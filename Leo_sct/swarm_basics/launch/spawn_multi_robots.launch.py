import os
import math
import random
import time
import xml.etree.ElementTree as ET
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnShutdown
from launch.actions import OpaqueFunction, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from swarm_basics.launch_defaults import MOVING_ARUCO_DEFAULTS


def _parse_pose(pose_text: str | None):
    if not pose_text:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    vals = [float(v) for v in pose_text.split()]
    while len(vals) < 6:
        vals.append(0.0)
    return tuple(vals[:6])


def _load_world_obstacles(world_path: str):
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    if world is None:
        return []

    obstacles = []
    for model in world.findall("model"):
        name = model.get("name", "")
        if name == "ground_plane":
            continue
        model_pose = _parse_pose(model.findtext("pose"))
        for link in model.findall("link"):
            link_pose = _parse_pose(link.findtext("pose"))
            for collision in link.findall("collision"):
                collision_pose = _parse_pose(collision.findtext("pose"))
                cx = model_pose[0] + link_pose[0] + collision_pose[0]
                cy = model_pose[1] + link_pose[1] + collision_pose[1]
                yaw = model_pose[5] + link_pose[5] + collision_pose[5]

                box_size = collision.findtext("geometry/box/size")
                cyl_radius = collision.findtext("geometry/cylinder/radius")
                if box_size:
                    sx, sy, _ = (float(v) for v in box_size.split())
                    obstacles.append(("box", cx, cy, sx, sy, yaw))
                elif cyl_radius:
                    obstacles.append(("cylinder", cx, cy, float(cyl_radius)))
    return obstacles


def _point_in_rotated_box(px: float, py: float, cx: float, cy: float, sx: float, sy: float, yaw: float, margin: float):
    dx = px - cx
    dy = py - cy
    c = math.cos(-yaw)
    s = math.sin(-yaw)
    lx = dx * c - dy * s
    ly = dx * s + dy * c
    return abs(lx) <= (sx * 0.5 + margin) and abs(ly) <= (sy * 0.5 + margin)


def _point_is_free(px: float, py: float, obstacles, wall_margin: float, obstacle_margin: float):
    if abs(px) > (5.0 - wall_margin) or abs(py) > (5.0 - wall_margin):
        return False
    for obstacle in obstacles:
        if obstacle[0] == "box":
            _, cx, cy, sx, sy, yaw = obstacle
            if _point_in_rotated_box(px, py, cx, cy, sx, sy, yaw, obstacle_margin):
                return False
        else:
            _, cx, cy, radius = obstacle
            if math.hypot(px - cx, py - cy) <= (radius + obstacle_margin):
                return False
    return True


def _build_robot_spawn_slots(total_robots: int, world_path: str):
    obstacles = _load_world_obstacles(world_path)
    candidate_x = [-3.25, -1.75, -0.25, 1.25, 2.75, 3.5]
    candidate_y = [-3.25, -1.75, -0.25, 1.25, 2.75, 3.5]
    min_robot_spacing = 1.25
    wall_margin = 0.6
    obstacle_margin = 0.55

    candidates = []
    for y in candidate_y:
        for x in candidate_x:
            if _point_is_free(x, y, obstacles, wall_margin, obstacle_margin):
                # Prefer far-apart outer slots first so robots start dispersed.
                candidates.append((x, y))

    if not candidates:
        raise RuntimeError(f"No safe spawn slots found in {world_path}.")

    robots = []
    # Greedy farthest-point sampling:
    # 1) start with the outermost free slot
    # 2) repeatedly add the slot with the largest distance to the current set
    first_x, first_y = max(candidates, key=lambda p: abs(p[0]) + abs(p[1]))
    robots.append({"x": first_x, "y": first_y, "yaw": 0.0})

    remaining = [p for p in candidates if p != (first_x, first_y)]
    while remaining and len(robots) < total_robots:
        feasible = []
        for x, y in remaining:
            distances = [math.hypot(x - robot["x"], y - robot["y"]) for robot in robots]
            min_dist = min(distances)
            if min_dist >= min_robot_spacing:
                feasible.append((min_dist, abs(x) + abs(y), x, y))
        if not feasible:
            break
        # Prefer the point with the largest minimum distance to existing robots.
        # Break ties toward outer slots.
        _, _, x, y = max(feasible, key=lambda item: (item[0], item[1]))
        robots.append({"x": x, "y": y, "yaw": 0.0})
        remaining = [p for p in remaining if p != (x, y)]

    if len(robots) < total_robots:
        raise RuntimeError(
            f"Only found {len(robots)} safe spawn slots in {world_path}, need {total_robots}."
        )
    return robots


def _build_middle_circle_slots(total_robots: int, world_path: str):
    obstacles = _load_world_obstacles(world_path)
    center_x = 0.0
    center_y = 0.0
    wall_margin = 0.6
    obstacle_margin = 0.45

    # Try a few radii until every robot can be placed safely on the ring.
    for radius in (1.2, 1.4, 1.6, 1.8, 2.0):
        robots = []
        ok = True
        for i in range(total_robots):
            angle = (2.0 * math.pi * i) / float(total_robots)
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            if not _point_is_free(x, y, obstacles, wall_margin, obstacle_margin):
                ok = False
                break
            robots.append({
                "x": x,
                "y": y,
                # Face outward from the circle center.
                "yaw": angle,
            })
        if ok:
            return robots

    raise RuntimeError(
        f"Could not find a safe middle_circle spawn ring for {total_robots} robots in {world_path}."
    )


def _build_random_safe_slots(total_robots: int, world_path: str):
    obstacles = _load_world_obstacles(world_path)
    rng = random.Random(time.time_ns())
    min_robot_spacing = 1.25
    wall_margin = 0.75
    obstacle_margin = 0.7
    max_attempts = 10000

    robots = []
    for _ in range(max_attempts):
        if len(robots) >= total_robots:
            break
        x = rng.uniform(-5.0 + wall_margin, 5.0 - wall_margin)
        y = rng.uniform(-5.0 + wall_margin, 5.0 - wall_margin)
        if not _point_is_free(x, y, obstacles, wall_margin, obstacle_margin):
            continue
        if any(math.hypot(x - robot["x"], y - robot["y"]) < min_robot_spacing for robot in robots):
            continue
        robots.append({
            "x": x,
            "y": y,
            "yaw": rng.uniform(-math.pi, math.pi),
        })

    if len(robots) < total_robots:
        raise RuntimeError(
            f"Only sampled {len(robots)} random safe spawn slots in {world_path}, "
            f"need {total_robots}."
        )
    return robots


def generate_launch_description():

    leo_description = get_package_share_directory("leo_description")
    swarm_basics_dir = get_package_share_directory("swarm_basics")
    leo_follow_aruco_dir = get_package_share_directory("leo_example_follow_aruco_marker")
    run_id = time.strftime("run_%Y%m%d_%H%M%S")

    auto_start_supervisor = LaunchConfiguration("auto_start_supervisor")
    spawn_moving_aruco = LaunchConfiguration("spawn_moving_aruco")
    moving_aruco_x = LaunchConfiguration("moving_aruco_x")
    moving_aruco_y = LaunchConfiguration("moving_aruco_y")
    moving_aruco_z = LaunchConfiguration("moving_aruco_z")
    moving_aruco_radius = LaunchConfiguration("moving_aruco_radius")
    moving_aruco_speed = LaunchConfiguration("moving_aruco_speed")
    moving_aruco_update_rate = LaunchConfiguration("moving_aruco_update_rate")
    auto_start_supervisor_arg = DeclareLaunchArgument(
        "auto_start_supervisor",
        default_value="true",
        description="Enable robot_supervisor_3_movements on launch",
    )
    spawn_moving_aruco_arg = DeclareLaunchArgument(
        "spawn_moving_aruco",
        default_value=MOVING_ARUCO_DEFAULTS["spawn_moving_aruco"],
        description="Spawn a moving ArUco target box in Gazebo.",
    )
    moving_aruco_x_arg = DeclareLaunchArgument(
        "moving_aruco_x",
        default_value=MOVING_ARUCO_DEFAULTS["moving_aruco_x"],
        description="Center X position for the moving ArUco target square path.",
    )
    moving_aruco_y_arg = DeclareLaunchArgument(
        "moving_aruco_y",
        default_value=MOVING_ARUCO_DEFAULTS["moving_aruco_y"],
        description="Center Y position for the moving ArUco target square path.",
    )
    moving_aruco_z_arg = DeclareLaunchArgument(
        "moving_aruco_z",
        default_value=MOVING_ARUCO_DEFAULTS["moving_aruco_z"],
        description="Z position for the moving ArUco target.",
    )
    moving_aruco_radius_arg = DeclareLaunchArgument(
        "moving_aruco_radius",
        default_value=MOVING_ARUCO_DEFAULTS["moving_aruco_radius"],
        description="Side length of the square ArUco path.",
    )
    moving_aruco_speed_arg = DeclareLaunchArgument(
        "moving_aruco_speed",
        default_value=MOVING_ARUCO_DEFAULTS["moving_aruco_speed"],
        description="Linear speed in m/s along the square ArUco path.",
    )
    moving_aruco_update_rate_arg = DeclareLaunchArgument(
        "moving_aruco_update_rate",
        default_value=MOVING_ARUCO_DEFAULTS["moving_aruco_update_rate"],
        description="Pose update rate in Hz for the moving ArUco target.",
    )
    spawn_layout_arg = DeclareLaunchArgument(
        "spawn_layout",
        default_value="spread",
        description="Robot spawn layout: 'spread', 'middle_circle', or 'random_safe'.",
    )

    # total_robots_arg = LaunchConfiguration("total_robots")

    plot_node = Node(
            package="swarm_basics",
            executable="coverage_counter",
            name="coverage_counter",
            parameters=[
                {"run_id": run_id},
                {"run_duration": LaunchConfiguration("run_duration")},
                {"results_dir": LaunchConfiguration("results_dir")},
                {"metadata_yaml_path": LaunchConfiguration("metadata_yaml_path")},
                {"prompt_text": LaunchConfiguration("prompt_text")},
                {"prompt_file_path": LaunchConfiguration("prompt_file_path")},
            ],
            output="screen"
    )
 

    # --- Function to create all robot nodes ---
    def create_all_robot_nodes(context):
        total_robots_value = int(LaunchConfiguration("total_robots").perform(context))
        total_robots = max(1, total_robots_value)
        world_path = os.path.join(swarm_basics_dir, "worlds", "random_world.sdf")
        spawn_layout = LaunchConfiguration("spawn_layout").perform(context).strip().lower()
        if spawn_layout == "spread":
            base_slots = _build_robot_spawn_slots(total_robots, world_path)
        elif spawn_layout == "middle_circle":
            base_slots = _build_middle_circle_slots(total_robots, world_path)
        elif spawn_layout == "random_safe":
            base_slots = _build_random_safe_slots(total_robots, world_path)
        else:
            raise RuntimeError(
                f"Unknown spawn_layout '{spawn_layout}'. Use 'spread', 'middle_circle', or 'random_safe'."
            )
        robots = []
        for i, slot in enumerate(base_slots):
            robots.append({
                "ns": f"robot_{i}",
                "x": slot["x"],
                "y": slot["y"],
                "yaw": slot["yaw"],
            })

        nodes = []

        if LaunchConfiguration("spawn_moving_aruco").perform(context).lower() == "true":
            marker_model_path = os.path.join(
                swarm_basics_dir,
                "models",
                "aruco_marker_0",
                "model.sdf",
            )
            marker_spawn = Node(
                package="ros_gz_sim",
                executable="create",
                name="moving_aruco_box_spawner",
                arguments=[
                    "-name", "moving_aruco_box",
                    "-x", str(float(moving_aruco_x.perform(context)) - 0.5 * float(moving_aruco_radius.perform(context))),
                    "-y", str(float(moving_aruco_y.perform(context)) - 0.5 * float(moving_aruco_radius.perform(context))),
                    "-z", moving_aruco_z,
                    "-file", marker_model_path,
                ],
                output="screen",
            )
            marker_mover = Node(
                package="swarm_basics",
                executable="aruco_mover",
                name="aruco_mover",
                parameters=[
                    {"world_name": "random_world"},
                    {"entity_name": "moving_aruco_box"},
                    {"center_x": moving_aruco_x},
                    {"center_y": moving_aruco_y},
                    {"z": moving_aruco_z},
                    {"radius": moving_aruco_radius},
                    {"angular_speed": moving_aruco_speed},
                    {"update_rate_hz": moving_aruco_update_rate},
                ],
                output="screen",
            )
            # nodes += [
            #     marker_spawn,
            #     TimerAction(
            #         period=2.0,
            #         actions=[marker_mover],
            #     ),
            # ]

        # --- One bridge for all robots ---
        bridge_args = []
        for robot in robots:
            ns = robot["ns"]
            bridge_args += [
                f"/{ns}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                f"/{ns}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                # f"/{ns}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                f"/{ns}/depth_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
                # RGB image bridge disabled while ArUco is not in use.
                # f"/{ns}/depth_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
                f"/world/random_world/model/{ns}/link/{ns}/base_footprint/sensor/contact_sensor/contact"
                f"@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
            ]

        bridge_args += [
            "/world/random_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
        ]    

        bridge_node = Node(
            package="ros_ign_bridge", # ros_gz_bridge
            executable="parameter_bridge",
            name="all_robots_bridge",
            arguments=bridge_args,
            parameters=[{"qos_overrides./tf_static.publisher.durability": "transient_local"}],
            output="screen"
        )
        nodes.append(bridge_node)

        # --- Create each robot ---
        for robot in robots:
            ns = robot["ns"]
            x = robot["x"]
            y = robot["y"]
            yaw = robot["yaw"]

            # URDF with per-robot namespace mapping
            xacro_file = os.path.join(leo_description, 'urdf', 'leo_sim.urdf.xacro')
            doc = xacro.process_file(xacro_file, mappings={"robot_ns": ns})
            robot_description = doc.toxml()

            # State publisher (per robot)
            state_pub = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=ns,
                parameters=[{
                    "use_sim_time": True,
                    "robot_description": robot_description
                }],
                remappings=[("/joint_states", f"{ns}/joint_states")],
                output="screen"
            )

            # Spawn robot in Gazebo
            spawn_node = Node(
                package="ros_gz_sim",
                executable="create",
                namespace=ns,
                arguments=[
                    "-name", ns,
                    "-x", str(x),
                    "-y", str(y),
                    "-z", "0.1",
                    "-Y", str(yaw),
                    "-topic", f"/{ns}/robot_description"
                ],
                output="screen"
            )

            # Controller node (per robot)
            behavior_node = Node(
                package="swarm_basics",
                executable="robot_supervisor",
                name="robot_supervisor",
                namespace=ns,
                parameters=[
                    {"spawn_x": x},
                    {"spawn_y": y},
                    {"enabled": auto_start_supervisor},
                    {"supervisor_yaml_path": LaunchConfiguration("metadata_yaml_path")},
                ],
                output="screen",
            )

            cpp_node = Node(
                package="leo_image",
                executable="image_processor",
                name="image_processor",
                namespace=ns,
                parameters=[
                    {"use_sim_time": True},
                    {"depth_topic": f"/{ns}/depth_camera/depth_image"},
                    {"aruco_enabled": False},
                    {"zone_log_enabled": True},
                    {"process_every_nth_depth_frame": 5},
                    {"depth_watchdog_enabled": True},
                    {"depth_stall_timeout_s": 4.0},
                    # RGB / ArUco disabled for current runs.
                    # {"rgb_topic": f"/{ns}/depth_camera/image"},
                    # {"aruco_dictionary_id": 0},
                    # {"aruco_target_id": 0},
                    # {"aruco_seen_hold_ms": 200},
                ],
                output="screen"
            )

            aruco_tracker_node = Node(
                package="aruco_opencv",
                executable="aruco_tracker_autostart",
                name="aruco_tracker",
                namespace=ns,
                parameters=[
                    os.path.join(leo_follow_aruco_dir, "config", "tracker.yaml"),
                    {
                        "use_sim_time": True,
                        "cam_base_topic": "depth_camera/image",
                        "output_frame": "",
                        "publish_tf": False,
                        "marker_size": 0.15,
                    },
                ],
                output="screen",
            )

            aruco_follower_node = Node(
                package="leo_example_follow_aruco_marker",
                executable="aruco_follower",
                name="aruco_follower",
                namespace=ns,
                parameters=[
                    os.path.join(leo_follow_aruco_dir, "config", "follower.yaml"),
                    {
                        "use_sim_time": True,
                        "follow_id": 0,
                        "follow_enabled": True,
                    },
                ],
                remappings=[
                    ("merged_odom", "odom"),
                    ("cmd_vel", "aruco_follower/cmd_vel"),
                ],
                output="screen",
            )

            nodes += [
                state_pub,
                spawn_node,
                behavior_node,
                TimerAction(
                    period=2.0,
                    actions=[cpp_node],
                ),
                # aruco_tracker_node,
                # aruco_follower_node,
            ]

        return nodes

    return LaunchDescription([
        auto_start_supervisor_arg,
        spawn_moving_aruco_arg,
        moving_aruco_x_arg,
        moving_aruco_y_arg,
        moving_aruco_z_arg,
        moving_aruco_radius_arg,
        moving_aruco_speed_arg,
        moving_aruco_update_rate_arg,
        spawn_layout_arg,
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    ExecuteProcess(
                        cmd=["bash", "-lc", "pkill -f parameter_bridge || true"],
                        output="screen",
                    )
                ]
            )
        ),
        TimerAction(
            period=LaunchConfiguration("run_duration"),
            actions=[Shutdown(reason="run_duration reached")],
        ),
        plot_node,
        Node(
            package="swarm_basics",
            executable="bump_counter",
            name="bump_counter",
            parameters=[
                {"run_id": run_id},
                {"results_dir": LaunchConfiguration("results_dir")},
            ],
            output="screen",
        ),
        OpaqueFunction(function=create_all_robot_nodes)
    ])
