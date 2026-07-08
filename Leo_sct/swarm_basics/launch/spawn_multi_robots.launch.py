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


def _build_origin_slots(total_robots: int, world_path: str):
    if total_robots != 1:
        raise RuntimeError("spawn_layout 'origin' is only valid with total_robots:=1.")

    obstacles = _load_world_obstacles(world_path)
    if not _point_is_free(0.0, 0.0, obstacles, wall_margin=0.6, obstacle_margin=0.55):
        raise RuntimeError(f"Origin is not a safe spawn point in {world_path}.")

    return [{"x": 0.0, "y": 0.0, "yaw": 0.0}]


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


def _build_random_safe_slots(total_robots: int, world_path: str, seed: int):
    obstacles = _load_world_obstacles(world_path)
    rng = random.Random(seed)
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


def _resolve_seed(value: str, name: str) -> int:
    text = str(value).strip().lower()
    if text in {"", "auto", "random"}:
        return random.SystemRandom().randint(1, 2_147_483_647)
    try:
        return int(text)
    except ValueError as exc:
        raise RuntimeError(
            f"Invalid {name} '{value}'. Use an integer seed or 'auto'."
        ) from exc


def generate_launch_description():

    leo_description = get_package_share_directory("leo_description")
    swarm_basics_dir = get_package_share_directory("swarm_basics")
    run_id = time.strftime("run_%Y%m%d_%H%M%S")

    auto_start_supervisor = LaunchConfiguration("auto_start_supervisor")
    auto_start_supervisor_arg = DeclareLaunchArgument(
        "auto_start_supervisor",
        default_value="true",
        description="Enable robot_supervisor_3_movements on launch",
    )
    spawn_layout_arg = DeclareLaunchArgument(
        "spawn_layout",
        default_value="spread",
        description="Robot spawn layout: 'spread', 'origin', 'middle_circle', or 'random_safe'.",
    )
    spawn_seed_arg = DeclareLaunchArgument(
        "spawn_seed",
        default_value="auto",
        description="Seed for randomized spawn layouts such as random_safe. Use 'auto' for a fresh seed each run.",
    )
    random_seed_arg = DeclareLaunchArgument(
        "random_seed",
        default_value="auto",
        description="Base seed for per-robot supervisor random choices. Use 'auto' for a fresh seed each run.",
    )
    sct_choice_mode_arg = DeclareLaunchArgument(
        "sct_choice_mode",
        default_value="random",
        description="SCT controllable choice mode: 'random' or deterministic 'first'.",
    )
    controller_mode_arg = DeclareLaunchArgument(
        "controller_mode",
        default_value="sct",
        description="Per-robot controller: 'sct' or 'random_walk_cbf'.",
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
        spawn_seed = _resolve_seed(LaunchConfiguration("spawn_seed").perform(context), "spawn_seed")
        random_seed = _resolve_seed(LaunchConfiguration("random_seed").perform(context), "random_seed")
        sct_choice_mode = LaunchConfiguration("sct_choice_mode").perform(context).strip().lower()
        controller_mode = LaunchConfiguration("controller_mode").perform(context).strip().lower()
        if controller_mode not in {"sct", "random_walk_cbf"}:
            raise RuntimeError(
                f"Unknown controller_mode '{controller_mode}'. Use 'sct' or 'random_walk_cbf'."
            )
        print(
            f"[spawn_multi_robots] run_id={run_id} spawn_seed={spawn_seed} "
            f"random_seed={random_seed} sct_choice_mode={sct_choice_mode} "
            f"controller_mode={controller_mode}",
            flush=True,
        )
        if spawn_layout == "spread":
            base_slots = _build_robot_spawn_slots(total_robots, world_path)
        elif spawn_layout == "origin":
            base_slots = _build_origin_slots(total_robots, world_path)
        elif spawn_layout == "middle_circle":
            base_slots = _build_middle_circle_slots(total_robots, world_path)
        elif spawn_layout == "random_safe":
            base_slots = _build_random_safe_slots(total_robots, world_path, spawn_seed)
        else:
            raise RuntimeError(
                f"Unknown spawn_layout '{spawn_layout}'. Use 'spread', 'origin', 'middle_circle', or 'random_safe'."
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

        # --- One bridge for all robots ---
        bridge_args = []
        for robot in robots:
            ns = robot["ns"]
            bridge_args += [
                f"/{ns}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                f"/{ns}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                f"/{ns}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                f"/{ns}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
                f"/{ns}/depth_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
                f"/{ns}/depth_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
                f"/{ns}/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                f"/world/random_world/model/{ns}/link/{ns}/base_footprint/sensor/contact_sensor/contact"
                f"@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
            ]

        bridge_node = Node(
            package="ros_ign_bridge", # ros_gz_bridge
            executable="parameter_bridge",
            name="all_robots_bridge",
            arguments=bridge_args,
            parameters=[{"qos_overrides./tf_static.publisher.durability": "transient_local"}],
            remappings=[
                *[(f"/{robot['ns']}/tf", "/tf") for robot in robots],
            ],
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
            robot_description = doc.toxml().replace(
                "<frame_id>odom</frame_id>",
                f"<frame_id>{ns}/odom</frame_id>",
            )

            # State publisher (per robot)
            state_pub = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=ns,
                parameters=[{
                    "use_sim_time": True,
                    "robot_description": robot_description
                }],
                remappings=[
                    ("joint_states", f"/{ns}/joint_states"),
                    ("tf", "/tf"),
                    ("tf_static", "/tf_static"),
                ],
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

            spawn_offset_tf = Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"{ns}_spawn_offset_tf",
                arguments=[
                    str(x),
                    str(y),
                    "0.0",
                    str(yaw),
                    "0.0",
                    "0.0",
                    "odom",
                    f"{ns}/odom",
                ],
                output="screen",
            )

            # Controller node (per robot)
            if controller_mode == "random_walk_cbf":
                behavior_node = Node(
                    package="swarm_basics",
                    executable="random_walk_cbf_controller",
                    name="random_walk_cbf_controller",
                    namespace=ns,
                    parameters=[
                        {"enabled": auto_start_supervisor},
                        {"random_seed": random_seed},
                        {"run_id": run_id},
                        {"results_dir": LaunchConfiguration("results_dir")},
                        {"cbf_filter_log_enabled": False},
                    ],
                    output="screen",
                )
            else:
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
                        {"random_seed": random_seed},
                        {"sct_choice_mode": sct_choice_mode},
                        {"run_id": run_id},
                        {"results_dir": LaunchConfiguration("results_dir")},
                        {"peer_warning_log_enabled": False},
                    ],
                    output="screen",
                )

            proximity_warner_node = Node(
                package="swarm_basics",
                executable="robot_proximity_warner",
                name="robot_proximity_warner",
                namespace=ns,
                parameters=[
                    {"total_robots": total_robots},
                    {"warning_distance_m": 1.0},
                    {"critical_distance_m": 0.55},
                    {"zone_timeout_s": 0.8},
                    {"publish_period_s": 0.1},
                    {"publish_to_self": False},
                    {"run_id": run_id},
                    {"results_dir": LaunchConfiguration("results_dir")},
                    {"warner_diagnostic_log_enabled": True},
                    {"diagnostic_distance_margin_m": 0.25},
                ],
                output="screen",
            )

            robot_id_warning_relay_node = Node(
                package="swarm_basics",
                executable="robot_id_warning_relay",
                name="robot_id_warning_relay",
                namespace=ns,
                parameters=[
                    {"total_robots": total_robots},
                    {"warning_distance_m": 1.0},
                    {"warning_source": "global_pose"},
                    {"critical_distance_m": 0.55},
                    {"front_angle_rad": math.radians(50.0)},
                    {"pose_timeout_s": 0.6},
                    {"publish_period_s": 0.1},
                    {"min_confidence": 0.0},
                    {"classified_robot_detections_topic": "classified_robot_detections"},
                    {"no_detection_warn_period_s": 10.0},
                    {"run_id": run_id},
                    {"results_dir": LaunchConfiguration("results_dir")},
                    {"robot_id_warning_log_enabled": True},
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
                    {"zone_log_enabled": True},
                    {"process_every_nth_depth_frame": 1},
                    {"enter_thresh": 0.70},
                    {"exit_thresh": 0.90},
                    {"side_enter_thresh": 0.70},
                    {"side_exit_thresh": 0.90},
                    {"hold_ms": 250},
                    {"corner_hold_ms": 250},
                    {"side_hold_ms": 250},
                    {"depth_watchdog_enabled": True},
                    {"depth_stall_timeout_s": 4.0},
                ],
                output="screen"
            )

            nodes += [
                state_pub,
                spawn_node,
                spawn_offset_tf,
                behavior_node,
                proximity_warner_node,
                robot_id_warning_relay_node,
                TimerAction(
                    period=2.0,
                    actions=[cpp_node],
                ),
            ]

        return nodes

    return LaunchDescription([
        auto_start_supervisor_arg,
        spawn_layout_arg,
        spawn_seed_arg,
        random_seed_arg,
        sct_choice_mode_arg,
        controller_mode_arg,
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
                {"global_mode": True},
                {"run_id": run_id},
                {"results_dir": LaunchConfiguration("results_dir")},
                {"total_robots": LaunchConfiguration("total_robots")},
                {"detection_context_timeout_s": 1.0},
            ],
            output="screen",
        ),
        OpaqueFunction(function=create_all_robot_nodes)
    ])
