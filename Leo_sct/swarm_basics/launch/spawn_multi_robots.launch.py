import os
import math
import time
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

    # total_robots_arg = LaunchConfiguration("total_robots")

    plot_node = Node(
            package="swarm_basics",
            executable="coverage_counter",
            name="coverage_counter",
            parameters=[
                {"run_duration": LaunchConfiguration("run_duration")},
                {"results_dir": LaunchConfiguration("results_dir")},
            ],
            output="screen"
    )
 

    # --- Function to create all robot nodes ---
    def create_all_robot_nodes(context):
        total_robots_value = int(LaunchConfiguration("total_robots").perform(context))
        total_robots = max(1, total_robots_value)

        center_x = -2.0
        center_y = 0.0
        robot_spacing = 1.25
        robots = []
        start_y = center_y - 0.5 * robot_spacing * (total_robots - 1)
        for i in range(total_robots):
            robots.append({
                "ns": f"robot_{i}",
                "x": center_x,
                "y": start_y + i * robot_spacing,
                "yaw": 0.0,
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
            nodes += [
                marker_spawn,
                TimerAction(
                    period=2.0,
                    actions=[marker_mover],
                ),
            ]

        # --- One bridge for all robots ---
        bridge_args = []
        for robot in robots:
            ns = robot["ns"]
            bridge_args += [
                f"/{ns}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                f"/{ns}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                # f"/{ns}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                f"/{ns}/depth_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",
                f"/{ns}/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
                f"/{ns}/depth_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
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
                    {"rgb_topic": f"/{ns}/depth_camera/image"},
                    {"aruco_dictionary_id": 0},
                    {"aruco_target_id": 0},
                    {"aruco_seen_hold_ms": 200},
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
                cpp_node,
                aruco_tracker_node,
                aruco_follower_node,
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
                {"results_dir": LaunchConfiguration("results_dir")},
            ],
            output="screen",
        ),
        OpaqueFunction(function=create_all_robot_nodes)
    ])
