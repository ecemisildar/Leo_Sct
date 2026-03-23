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


def generate_launch_description():

    leo_description = get_package_share_directory("leo_description")
    swarm_basics_dir = get_package_share_directory("swarm_basics")
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
        default_value="true",
        description="Spawn a moving ArUco target box in Gazebo.",
    )
    moving_aruco_x_arg = DeclareLaunchArgument(
        "moving_aruco_x",
        default_value="1.0",
        description="Center X position for the moving ArUco target.",
    )
    moving_aruco_y_arg = DeclareLaunchArgument(
        "moving_aruco_y",
        default_value="1.0",
        description="Center Y position for the moving ArUco target.",
    )
    moving_aruco_z_arg = DeclareLaunchArgument(
        "moving_aruco_z",
        default_value="0.25",
        description="Z position for the moving ArUco target.",
    )
    moving_aruco_radius_arg = DeclareLaunchArgument(
        "moving_aruco_radius",
        default_value="1.5",
        description="Radius of the circular ArUco motion.",
    )
    moving_aruco_speed_arg = DeclareLaunchArgument(
        "moving_aruco_speed",
        default_value="0.35",
        description="Angular speed in rad/s for the moving ArUco target.",
    )
    moving_aruco_update_rate_arg = DeclareLaunchArgument(
        "moving_aruco_update_rate",
        default_value="5.0",
        description="Pose update rate in Hz for the moving ArUco target.",
    )

    # total_robots_arg = LaunchConfiguration("total_robots")

    plot_node = Node(
            package="swarm_basics",
            executable="coverage_counter",
            name="coverage_counter",
            parameters=[
                {"run_duration": LaunchConfiguration("run_duration")},
                {"run_id": run_id},
                {"results_dir": LaunchConfiguration("results_dir")},
            ],
            output="screen"
    )
 

    # --- Function to create all robot nodes ---
    def create_all_robot_nodes(context):
        total_robots_value = int(LaunchConfiguration("total_robots").perform(context))
        total_robots = max(1, total_robots_value)

        center_x = 0.0
        center_y = 0.0
        formation_radius = 2.0
        robots = []
        for i in range(total_robots):
            yaw = (2 * math.pi / total_robots) * i
            robots.append({
                "ns": f"robot_{i}",
                "x": center_x + formation_radius * math.cos(yaw),
                "y": center_y + formation_radius * math.sin(yaw),
                "yaw": yaw
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
                    "-x", str(float(moving_aruco_x.perform(context)) + float(moving_aruco_radius.perform(context))),
                    "-y", moving_aruco_y,
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
                # f"/{ns}/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
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
                    {"aruco_dictionary_id": 1},
                    {"aruco_target_id": 0},
                ],
                output="screen"
            )

            nodes += [state_pub, spawn_node, behavior_node, cpp_node]

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
                {"run_id": run_id},
                {"results_dir": LaunchConfiguration("results_dir")},
            ],
            output="screen",
        ),
        OpaqueFunction(function=create_all_robot_nodes)
    ])
