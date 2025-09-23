import os
import xacro

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener


def generate_launch_description():

    leo_description = get_package_share_directory("leo_description")

    # --- Total robots (edit here if you want more/less) ---
    total_robots = 6

    # --- Initial positions for each robot ---
    robot_positions = [
        (0.0, 0.0),
        (3.0, 0.0),
        (0.0, 3.0),
        (3.0, 3.0),
        (5.0, 0.0),
        (0.0, 5.0),
    ]

    robots_to_spawn = []
    for i in range(total_robots):
        x, y = robot_positions[i]
        robots_to_spawn.append({
            "ns": f"robot_{i}",
            "x": x,
            "y": y
        })

    # --- Function to create all robot nodes ---
    def create_all_robot_nodes(context, robots):
        nodes = []

        # One bridge for all robots
        bridge_args = []
        for robot in robots:
            ns = robot["ns"]
            bridge_args += [
                f"/{ns}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                f"/{ns}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                f"/{ns}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                f"/{ns}/depth_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",
                f"/{ns}/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            ]

        bridge_node = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="all_robots_bridge",
            arguments=bridge_args,
            parameters=[{"qos_overrides./tf_static.publisher.durability": "transient_local"}],
            output="screen"
        )
        nodes.append(bridge_node)

        # Create each robot
        for robot in robots:
            ns = robot["ns"]
            x = robot["x"]
            y = robot["y"]

            # URDF
            xacro_file = os.path.join(leo_description, 'urdf', 'leo_sim.urdf.xacro')
            doc = xacro.process_file(xacro_file, mappings={
                "robot_ns": ns,
            })
            robot_description = doc.toxml()

            # State publisher
            state_pub = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=ns,
                parameters=[{
                    "use_sim_time": True,
                    "robot_description": robot_description,
                    "tf_prefix": ns
                }],
                output="screen"
            )

            # Spawn robot
            spawn_node = Node(
                package="ros_gz_sim",
                executable="create",
                namespace=ns,
                output="screen",
                arguments=[
                    "-name", ns,
                    "-x", str(x),
                    "-y", str(y),
                    "-z", "0.1",
                    "-topic", "robot_description"
                ]
            )

            # Controller node
            behavior_node = Node(
                package="swarm_basics",
                executable="robot_circle_controller",
                name="robot_circle_controller",
                namespace=ns,
                parameters=[
                    {"spawn_x": x},
                    {"spawn_y": y}
                ],
                output="screen"
            )

            # Start controller after spawn
            behavior_after_spawn = RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_node,
                    on_exit=[behavior_node]
                )
            )

            nodes += [state_pub, spawn_node, behavior_after_spawn]

        return nodes

    return LaunchDescription([
        OpaqueFunction(function=lambda context: create_all_robot_nodes(context, robots_to_spawn))
    ])
