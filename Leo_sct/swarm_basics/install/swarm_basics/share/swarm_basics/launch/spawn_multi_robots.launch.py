import os
import xacro

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    leo_description = get_package_share_directory("leo_description")

    # --- Total robots ---
    total_robots = 1

    # --- Initial positions for each robot ---
    robot_positions = [
        (0.0, 0.0),
        # (1.0, 0.0),
        # (0.0, 1.0),
        # (-1.0, 1.0),
        # (-1.0, 0.0),
        # (0.0, -1.0),
        # (2.0, 0.0),
        # (0.0, 2.0),
        # (-2.0, 2.0),
        # (-2.0, 0.0),
    ]

    robots_to_spawn = []
    for i in range(total_robots):
        x, y = robot_positions[i]
        robots_to_spawn.append({
            "ns": f"robot_{i}",
            "x": x,
            "y": y
        })

    plot_node = Node(
            package="swarm_basics",
            executable="coverage_plotter",
            name="coverage_plotter",
            output="screen"
    )       
 

    # --- Function to create all robot nodes ---
    def create_all_robot_nodes(context, robots):
        nodes = []

        # --- One bridge for all robots ---
        bridge_args = []
        for robot in robots:
            ns = robot["ns"]
            bridge_args += [
                f"/{ns}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                f"/{ns}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                f"/{ns}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
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
            package="ros_gz_bridge",
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
                    "-topic", f"/{ns}/robot_description"
                ],
                output="screen"
            )

            # Controller node (per robot)
            behavior_node = Node(
                package="swarm_basics",
                executable="robot_supervisor_3_movements",
                name="robot_supervisor",
                namespace=ns,
                parameters=[
                    {"spawn_x": x},
                    {"spawn_y": y}
                ],
                output="screen"
            )

            cpp_node = Node(
                package="leo_image",
                executable="image_processor",
                name="image_processor",
                namespace=ns,
                output="screen"
            )

            bump_node = Node(
                package="swarm_basics",
                executable="bump_counter",
                name="bump_counter",
                namespace=ns,
                output="screen",
                remappings=[
                    ('contact', f"/world/random_world/model/{ns}/link/{ns}/base_footprint/sensor/contact_sensor/contact"),
                ],
            )

            nodes += [state_pub, spawn_node, behavior_node, cpp_node, bump_node]

        return nodes

    return LaunchDescription([
        plot_node,
        OpaqueFunction(function=lambda context: create_all_robot_nodes(context, robots_to_spawn))
    ])