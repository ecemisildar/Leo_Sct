import os
import xacro

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    leo_description = get_package_share_directory("leo_description")

    # --- Color map ---
    color_map = {
        "red": "1 0 0 1",
        "green": "0 1 0 1",
        "yellow": "1 1 0 1",
        "blue": "0 0 1 1"
    }

    robots_to_spawn = []

    # --- Followers ---
    follower_positions = [
        (3.0, 0.0), (2.0, 1.0), (4.0, 1.0), (3.0, -3.0), (0.0, 3.0),
        (2.0, 0.0), (1.0, 1.0), (5.0, 1.0), (2.0, -3.0), (0.0, 5.0)]
    for i, pos in enumerate(follower_positions):
        robots_to_spawn.append({
            "ns": f"robot_follower_{i}",
            "color": color_map["yellow"],
            "x": pos[0],
            "y": pos[1]
        })

    # --- Function to create all robot nodes ---
    def create_all_robot_nodes(context, robots):
        nodes = []

        # --- Single bridge for all robots ---
        bridge_args = []
        for robot in robots:
            ns = robot["ns"]
            bridge_args += [
                f"/{ns}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                f"/{ns}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                f"/{ns}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                f"/{ns}/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                f"/{ns}/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
                f"/{ns}/camera/image_raw@sensor_msgs/msg/CompressedImage[ignition.msgs.Image",
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
        for i, robot in enumerate(robots):
            ns = robot["ns"]
            color = robot["color"]
            x = robot["x"]
            y = robot["y"]

            # URDF
            xacro_file = os.path.join(leo_description, 'urdf', 'leo_sim.urdf.xacro')
            doc = xacro.process_file(xacro_file, mappings={
                "robot_ns": ns,
                "chassis_color": color,
            })
            robot_description = doc.toxml()

            # State publisher
            state_pub = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=ns,
                parameters=[{"use_sim_time": True, "robot_description": robot_description}],
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

            # Behavior node
            behavior_node = Node(
                package="swarm_aggregation",
                executable="aggregation_node",
                name="aggregation_node",
                namespace=ns,
                parameters=[
                    {"color": color},
                    {"spawn_x": x},
                    {"spawn_y": y},
                ],
                output="screen"
            )

            # # Camera image bridge
            # image_bridge = Node(
            #     package="ros_gz_image",
            #     executable="image_bridge",
            #     name="image_bridge",
            #     arguments=[f"/{ns}/camera/image_raw"],
            #     output="screen",
            # )

            # Only start behavior node after spawn finishes
            behavior_after_spawn = RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_node,
                    on_exit=[behavior_node]
                )
            )

            # Add all nodes for this robot
            nodes += [state_pub, spawn_node, behavior_after_spawn]

        return nodes

    # --- Launch all robots in parallel ---
    return LaunchDescription([
        OpaqueFunction(function=lambda context: create_all_robot_nodes(context, robots_to_spawn))
    ])
