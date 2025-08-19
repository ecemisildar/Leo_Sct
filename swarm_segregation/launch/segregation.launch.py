import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import xacro


def create_robot(ns, color, x, y, is_leader):
    pkg_project_description = get_package_share_directory("leo_description")

    # Process URDF with namespace + color
    robot_desc = xacro.process(
        os.path.join(pkg_project_description, "urdf", "leo_sim.urdf.xacro"),
        mappings={
            "robot_ns": ns,
            "color": color,   # pass color if your URDF supports it
            "is_leader": str(is_leader).lower(),
        },
    )

    robot_gazebo_name = ns
    node_name_prefix = ns + "_"

    robot_state_publisher = Node(
        namespace=ns,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_desc},
        ],
    )

    leo_rover = Node(
        namespace=ns,
        package="ros_gz_sim",
        executable="create",
        name="ros_gz_sim_create",
        output="both",
        arguments=[
            "-topic", "robot_description",
            "-name", robot_gazebo_name,
            "-z", "1.00",
            "-x", str(x),
            "-y", str(y),
        ],
    )

    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=node_name_prefix + "parameter_bridge",
        arguments=[
            ns + "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            ns + "/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            ns + "/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
            ns + "/imu/data_raw@sensor_msgs/msg/Imu@ignition.msgs.IMU",
            ns + "/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            ns + "/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model",
        ],
        parameters=[{"qos_overrides./tf_static.publisher.durability": "transient_local"}],
        output="screen",
    )

    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name=node_name_prefix + "image_bridge",
        arguments=[ns + "/camera/image_raw"],
        output="screen",
    )

    return [robot_state_publisher, leo_rover, topic_bridge, image_bridge]


def generate_launch_description():
    launch_description = []

    # Spawn 3 leaders
    leader_colors = ["red", "green", "yellow"]
    for i in range(3):
        ns = f"robot_leader_{i}"
        color = leader_colors[i % len(leader_colors)]
        x, y = i * 3.0, 0.0
        launch_description += create_robot(ns, color, x, y, is_leader=True)

    # Spawn 5 followers
    for i in range(5):
        ns = f"robot_follower_{i}"
        color = "blue"
        x, y = i * 1.5, 2.0
        launch_description += create_robot(ns, color, x, y, is_leader=False)

    return LaunchDescription(launch_description)
