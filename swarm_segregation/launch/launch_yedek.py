import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get directories
    leo_description = get_package_share_directory("leo_description")

    # ==================== LEADER ROBOT ====================
    leader_namespace = "robot_red"
    leader_color = "red"

    leader_robot_description = os.popen(
        f"xacro {leo_description}/urdf/leo_sim.urdf.xacro robot_ns:={leader_namespace} robot_color:={leader_color}"
    ).read()

    leader_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=leader_namespace,
        parameters=[{
            "use_sim_time": True,
            "robot_description": leader_robot_description,
        }],
        output="screen",
    )

    leader_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=leader_namespace,
        output="screen",
        arguments=[
            "-name", "leo_" + leader_namespace,
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1",
            "-topic", "robot_description"
        ]
    )

    leader_node = Node(
        package="swarm_segregation",
        executable="leader_node",
        name="leader_node",
        namespace=leader_namespace,
        parameters=[{"color": leader_color}],
        output="screen",
    )

    # ==================== FOLLOWER ROBOT ====================
    follower_namespace = "robot_blue"
    follower_color = "blue"

    follower_robot_description = os.popen(
        f"xacro {leo_description}/urdf/leo_sim.urdf.xacro robot_ns:={follower_namespace} robot_color:={follower_color}"
    ).read()

    follower_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=follower_namespace,
        parameters=[{
            "use_sim_time": True,
            "robot_description": follower_robot_description,
        }],
        output="screen",
    )

    follower_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=follower_namespace,
        output="screen",
        arguments=[
            "-name", "leo_" + follower_namespace,
            "-x", "1.0",
            "-y", "1.0",
            "-z", "0.1",
            "-topic", "robot_description"
        ]
    )

    follower_node = Node(
        package="swarm_segregation",
        executable="follower_node",
        name="follower_node",
        namespace=follower_namespace,
        output="screen",
    )

    follower_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    name="follower_bridge",
    arguments=[
        "/robot_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
        "/robot_blue/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        "/robot_blue/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
        "/robot_blue/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"
    ],
    parameters=[
        {"qos_overrides./tf_static.publisher.durability": "transient_local"}
    ],
    output="screen",
)


    # ==================== CLOCK BRIDGE ====================
    # Bridge ROS topics and Gazebo messages for establishing communication
    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    return LaunchDescription([
        topic_bridge,
        leader_state_publisher,
        leader_spawn,
        leader_node,
        follower_state_publisher,
        follower_spawn,
        follower_node,
        follower_bridge
    ])