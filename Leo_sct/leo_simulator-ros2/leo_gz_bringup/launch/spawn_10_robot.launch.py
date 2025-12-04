import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def spawn_robot(robot_ns: str, x: float = 0.0, y: float = 0.0, z: float = 0.1):
    pkg_project_description = get_package_share_directory("leo_description")

    robot_desc = xacro.process(
        os.path.join(pkg_project_description, "urdf", "leo_sim.urdf.xacro"),
        mappings={"robot_ns": robot_ns},
    )

    robot_gazebo_name = "leo_rover" if robot_ns == "" else "leo_rover_" + robot_ns
    node_name_prefix = "" if robot_ns == "" else robot_ns + "_"

    # Robot state publisher
    robot_state_publisher = Node(
        namespace=robot_ns,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_desc},
        ],
    )

    # Spawn robot in Gazebo
    leo_rover = Node(
        namespace=robot_ns,
        package="ros_gz_sim",
        executable="create",
        name="ros_gz_sim_create",
        output="both",
        arguments=[
            "-topic", "robot_description",
            "-name", robot_gazebo_name,
            "-x", str(x),
            "-y", str(y),
            "-z", str(z),
        ],
    )

    # Bridge ROS <-> Gazebo
    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=node_name_prefix + "parameter_bridge",
        arguments=[
            robot_ns + "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            robot_ns + "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            robot_ns + "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            robot_ns + "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    return [robot_state_publisher, leo_rover, topic_bridge]


def generate_launch_description():
    ld = LaunchDescription()

    # spawn 10 robots, fixed x=0.0, y spaced by 2.0 meters
    for i in range(15):
        ns = f"robot_{i}"
        y = i * 2.0
        for action in spawn_robot(ns, x=0.0, y=y, z=0.1):
            ld.add_action(action)

    return ld
