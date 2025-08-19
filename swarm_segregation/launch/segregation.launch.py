import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command 
from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get directories
    leo_description = get_package_share_directory("leo_description")
    launch_description = []

    def create_robot(ns, color, x, y, is_leader=False):

        xacro_file = os.path.join(leo_description, 'urdf', 'leo_sim.urdf.xacro')
        
        doc = xacro.process_file(xacro_file, mappings={
            "robot_ns": ns,
            "chassis_color": color,
            "is_leader": str(is_leader).lower()
        })
        robot_description = doc.toxml()

        # State Publisher
        state_pub = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=ns,
            parameters=[{
                "use_sim_time": True,
                "robot_description": robot_description,
            }],
            output="screen",
        )

        # Spawn in Gazebo
        spawn = Node(
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

        # Behavior Node (leader/follower)
        params = [
            {"color": color}
        ]
        
        if is_leader: 
            params.append({"is_leader": True})  

        behavior_node = Node(
            package="swarm_segregation",
            executable="leader_node" if is_leader else "follower_node",
            name="leader_node" if is_leader else "follower_node",
            namespace=ns,
            parameters=params,
            output="screen",
        )

        bridges = []
        if not is_leader: 
            bridges.append(Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"{ns}_bridge",
            arguments=[
                f"/{ns}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                f"/{ns}/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
                f"/{ns}/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
                f"/{ns}/tf_static@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
                f"/{ns}/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image", 
            ],
            parameters=[{"qos_overrides./tf_static.publisher.durability": "transient_local"}],
            output="screen",    
            ))

        return [state_pub, spawn, behavior_node] + bridges    

    color_map = {
        "red": "1 0 0 1",
        "green": "0 1 0 1",
        "yellow": "1 1 0 1",
        "blue": "0 0 1 1",
        "magenta": "1 0 1 1",
        "cyan": "0 1 1 1",
        "gray": "0.5 0.5 0.5 1",
        "orange": "1 0.5 0 1"
    }

    # Spawn 3 Leaders
    leader_colors = ["red", "green", "blue"]
    for i in range(3):
        ns = f"robot_leader_{i}"
        color_name = leader_colors[i % len(leader_colors)]
        color_rgba = color_map[color_name]
        x, y = i * 3.0, 0.0 
        aruco_id = i + 1
        launch_description += create_robot(ns, color_rgba, x, y, is_leader=True)

    # Spawn 5 followers
    for i in range(5):
        ns = f"robot_follower_{i}"
        color_rgba = color_map["yellow"]
        x, y = i * 1.5, 2.0 
        aruco_id = i + 10
        launch_description += create_robot(ns, color_rgba, x, y, is_leader=False)
    

    return LaunchDescription(launch_description)