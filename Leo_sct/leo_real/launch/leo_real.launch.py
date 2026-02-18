from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- Launch args ---
    robot_ns = LaunchConfiguration("robot_ns")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")

    enable_supervisor = LaunchConfiguration("enable_supervisor")
    static_mode = LaunchConfiguration("static")

    robot_ns_arg = DeclareLaunchArgument(
        "robot_ns",
        default_value="rob_1",
        description="Robot namespace, e.g. rob_1, rob_2, rob_3, rob_4",
    )
    spawn_x_arg = DeclareLaunchArgument("spawn_x", default_value="0.0")
    spawn_y_arg = DeclareLaunchArgument("spawn_y", default_value="0.0")

    enable_supervisor_arg = DeclareLaunchArgument(
        "enable_supervisor",
        default_value="false",
        description="Start robot_supervisor enabled (true/false).",
    )
    static_mode_arg = DeclareLaunchArgument(
        "static",
        default_value="false",
        description="Force robot_supervisor cmd_vel output to zero for testing.",
    )

    # --- Paths ---
    pkg_leo_real = get_package_share_directory("leo_real")
    camera_params_file = os.path.join(pkg_leo_real, "config", "real_camera.yaml")

    pkg_rs = get_package_share_directory("realsense2_camera")
    realsense_launch = os.path.join(pkg_rs, "launch", "rs_launch.py")

    # --- Supervisor node (publishes cmd_vel) ---
    supervisor_node = Node(
        package="leo_real",
        executable="robot_supervisor",
        name="robot_supervisor",
        namespace=robot_ns,
        parameters=[
            {"spawn_x": spawn_x},
            {"spawn_y": spawn_y},
            {"enabled": enable_supervisor},
            {"static": static_mode},
        ],
        output="screen",
    )

    # --- RealSense launch (namespaced) ---
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            "camera_name": "camera",
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": "true",
            "depth_module.depth_profile": "424x240x15",
            "rgb_camera.color_profile": "640x480x15",
            "initial_reset": "false",
        }.items(),
    )

    realsense_group = GroupAction([
        PushRosNamespace(robot_ns),
        realsense_node,
    ])

    # --- Image processor (reads depth) ---
    image_proc_node = Node(
        package="leo_image",
        executable="image_processor",
        name="image_processor",
        namespace=robot_ns,
        parameters=[
            camera_params_file,
            {"rgb_topic": "camera/camera/color/image_raw"},
        ],
        remappings=[
            ("depth_camera/depth_image", "camera/camera/aligned_depth_to_color/image_raw"),
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_ns_arg, spawn_x_arg, spawn_y_arg,
        enable_supervisor_arg, static_mode_arg,
        supervisor_node, realsense_group, image_proc_node
    ])
