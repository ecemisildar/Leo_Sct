from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- Launch args ---
    robot_ns = LaunchConfiguration("robot_ns")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")

    enable_supervisor = LaunchConfiguration("enable_supervisor")
    supervisor_yaml_path = LaunchConfiguration("supervisor_yaml_path")
    enable_aruco = LaunchConfiguration("enable_aruco")
    static_mode = LaunchConfiguration("static")

    robot_ns_arg = DeclareLaunchArgument(
        "robot_ns",
        default_value="",  # IMPORTANT: empty by default
        description="Robot namespace (rob_1, rob_2, ...). Leave empty for no namespace.",
    )
    spawn_x_arg = DeclareLaunchArgument("spawn_x", default_value="0.0")
    spawn_y_arg = DeclareLaunchArgument("spawn_y", default_value="0.0")

    enable_supervisor_arg = DeclareLaunchArgument(
        "enable_supervisor",
        default_value="false",
        description="Start robot_supervisor enabled (true/false).",
    )
    supervisor_yaml_path_arg = DeclareLaunchArgument(
        "supervisor_yaml_path",
        default_value="",
        description="Optional absolute path to an SCT supervisor YAML.",
    )
    enable_aruco_arg = DeclareLaunchArgument(
        "enable_aruco",
        default_value="false",
        description="Enable ArUco marker detection/following topics.",
    )
    static_mode_arg = DeclareLaunchArgument(
        "static",
        default_value="false",
        description="Force robot_supervisor cmd_vel output to zero for testing.",
    )

    # --- Conditions ---
    # True if robot_ns is NOT empty
    use_ns = PythonExpression(["'", robot_ns, "' != ''"])

    # --- Paths ---
    pkg_leo_real = get_package_share_directory("leo_real")
    camera_params_file = os.path.join(pkg_leo_real, "config", "real_camera.yaml")

    pkg_rs = get_package_share_directory("realsense2_camera")
    realsense_launch = os.path.join(pkg_rs, "launch", "rs_launch.py")

    # --- RealSense include ---
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

    # --- Supervisor node (publishes cmd_vel) ---
    supervisor_node = Node(
        package="leo_real",
        executable="robot_supervisor_rgb",
        name="robot_supervisor_rgb",
        parameters=[
            {"spawn_x": spawn_x},
            {"spawn_y": spawn_y},
            {"enabled": enable_supervisor},
            {"supervisor_yaml_path": supervisor_yaml_path},
            {"enable_aruco": enable_aruco},
            {"static": static_mode},
            {"green_centering_gain": 0.8},
            {"green_centering_max_angular": 0.45},
            {"green_centering_deadband": 0.08},
            {"green_offset_timeout_s": 0.5},
            {"green_follow_linear_x": 0.06},
            {"green_follow_centered_deadband": 0.18},
        ],
        output="screen",
    )

    # --- Image processor (reads depth) ---
    image_proc_node = Node(
        package="leo_image",
        executable="image_processor_rgb",
        name="image_processor_rgb",
        parameters=[
            camera_params_file,
            {"aruco_enabled": enable_aruco},
        ],
        remappings=[
            ("depth_camera/depth_image", "camera/camera/aligned_depth_to_color/image_raw"),
        ],
        output="screen",
    )

    # --- Group when namespace is used ---
    group_with_ns = GroupAction(
        actions=[
            PushRosNamespace(robot_ns),
            realsense_node,
            supervisor_node,
            image_proc_node,
        ],
        condition=IfCondition(use_ns),
    )

    # --- Group when NO namespace (root topics) ---
    group_no_ns = GroupAction(
        actions=[
            realsense_node,
            supervisor_node,
            image_proc_node,
        ],
        condition=UnlessCondition(use_ns),
    )

    return LaunchDescription([
        robot_ns_arg, spawn_x_arg, spawn_y_arg,
        enable_supervisor_arg, supervisor_yaml_path_arg, enable_aruco_arg, static_mode_arg,
        group_with_ns,
        group_no_ns,
    ])
