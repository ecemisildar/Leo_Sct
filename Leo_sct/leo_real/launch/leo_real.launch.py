from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    enable_supervisor = LaunchConfiguration("enable_supervisor")
    static_mode = LaunchConfiguration("static")
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

    # List your real robots here
    robots = [
        {"ns": "rob_2", "spawn_x": 0.0, "spawn_y": 0.0},
    ]

    nodes = []

    pkg_leo_real = get_package_share_directory("leo_real")
    camera_params_file = os.path.join(pkg_leo_real, "config", "real_camera.yaml")
    pkg_rs = get_package_share_directory("realsense2_camera")
    realsense_launch = os.path.join(pkg_rs, "launch", "rs_launch.py")

    for robot in robots:
        ns = robot["ns"]
        sx = robot["spawn_x"]
        sy = robot["spawn_y"]
        cmd_vel_topic = robot.get("cmd_vel_topic", f"/{ns}/cmd_vel")


        # --- Supervisor node (publishes cmd_vel) ---
        supervisor_node = Node(
            package="leo_real",
            executable="robot_supervisor",
            name="robot_supervisor",
            namespace=ns,
            parameters=[
                {"spawn_x": sx},
                {"spawn_y": sy},
                {"enabled": enable_supervisor},
                {"static": static_mode},
            ],
            # If your node uses *relative* 'cmd_vel' in this namespace,
            # topic becomes /robot_i/cmd_vel automatically and you can
            # REMOVE remappings.
            #
            # If it uses an absolute /<ns>/cmd_vel internally, you also
            # don't need remapping.
            #
            # Only if it publishes just '/cmd_vel' *without* namespace,
            # you’d need something like:
            #
            remappings=[
                # ("cmd_vel", f"{ns}/cmd_vel"),
                ("cmd_vel", cmd_vel_topic),
            ],
            output="screen",
        )

        # --- Image processor (reads depth) ---
        realsense_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                "camera_name": "camera",
                "enable_color": "true",
                "enable_depth": "true",
                "align_depth": "true",
                "depth_module.profile": "424x240x15",
                "rgb_camera.profile": "640x480x15",
                "initial_reset": "true",
            }.items()
        )
        
        image_proc_node = Node(
            package="leo_image",
            executable="image_processor",
            name="image_processor",
            namespace=ns,
            parameters=[
                camera_params_file,
                {"rgb_topic": f"/{ns}/camera/camera/color/image_raw"},
            ],
            remappings=[
                ("depth_camera/depth_image", f"/{ns}/camera/camera/aligned_depth_to_color/image_raw"),
                # fallback if aligned topic doesn't exist:
                # ("depth_camera/depth_image", f"/{ns}/camera/depth/image_rect_raw"),
            ],
            output="screen",
        )

        realsense_group = GroupAction([
            PushRosNamespace(ns),
            realsense_node,
        ])


        nodes += [supervisor_node, realsense_group, image_proc_node]

    return LaunchDescription([enable_supervisor_arg, static_mode_arg, *nodes])
