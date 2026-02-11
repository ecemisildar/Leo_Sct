from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    enable_supervisor = LaunchConfiguration("enable_supervisor")
    enable_supervisor_arg = DeclareLaunchArgument(
        "enable_supervisor",
        default_value="false",
        description="Start robot_supervisor enabled (true/false).",
    )

    # List your real robots here
    robots = [
        {"ns": "rob_2", "spawn_x": 0.0, "spawn_y": 0.0},
    ]

    nodes = []

    pkg_leo_real = get_package_share_directory("leo_real")
    camera_params_file = os.path.join(pkg_leo_real, "config", "real_camera.yaml")

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
        # In sim you had /robot_i/depth_camera/depth_image.
        # Here we connect that to the REAL rover's depth topic.
        # --- RealSense camera driver ---
        realsense_node = Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="realsense",
            namespace=ns,  # => /rob_2/...
            parameters=[{
		"camera_name": "camera",
		"enable_color": True,
		"enable_depth": True,
		"enable_infra1": False,
		"enable_infra2": False,
		"align_depth": True,

		"depth_width": 424,
		"depth_height": 240,
		"depth_fps": 15,

		"color_width": 640,
		"color_height": 480,
		"color_fps": 15,

		"initial_reset": True,
	    }],
            output="screen",
        )
        
        image_proc_node = Node(
            package="leo_image",
            executable="image_processor",
            name="image_processor",
            namespace=ns,
            parameters=[
                camera_params_file,
                {"rgb_topic": f"/{ns}/realsense/color/image_raw"},
            ],
            remappings=[
                # left side: what your node expects (sim convention)
                # right side: REAL depth topic for this robot
                (f"/{ns}/depth_camera/depth_image",
                 f"/{ns}/realsense/depth/image_rect_raw"),
                # adjust right side to match your camera driver topics
            ],
            output="screen",
        )

        nodes += [supervisor_node, realsense_node, image_proc_node]

    return LaunchDescription([enable_supervisor_arg, *nodes])
