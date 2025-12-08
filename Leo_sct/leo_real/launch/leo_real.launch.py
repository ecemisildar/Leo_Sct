from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # List your real robots here
    robots = [
        {"ns": "rob_2", "spawn_x": 0.0, "spawn_y": 0.0},
    ]

    nodes = []

    for robot in robots:
        ns = robot["ns"]
        sx = robot["spawn_x"]
        sy = robot["spawn_y"]
        cmd_vel_topic = robot.get("cmd_vel_topic", f"/{ns}/cmd_vel")


        # --- Supervisor node (publishes cmd_vel) ---
        supervisor_node = Node(
            package="leo_real",
            executable="robot_supervisor_3_movements",
            name="robot_supervisor",
            namespace=ns,
            parameters=[
                {"spawn_x": sx},
                {"spawn_y": sy},
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
        image_proc_node = Node(
            package="leo_image",
            executable="image_processor",
            name="image_processor",
            namespace=ns,
            remappings=[
                # left side: what your node expects (sim convention)
                # right side: REAL depth topic for this robot
                (f"/{ns}/depth_camera/depth_image",
                 f"/{ns}/camera/aligned_depth_to_color/image_raw"),
                # adjust right side to match your camera driver topics
            ],
            output="screen",
        )

        nodes += [supervisor_node]

    return LaunchDescription(nodes)

