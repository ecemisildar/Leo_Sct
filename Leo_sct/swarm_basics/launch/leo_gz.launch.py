# Copyright 2023 Fictionlab sp. z o.o.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown, TimerAction, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Setup project paths
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_project_gazebo = get_package_share_directory("swarm_basics")
    pkg_project_worlds = get_package_share_directory("leo_gz_worlds")
    existing_ign_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    model_path = os.path.join(pkg_project_gazebo, "models")
    ign_resource_path = os.pathsep.join(
        [p for p in [pkg_project_gazebo, model_path, existing_ign_path] if p]
    )
    gz_resource_path = os.pathsep.join(
        [p for p in [pkg_project_gazebo, model_path, existing_gz_path] if p]
    )

    sim_world = DeclareLaunchArgument(
        "sim_world",
        default_value=os.path.join(pkg_project_gazebo, "worlds", "random_world.sdf"),
        # default_value=os.path.join(pkg_project_worlds, "worlds", "leo_empty.sdf"),
        description="Path to the Gazebo world file",
    )
    headless = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="Run Gazebo headless (no GUI)",
    )
    auto_start = DeclareLaunchArgument(
        "auto_start",
        default_value="true",
        description="Start physics immediately (run without manual play)",
    )

    robot_ns = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Robot namespace",
    )
    run_duration = DeclareLaunchArgument(
        "run_duration",
        default_value="50000.0",
        description="Seconds before shutting down the launch",
    )
    total_robots = DeclareLaunchArgument(
        "total_robots",
        default_value="10",
        description="Number of robots to spawn in the star formation",
    )
    auto_start_supervisor = DeclareLaunchArgument(
        "auto_start_supervisor",
        default_value="true",
        description="Enable robot_supervisor_3_movements on launch",
    )
    spawn_moving_aruco = DeclareLaunchArgument(
        "spawn_moving_aruco",
        default_value="true",
        description="Spawn a moving ArUco target box.",
    )
    moving_aruco_x = DeclareLaunchArgument(
        "moving_aruco_x",
        default_value="1.0",
        description="Center X position for the moving ArUco target.",
    )
    moving_aruco_y = DeclareLaunchArgument(
        "moving_aruco_y",
        default_value="1.0",
        description="Center Y position for the moving ArUco target.",
    )
    moving_aruco_z = DeclareLaunchArgument(
        "moving_aruco_z",
        default_value="0.25",
        description="Z position for the moving ArUco target.",
    )
    moving_aruco_radius = DeclareLaunchArgument(
        "moving_aruco_radius",
        default_value="1.5",
        description="Circular path radius for the moving ArUco target.",
    )
    moving_aruco_speed = DeclareLaunchArgument(
        "moving_aruco_speed",
        default_value="0.35",
        description="Circular path angular speed for the moving ArUco target.",
    )
    moving_aruco_update_rate = DeclareLaunchArgument(
        "moving_aruco_update_rate",
        default_value="5.0",
        description="Circular path update rate in Hz for the moving ArUco target.",
    )
    results_dir = DeclareLaunchArgument(
        "results_dir",
        default_value=os.path.join(
            os.path.expanduser("~"),
            "ros2_ws",
            "src",
            "Leo_sct",
            "results",
        ),
        description="Directory to write run artifacts",
    )

    # Setup to launch the simulator and Gazebo world
    gz_args = PythonExpression([
        "'",
        LaunchConfiguration("sim_world"),
        "' + (' -s' if '",
        LaunchConfiguration("headless"),
        "' == 'true' else '') + (' -r' if '",
        LaunchConfiguration("auto_start"),
        "' == 'true' else '')",
    ])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_gazebo, "launch", "spawn_multi_robots.launch.py")
        ),
        launch_arguments={
            "robot_ns": LaunchConfiguration("robot_ns"),
            "run_duration": LaunchConfiguration("run_duration"),
            "total_robots": LaunchConfiguration("total_robots"),
            "results_dir": LaunchConfiguration("results_dir"),
            "auto_start_supervisor": LaunchConfiguration("auto_start_supervisor"),
            "spawn_moving_aruco": LaunchConfiguration("spawn_moving_aruco"),
            "moving_aruco_x": LaunchConfiguration("moving_aruco_x"),
            "moving_aruco_y": LaunchConfiguration("moving_aruco_y"),
            "moving_aruco_z": LaunchConfiguration("moving_aruco_z"),
            "moving_aruco_radius": LaunchConfiguration("moving_aruco_radius"),
            "moving_aruco_speed": LaunchConfiguration("moving_aruco_speed"),
            "moving_aruco_update_rate": LaunchConfiguration("moving_aruco_update_rate"),
        }.items(),
    )

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

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=ign_resource_path,
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=gz_resource_path,
            ),
            sim_world,
            headless,
            auto_start,
            robot_ns,
            run_duration,
            total_robots,
            auto_start_supervisor,
            spawn_moving_aruco,
            moving_aruco_x,
            moving_aruco_y,
            moving_aruco_z,
            moving_aruco_radius,
            moving_aruco_speed,
            moving_aruco_update_rate,
            results_dir,
            gz_sim,
            spawn_robot,
            topic_bridge,
            TimerAction(
                period=LaunchConfiguration("run_duration"),
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "bash",
                            "-lc",
                            "pkill -9 -f 'ruby .*gz sim' || true; "
                            "pkill -9 -f 'gz sim' || true; "
                            "pkill -9 -f 'ign gazebo' || true; "
                            "pkill -9 -f 'gzserver' || true; "
                            "pkill -9 -f 'gzclient' || true",
                        ],
                        output="screen",
                    ),
                    Shutdown(reason="Run duration reached"),
                ],
            ),
        ]
    )
