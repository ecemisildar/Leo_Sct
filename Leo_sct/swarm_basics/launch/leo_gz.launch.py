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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Setup project paths
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_project_gazebo = get_package_share_directory("swarm_basics")
    pkg_project_worlds = get_package_share_directory("leo_gz_worlds")

    sim_world = DeclareLaunchArgument(
        "sim_world",
        default_value=os.path.join(pkg_project_gazebo, "worlds", "random_world.sdf"),
        # default_value=os.path.join(pkg_project_worlds, "worlds", "leo_empty.sdf"),
        description="Path to the Gazebo world file",
    )
    headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
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
        default_value="500.0",
        description="Seconds before shutting down the launch",
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
            sim_world,
            headless,
            auto_start,
            robot_ns,
            run_duration,
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
