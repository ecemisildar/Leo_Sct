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
import subprocess
import time

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, TimerAction, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _kill_gazebo_processes(*_args, **_kwargs):
    patterns = [
        r"ruby .*gz sim",
        r"^gz sim ",
        r"^ign gazebo ",
        r"ign gazebo .*random_world\.sdf",
        r"gzserver",
        r"gzclient",
        r"parameter_bridge",
    ]
    for signal in ("-TERM", "-KILL"):
        for pattern in patterns:
            subprocess.run(
                ["pkill", signal, "-f", pattern],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        time.sleep(0.3)
    return []


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
        default_value="300.0",
        description="Seconds before shutting down the launch",
    )
    total_robots = DeclareLaunchArgument(
        "total_robots",
        default_value="5",
        description="Number of robots to spawn in the star formation",
    )
    spawn_layout = DeclareLaunchArgument(
        "spawn_layout",
        default_value="spread",
        description="Robot spawn layout: 'spread', 'origin', 'middle_circle', or 'random_safe'.",
    )
    spawn_seed = DeclareLaunchArgument(
        "spawn_seed",
        default_value="auto",
        description="Seed for randomized spawn layouts such as random_safe. Use 'auto' for a fresh seed each run.",
    )
    random_seed = DeclareLaunchArgument(
        "random_seed",
        default_value="auto",
        description="Base seed for per-robot supervisor random choices. Use 'auto' for a fresh seed each run.",
    )
    sct_choice_mode = DeclareLaunchArgument(
        "sct_choice_mode",
        default_value="random",
        description="SCT controllable choice mode: 'random' or deterministic 'first'.",
    )
    controller_mode = DeclareLaunchArgument(
        "controller_mode",
        default_value="random_walk_cbf",
        description="Per-robot controller: 'sct' or 'random_walk_cbf'.",
    )
    exploration_algorithm = DeclareLaunchArgument(
        "exploration_algorithm",
        default_value="random_walk",
        description="Exploration mode for random_walk_cbf: 'random_walk' or goal-free 'bug'.",
    )
    peer_warning_enabled = DeclareLaunchArgument(
        "peer_warning_enabled",
        default_value="true",
        description="Enable peer-warning messages in per-robot controllers.",
    )
    auto_start_supervisor = DeclareLaunchArgument(
        "auto_start_supervisor",
        default_value="true",
        description="Enable robot_supervisor_3_movements on launch",
    )
    results_dir = DeclareLaunchArgument(
        "results_dir",
        default_value=os.path.join(
            os.path.expanduser("~"),
            "sct_llm_ws",
            "src",
            "Leo_Sct",
            "Leo_sct",
            "results_exp",
        ),
        description="Directory to write run artifacts",
    )
    metadata_yaml_path = DeclareLaunchArgument(
        "metadata_yaml_path",
        default_value=os.path.join(
            pkg_project_gazebo,
            "config",
            "sup_gpt.yaml",
        ),
        description="YAML file to copy into each run folder",
    )
    prompt_text = DeclareLaunchArgument(
        "prompt_text",
        default_value="",
        description="Prompt text to save into each run folder as prompt.txt",
    )
    prompt_file_path = DeclareLaunchArgument(
        "prompt_file_path",
        default_value="",
        description="Prompt text file to copy into each run folder as prompt.txt",
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
            "sim_world": LaunchConfiguration("sim_world"),
            "headless": LaunchConfiguration("headless"),
            "auto_start": LaunchConfiguration("auto_start"),
            "robot_ns": LaunchConfiguration("robot_ns"),
            "run_duration": LaunchConfiguration("run_duration"),
            "total_robots": LaunchConfiguration("total_robots"),
            "spawn_layout": LaunchConfiguration("spawn_layout"),
            "spawn_seed": LaunchConfiguration("spawn_seed"),
            "random_seed": LaunchConfiguration("random_seed"),
            "sct_choice_mode": LaunchConfiguration("sct_choice_mode"),
            "controller_mode": LaunchConfiguration("controller_mode"),
            "exploration_algorithm": LaunchConfiguration("exploration_algorithm"),
            "peer_warning_enabled": LaunchConfiguration("peer_warning_enabled"),
            "results_dir": LaunchConfiguration("results_dir"),
            "metadata_yaml_path": LaunchConfiguration("metadata_yaml_path"),
            "prompt_text": LaunchConfiguration("prompt_text"),
            "prompt_file_path": LaunchConfiguration("prompt_file_path"),
            "auto_start_supervisor": LaunchConfiguration("auto_start_supervisor"),
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
            spawn_layout,
            spawn_seed,
            random_seed,
            sct_choice_mode,
            controller_mode,
            exploration_algorithm,
            peer_warning_enabled,
            auto_start_supervisor,
            results_dir,
            metadata_yaml_path,
            prompt_text,
            prompt_file_path,
            gz_sim,
            spawn_robot,
            topic_bridge,
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[OpaqueFunction(function=_kill_gazebo_processes)],
                )
            ),
            TimerAction(
                period=LaunchConfiguration("run_duration"),
                actions=[
                    Shutdown(reason="Run duration reached"),
                ],
            ),
        ]
    )
