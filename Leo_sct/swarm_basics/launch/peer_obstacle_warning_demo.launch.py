import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_swarm_basics = get_package_share_directory("swarm_basics")

    gazebo_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_swarm_basics, "launch", "leo_gz.launch.py")
        ),
        launch_arguments={
            "headless": "false",
            "auto_start": "true",
            "run_duration": "90",
            "total_robots": "2",
            "spawn_layout": "peer_obstacle_test",
            "controller_mode": "random_walk_cbf",
            "exploration_algorithm": "bug",
            "peer_warning_enabled": "true",
        }.items(),
    )

    warning_test = TimerAction(
        period=11.0,
        actions=[
            Node(
                package="swarm_basics",
                executable="peer_obstacle_warning_test",
                name="peer_obstacle_warning_test",
                parameters=[
                    {"source_robot": "robot_0"},
                    {"target_robot": "robot_1"},
                    {"warning_zone": "BACK"},
                    {"distance_m": 0.75},
                    {"obstacle_distance_m": 0.45},
                    {"test_duration_s": 20.0},
                    {"publish_period_s": 0.1},
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([gazebo_demo, warning_test])
