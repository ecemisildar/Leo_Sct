#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
import xacro
import os
import time

from ament_index_python.packages import get_package_share_directory

class RobotSpawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')

        # Path to your XACRO
        self.leo_description = os.path.join(
            get_package_share_directory("leo_description"),
            "urdf",
            "leo_sim.urdf.xacro"
        )

        # Robot configurations
        self.robots = [
            {"ns": "robot_leader_0", "color": "1 0 0 1", "x": 0.0, "y": 0.0, "is_leader": True},
            {"ns": "robot_leader_1", "color": "0 1 0 1", "x": 3.0, "y": 3.0, "is_leader": True},
            {"ns": "robot_leader_2", "color": "0 0 1 1", "x": 6.0, "y": 0.0, "is_leader": True},
            {"ns": "robot_follower_0", "color": "1 1 0 1", "x": 3.0, "y": 0.0, "is_leader": False},
            {"ns": "robot_follower_1", "color": "1 1 0 1", "x": 1.0, "y": 1.0, "is_leader": False},
            {"ns": "robot_follower_2", "color": "1 1 0 1", "x": 4.0, "y": 1.0, "is_leader": False},
            {"ns": "robot_follower_3", "color": "1 1 0 1", "x": 3.0, "y": -3.0, "is_leader": False},
            {"ns": "robot_follower_4", "color": "1 1 0 1", "x": 0.0, "y": 3.0, "is_leader": False},
        ]

        self.spawn_index = 0

        # DDS / ROS 2 bridge service
        self.service_name = '/world/leo_circles/create'
        self.cli = self.create_client(SpawnEntity, self.service_name)

        # Wait for the service reliably
        self.get_logger().info(f"Waiting for {self.service_name}...")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service not ready, retrying... Make sure ros_gz_bridge is running!")

        self.get_logger().info("Service available! Starting spawn sequence...")
        self.spawn_next_robot()

    def spawn_next_robot(self):
        if self.spawn_index >= len(self.robots):
            self.get_logger().info("All robots spawned!")
            return

        robot = self.robots[self.spawn_index]

        # Convert XACRO to URDF XML
        doc = xacro.process_file(self.leo_description, mappings={
            "robot_ns": robot["ns"],
            "chassis_color": robot["color"],
            "is_leader": str(robot["is_leader"]).lower()
        })
        robot_description = doc.toxml()

        # Prepare the SpawnEntity request
        request = SpawnEntity.Request()
        request.name = robot["ns"]
        request.xml = robot_description
        request.pose.position.x = robot["x"]
        request.pose.position.y = robot["y"]
        request.pose.position.z = 0.1
        request.allow_renaming = True  # Important in Ignition

        self.get_logger().info(f"Spawning {robot['ns']}...")

        # Call the service asynchronously
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # Check response
        resp = future.result()
        if resp is not None:
            if resp.success:
                self.get_logger().info(f"{robot['ns']} spawned successfully!")
            else:
                self.get_logger().error(f"Failed to spawn {robot['ns']}: {resp.status_message}")
        else:
            self.get_logger().error(f"Failed to call service for {robot['ns']}")

        # Move to next robot
        self.spawn_index += 1
        # Short delay for stability
        time.sleep(0.5)
        self.spawn_next_robot()


def main(args=None):
    rclpy.init(args=args)
    node = RobotSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
