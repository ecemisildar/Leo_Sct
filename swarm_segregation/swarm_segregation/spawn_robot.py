#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
import xacro
from ament_index_python.packages import get_package_share_directory

class SpawnRobots(Node):
    def __init__(self):
        super().__init__('spawn_robot')

        # Get URDF path
        leo_description = get_package_share_directory("leo_description")
        self.xacro_file = os.path.join(leo_description, 'urdf', 'leo_sim.urdf.xacro')

        # Define robots to spawn: name, color RGBA, x, y, is_leader
        self.robots = []

        # Leader robots
        leader_colors = ["red", "green", "blue"]
        color_map = {
            "red": "1 0 0 1",
            "green": "0 1 0 1",
            "yellow": "1 1 0 1",
            "blue": "0 0 1 1",
            "magenta": "1 0 1 1",
            "cyan": "0 1 1 1",
            "gray": "0.5 0.5 0.5 1",
            "orange": "1 0.5 0 1"
        }

        leader_positions = [(0.0, 0.0), (3.0, 3.0), (6.0, 0.0)]
        for i, (color_name, pos) in enumerate(zip(leader_colors, leader_positions)):
            ns = f"robot_leader_{i}"
            color_rgba = color_map[color_name]
            self.robots.append({
                "name": ns,
                "color": color_rgba,
                "x": pos[0],
                "y": pos[1],
                "is_leader": True
            })

        # Follower robots
        follower_positions = [(3.0, 0.0)]  # add more if needed
        for i, pos in enumerate(follower_positions):
            ns = f"robot_follower_{i}"
            color_rgba = color_map["yellow"]
            self.robots.append({
                "name": ns,
                "color": color_rgba,
                "x": pos[0],
                "y": pos[1],
                "is_leader": False
            })

        # Create client to /spawn_entity service
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /spawn_entity service...")

        # Spawn all robots
        for robot in self.robots:
            self.spawn_robot(robot)

    def spawn_robot(self, robot):
        # Process XACRO to URDF
        doc = xacro.process_file(self.xacro_file, mappings={
            "robot_ns": robot["name"],
            "chassis_color": robot["color"],
            "is_leader": str(robot["is_leader"]).lower()
        })
        robot_urdf = doc.toxml()

        # Prepare request
        req = SpawnEntity.Request()
        req.name = robot["name"]
        req.xml = robot_urdf
        req.robot_namespace = robot["name"]
        req.initial_pose.position.x = robot["x"]
        req.initial_pose.position.y = robot["y"]
        req.initial_pose.position.z = 0.1
        req.reference_frame = "world"

        # Call service
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Spawned {robot['name']} at ({robot['x']},{robot['y']})")
        else:
            self.get_logger().error(f"Failed to spawn {robot['name']}")

def main(args=None):
    rclpy.init(args=args)
    node = SpawnRobots()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
