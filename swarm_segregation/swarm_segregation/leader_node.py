import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import os
from ament_index_python.packages import get_package_share_directory
from swarm_segregation.sct import SCT
import time


class LeaderNode(Node):
    def __init__(self):
        super().__init__('leader_node')

        yaml_path = os.path.join(get_package_share_directory('swarm_segregation'), 'config', 'supervisor.yaml')

        self.supervisor = SCT(yaml_path)
        self.EV, _ = self.supervisor.get_events()
        self.signals = {ev: False for ev in self.EV}

        # self.register_callbacks()

        self.declare_parameter('color', 'red')
        raw_color = self.get_parameter('color').get_parameter_value().string_value

        code_to_color = {
            "1 0 0 1": "red",
            "0 1 0 1": "green",
            "0 0 1 1": "blue",
            "1 1 0 0": "yellow",
        }

        self.color = code_to_color.get(raw_color, raw_color)

        self.publisher = self.create_publisher(String, f"/leader_broadcast/{self.color}", 10)
        self.ready = False

        self.ready_timer = self.create_timer(0.1, self.check_ready)

        # Main publish timer, will only publish if ready
        self.publish_timer = self.create_timer(0.2, self.broadcast_color)

    def check_ready(self):
        topics = [t[0] for t in self.get_topic_names_and_types()]
        # self.get_logger().info("Waiting 10 seconds before publishing...")
        # time.sleep(10)
        if f"/leader_broadcast/{self.color}" in topics:
            self.ready = True
            self.get_logger().info(f"Leader topic ready: {self.color}")
            self.ready_timer.cancel()  # stop checking

    def broadcast_color(self):
        if not self.ready:
            return  # Do not publish until ready
            
        if self.color == "red":
            self.signals["EV_sendR"] = True
        if self.color == "green":
            self.signals["EV_sendG"] = True
        if self.color == "blue":
            self.signals["EV_sendB"] = True

        msg = String()
        msg.data = self.color
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
