import rclpy
from rclpy.node import Node

import os
from ament_index_python.packages import get_package_share_directory

from swarm_segregation.sct import SCT

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

# TO DO
# limited comm range e.g. 3 meters using aruco markers
# add collision avoidance with each robot

class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')


        yaml_path = os.path.join(get_package_share_directory('swarm_segregation'), 'config', 'supervisor.yaml')

        self.supervisor = SCT(yaml_path)
        self.EV, _ = self.supervisor.get_events()
        self.signals = {ev: False for ev in self.EV}

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_subscription(String, '/leader_broadcast/red', self.on_red, 10)
        self.create_subscription(String, '/leader_broadcast/green', self.on_green, 10)
        self.create_subscription(String, '/leader_broadcast/blue', self.on_blue, 10)

        self.register_callbacks()

        self.signals["EV_press"] = True
        self.signals["EV_getR"] = True
        self.signals["EV_getG"] = True

        self.timer = self.create_timer(0.2, self.step)   

    def on_red(self, msg):
        self.get_logger().info("Received red leader message")
        self.signals["EV_getR"] = True
        self.signals["EV_getNotB"] = True
        self.signals["EV_getNotG"] = True
        self.signals["EV_sendR"] = True
        self.signals["EV_sendNothing"] = True

    def on_green(self, msg):
        self.get_logger().info("Received green leader message")
        self.signals["EV_getG"] = True
        self.signals["EV_getNotB"] = True
        self.signals["EV_getNotR"] = True 
        self.signals["EV_sendG"] = True
        self.signals["EV_sendNothing"] = True

    def on_blue(self, msg):
        self.get_logger().info("Received blue leader message")
        self.signals["EV_getB"] = True
        self.signals["EV_getNotR"] = True
        self.signals["EV_getNotG"] = True 
        self.signals["EV_sendB"] = True
        self.signals["EV_sendNothing"] = True

    def register_callbacks(self):

        for ev, ev_id in self.EV.items():
            # Sensor input logic
            print(ev)
            if ev in ["EV_getR", "EV_getG", "EV_getB", 
                        "EV_getNotR", "EV_getNotG", "EV_getNotB"]:
                self.supervisor.add_callback(ev_id, self.noop, self.make_input_checker(ev), None)

            # Movement logic
            elif ev == "EV_moveFW":
                self.supervisor.add_callback(ev_id, self.move_forward, self.always_true, None)
            elif ev == "EV_turnCW":
                self.supervisor.add_callback(ev_id, self.turn_CW, self.always_true, None)
            elif ev == "EV_turnCCW":
                self.supervisor.add_callback(ev_id, self.turn_CCW, self.always_true, None)
            elif ev == "EV_moveStop":
                self.supervisor.add_callback(ev_id, self.stop_moving, self.always_true, None)

            # Anything else we don't handle → ignore safely
            else:
                self.supervisor.add_callback(ev_id, self.noop, self.always_true, None)   

    def make_input_checker(self, ev):
        def check(_): return self.signals.get(ev, False)   
        return check

    def step(self):
        #self.get_logger().info(f"Step called. Current signals: {self.signals}")

        current_sct_states = self.supervisor.get_current_state()
        #self.get_logger().info(f"SCT Current States (Sup0, Sup1, Sup2): {current_sct_states}")

        self.supervisor.run_step()

        enabled_sct_events = self.supervisor.get_enabled_events()
        #self.get_logger().info(f"SCT Enabled Events AFTER run_step: {enabled_sct_events}")

        self.signals = {ev: False for ev in self.EV}
    

    def move_forward(self, _):
        self.get_logger().info("Moving forward")
        msg = Twist()
        msg.linear.x = 0.2
        self.cmd_pub.publish(msg)

    def turn_CW(self, _):
        self.get_logger().info("Turning CW")
        msg = Twist()
        msg.angular.z = -0.5
        self.cmd_pub.publish(msg)

    def turn_CCW(self, _):
        self.get_logger().info("Turning CCW")
        msg = Twist()
        msg.angular.z = 0.5
        self.cmd_pub.publish(msg)

    def stop_moving(self, _):
        self.cmd_pub.publish(Twist())  

    def always_true(self, _): return True

    def noop(self, _): pass


def main(args=None):

    rclpy.init(args=args)
    node = FollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()      



