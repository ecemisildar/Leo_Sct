import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from swarm_segregation.sct import SCT
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
import time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower')
        self.signals = {"red": False, "green": False, "blue": False}

        self.ready = False

        # Load your SCT automaton
        yaml_path = os.path.join(get_package_share_directory('swarm_segregation'), 'config', 'supervisor.yaml')
        self.sct = SCT(yaml_path)

        for event_name, idx in self.sct.EV.items():
            # Add a dummy callback for all events if you don't need them
            self.sct.add_callback(
                event=idx,
                clbk=lambda data: None,  # does nothing
                ci=lambda data: True,    # always returns True for input reading
                sup_data=None
            )
        self.signals["EV_press"] = True

        self.dist2red = float("100")
        self.dist2green = float("100")
        self.dist2blue = float("100")

        self.red_leader_pos   = np.array([0.0, 0.0])
        self.green_leader_pos = np.array([3.0, 3.0])
        self.blue_leader_pos  = np.array([6.0, 0.0])

        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("color", "yellow")
        self.declare_parameter("is_leader", False)

        # Get values
        self.spawn_x = self.get_parameter("spawn_x").get_parameter_value().double_value
        self.spawn_y = self.get_parameter("spawn_y").get_parameter_value().double_value
        self.color = self.get_parameter("color").get_parameter_value().string_value
        self.is_leader = self.get_parameter("is_leader").get_parameter_value().bool_value


        # Subscriptions to leader colors
        self.create_subscription(String, '/leader_broadcast/red', self.on_red, 10)
        self.create_subscription(String, '/leader_broadcast/green', self.on_green, 10)
        self.create_subscription(String, '/leader_broadcast/blue', self.on_blue, 10)

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)


        # Publisher for robot movement
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for main loop
        self.create_timer(0.2, self.run_step)  # 5 Hz


        self.ready_timer = self.create_timer(0.2, self.check_ready)

    def check_ready(self):
        # Check if required topics exist
        topics = [t[0] for t in self.get_topic_names_and_types()]
        odom_topic = f'{self.get_namespace()}/odom'
        leader_topics = [f'/leader_broadcast/{c}' for c in ["red", "green", "blue"]]

        if odom_topic in topics and all(t in topics for t in leader_topics):
            self.ready = True
            self.get_logger().info("All required topics ready, starting behavior!")
            self.ready_timer.cancel()  # Stop checking

    def odom_callback(self, msg: Odometry):
        global_x = self.spawn_x + msg.pose.pose.position.x
        global_y = self.spawn_y + msg.pose.pose.position.y

        follower_pos = np.array([global_x, global_y])

        self.dist2red = np.linalg.norm(self.red_leader_pos - follower_pos)
        self.dist2green = np.linalg.norm(self.green_leader_pos - follower_pos)
        self.dist2blue = np.linalg.norm(self.blue_leader_pos - follower_pos)
        self.get_logger().info(f"Distances: dist_red={self.dist2red}, dist_green={self.dist2green}, dist_blue={self.dist2blue}")
    

        
    # Callbacks for leader signals
    def on_red(self, msg):
        if self.dist2red < 3.0:
            self.signals["red"] = True

    def on_green(self, msg):
        if self.dist2green < 3.0:
            self.signals["green"] = True

    def on_blue(self, msg):
        if self.dist2blue < 3.0:
            self.signals["blue"] = True

    # Map SCT controllable events to robot commands
    def exec_sct_action(self, event_name):
        twist = Twist()
        if event_name == "EV_moveFW":
            self.get_logger().info("Moving forward")
            twist.linear.x = 10.8
            self.pub.publish(twist)
            time.sleep(1.0)
        elif event_name == "EV_moveStop":
            self.get_logger().info("Stopping")
            twist.linear.x = 0.0
            self.pub.publish(twist)
            time.sleep(1.0)
        elif event_name == "EV_turnCW":
            self.get_logger().info("Turning CW")
            twist.angular.z = -10.8
            self.pub.publish(twist)
            time.sleep(1.0)
        elif event_name == "EV_turnCCW":
            self.get_logger().info("Turning CCW")
            twist.angular.z = 10.8
            self.pub.publish(twist)
            time.sleep(1.0)
        

    # Main loop
    def run_step(self):
        if not self.ready:
            return  # Do nothing until topics exist

        # Determine which colors were received this timestep
        colors_received = [c for c in ["red", "green", "blue"] if self.signals[c]]

        # Reset SCT input buffer
        self.sct.input_buffer = []

        if len(colors_received) <= 1:
            # Received 0 or 1 colors → do not move
            #self.get_logger().info(f"Received {len(colors_received)} color(s) ({colors_received}); robot stays put")
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)
        else:
            # Received more than one color → feed them to SCT
            for color in colors_received:
                ev_name = f"EV_get{color[0].upper()}"  # EV_getR, EV_getG, EV_getB
                self.sct.input_buffer.append(self.sct.EV[ev_name])
                #self.get_logger().info(f"Received {color} leader signal; feeding {ev_name} to SCT")

            # Execute SCT step
            self.sct.run_step()

            # Execute all enabled controllable events as robot actions
            for event_name in self.sct.get_enabled_events():
                self.exec_sct_action(event_name)

        # Reset signals for next iteration
        for k in self.signals:
            self.signals[k] = False

def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == "__main__":
    main()