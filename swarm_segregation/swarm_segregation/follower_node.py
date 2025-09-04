import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from swarm_segregation.movement_controller import MovementController
from swarm_segregation.position_tracker import PositionTracker
from swarm_segregation.leader_signals import LeaderSignalHandler
from swarm_segregation.sct_wrapper import SCTWrapper


# ps aux | grep -E "gz|ign"
class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower')

        self.ready = False

        # Parameters
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("color", "yellow")
        self.declare_parameter("is_leader", False)

        spawn_x = self.get_parameter("spawn_x").get_parameter_value().double_value
        spawn_y = self.get_parameter("spawn_y").get_parameter_value().double_value

        # Modules
        self.tracker = PositionTracker(spawn_x, spawn_y, self.get_logger())
        self.signals = LeaderSignalHandler()
        self.sct = SCTWrapper()
        self.mover = MovementController(self.create_publisher(Twist, 'cmd_vel', 10), self.get_logger())

        # Subscriptions
        self.create_subscription(String, '/leader_broadcast/red',   lambda msg: self.signals.on_red(msg, self.tracker.dist2red), 10)
        self.create_subscription(String, '/leader_broadcast/green', lambda msg: self.signals.on_green(msg, self.tracker.dist2green), 10)
        self.create_subscription(String, '/leader_broadcast/blue',  lambda msg: self.signals.on_blue(msg, self.tracker.dist2blue), 10)
        self.create_subscription(Odometry, 'odom', self.tracker.odom_callback, 10)

        self.twist = Twist()
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Timer
        self.create_timer(1.0, self.run_step)
        self.timer = self.create_timer(0.1, self.publisher)
        self.last_cmd_time = 0.0

        self.ready_timer = self.create_timer(1.0, self.check_ready)

    def check_ready(self):
        # Check if required topics exist
        topics = [t[0] for t in self.get_topic_names_and_types()]
        odom_topic = f'{self.get_namespace()}/odom'
        leader_topics = [f'/leader_broadcast/{c}' for c in ["red", "green", "blue"]]

        if odom_topic in topics and all(t in topics for t in leader_topics):
            self.ready = True
            self.get_logger().info("All required topics ready, starting behavior!")
            self.ready_timer.cancel()  # Stop checking

    def run_step(self):
        if not self.ready:
            return  # Do nothing until topics exist
        colors_received = self.signals.get_active_signals()
        # self.get_logger().info(f'colors received: {colors_received}')
        self.sct.sct.input_buffer = []

        if len(colors_received) <= 1:
            if time.time() - self.last_cmd_time > 2.0:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
            return

        linear, angular = self.twist.linear.x, self.twist.angular.z

        for color in colors_received:
            ev_name = f"EV_get{color[0].upper()}"
            self.sct.add_event(ev_name)

        for ev in self.sct.run():
            if ev == "EV_moveFW":
                self.get_logger().info(f'ev: {ev}')
                linear = 0.4
            elif ev == "EV_moveStop":
                self.get_logger().info(f'ev: {ev}')
                linear = 0.0
                angular = 0.0
            elif ev == "EV_turnCW":
                self.get_logger().info(f'ev: {ev}')
                angular = -0.4
            elif ev == "EV_turnCCW":
                self.get_logger().info(f'ev: {ev}')
                angular = 0.4

        self.twist.linear.x = linear
        self.twist.angular.z = angular
        self.last_cmd_time = time.time()  

    def publisher(self):
        self.cmd_pub.publish(self.twist)
    

def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
