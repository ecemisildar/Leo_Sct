import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
import math


class GoalNavigator(Node):
    def __init__(self):
        super().__init__('goal_navigator_bug2')

        # --- Goal parameters ---
        self.goal = (8.0, 0.0)  # [m] target coordinate
        self.goal_tolerance = 0.2

        # --- Robot state ---
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        # --- Flags for Bug2 logic ---
        self.state = "GOAL_SEEK"   # ["GOAL_SEEK", "WALL_FOLLOW"]
        self.hit_point = None      # (x, y)
        self.leave_point = None
        self.obstacle_detected = False
        self.min_distance_to_goal = float("inf")

        # --- ROS interfaces ---
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.obstacle_sub = self.create_subscription(Bool, 'obstacle_detected', self.obstacle_callback, 10)
        self.nav_pub = self.create_publisher(String, 'nav_hint', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f"Bug2 GoalNavigator started. Goal: {self.goal}")

    # -----------------------------
    # Callbacks
    # -----------------------------
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def obstacle_callback(self, msg: Bool):
        self.obstacle_detected = msg.data

    # -----------------------------
    # Helpers
    # -----------------------------
    def distance_to_goal(self):
        gx, gy = self.goal
        return math.hypot(gx - self.x, gy - self.y)

    def on_mline(self, threshold=0.1):
        """Check if robot is close to the line from start to goal."""
        x0, y0 = 0.0, 0.0  # start assumed at origin
        gx, gy = self.goal
        num = abs((gy - y0) * self.x - (gx - x0) * self.y)
        den = math.hypot(gx - x0, gy - y0)
        return (num / den) < threshold

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def heading_to_goal(self):
        gx, gy = self.goal
        dx = gx - self.x
        dy = gy - self.y
        return math.atan2(dy, dx)

    # -----------------------------
    # Main Behavior
    # -----------------------------
    def timer_callback(self):
        if not self.odom_received:
            return

        distance = self.distance_to_goal()
        msg = String()

        # --- Check if goal reached ---
        if distance < self.goal_tolerance:
            msg.data = "AT_GOAL"
            self.state = "GOAL_REACHED"
            self.nav_pub.publish(msg)
            self.get_logger().info("Reached goal")
            return

        # --- Bug2 state machine ---
        if self.state == "GOAL_SEEK":
            if self.obstacle_detected:
                self.get_logger().info("Hit obstacle, switching to WALL_FOLLOW.")
                self.hit_point = (self.x, self.y)
                self.state = "WALL_FOLLOW"
                self.min_distance_to_goal = distance
                msg.data = "LEFT"  # start wall following left
            else:
                # Move toward goal direction
                angle_diff = self.normalize_angle(self.heading_to_goal() - self.yaw)
                if abs(angle_diff) < math.radians(15):
                    msg.data = "FORWARD"
                elif angle_diff > 0:
                    msg.data = "LEFT"
                else:
                    msg.data = "RIGHT"

        elif self.state == "WALL_FOLLOW":
            if not self.obstacle_detected and self.on_mline() and distance < self.min_distance_to_goal:
                self.get_logger().info("Back on M-line closer to goal, switching to GOAL_SEEK.")
                self.state = "GOAL_SEEK"
                msg.data = "FORWARD"
            else:
                # Continue following wall
                msg.data = "LEFT" if self.obstacle_detected else "FORWARD"
                self.min_distance_to_goal = min(self.min_distance_to_goal, distance)

        else:
            msg.data = "STOP"

        self.nav_pub.publish(msg)
        self.get_logger().debug(f"Bug2 → {self.state} → {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
