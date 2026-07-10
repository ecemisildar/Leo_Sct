import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class PeerObstacleWarningTest(Node):
    """Inject a peer obstacle-context warning and print the target reaction."""

    def __init__(self):
        super().__init__("peer_obstacle_warning_test")

        self.target_robot = str(self.declare_parameter("target_robot", "robot_1").value)
        self.source_robot = str(self.declare_parameter("source_robot", "robot_0").value)
        self.warning_zone = (
            str(self.declare_parameter("warning_zone", "BACK").value).strip().upper()
        )
        self.distance_m = float(self.declare_parameter("distance_m", 0.75).value)
        self.obstacle_distance_m = float(
            self.declare_parameter("obstacle_distance_m", 0.45).value
        )
        self.publish_period_s = float(
            self.declare_parameter("publish_period_s", 0.1).value
        )
        self.test_duration_s = float(self.declare_parameter("test_duration_s", 8.0).value)
        self.print_period_s = float(self.declare_parameter("print_period_s", 0.5).value)

        self.start_time = time.time()
        self.last_print_time = 0.0
        self.last_cmd_time = 0.0
        self.last_cmd = Twist()

        self.warning_pub = self.create_publisher(
            String,
            f"/{self.target_robot}/peer_warning_zone",
            10,
        )
        self.create_subscription(
            Twist,
            f"/{self.target_robot}/cmd_vel",
            self.cmd_callback,
            10,
        )
        self.timer = self.create_timer(self.publish_period_s, self.timer_callback)

        self.get_logger().info(
            "peer_obstacle_warning_test active "
            f"(source={self.source_robot}, target={self.target_robot}, "
            f"zone={self.warning_zone}, duration_s={self.test_duration_s:.1f})"
        )
        self.get_logger().info(
            "Watch the target CBF log for reasons such as "
            "peer_warning_stop_backward_obstacle or peer_warning_stop_backward."
        )

    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = time.time()

    def warning_message(self) -> String:
        msg = String()
        msg.data = ",".join(
            [
                self.warning_zone,
                f"source={self.source_robot}",
                "source_zone=test",
                "target_zone=unknown",
                "zone_reason=test_peer_obstacle_context",
                f"distance={self.distance_m:.3f}",
                "bearing_rad=3.141593",
                "bearing_deg=180.000",
                "bearing_zone_reason=test_rear_sector",
                f"target={self.target_robot}",
                "obstacle_behind=true",
                f"obstacle_zone={self.warning_zone}",
                f"obstacle_distance={self.obstacle_distance_m:.3f}",
                "obstacle_zone_age_s=0.000",
                "obstacle_distance_age_s=0.000",
            ]
        )
        return msg

    def timer_callback(self):
        now = time.time()
        elapsed = now - self.start_time
        if elapsed > self.test_duration_s:
            self.get_logger().info("test complete")
            rclpy.shutdown()
            return

        self.warning_pub.publish(self.warning_message())

        if now - self.last_print_time >= self.print_period_s:
            self.last_print_time = now
            cmd_age = now - self.last_cmd_time if self.last_cmd_time > 0.0 else float("inf")
            self.get_logger().info(
                f"sent warning to /{self.target_robot}/peer_warning_zone; "
                f"latest /{self.target_robot}/cmd_vel "
                f"linear.x={self.last_cmd.linear.x:.3f}, "
                f"angular.z={self.last_cmd.angular.z:.3f}, "
                f"age_s={cmd_age:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PeerObstacleWarningTest()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
