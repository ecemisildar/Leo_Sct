import csv
import math
import random
import time
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool


class RandomWalkCbfController(Node):
    """Random-walk controller with a camera-derived CBF safety filter.

    This node does not load or execute an SCT supervisor. It creates random
    velocity pulses and filters them using the depth image processor outputs:
    detected_zones and front_obstacle_distance.
    """

    def __init__(self):
        super().__init__("random_walk_cbf_controller")

        self.ns = self.get_namespace().strip("/") or "root"
        self.robot_index = self._namespace_index()

        self.enabled = bool(self.declare_parameter("enabled", True).value)
        self.static_mode = bool(self.declare_parameter("static", False).value)
        self.control_period_s = float(
            self.declare_parameter("control_period_s", 0.1).value
        )
        self.motion_hold_duration_s = float(
            self.declare_parameter("motion_hold_duration_s", 0.35).value
        )
        self.forward_min_mps = float(self.declare_parameter("forward_min_mps", 0.10).value)
        self.forward_max_mps = float(self.declare_parameter("forward_max_mps", 0.35).value)
        self.turn_probability = float(
            self.declare_parameter("turn_probability", 0.40).value
        )
        self.turn_min_radps = float(self.declare_parameter("turn_min_radps", 0.35).value)
        self.turn_max_radps = float(self.declare_parameter("turn_max_radps", 0.90).value)
        self.max_turn_hold_s = float(self.declare_parameter("max_turn_hold_s", 1.4).value)

        self.zone_memory_s = float(self.declare_parameter("zone_memory_s", 0.4).value)
        self.cbf_enabled = bool(self.declare_parameter("cbf_enabled", True).value)
        self.cbf_safety_distance_m = float(
            self.declare_parameter("cbf_safety_distance_m", 0.60).value
        )
        self.cbf_alpha = float(self.declare_parameter("cbf_alpha", 1.2).value)
        self.cbf_distance_timeout_s = float(
            self.declare_parameter("cbf_distance_timeout_s", 0.4).value
        )
        self.cbf_stop_on_stale_distance = bool(
            self.declare_parameter("cbf_stop_on_stale_distance", True).value
        )
        self.cbf_zone_stop_forward = bool(
            self.declare_parameter("cbf_zone_stop_forward", True).value
        )
        self.cbf_zone_avoid_turning_into_obstacle = bool(
            self.declare_parameter("cbf_zone_avoid_turning_into_obstacle", True).value
        )
        self.cbf_filter_log_enabled = bool(
            self.declare_parameter("cbf_filter_log_enabled", False).value
        )
        self.results_dir = str(self.declare_parameter("results_dir", "").value).strip()
        self.run_id = str(self.declare_parameter("run_id", "").value).strip()

        base_seed = int(self.declare_parameter("random_seed", 12345).value)
        self.robot_seed = base_seed + self.robot_index
        self.rng = random.Random(self.robot_seed)

        self.raw_zone = "CLEAR"
        self.last_non_clear_zone = "CLEAR"
        self.last_non_clear_zone_time = 0.0
        self.front_obstacle_distance_m = float("inf")
        self.last_front_obstacle_distance_time = 0.0
        self.active_twist = Twist()
        self.motion_until = 0.0
        self.stop_sent = False
        self.cbf_filter_log_path: Optional[Path] = None
        self._init_cbf_filter_log()

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(String, "detected_zones", self.zone_callback, 10)
        self.create_subscription(
            Float32,
            "front_obstacle_distance",
            self.front_obstacle_distance_callback,
            10,
        )
        self.create_service(SetBool, "enable_random_walk_cbf", self.enable_callback)
        self.timer = self.create_timer(self.control_period_s, self.timer_callback)

        self.get_logger().info(
            "random_walk_cbf_controller active "
            f"(robot={self.ns}, enabled={self.enabled}, seed={self.robot_seed}, "
            f"cbf_safety_distance_m={self.cbf_safety_distance_m:.2f})"
        )

    def _namespace_index(self) -> int:
        if "_" not in self.ns:
            return 0
        try:
            return int(self.ns.rsplit("_", 1)[1])
        except ValueError:
            return 0

    def _run_log_dir(self) -> Path:
        if self.results_dir:
            base_dir = Path(self.results_dir)
            return base_dir / self.run_id if self.run_id else base_dir
        return Path.cwd()

    def _init_cbf_filter_log(self):
        if not self.cbf_filter_log_enabled:
            return

        log_dir = self._run_log_dir()
        log_dir.mkdir(parents=True, exist_ok=True)
        self.cbf_filter_log_path = log_dir / f"random_walk_cbf_{self.ns}.csv"
        if self.cbf_filter_log_path.exists():
            return

        with self.cbf_filter_log_path.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "wall_time",
                    "robot",
                    "reasons",
                    "raw_zone",
                    "effective_zone",
                    "front_distance_m",
                    "distance_age_s",
                    "cbf_safety_distance_m",
                    "cbf_alpha",
                    "requested_linear_x",
                    "requested_angular_z",
                    "filtered_linear_x",
                    "filtered_angular_z",
                ]
            )

    def enable_callback(self, request, response):
        self.enabled = bool(request.data)
        self.stop_sent = False
        if not self.enabled:
            self.active_twist = Twist()
            self.motion_until = 0.0
            self._publish_stop()
        response.success = True
        response.message = f"random_walk_cbf enabled={self.enabled}"
        return response

    def zone_callback(self, msg: String):
        zone = self._parse_zone(msg.data)
        now = time.time()
        self.raw_zone = zone
        if zone != "CLEAR":
            self.last_non_clear_zone = zone
            self.last_non_clear_zone_time = now

    def front_obstacle_distance_callback(self, msg: Float32):
        distance = float(msg.data)
        self.front_obstacle_distance_m = (
            distance if math.isfinite(distance) else float("inf")
        )
        self.last_front_obstacle_distance_time = time.time()

    def _parse_zone(self, data: str) -> str:
        token = data.strip().upper()
        if ":" in token:
            token = token.split(":", 1)[0].strip()
        if "," in token:
            token = token.split(",", 1)[0].strip()
        return token if token in {"LEFT", "RIGHT", "CORNER", "CLEAR"} else "CLEAR"

    def _effective_zone(self) -> str:
        if self.raw_zone != "CLEAR":
            return self.raw_zone
        if (
            self.last_non_clear_zone != "CLEAR"
            and (time.time() - self.last_non_clear_zone_time) <= self.zone_memory_s
        ):
            return self.last_non_clear_zone
        return "CLEAR"

    def _new_random_walk_twist(self):
        twist = Twist()
        twist.linear.x = self.rng.uniform(self.forward_min_mps, self.forward_max_mps)
        hold = self.motion_hold_duration_s

        if self.rng.random() < self.turn_probability:
            turn_angle = self.rng.uniform(-math.pi, math.pi)
            twist.angular.z = math.copysign(
                self.rng.uniform(self.turn_min_radps, self.turn_max_radps),
                turn_angle,
            )
            hold = min(
                abs(turn_angle) / max(1e-6, abs(twist.angular.z)),
                self.max_turn_hold_s,
            )
        return twist, hold

    def _log_cbf_filter(self, requested: Twist, filtered: Twist, reasons, distance_age: float):
        if self.cbf_filter_log_path is None or not reasons:
            return

        front_distance = (
            f"{self.front_obstacle_distance_m:.6f}"
            if math.isfinite(self.front_obstacle_distance_m)
            else "inf"
        )
        with self.cbf_filter_log_path.open("a", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    f"{time.time():.6f}",
                    self.ns,
                    "|".join(reasons),
                    self.raw_zone,
                    self._effective_zone(),
                    front_distance,
                    f"{distance_age:.6f}" if math.isfinite(distance_age) else "inf",
                    f"{self.cbf_safety_distance_m:.6f}",
                    f"{self.cbf_alpha:.6f}",
                    f"{requested.linear.x:.6f}",
                    f"{requested.angular.z:.6f}",
                    f"{filtered.linear.x:.6f}",
                    f"{filtered.angular.z:.6f}",
                ]
            )

    def _cbf_filter_twist(self, twist: Twist) -> Twist:
        filtered = Twist()
        filtered.linear.x = float(twist.linear.x)
        filtered.linear.y = float(twist.linear.y)
        filtered.linear.z = float(twist.linear.z)
        filtered.angular.x = float(twist.angular.x)
        filtered.angular.y = float(twist.angular.y)
        filtered.angular.z = float(twist.angular.z)

        if not self.cbf_enabled:
            return filtered

        now = time.time()
        zone = self._effective_zone()
        distance_age = now - self.last_front_obstacle_distance_time
        reasons = []

        if (
            self.cbf_zone_stop_forward
            and filtered.linear.x > 0.0
            and zone in {"LEFT", "RIGHT", "CORNER"}
        ):
            filtered.linear.x = 0.0
            reasons.append("zone_stop_forward")

        if self.cbf_zone_avoid_turning_into_obstacle:
            if zone == "LEFT" and filtered.angular.z > 0.0:
                filtered.angular.z = 0.0
                reasons.append("turn_left_blocked")
            if zone == "RIGHT" and filtered.angular.z < 0.0:
                filtered.angular.z = 0.0
                reasons.append("turn_right_blocked")

        if filtered.linear.x <= 0.0:
            self._log_cbf_filter(twist, filtered, reasons, distance_age)
            return filtered

        distance_is_fresh = (
            self.last_front_obstacle_distance_time > 0.0
            and distance_age <= self.cbf_distance_timeout_s
            and math.isfinite(self.front_obstacle_distance_m)
        )
        if not distance_is_fresh:
            if self.cbf_stop_on_stale_distance and zone != "CLEAR":
                filtered.linear.x = 0.0
                reasons.append("stale_distance_stop")
            self._log_cbf_filter(twist, filtered, reasons, distance_age)
            return filtered

        h = self.front_obstacle_distance_m - self.cbf_safety_distance_m
        max_forward_v = max(0.0, self.cbf_alpha * h)
        if filtered.linear.x > max_forward_v:
            filtered.linear.x = max_forward_v
            reasons.append("distance_cbf_cap")

        self._log_cbf_filter(twist, filtered, reasons, distance_age)
        return filtered

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())
        self.stop_sent = True

    def _publish_cmd(self, twist: Twist):
        if self.static_mode:
            self.cmd_pub.publish(Twist())
            return
        self.cmd_pub.publish(self._cbf_filter_twist(twist))

    def timer_callback(self):
        if not self.enabled:
            if not self.stop_sent:
                self._publish_stop()
            return

        self.stop_sent = False
        now = time.time()
        if now >= self.motion_until:
            self.active_twist, hold = self._new_random_walk_twist()
            self.motion_until = now + hold

        self._publish_cmd(self.active_twist)


def main(args=None):
    rclpy.init(args=args)
    node = RandomWalkCbfController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
