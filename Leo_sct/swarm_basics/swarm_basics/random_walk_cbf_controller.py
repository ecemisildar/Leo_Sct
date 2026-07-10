import csv
import math
import random
import time
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool


class RandomWalkCbfController(Node):
    """Random-walk controller with a camera/peer-warning CBF safety filter.

    This node does not load or execute an SCT supervisor. It creates random
    velocity pulses and filters them using the depth image processor outputs
    and peer_warning_zone messages from the warning relay.
    """

    def __init__(self):
        super().__init__("random_walk_cbf_controller")

        self.ns = self.get_namespace().strip("/") or "root"
        self.robot_index = self._namespace_index()

        # General runtime switch and control-loop timing.
        self.enabled = bool(self.declare_parameter("enabled", True).value)
        self.control_period_s = float(
            self.declare_parameter("control_period_s", 0.1).value
        )

        # Exploration command generator. "random_walk" keeps the old behavior;
        # "bug" drives straight until an obstacle appears, then follows/turns
        # around the detected side without using a goal point.
        self.exploration_algorithm = (
            str(self.declare_parameter("exploration_algorithm", "random_walk").value)
            .strip()
            .lower()
        )
        if self.exploration_algorithm not in {"random_walk", "bug"}:
            self.get_logger().warn(
                f"Unknown exploration_algorithm='{self.exploration_algorithm}', "
                "falling back to random_walk"
            )
            self.exploration_algorithm = "random_walk"

        # Random-walk command limits. A pulse is reused for a short hold time so
        # the robot moves in smooth segments instead of changing every timer tick.
        self.motion_hold_duration_s = float(
            self.declare_parameter("motion_hold_duration_s", 0.6).value
        )
        self.forward_min_mps = float(self.declare_parameter("forward_min_mps", 0.10).value) # meter per second
        self.forward_max_mps = float(self.declare_parameter("forward_max_mps", 0.35).value)
        self.turn_probability = float(
            self.declare_parameter("turn_probability", 0.40).value
        )
        self.turn_min_radps = float(self.declare_parameter("turn_min_radps", 0.35).value)
        self.turn_max_radps = float(self.declare_parameter("turn_max_radps", 0.90).value)
        self.max_turn_hold_s = float(self.declare_parameter("max_turn_hold_s", 1.4).value)

        # Goal-free Bug-style exploration speeds used when exploration_algorithm
        # is "bug". CBF filtering still clamps these commands before publishing.
        self.bug_forward_mps = float(
            self.declare_parameter("bug_forward_mps", 0.28).value
        )
        self.bug_wall_follow_mps = float(
            self.declare_parameter("bug_wall_follow_mps", 0.10).value
        )
        self.bug_turn_radps = float(
            self.declare_parameter("bug_turn_radps", 0.45).value
        )

        # CBF and camera-zone parameters. The zone memory bridges brief CLEAR
        # gaps, while distance freshness decides whether forward motion is safe.
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

        # Peer warnings come from other robots. Priority uses robot indices to
        # decide who yields when two robots warn each other at the same time.
        self.peer_warning_enabled = bool(
            self.declare_parameter("peer_warning_enabled", True).value
        )
        self.peer_warning_timeout_s = float(
            self.declare_parameter("peer_warning_timeout_s", 0.6).value
        )
        self.peer_warning_priority_enabled = bool(
            self.declare_parameter("peer_warning_priority_enabled", True).value
        )
        self.peer_warning_yield_linear_scale = float(
            self.declare_parameter("peer_warning_yield_linear_scale", 0.45).value
        )
        self.peer_warning_pass_linear_scale = float(
            self.declare_parameter("peer_warning_pass_linear_scale", 0.75).value
        )
        self.peer_warning_avoid_turn_radps = float(
            self.declare_parameter("peer_warning_avoid_turn_radps", 0.65).value
        )
        self.peer_warning_corner_slow_distance_m = float(
            self.declare_parameter("peer_warning_corner_slow_distance_m", 0.85).value
        )

        # Contact recovery handles physical collision reports from Gazebo by
        # temporarily overriding the random walk with a backing/turning command.
        self.contact_recovery_enabled = bool(
            self.declare_parameter("contact_recovery_enabled", True).value
        )
        self.contact_recovery_duration_s = float(
            self.declare_parameter("contact_recovery_duration_s", 1.2).value
        )
        self.contact_recovery_linear_x = float(
            self.declare_parameter("contact_recovery_linear_x", -0.16).value
        )
        self.contact_recovery_angular_z = float(
            self.declare_parameter("contact_recovery_angular_z", 0.8).value
        )
        self.contact_recovery_retrigger_block_s = float(
            self.declare_parameter("contact_recovery_retrigger_block_s", 0.4).value
        )
        self.contact_topic = str(
            self.declare_parameter(
                "contact_topic",
                f"/world/random_world/model/{self.ns}/link/{self.ns}/base_footprint/"
                "sensor/contact_sensor/contact",
            ).value
        )

        # Optional run metadata used only for CSV logging.
        self.results_dir = str(self.declare_parameter("results_dir", "").value).strip()
        self.run_id = str(self.declare_parameter("run_id", "").value).strip()

        # Offset the seed by robot index so a swarm does not choose identical
        # random-walk pulses when every node receives the same base seed.
        base_seed = int(self.declare_parameter("random_seed", 12345).value)
        self.robot_seed = base_seed + self.robot_index
        self.rng = random.Random(self.robot_seed)

        # Perception state is kept as short-lived memory so one dropped message
        # does not immediately clear an obstacle or peer warning.
        self.raw_zone = "CLEAR"
        self.last_non_clear_zone = "CLEAR"
        self.last_non_clear_zone_time = 0.0
        self.peer_warning_zone = "CLEAR"
        self.peer_warning_source = ""
        self.last_peer_warning_time = 0.0
        self.front_obstacle_distance_m = float("inf")
        self.last_front_obstacle_distance_time = 0.0
        self.active_twist = Twist()
        self.motion_until = 0.0
        self.bug_turn_sign = -1.0 if (self.robot_index % 2 == 0) else 1.0
        self.contact_recovery_until = 0.0
        self.last_contact_recovery_started_at = 0.0
        self.contact_recovery_source = ""
        self.stop_sent = False
        self.cbf_filter_log_path: Optional[Path] = None
        self._init_cbf_filter_log()

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(String, "detected_zones", self.zone_callback, 10)
        self.create_subscription(String, "peer_warning_zone", self.peer_warning_callback, 10)
        self.create_subscription(
            Float32,
            "front_obstacle_distance",
            self.front_obstacle_distance_callback,
            10,
        )
        self.create_subscription(Contacts, self.contact_topic, self.contact_callback, 10)
        self.create_service(SetBool, "enable_random_walk_cbf", self.enable_callback)
        self.timer = self.create_timer(self.control_period_s, self.timer_callback)

        self.get_logger().info(
            "random_walk_cbf_controller active "
            f"(robot={self.ns}, enabled={self.enabled}, seed={self.robot_seed}, "
            f"exploration_algorithm={self.exploration_algorithm}, "
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
        """Create the optional CSV log used to audit CBF velocity changes."""
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
                    "peer_warning_zone",
                    "peer_warning_age_s",
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
        """Enable or disable motion through a ROS service."""
        self.enabled = bool(request.data)
        self.stop_sent = False
        if not self.enabled:
            self.active_twist = Twist()
            self.motion_until = 0.0
            self.contact_recovery_until = 0.0
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

    def peer_warning_callback(self, msg: String):
        if not self.peer_warning_enabled:
            return

        zone = self._parse_zone(msg.data)
        if zone == "CLEAR":
            return

        self.peer_warning_zone = zone
        self.peer_warning_source = msg.data
        self.last_peer_warning_time = time.time()

    def contact_callback(self, msg: Contacts):
        if not self.contact_recovery_enabled or not self.enabled or not msg.contacts:
            return

        now = time.time()
        # Contacts can arrive in bursts from the simulator; throttle recovery
        # restarts so one collision produces one continuous escape maneuver.
        if (
            now - self.last_contact_recovery_started_at
            < self.contact_recovery_retrigger_block_s
        ):
            return

        self.last_contact_recovery_started_at = now
        self.contact_recovery_until = max(
            self.contact_recovery_until,
            now + self.contact_recovery_duration_s,
        )
        self.contact_recovery_source = self._contact_summary(msg)
        self.active_twist = Twist()
        self.motion_until = 0.0
        self.get_logger().info(
            "CONTACT RECOVERY activated "
            f"(source={self.contact_recovery_source}, "
            f"duration_s={self.contact_recovery_duration_s:.2f})"
        )

    def _contact_summary(self, msg: Contacts) -> str:
        names = []
        for contact in msg.contacts[:3]:
            for collision in (contact.collision1, contact.collision2):
                name = getattr(collision, "name", str(collision))
                if self.ns not in name:
                    names.append(name)
        return ",".join(names[:3]) or "local_contact"

    def _contact_recovery_twist(self) -> Twist:
        twist = Twist()
        turn_sign = 1.0 if (self.robot_index % 2 == 0) else -1.0
        zone = self._effective_zone()

        # Recovery normally backs up and turns. If another robot is reported
        # behind us, rotate in place instead of backing into it.
        if zone == "BACK" and self._peer_warning_is_active():
            twist.linear.x = 0.0
        elif zone == "LEFT":
            turn_sign = -1.0
        elif zone == "RIGHT":
            turn_sign = 1.0

        if zone != "BACK":
            twist.linear.x = min(0.0, self.contact_recovery_linear_x)
        twist.angular.z = turn_sign * abs(self.contact_recovery_angular_z)
        return twist

    def _parse_zone(self, data: str) -> str:
        token = data.strip().upper()
        if ":" in token:
            token = token.split(":", 1)[0].strip()
        if "," in token:
            token = token.split(",", 1)[0].strip()
        return token if token in {"LEFT", "RIGHT", "CORNER", "BACK", "CLEAR"} else "CLEAR"

    def _effective_zone(self) -> str:
        """Return the active obstacle zone, including recent and peer warnings."""
        if self.raw_zone != "CLEAR":
            return self.raw_zone
        if self._peer_warning_is_active():
            return self.peer_warning_zone
        if (
            self.last_non_clear_zone != "CLEAR"
            and (time.time() - self.last_non_clear_zone_time) <= self.zone_memory_s
        ):
            return self.last_non_clear_zone
        return "CLEAR"

    def _peer_warning_is_active(self) -> bool:
        return (
            self.peer_warning_enabled
            and self.peer_warning_zone != "CLEAR"
            and (time.time() - self.last_peer_warning_time) <= self.peer_warning_timeout_s
        )

    def _peer_warning_fields(self):
        fields = {}
        for part in self.peer_warning_source.split(","):
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            fields[key.strip().lower()] = value.strip()
        return fields

    def _peer_warning_float(self, key: str):
        try:
            return float(self._peer_warning_fields().get(key, ""))
        except (TypeError, ValueError):
            return None

    def _robot_index_from_name(self, name: str):
        text = str(name or "").strip()
        if text.isdigit():
            return int(text)
        if "_" not in text:
            return None
        try:
            return int(text.rsplit("_", 1)[1])
        except ValueError:
            return None

    def _should_yield_to_peer(self) -> bool:
        """Use robot index as a deterministic tie-breaker for peer conflicts."""
        if not self.peer_warning_priority_enabled:
            return True
        source_idx = self._robot_index_from_name(
            self._peer_warning_fields().get("source", "")
        )
        if source_idx is None:
            return True
        return self.robot_index > source_idx

    def _scale_positive_linear(self, twist: Twist, scale: float):
        if twist.linear.x > 0.0:
            twist.linear.x *= max(0.0, min(1.0, scale))

    def _turn_away_from_peer_warning(self, twist: Twist, zone: str, reasons):
        """Adjust angular velocity so the robot rotates away from a peer."""
        turn = abs(self.peer_warning_avoid_turn_radps)
        if zone == "LEFT":
            if twist.angular.z > -turn:
                twist.angular.z = -turn
                reasons.append("peer_warning_turn_away_left")
        elif zone == "RIGHT":
            if twist.angular.z < turn:
                twist.angular.z = turn
                reasons.append("peer_warning_turn_away_right")
        elif zone == "CORNER":
            bearing = self._peer_warning_float("bearing_rad")
            if bearing is None:
                turn_sign = -1.0 if (self.robot_index % 2 == 0) else 1.0
            else:
                turn_sign = -1.0 if bearing > 0.0 else 1.0
            desired = turn_sign * turn
            if abs(twist.angular.z) < turn:
                twist.angular.z = desired
                reasons.append("peer_warning_turn_away_front")

    def _apply_peer_warning_avoidance(self, twist: Twist, reasons):
        """Apply cooperative collision-avoidance rules before distance CBF."""
        if not self._peer_warning_is_active():
            return

        zone = self.peer_warning_zone
        yielding = self._should_yield_to_peer()
        slow_scale = (
            self.peer_warning_yield_linear_scale
            if yielding
            else self.peer_warning_pass_linear_scale
        )

        if zone == "BACK":
            if twist.linear.x < 0.0:
                twist.linear.x = 0.0
                reasons.append("peer_warning_stop_backward")
            return

        if zone in {"LEFT", "RIGHT"}:
            self._turn_away_from_peer_warning(twist, zone, reasons)
            self._scale_positive_linear(twist, slow_scale)
            reasons.append(
                "peer_warning_yield_slow_side" if yielding else "peer_warning_pass_slow_side"
            )
            return

        if zone == "CORNER":
            distance = self._peer_warning_float("distance")
            self._turn_away_from_peer_warning(twist, zone, reasons)
            if (
                distance is not None
                and distance > self.peer_warning_corner_slow_distance_m
                and twist.linear.x > 0.0
            ):
                self._scale_positive_linear(twist, slow_scale)
                reasons.append(
                    "peer_warning_yield_slow_front"
                    if yielding
                    else "peer_warning_pass_slow_front"
                )
            elif twist.linear.x > 0.0:
                twist.linear.x = 0.0
                reasons.append("peer_warning_stop_front_close")

    def _new_random_walk_twist(self):
        """Generate one random motion pulse and its hold duration."""
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

    def _new_bug_exploration_twist(self):
        """Generate a goal-free Bug-style exploration command from obstacle zones."""
        twist = Twist()
        zone = self._effective_zone()
        turn = abs(self.bug_turn_radps)

        if zone == "LEFT":
            twist.linear.x = self.bug_wall_follow_mps
            twist.angular.z = -turn
        elif zone == "RIGHT":
            twist.linear.x = self.bug_wall_follow_mps
            twist.angular.z = turn
        elif zone == "CORNER":
            twist.linear.x = self.bug_wall_follow_mps
            twist.angular.z = self.bug_turn_sign * turn
        elif zone == "BACK":
            twist.linear.x = self.bug_forward_mps
        else:
            twist.linear.x = self.bug_forward_mps

        return twist, self.control_period_s

    def _new_exploration_twist(self):
        if self.exploration_algorithm == "bug":
            return self._new_bug_exploration_twist()
        return self._new_random_walk_twist()

    def _log_cbf_filter(self, requested: Twist, filtered: Twist, reasons, distance_age: float):
        if self.cbf_filter_log_path is None or not reasons:
            return

        front_distance = (
            f"{self.front_obstacle_distance_m:.6f}"
            if math.isfinite(self.front_obstacle_distance_m)
            else "inf"
        )
        peer_warning_age = (
            time.time() - self.last_peer_warning_time
            if self.last_peer_warning_time > 0.0
            else float("inf")
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
                    self.peer_warning_zone if self._peer_warning_is_active() else "CLEAR",
                    f"{peer_warning_age:.6f}" if math.isfinite(peer_warning_age) else "inf",
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
        """Clamp the requested velocity according to obstacle and peer state."""
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

        self._apply_peer_warning_avoidance(filtered, reasons)

        if (
            self.cbf_zone_stop_forward
            and filtered.linear.x > 0.0
            and zone == "CORNER"
            and not self._peer_warning_is_active()
        ):
            # A visual front-zone warning is treated as a hard stop unless it is
            # only a peer warning, which has its own yielding/pass behavior.
            filtered.linear.x = 0.0
            reasons.append("zone_stop_forward")

        if self.cbf_zone_avoid_turning_into_obstacle:
            # Side-zone warnings only block turns that would steer toward the
            # occupied side; turning away remains available.
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
            if (
                self.cbf_stop_on_stale_distance
                and zone in {"LEFT", "RIGHT", "CORNER"}
                and (not self._peer_warning_is_active() or self.raw_zone != "CLEAR")
            ):
                # If a camera zone says an obstacle is present but the measured
                # distance is stale, fail closed by stopping forward motion.
                filtered.linear.x = 0.0
                reasons.append("stale_distance_stop")
            self._log_cbf_filter(twist, filtered, reasons, distance_age)
            return filtered

        # Standard CBF condition: h = d - d_safe, with v <= alpha * h.
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
        self.cmd_pub.publish(self._cbf_filter_twist(twist))

    def timer_callback(self):
        """Main control loop: stop, recover, or publish a filtered random walk."""
        if not self.enabled:
            if not self.stop_sent:
                self._publish_stop()
            return

        self.stop_sent = False
        now = time.time()
        if now < self.contact_recovery_until:
            self.active_twist = self._contact_recovery_twist()
            self._publish_cmd(self.active_twist)
            return

        if now >= self.motion_until:
            self.active_twist, hold = self._new_exploration_twist()
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
