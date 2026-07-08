import csv
import math
import time
from pathlib import Path
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import String


def _wrap_to_pi(angle: float) -> float:
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    while angle > math.pi:
        angle -= 2.0 * math.pi
    return angle


def _yaw_from_quat(q) -> float:
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class RobotProximityWarner(Node):
    def __init__(self):
        super().__init__("robot_proximity_warner")

        self.ns = self.get_namespace().strip("/") or "root"
        self.total_robots = int(self.declare_parameter("total_robots", 5).value)
        self.warning_distance_m = float(
            self.declare_parameter("warning_distance_m", 1.0).value
        )
        self.critical_distance_m = float(
            self.declare_parameter("critical_distance_m", 0.55).value
        )
        self.front_angle_rad = float(
            self.declare_parameter("front_angle_rad", math.radians(50.0)).value
        )
        self.pose_timeout_s = float(self.declare_parameter("pose_timeout_s", 0.6).value)
        self.zone_timeout_s = float(self.declare_parameter("zone_timeout_s", 0.8).value)
        self.publish_period_s = float(
            self.declare_parameter("publish_period_s", 0.1).value
        )
        self.publish_to_peer = bool(self.declare_parameter("publish_to_peer", True).value)
        self.publish_to_self = bool(self.declare_parameter("publish_to_self", False).value)
        self.results_dir = str(self.declare_parameter("results_dir", "").value).strip()
        self.run_id = str(self.declare_parameter("run_id", "").value).strip()
        self.diagnostic_log_enabled = bool(
            self.declare_parameter("warner_diagnostic_log_enabled", True).value
        )
        self.diagnostic_distance_margin_m = float(
            self.declare_parameter("diagnostic_distance_margin_m", 0.25).value
        )

        self.poses: Dict[str, Tuple[float, float, float, float]] = {}
        self.zones: Dict[str, Tuple[str, float]] = {}
        self._diagnostic_log_path = self._make_diagnostic_log_path()
        self.warn_publishers = {}
        for idx in range(self.total_robots):
            robot = f"robot_{idx}"
            self.create_subscription(
                Odometry,
                f"/{robot}/odom",
                self._make_odom_callback(robot),
                10,
            )
            self.warn_publishers[robot] = self.create_publisher(
                String,
                f"/{robot}/peer_warning_zone",
                10,
            )
            self.create_subscription(
                String,
                f"/{robot}/detected_zones",
                self._make_zone_callback(robot),
                10,
            )

        self.timer = self.create_timer(self.publish_period_s, self.timer_callback)
        self.get_logger().info(
            "robot_proximity_warner active "
            f"(robot={self.ns}, total_robots={self.total_robots}, "
            f"warning_distance_m={self.warning_distance_m:.2f}, "
            f"zone_timeout_s={self.zone_timeout_s:.2f}, "
            f"diagnostic_log={self._diagnostic_log_path or 'disabled'})"
        )

    def _make_diagnostic_log_path(self):
        if not self.diagnostic_log_enabled or not self.results_dir or not self.run_id:
            return None

        log_dir = Path(self.results_dir).expanduser() / self.run_id
        log_dir.mkdir(parents=True, exist_ok=True)
        path = log_dir / f"proximity_warner_{self.ns}.csv"
        if not path.exists():
            with path.open("w", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(
                    [
                        "wall_time",
                        "source",
                        "target",
                        "decision",
                        "skip_reason",
                        "source_zone",
                        "source_zone_active",
                        "source_zone_age_s",
                        "target_zone",
                        "target_zone_active",
                        "target_zone_age_s",
                        "distance_m",
                        "warning_distance_m",
                        "source_pose_fresh",
                        "target_pose_fresh",
                        "source_pose_age_s",
                        "target_pose_age_s",
                        "warning_zone",
                        "bearing_deg",
                        "zone_reason",
                    ]
                )
        return path

    def _make_odom_callback(self, robot: str):
        def callback(msg: Odometry):
            self.poses[robot] = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                _yaw_from_quat(msg.pose.pose.orientation),
                time.time(),
            )

        return callback

    def _make_zone_callback(self, robot: str):
        def callback(msg: String):
            self.zones[robot] = (self._parse_zone(msg.data), time.time())

        return callback

    def _parse_zone(self, data: str) -> str:
        token = data.strip().upper()
        if ":" in token:
            token = token.split(":", 1)[0].strip()
        if "," in token:
            token = token.split(",", 1)[0].strip()
        if token not in ("LEFT", "RIGHT", "CORNER", "CLEAR"):
            token = "CLEAR"
        return token

    def _robot_zone(self, robot: str) -> str:
        return self.zones.get(robot, ("CLEAR", 0.0))[0]

    def _robot_zone_is_active(self, robot: str, now: float) -> bool:
        zone, stamp = self.zones.get(robot, ("CLEAR", 0.0))
        return zone != "CLEAR" and (now - stamp) <= self.zone_timeout_s

    def _zone_context(self, robot: str, now: float):
        zone, stamp = self.zones.get(robot, ("CLEAR", 0.0))
        age = "" if stamp <= 0.0 else now - stamp
        active = zone != "CLEAR" and stamp > 0.0 and age <= self.zone_timeout_s
        return zone, active, age

    def _pose_context(self, robot: str, now: float):
        pose = self.poses.get(robot)
        if pose is None:
            return None, False, ""
        age = now - pose[3]
        return pose, age <= self.pose_timeout_s, age

    def _fmt(self, value):
        if value == "" or value is None:
            return ""
        if isinstance(value, bool):
            return str(value).lower()
        if isinstance(value, float):
            return f"{value:.6f}"
        return value

    def _log_decision(
        self,
        now: float,
        target: str,
        decision: str,
        skip_reason: str,
        source_zone: str,
        source_zone_active: bool,
        source_zone_age,
        target_zone: str,
        target_zone_active: bool,
        target_zone_age,
        distance,
        source_pose_fresh: bool,
        target_pose_fresh: bool,
        source_pose_age,
        target_pose_age,
        warning_zone: str = "",
        bearing_rad=None,
        zone_reason: str = "",
    ):
        if self._diagnostic_log_path is None:
            return

        bearing_deg = "" if bearing_rad is None else math.degrees(bearing_rad)
        with self._diagnostic_log_path.open("a", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    self._fmt(now),
                    self.ns,
                    target,
                    decision,
                    skip_reason,
                    source_zone,
                    self._fmt(source_zone_active),
                    self._fmt(source_zone_age),
                    target_zone,
                    self._fmt(target_zone_active),
                    self._fmt(target_zone_age),
                    self._fmt(distance),
                    self._fmt(self.warning_distance_m),
                    self._fmt(source_pose_fresh),
                    self._fmt(target_pose_fresh),
                    self._fmt(source_pose_age),
                    self._fmt(target_pose_age),
                    warning_zone,
                    self._fmt(bearing_deg),
                    zone_reason,
                ]
            )

    def _zone_from_target_to_source(self, target: str, source: str, distance: float):
        tx, ty, tyaw, _ = self.poses[target]
        sx, sy, _, _ = self.poses[source]
        bearing = _wrap_to_pi(math.atan2(sy - ty, sx - tx) - tyaw)

        if distance <= self.critical_distance_m:
            return "CORNER", bearing, "critical_distance"
        if abs(bearing) <= self.front_angle_rad:
            return "CORNER", bearing, "front_sector"
        return ("LEFT" if bearing > 0.0 else "RIGHT"), bearing, "side_sector"

    def _publish_warning(self, target: str, source: str, distance: float):
        zone, bearing, zone_reason = self._zone_from_target_to_source(target, source, distance)
        tx, ty, tyaw, _ = self.poses[target]
        sx, sy, syaw, _ = self.poses[source]
        msg = String()
        msg.data = (
            f"{zone},source={source},source_zone={self._robot_zone(source)},"
            f"target_zone={self._robot_zone(target)},"
            f"distance={distance:.3f},"
            f"bearing_rad={bearing:.6f},bearing_deg={math.degrees(bearing):.3f},"
            f"zone_reason={zone_reason},"
            f"source_x={sx:.6f},source_y={sy:.6f},source_yaw={syaw:.6f},"
            f"target_x={tx:.6f},target_y={ty:.6f},target_yaw={tyaw:.6f}"
        )
        self.warn_publishers[target].publish(msg)
        return zone, bearing, zone_reason

    def timer_callback(self):
        now = time.time()
        source_zone, source_zone_active, source_zone_age = self._zone_context(self.ns, now)
        own_pose, own_pose_fresh, own_pose_age = self._pose_context(self.ns, now)
        if own_pose is None:
            return
        if not own_pose_fresh:
            return

        ox, oy, _, _ = own_pose
        nearest_self = None

        for robot, pose in self.poses.items():
            if robot == self.ns:
                continue
            px, py, _, stamp = pose
            target_zone, target_zone_active, target_zone_age = self._zone_context(robot, now)
            target_pose_age = now - stamp
            target_pose_fresh = target_pose_age <= self.pose_timeout_s
            if not target_pose_fresh:
                self._log_decision(
                    now,
                    robot,
                    "skipped",
                    "target_pose_stale",
                    source_zone,
                    source_zone_active,
                    source_zone_age,
                    target_zone,
                    target_zone_active,
                    target_zone_age,
                    "",
                    own_pose_fresh,
                    target_pose_fresh,
                    own_pose_age,
                    target_pose_age,
                )
                continue

            distance = math.hypot(px - ox, py - oy)
            log_near_pair = (
                distance <= (self.warning_distance_m + self.diagnostic_distance_margin_m)
            )
            warning_zone = ""
            bearing = None
            zone_reason = ""
            if log_near_pair:
                warning_zone, bearing, zone_reason = self._zone_from_target_to_source(
                    robot, self.ns, distance
                )

            if not source_zone_active:
                if log_near_pair:
                    self._log_decision(
                        now,
                        robot,
                        "skipped",
                        "source_zone_inactive",
                        source_zone,
                        source_zone_active,
                        source_zone_age,
                        target_zone,
                        target_zone_active,
                        target_zone_age,
                        distance,
                        own_pose_fresh,
                        target_pose_fresh,
                        own_pose_age,
                        target_pose_age,
                        warning_zone,
                        bearing,
                        zone_reason,
                    )
                continue

            if distance > self.warning_distance_m:
                if log_near_pair:
                    self._log_decision(
                        now,
                        robot,
                        "skipped",
                        "distance_above_warning_threshold",
                        source_zone,
                        source_zone_active,
                        source_zone_age,
                        target_zone,
                        target_zone_active,
                        target_zone_age,
                        distance,
                        own_pose_fresh,
                        target_pose_fresh,
                        own_pose_age,
                        target_pose_age,
                        warning_zone,
                        bearing,
                        zone_reason,
                    )
                continue

            # Warn only if this robot sees a risk and the peer does not.
            if target_zone_active:
                self._log_decision(
                    now,
                    robot,
                    "skipped",
                    "target_zone_active",
                    source_zone,
                    source_zone_active,
                    source_zone_age,
                    target_zone,
                    target_zone_active,
                    target_zone_age,
                    distance,
                    own_pose_fresh,
                    target_pose_fresh,
                    own_pose_age,
                    target_pose_age,
                    warning_zone,
                    bearing,
                    zone_reason,
                )
                continue

            if nearest_self is None or distance < nearest_self[1]:
                nearest_self = (robot, distance)

            if self.publish_to_peer:
                warning_zone, bearing, zone_reason = self._publish_warning(
                    robot, self.ns, distance
                )
                self._log_decision(
                    now,
                    robot,
                    "sent",
                    "",
                    source_zone,
                    source_zone_active,
                    source_zone_age,
                    target_zone,
                    target_zone_active,
                    target_zone_age,
                    distance,
                    own_pose_fresh,
                    target_pose_fresh,
                    own_pose_age,
                    target_pose_age,
                    warning_zone,
                    bearing,
                    zone_reason,
                )
            else:
                self._log_decision(
                    now,
                    robot,
                    "skipped",
                    "publish_to_peer_disabled",
                    source_zone,
                    source_zone_active,
                    source_zone_age,
                    target_zone,
                    target_zone_active,
                    target_zone_age,
                    distance,
                    own_pose_fresh,
                    target_pose_fresh,
                    own_pose_age,
                    target_pose_age,
                    warning_zone,
                    bearing,
                    zone_reason,
                )

        if self.publish_to_self and nearest_self is not None:
            source, distance = nearest_self
            self._publish_warning(self.ns, source, distance)


def main(args=None):
    rclpy.init(args=args)
    node = RobotProximityWarner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
