import csv
import math
import time
from pathlib import Path
from typing import Dict, List, Optional

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


class RobotIdWarningRelay(Node):
    """Relay robot-ID warnings into peer_warning_zone messages.

    In classifier mode, this node is distributed: it does not subscribe to
    global odometry or any other robot's pose. It expects the local
    perception/classifier to publish String messages on classified_robot_detections.

    In global_pose mode, this node is simulation-only and uses robot odometry to
    publish warnings before the image classifier exists.

    Accepted detection formats, separated by semicolons for multiple detections:
      robot_3,CORNER,distance=0.62,confidence=0.91
      target=robot_3,zone=LEFT,distance=0.80,confidence=0.75

    The relay sends the target robot a peer_warning_zone message using the same
    wire format consumed by robot_supervisor and bump_counter.
    """

    def __init__(self):
        super().__init__("robot_id_warning_relay")

        self.ns = self.get_namespace().strip("/") or "root"
        self.total_robots = int(self.declare_parameter("total_robots", 5).value)
        self.warning_distance_m = float(
            self.declare_parameter("warning_distance_m", 1.0).value
        )
        self.min_confidence = float(self.declare_parameter("min_confidence", 0.0).value)
        self.warning_source = str(
            self.declare_parameter("warning_source", "classifier").value
        ).strip().lower()
        if self.warning_source not in {"classifier", "global_pose"}:
            raise ValueError(
                f"Unknown warning_source '{self.warning_source}'. "
                "Use 'classifier' or 'global_pose'."
            )
        self.critical_distance_m = float(
            self.declare_parameter("critical_distance_m", 0.55).value
        )
        self.front_angle_rad = float(
            self.declare_parameter("front_angle_rad", math.radians(50.0)).value
        )
        self.pose_timeout_s = float(self.declare_parameter("pose_timeout_s", 0.6).value)
        self.publish_period_s = float(
            self.declare_parameter("publish_period_s", 0.1).value
        )
        self.log_enabled = bool(
            self.declare_parameter("robot_id_warning_log_enabled", True).value
        )
        self.detections_topic = str(
            self.declare_parameter(
                "classified_robot_detections_topic",
                "classified_robot_detections",
            ).value
        ).strip()
        self.no_detection_warn_period_s = float(
            self.declare_parameter("no_detection_warn_period_s", 10.0).value
        )
        self.results_dir = str(self.declare_parameter("results_dir", "").value).strip()
        self.run_id = str(self.declare_parameter("run_id", "").value).strip()
        self.detection_count = 0
        self.last_detection_time = 0.0
        self.poses = {}

        self.warn_publishers = {}
        for idx in range(self.total_robots):
            robot = f"robot_{idx}"
            self.warn_publishers[robot] = self.create_publisher(
                String,
                f"/{robot}/peer_warning_zone",
                10,
            )
            if self.warning_source == "global_pose":
                self.create_subscription(
                    Odometry,
                    f"/{robot}/odom",
                    self._make_odom_callback(robot),
                    10,
                )

        self.log_path = self._make_log_path()
        if self.warning_source == "classifier":
            self.create_subscription(
                String,
                self.detections_topic,
                self.detections_callback,
                10,
            )
            self.diagnostic_timer = self.create_timer(
                self.no_detection_warn_period_s,
                self.diagnostic_callback,
            )
        else:
            self.timer = self.create_timer(
                self.publish_period_s,
                self.global_pose_timer_callback,
            )
        self.get_logger().info(
            "robot_id_warning_relay active "
            f"(robot={self.ns}, total_robots={self.total_robots}, "
            f"warning_distance_m={self.warning_distance_m:.2f}, "
            f"warning_source={self.warning_source}, "
            f"detections_topic={self.detections_topic})"
        )

    def _make_log_path(self) -> Optional[Path]:
        if not self.log_enabled or not self.results_dir or not self.run_id:
            return None

        log_dir = Path(self.results_dir).expanduser() / self.run_id
        log_dir.mkdir(parents=True, exist_ok=True)
        path = log_dir / f"robot_id_warnings_{self.ns}.csv"
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
                        "warning_zone",
                        "distance_m",
                        "confidence",
                        "raw_detection",
                        "raw_message",
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

    def _log(
        self,
        target: str,
        decision: str,
        skip_reason: str,
        zone: str,
        distance,
        confidence,
        raw_detection: str,
        raw_message: str,
    ):
        if self.log_path is None:
            return

        with self.log_path.open("a", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    f"{time.time():.6f}",
                    self.ns,
                    target,
                    decision,
                    skip_reason,
                    zone,
                    self._fmt(distance),
                    self._fmt(confidence),
                    raw_detection,
                    raw_message,
                ]
            )

    def _fmt(self, value):
        if value is None or value == "":
            return ""
        if isinstance(value, float):
            return f"{value:.6f}" if math.isfinite(value) else "inf"
        return str(value)

    def detections_callback(self, msg: String):
        self.detection_count += 1
        self.last_detection_time = time.time()
        for raw_detection in self._split_detections(msg.data):
            detection = self._parse_detection(raw_detection)
            target = detection.get("target", "")
            zone = detection.get("zone", "CLEAR")
            distance = detection.get("distance")
            confidence = detection.get("confidence")

            skip_reason = self._skip_reason(target, zone, distance, confidence)
            if skip_reason:
                self._log(
                    target,
                    "skipped",
                    skip_reason,
                    zone,
                    distance,
                    confidence,
                    raw_detection,
                    msg.data,
                )
                continue

            warning = String()
            warning.data = self._warning_message(target, zone, distance, confidence)
            self.warn_publishers[target].publish(warning)
            self._log(
                target,
                "sent",
                "",
                zone,
                distance,
                confidence,
                raw_detection,
                msg.data,
            )

    def diagnostic_callback(self):
        if self.detection_count > 0:
            age = time.time() - self.last_detection_time
            self.get_logger().info(
                "robot ID detections received "
                f"(count={self.detection_count}, last_age_s={age:.2f}, "
                f"topic={self.detections_topic})"
            )
            return

        self.get_logger().warn(
            "No robot ID detections received yet. Expected std_msgs/String on "
            f"topic '{self.detections_topic}' in namespace '{self.ns}'."
        )

    def global_pose_timer_callback(self):
        now = time.time()
        source_pose = self.poses.get(self.ns)
        if source_pose is None:
            return

        source_age = now - source_pose[3]
        if source_age > self.pose_timeout_s:
            return

        sx, sy, _, _ = source_pose
        for target, target_pose in self.poses.items():
            if target == self.ns:
                continue

            target_age = now - target_pose[3]
            if target_age > self.pose_timeout_s:
                self._log(
                    target,
                    "skipped",
                    "target_pose_stale",
                    "",
                    "",
                    "",
                    "global_pose",
                    f"source_age={source_age:.3f},target_age={target_age:.3f}",
                )
                continue

            tx, ty, _, _ = target_pose
            distance = math.hypot(tx - sx, ty - sy)
            zone, bearing, zone_reason = self._zone_from_target_to_source(
                target,
                self.ns,
                distance,
            )

            if distance > self.warning_distance_m:
                if distance <= self.warning_distance_m + 0.25:
                    self._log(
                        target,
                        "skipped",
                        "distance_above_warning_threshold",
                        zone,
                        distance,
                        "",
                        "global_pose",
                        (
                            f"bearing_deg={math.degrees(bearing):.3f},"
                            f"zone_reason={zone_reason}"
                        ),
                    )
                continue

            warning = String()
            warning.data = self._global_pose_warning_message(
                target,
                self.ns,
                zone,
                distance,
                bearing,
                zone_reason,
            )
            self.warn_publishers[target].publish(warning)
            self._log(
                target,
                "sent",
                "",
                zone,
                distance,
                "",
                "global_pose",
                (
                    f"bearing_deg={math.degrees(bearing):.3f},"
                    f"zone_reason={zone_reason}"
                ),
            )

    def _split_detections(self, data: str) -> List[str]:
        return [part.strip() for part in data.replace("\n", ";").split(";") if part.strip()]

    def _parse_detection(self, raw: str) -> Dict[str, object]:
        fields: Dict[str, object] = {}
        positional = []
        for part in raw.split(","):
            token = part.strip()
            if not token:
                continue
            if "=" in token:
                key, value = token.split("=", 1)
                key = key.strip().lower()
                value = value.strip()
                fields[key] = value
            else:
                positional.append(token)

        if "target" not in fields and positional:
            fields["target"] = positional[0]
        if "zone" not in fields and len(positional) >= 2:
            fields["zone"] = positional[1]

        target = str(fields.get("target", "")).strip()
        if target.isdigit():
            target = f"robot_{target}"
        fields["target"] = target
        fields["zone"] = self._parse_zone(str(fields.get("zone", "")))

        for key in ("distance", "confidence"):
            if key not in fields:
                continue
            try:
                fields[key] = float(fields[key])
            except (TypeError, ValueError):
                fields[key] = None
        return fields

    def _parse_zone(self, value: str) -> str:
        token = value.strip().upper()
        if token not in {"LEFT", "RIGHT", "CORNER", "CLEAR"}:
            return "CLEAR"
        return token

    def _skip_reason(self, target: str, zone: str, distance, confidence) -> str:
        if not target:
            return "missing_target"
        if target == self.ns:
            return "self_detection"
        if target not in self.warn_publishers:
            return "unknown_target"
        if zone == "CLEAR":
            return "clear_or_invalid_zone"
        if confidence is not None and confidence < self.min_confidence:
            return "low_confidence"
        if distance is not None and distance > self.warning_distance_m:
            return "distance_above_warning_threshold"
        return ""

    def _zone_from_target_to_source(self, target: str, source: str, distance: float):
        tx, ty, tyaw, _ = self.poses[target]
        sx, sy, _, _ = self.poses[source]
        bearing = _wrap_to_pi(math.atan2(sy - ty, sx - tx) - tyaw)

        if distance <= self.critical_distance_m:
            return "CORNER", bearing, "critical_distance"
        if abs(bearing) <= self.front_angle_rad:
            return "CORNER", bearing, "front_sector"
        return ("LEFT" if bearing > 0.0 else "RIGHT"), bearing, "side_sector"

    def _warning_message(self, target: str, zone: str, distance, confidence) -> str:
        fields = [
            zone,
            f"source={self.ns}",
            f"source_zone={zone}",
            "target_zone=unknown",
            "zone_reason=robot_id_classifier",
        ]
        if distance is not None:
            fields.append(f"distance={distance:.3f}")
        if confidence is not None:
            fields.append(f"confidence={confidence:.3f}")
        fields.append(f"target={target}")
        return ",".join(fields)

    def _global_pose_warning_message(
        self,
        target: str,
        source: str,
        zone: str,
        distance: float,
        bearing: float,
        zone_reason: str,
    ) -> str:
        sx, sy, syaw, _ = self.poses[source]
        tx, ty, tyaw, _ = self.poses[target]
        fields = [
            zone,
            f"source={source}",
            f"source_zone=global_pose",
            "target_zone=unknown",
            "zone_reason=global_pose",
            f"distance={distance:.3f}",
            f"bearing_rad={bearing:.6f}",
            f"bearing_deg={math.degrees(bearing):.3f}",
            f"bearing_zone_reason={zone_reason}",
            f"target={target}",
            f"source_x={sx:.6f}",
            f"source_y={sy:.6f}",
            f"source_yaw={syaw:.6f}",
            f"target_x={tx:.6f}",
            f"target_y={ty:.6f}",
            f"target_yaw={tyaw:.6f}",
        ]
        return ",".join(fields)


def main(args=None):
    rclpy.init(args=args)
    node = RobotIdWarningRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
