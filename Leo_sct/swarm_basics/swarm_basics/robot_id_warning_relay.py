import csv
import math
import time
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from tf2_msgs.msg import TFMessage


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

    In global_pose mode, this node is simulation-only and uses Gazebo global
    dynamic poses to publish warnings before the image classifier exists.

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
            self.declare_parameter("warning_distance_m", 1.3).value
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
        self.global_pose_topic = str(
            self.declare_parameter(
                "global_pose_topic",
                "/world/random_world/dynamic_pose/info",
            ).value
        ).strip()
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
        self.peer_obstacle_warning_enabled = bool(
            self.declare_parameter("peer_obstacle_warning_enabled", True).value
        )
        self.peer_obstacle_warning_timeout_s = float(
            self.declare_parameter("peer_obstacle_warning_timeout_s", 0.6).value
        )
        self.peer_obstacle_warning_distance_m = float(
            self.declare_parameter("peer_obstacle_warning_distance_m", 0.85).value
        )
        self.results_dir = str(self.declare_parameter("results_dir", "").value).strip()
        self.run_id = str(self.declare_parameter("run_id", "").value).strip()
        self.detection_count = 0
        self.last_detection_time = 0.0
        self.poses = {}
        self.obstacle_zone = "CLEAR"
        self.last_obstacle_zone_time = 0.0
        self.front_obstacle_distance_m = float("inf")
        self.last_front_obstacle_distance_time = 0.0

        self.warn_publishers = {}
        for idx in range(self.total_robots):
            robot = f"robot_{idx}"
            self.warn_publishers[robot] = self.create_publisher(
                String,
                f"/{robot}/peer_warning_zone",
                10,
            )

        self.log_path = self._make_log_path()
        self.create_subscription(String, "detected_zones", self.obstacle_zone_callback, 10)
        self.create_subscription(
            Float32,
            "front_obstacle_distance",
            self.front_obstacle_distance_callback,
            10,
        )
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
            self.create_subscription(
                TFMessage,
                self.global_pose_topic,
                self.global_pose_callback,
                10,
            )
            self.timer = self.create_timer(
                self.publish_period_s,
                self.global_pose_timer_callback,
            )
        self.get_logger().info(
            "robot_id_warning_relay active "
            f"(robot={self.ns}, total_robots={self.total_robots}, "
            f"warning_distance_m={self.warning_distance_m:.2f}, "
            f"warning_source={self.warning_source}, "
            f"peer_obstacle_warning_enabled={self.peer_obstacle_warning_enabled}, "
            f"global_pose_topic={self.global_pose_topic}, "
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

    def global_pose_callback(self, msg: TFMessage):
        now = time.time()
        for transform in msg.transforms:
            robot = transform.child_frame_id.strip()
            if "/" in robot:
                continue
            if robot not in self.warn_publishers:
                continue

            self.poses[robot] = (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                _yaw_from_quat(transform.transform.rotation),
                now,
            )

    def obstacle_zone_callback(self, msg: String):
        self.obstacle_zone = self._parse_zone(msg.data)
        self.last_obstacle_zone_time = time.time()

    def front_obstacle_distance_callback(self, msg: Float32):
        distance = float(msg.data)
        self.front_obstacle_distance_m = (
            distance if math.isfinite(distance) else float("inf")
        )
        self.last_front_obstacle_distance_time = time.time()

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
        if token not in {"LEFT", "RIGHT", "CORNER", "BACK", "CLEAR"}:
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

        critical = distance <= self.critical_distance_m
        if abs(bearing) <= self.front_angle_rad:
            reason = "critical_front_sector" if critical else "front_sector"
            return "CORNER", bearing, reason
        if abs(bearing) >= (math.pi - self.front_angle_rad):
            reason = "critical_rear_sector" if critical else "rear_sector"
            return "BACK", bearing, reason
        reason = "critical_side_sector" if critical else "side_sector"
        return ("LEFT" if bearing > 0.0 else "RIGHT"), bearing, reason

    def _peer_obstacle_context_fields(self, peer_zone: str) -> List[str]:
        if not self.peer_obstacle_warning_enabled:
            return []

        now = time.time()
        zone_age = (
            now - self.last_obstacle_zone_time
            if self.last_obstacle_zone_time > 0.0
            else float("inf")
        )
        distance_age = (
            now - self.last_front_obstacle_distance_time
            if self.last_front_obstacle_distance_time > 0.0
            else float("inf")
        )
        obstacle_is_fresh = (
            self.obstacle_zone != "CLEAR"
            and zone_age <= self.peer_obstacle_warning_timeout_s
            and distance_age <= self.peer_obstacle_warning_timeout_s
            and math.isfinite(self.front_obstacle_distance_m)
            and self.front_obstacle_distance_m <= self.peer_obstacle_warning_distance_m
        )
        if not obstacle_is_fresh or self.obstacle_zone != peer_zone:
            return []

        return [
            "obstacle_behind=true",
            f"obstacle_zone={self.obstacle_zone}",
            f"obstacle_distance={self.front_obstacle_distance_m:.3f}",
            f"obstacle_zone_age_s={zone_age:.3f}",
            f"obstacle_distance_age_s={distance_age:.3f}",
        ]

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
        fields.extend(self._peer_obstacle_context_fields(zone))
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
        fields.extend(self._peer_obstacle_context_fields(zone))
        return ",".join(fields)


def main(args=None):
    rclpy.init(args=args)
    node = RobotIdWarningRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
