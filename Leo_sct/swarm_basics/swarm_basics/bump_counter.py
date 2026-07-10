#!/usr/bin/env python3
"""
bump_counter.py

Counts Gazebo/Ignition contact events (ros_gz_interfaces/msg/Contacts).

Two modes:
  1) global_mode:=True  (RECOMMENDED)
     - Run ONE node (no namespace needed).
     - Auto-subscribes to all contact topics matching a filter (default: topics ending with "/contact").
     - Counts UNIQUE PAIRS directly:
         robot-robot contact => +1 per robot pair (robot_i, robot_j)
         robot-obstacle      => +1 per (robot_i, obstacle_entity)
     - Debounces repeated contact for the same key using cooldown.

  2) global_mode:=False
     - Run inside a robot namespace and subscribe to a single CONTACT_TOPIC (default "contact").
     - Counts per-robot bump events against "other" entity names (good for per-robot stats),
       but robot-robot will naturally appear twice if you run one node per robot.

Publishes:
  bump_count           (UInt32) : total (robot-robot-pairs + robot-obstacle)
  bump_count_robot     (UInt32) : robot-robot PAIR events (global_mode) or robot-vs-robot events (per-robot mode)
  bump_count_obstacle  (UInt32) : robot-obstacle events
  last_bump_with       (String) : last other entity name (or "robot_i<->robot_j" in global mode)
  last_bump_type       (String) : "robot" or "obstacle"

Logs CSV to ~/ros_bump_logs/bumps_<label>_<YYYY-MM-DD>.csv
"""

import csv
import math
import re
import time
from collections import deque
from pathlib import Path
from typing import Dict, Tuple, Any, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import UInt32, String
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import Image

# ------------------ Defaults ------------------
DEFAULT_COOLDOWN_SEC = 0.5          # generic cooldown
DEFAULT_COOLDOWN_ROBOT_SEC = 0.8    # optional: slightly longer for robot-robot
DEFAULT_COOLDOWN_OBS_SEC = 0.5      # optional: for robot-obstacle
PRUNE_HZ = 10.0                     # prune inactive contacts at this rate
CONTACT_TOPIC = "contact"           # used only in per-robot mode
DEFAULT_LOG_DIR = Path.home() / "ros_bump_logs"
# ---------------------------------------------


def entity_name(ent) -> str:
    """Contacts.collision{1,2} are Entity objects with .name."""
    try:
        return ent.name
    except AttributeError:
        return str(ent)


def mean_xyz(pts):
    if not pts:
        return (math.nan, math.nan, math.nan)
    sx = sum(getattr(p, "x", 0.0) for p in pts)
    sy = sum(getattr(p, "y", 0.0) for p in pts)
    sz = sum(getattr(p, "z", 0.0) for p in pts)
    n = float(len(pts))
    return (sx / n, sy / n, sz / n)


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


class BumpCounter(Node):
    def __init__(self):
        super().__init__("bump_counter")

        # ---- Params ----
        self.global_mode = bool(self.declare_parameter("global_mode", True).value)

        # Topic discovery filter (global mode)
        self.contact_topic_substring = str(
            self.declare_parameter("contact_topic_substring", "/contact").value
        )
        # If you want to only subscribe to robot namespaces:
        # e.g. r"(?:^|/)robot_\d+/" to require "/robot_i/" in topic
        self.topic_regex = str(self.declare_parameter("topic_regex", "").value)

        # Cooldowns (debounce per key)
        cooldown_sec = float(self.declare_parameter("cooldown_sec", DEFAULT_COOLDOWN_SEC).value)
        cooldown_robot_sec = float(
            self.declare_parameter("cooldown_robot_sec", DEFAULT_COOLDOWN_ROBOT_SEC).value
        )
        cooldown_obstacle_sec = float(
            self.declare_parameter("cooldown_obstacle_sec", DEFAULT_COOLDOWN_OBS_SEC).value
        )
        self.cooldown_default = Duration(seconds=max(0.0, cooldown_sec))
        self.cooldown_robot = Duration(seconds=max(0.0, cooldown_robot_sec))
        self.cooldown_obstacle = Duration(seconds=max(0.0, cooldown_obstacle_sec))

        self.valid_robot_regex = str(self.declare_parameter("robot_name_regex", r"(robot_\d+)").value)
        self._robot_re = re.compile(self.valid_robot_regex)

        self.flush_interval_sec = float(self.declare_parameter("flush_interval_sec", 1.0).value)
        self.flush_max_rows = int(self.declare_parameter("flush_max_rows", 200).value)
        self.total_robots = int(self.declare_parameter("total_robots", 5).value)
        self.detection_context_timeout_s = float(
            self.declare_parameter("detection_context_timeout_s", 1.0).value
        )
        self.save_depth_pre_collision_images = bool(
            self.declare_parameter("save_depth_pre_collision_images", True).value
        )
        self.depth_history_frames = max(
            1,
            int(self.declare_parameter("depth_history_frames", 5).value),
        )
        self.depth_topic_template = str(
            self.declare_parameter(
                "depth_topic_template",
                "/{robot}/depth_camera/depth_image",
            ).value
        )

        # ---- Identity label for CSV/logging ----
        ns = self.get_namespace().strip("/") or "root"
        self.label = "global" if self.global_mode else ns

        # ---- CSV output dir ----
        results_dir = str(self.declare_parameter("results_dir", "").value).strip()
        run_id = str(self.declare_parameter("run_id", "").value).strip()
        if results_dir:
            base_dir = Path(results_dir)
            self.log_dir = base_dir / run_id if run_id else base_dir
        else:
            self.log_dir = DEFAULT_LOG_DIR

        # ---- State ----
        # active: key -> (last_seen_time, bump_type)
        self.active: Dict[Any, Tuple[rclpy.time.Time, str]] = {}

        self.bump_total = 0
        self.bump_robot = 0       # global_mode: robot-robot PAIR events
        self.bump_obstacle = 0
        self.robot_zones: Dict[str, Tuple[str, float]] = {}
        self.peer_warnings: Dict[Tuple[str, str], Tuple[float, Dict[str, str], str]] = {}
        self.robot_poses: Dict[str, Tuple[float, float, float, float]] = {}
        self.depth_buffers = {
            f"robot_{idx}": deque(maxlen=self.depth_history_frames)
            for idx in range(self.total_robots)
        }
        self.depth_snapshot_dir = self.log_dir / "collision_depth_frames"
        self.depth_snapshot_index_path = self.depth_snapshot_dir / "frames.csv"

        # ---- Publishers ----
        self.pub_total = self.create_publisher(UInt32, "bump_count", 10)
        self.pub_robot = self.create_publisher(UInt32, "bump_count_robot", 10)
        self.pub_obs = self.create_publisher(UInt32, "bump_count_obstacle", 10)
        self.pub_last = self.create_publisher(String, "last_bump_with", 10)
        self.pub_last_type = self.create_publisher(String, "last_bump_type", 10)
        self.pub_last_robot = self.create_publisher(String, "last_bump_robot", 10)

        # ---- Subscriptions ----
        self._subs = {}
        if self.global_mode:
            self._topic_scan_timer = self.create_timer(1.0, self._refresh_contact_subs)
        else:
            self.sub = self.create_subscription(Contacts, CONTACT_TOPIC, self._on_contacts, 10)

        for idx in range(self.total_robots):
            robot = f"robot_{idx}"
            self.create_subscription(
                String,
                f"/{robot}/detected_zones",
                self._make_zone_callback(robot),
                10,
            )
            self.create_subscription(
                String,
                f"/{robot}/peer_warning_zone",
                self._make_peer_warning_callback(robot),
                10,
            )
            self.create_subscription(
                Odometry,
                f"/{robot}/odom",
                self._make_odom_callback(robot),
                10,
            )
            if self.save_depth_pre_collision_images:
                self.create_subscription(
                    Image,
                    self.depth_topic_template.format(robot=robot),
                    self._make_depth_callback(robot),
                    10,
                )

        # prune timer
        self.create_timer(1.0 / PRUNE_HZ, self._prune)

        # ---- CSV logging ----
        self.log_dir.mkdir(parents=True, exist_ok=True)
        if self.save_depth_pre_collision_images:
            self.depth_snapshot_dir.mkdir(parents=True, exist_ok=True)
            self._ensure_depth_snapshot_index_header()
        self.csv_path = self._csv_path_for_today()
        self._ensure_csv_header()
        self._csv_rows = []
        self._csv_lock = False

        self._flush_timer = self.create_timer(self.flush_interval_sec, self._flush_csv_rows)

        self.get_logger().info(
            f"[{self.label}] global_mode={self.global_mode} "
            f"cooldown(robot)={cooldown_robot_sec:.2f}s cooldown(obs)={cooldown_obstacle_sec:.2f}s "
            f"Logging to {self.csv_path}"
        )

    # ---------------- CSV helpers ----------------
    def _csv_path_for_today(self) -> Path:
        timestr = time.strftime("%H%M%S")
        return self.log_dir / f"bumps_{self.label}_{timestr}.csv"

    def _ensure_csv_header(self):
        if not self.csv_path.exists():
            with self.csv_path.open("w", newline="") as f:
                w = csv.writer(f)
                w.writerow([
                    "stamp_sec", "stamp_nsec",
                    "label",
                    "total_index", "robot_index", "obstacle_index",
                    "bump_type",
                    "key",
                    "entity_a", "entity_b",
                    "avg_contact_x", "avg_contact_y", "avg_contact_z",
                    "source_topic",
                    "entity_a_bump_direction",
                    "entity_b_bump_direction",
                    "pair_detection_summary",
                    "depth_snapshot_dir",
                ])

    def _append_csv(self, stamp, bump_type: str, key: Any,
                    entity_a: str, entity_b: str, avg_xyz, source_topic: str):
        context = self._collision_detection_context(entity_a, entity_b, time.time())
        direction_a, direction_b = self._collision_direction_context(entity_a, entity_b, avg_xyz)
        depth_snapshot_dir = self._save_pre_collision_depth_frames(
            stamp,
            bump_type,
            key,
            entity_a,
            entity_b,
        )
        self._csv_rows.append([
            int(stamp.sec), int(stamp.nanosec),
            self.label,
            int(self.bump_total), int(self.bump_robot), int(self.bump_obstacle),
            bump_type,
            str(key),
            entity_a, entity_b,
            f"{avg_xyz[0]:.6f}" if isinstance(avg_xyz[0], float) else "",
            f"{avg_xyz[1]:.6f}" if isinstance(avg_xyz[1], float) else "",
            f"{avg_xyz[2]:.6f}" if isinstance(avg_xyz[2], float) else "",
            source_topic,
            direction_a,
            direction_b,
            context["pair_detection_summary"],
            depth_snapshot_dir,
        ])
        if len(self._csv_rows) >= self.flush_max_rows:
            self._flush_csv_rows()

    def _flush_csv_rows(self):
        if self._csv_lock or not self._csv_rows:
            return
        self._csv_lock = True
        try:
            with self.csv_path.open("a", newline="") as f:
                w = csv.writer(f)
                w.writerows(self._csv_rows)
            self._csv_rows.clear()
        finally:
            self._csv_lock = False

    def destroy_node(self):
        try:
            self._flush_csv_rows()
        except Exception:
            pass
        super().destroy_node()
    # --------------------------------------------

    # ---------------- Pre-collision depth snapshots ----------------
    def _ensure_depth_snapshot_index_header(self):
        if self.depth_snapshot_index_path.exists():
            return
        with self.depth_snapshot_index_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "bump_total",
                "stamp_sec",
                "stamp_nsec",
                "bump_type",
                "key",
                "entity_a",
                "entity_b",
                "robot",
                "frame_index_oldest_first",
                "frame_wall_time",
                "frame_stamp_sec",
                "frame_stamp_nsec",
                "encoding",
                "height",
                "width",
                "npy_path",
                "pgm_path",
            ])

    def _make_depth_callback(self, robot: str):
        def callback(msg: Image):
            frame = self._decode_depth_image(msg)
            if frame is None:
                return
            self.depth_buffers[robot].append(frame)

        return callback

    def _dtype_for_image_encoding(self, encoding: str):
        key = str(encoding or "").upper()
        mapping = {
            "32FC1": np.float32,
            "16UC1": np.uint16,
            "16SC1": np.int16,
            "8UC1": np.uint8,
            "8SC1": np.int8,
            "MONO8": np.uint8,
            "MONO16": np.uint16,
        }
        return mapping.get(key)

    def _decode_depth_image(self, msg: Image):
        dtype = self._dtype_for_image_encoding(msg.encoding)
        if dtype is None:
            if not hasattr(self, "_warned_depth_encodings"):
                self._warned_depth_encodings = set()
            if msg.encoding not in self._warned_depth_encodings:
                self.get_logger().warn(
                    f"Depth snapshot disabled for unsupported encoding '{msg.encoding}'."
                )
                self._warned_depth_encodings.add(msg.encoding)
            return None

        try:
            itemsize = np.dtype(dtype).itemsize
            row_stride = int(msg.step) // itemsize
            raw = np.frombuffer(msg.data, dtype=dtype)
            if row_stride <= 0 or raw.size < int(msg.height) * row_stride:
                return None
            image = raw.reshape((int(msg.height), row_stride))[:, : int(msg.width)].copy()
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(f"Could not decode depth image: {exc}")
            return None

        return {
            "wall_time": time.time(),
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nsec": int(msg.header.stamp.nanosec),
            "encoding": str(msg.encoding),
            "height": int(msg.height),
            "width": int(msg.width),
            "image": image,
        }

    def _write_depth_preview_pgm(self, path: Path, image: np.ndarray):
        arr = image.astype(np.float32, copy=False)
        finite = np.isfinite(arr)
        if not np.any(finite):
            preview = np.zeros(arr.shape, dtype=np.uint8)
        else:
            values = arr[finite]
            lo = float(np.percentile(values, 2.0))
            hi = float(np.percentile(values, 98.0))
            if hi <= lo:
                hi = lo + 1.0
            normalized = (arr - lo) / (hi - lo)
            normalized = np.where(finite, normalized, 0.0)
            preview = np.clip(normalized * 255.0, 0.0, 255.0).astype(np.uint8)

        with path.open("wb") as f:
            f.write(f"P5\n{preview.shape[1]} {preview.shape[0]}\n255\n".encode("ascii"))
            f.write(preview.tobytes())

    def _collision_robots(self, entity_a: str, entity_b: str):
        robots = []
        for entity in (entity_a, entity_b):
            robot = self._robot_from_entity_name(entity)
            if robot and robot not in robots:
                robots.append(robot)
        return robots

    def _safe_name(self, value: Any) -> str:
        return re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("_")[:80] or "item"

    def _save_pre_collision_depth_frames(
        self,
        stamp,
        bump_type: str,
        key: Any,
        entity_a: str,
        entity_b: str,
    ) -> str:
        if not self.save_depth_pre_collision_images:
            return ""

        robots = self._collision_robots(entity_a, entity_b)
        if not robots:
            return ""

        bump_dir = (
            self.depth_snapshot_dir
            / f"bump_{int(self.bump_total):04d}_{self._safe_name(key)}"
        )
        wrote_any = False
        index_rows = []
        for robot in robots:
            frames = list(self.depth_buffers.get(robot, []))[-self.depth_history_frames :]
            robot_dir = bump_dir / robot
            for frame_index, frame in enumerate(frames):
                robot_dir.mkdir(parents=True, exist_ok=True)
                prefix = f"frame_{frame_index:02d}_{frame['stamp_sec']}_{frame['stamp_nsec']}"
                npy_path = robot_dir / f"{prefix}.npy"
                pgm_path = robot_dir / f"{prefix}.pgm"
                np.save(npy_path, frame["image"])
                self._write_depth_preview_pgm(pgm_path, frame["image"])
                wrote_any = True
                index_rows.append([
                    int(self.bump_total),
                    int(stamp.sec),
                    int(stamp.nanosec),
                    bump_type,
                    str(key),
                    entity_a,
                    entity_b,
                    robot,
                    frame_index,
                    f"{frame['wall_time']:.6f}",
                    frame["stamp_sec"],
                    frame["stamp_nsec"],
                    frame["encoding"],
                    frame["height"],
                    frame["width"],
                    str(npy_path.relative_to(self.log_dir)),
                    str(pgm_path.relative_to(self.log_dir)),
                ])

        if index_rows:
            with self.depth_snapshot_index_path.open("a", newline="") as f:
                writer = csv.writer(f)
                writer.writerows(index_rows)

        return str(bump_dir.relative_to(self.log_dir)) if wrote_any else ""
    # ------------------------------------------------------------

    # ---------------- Detection / warning context ----------------
    def _parse_zone_token(self, data: str) -> str:
        token = str(data or "").strip().upper()
        if ":" in token:
            token = token.split(":", 1)[0].strip()
        if "," in token:
            token = token.split(",", 1)[0].strip()
        return token if token in {"LEFT", "RIGHT", "CORNER", "BACK", "CLEAR"} else "CLEAR"

    def _parse_key_values(self, data: str) -> Dict[str, str]:
        fields: Dict[str, str] = {}
        for part in str(data or "").split(","):
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            fields[key.strip().lower()] = value.strip()
        return fields

    def _make_zone_callback(self, robot: str):
        def callback(msg: String):
            self.robot_zones[robot] = (self._parse_zone_token(msg.data), time.time())

        return callback

    def _make_peer_warning_callback(self, receiver: str):
        def callback(msg: String):
            zone = self._parse_zone_token(msg.data)
            fields = self._parse_key_values(msg.data)
            source = fields.get("source", "")
            if not source:
                return
            self.peer_warnings[(receiver, source)] = (time.time(), fields, zone)

        return callback

    def _make_odom_callback(self, robot: str):
        def callback(msg: Odometry):
            self.robot_poses[robot] = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                _yaw_from_quat(msg.pose.pose.orientation),
                time.time(),
            )

        return callback

    def _direction_from_contact(self, robot: str, avg_xyz) -> str:
        if not robot or robot not in self.robot_poses:
            return ""
        cx, cy, _ = avg_xyz
        if not (math.isfinite(cx) and math.isfinite(cy)):
            return ""

        x, y, yaw, _ = self.robot_poses[robot]
        bearing = _wrap_to_pi(math.atan2(cy - y, cx - x) - yaw)
        abs_bearing = abs(bearing)
        if abs_bearing <= math.radians(45.0):
            return "front"
        if abs_bearing >= math.radians(135.0):
            return "back"
        return "left" if bearing > 0.0 else "right"

    def _collision_direction_context(self, entity_a: str, entity_b: str, avg_xyz) -> Tuple[str, str]:
        robot_a = self._robot_from_entity_name(entity_a)
        robot_b = self._robot_from_entity_name(entity_b)
        return (
            self._direction_from_contact(robot_a, avg_xyz),
            self._direction_from_contact(robot_b, avg_xyz),
        )

    def _robot_zone_context(self, robot: str, now_wall: float) -> Tuple[str, str, str]:
        zone, stamp = self.robot_zones.get(robot, ("CLEAR", 0.0))
        age = now_wall - stamp if stamp > 0.0 else float("inf")
        active = zone != "CLEAR" and age <= self.detection_context_timeout_s
        age_text = f"{age:.6f}" if math.isfinite(age) else "inf"
        return zone, age_text, str(active).lower()

    def _warning_context(self, receiver: str, source: str, now_wall: float) -> Dict[str, str]:
        stamp, fields, zone = self.peer_warnings.get((receiver, source), (0.0, {}, "CLEAR"))
        age = now_wall - stamp if stamp > 0.0 else float("inf")
        active = stamp > 0.0 and age <= self.detection_context_timeout_s
        return {
            "active": str(active).lower(),
            "age_s": f"{age:.6f}" if math.isfinite(age) else "inf",
            "zone": zone,
            "source_zone": fields.get("source_zone", ""),
            "target_zone": fields.get("target_zone", ""),
            "distance_m": fields.get("distance", ""),
        }

    def _collision_detection_context(self, entity_a: str, entity_b: str, now_wall: float) -> Dict[str, str]:
        context = {
            "entity_a_zone": "",
            "entity_a_zone_age_s": "",
            "entity_a_zone_active": "",
            "entity_b_zone": "",
            "entity_b_zone_age_s": "",
            "entity_b_zone_active": "",
            "entity_a_warning_from_b_active": "",
            "entity_a_warning_from_b_age_s": "",
            "entity_a_warning_from_b_zone": "",
            "entity_a_warning_from_b_source_zone": "",
            "entity_a_warning_from_b_target_zone": "",
            "entity_a_warning_from_b_distance_m": "",
            "entity_b_warning_from_a_active": "",
            "entity_b_warning_from_a_age_s": "",
            "entity_b_warning_from_a_zone": "",
            "entity_b_warning_from_a_source_zone": "",
            "entity_b_warning_from_a_target_zone": "",
            "entity_b_warning_from_a_distance_m": "",
            "pair_warning_active": "false",
            "pair_detection_summary": "not_robot_pair",
        }

        robot_a = self._robot_from_entity_name(entity_a)
        robot_b = self._robot_from_entity_name(entity_b)
        if not robot_a:
            return context

        a_zone, a_age, a_active = self._robot_zone_context(robot_a, now_wall)
        context["entity_a_zone"] = a_zone
        context["entity_a_zone_age_s"] = a_age
        context["entity_a_zone_active"] = a_active

        if not robot_b:
            context["pair_detection_summary"] = (
                f"robot_obstacle_detecting_{a_zone}"
                if a_active == "true"
                else "robot_obstacle_not_detecting"
            )
            return context

        b_zone, b_age, b_active = self._robot_zone_context(robot_b, now_wall)
        context["entity_b_zone"] = b_zone
        context["entity_b_zone_age_s"] = b_age
        context["entity_b_zone_active"] = b_active

        a_from_b = self._warning_context(robot_a, robot_b, now_wall)
        b_from_a = self._warning_context(robot_b, robot_a, now_wall)

        context["entity_a_warning_from_b_active"] = a_from_b["active"]
        context["entity_a_warning_from_b_age_s"] = a_from_b["age_s"]
        context["entity_a_warning_from_b_zone"] = a_from_b["zone"]
        context["entity_a_warning_from_b_source_zone"] = a_from_b["source_zone"]
        context["entity_a_warning_from_b_target_zone"] = a_from_b["target_zone"]
        context["entity_a_warning_from_b_distance_m"] = a_from_b["distance_m"]
        context["entity_b_warning_from_a_active"] = b_from_a["active"]
        context["entity_b_warning_from_a_age_s"] = b_from_a["age_s"]
        context["entity_b_warning_from_a_zone"] = b_from_a["zone"]
        context["entity_b_warning_from_a_source_zone"] = b_from_a["source_zone"]
        context["entity_b_warning_from_a_target_zone"] = b_from_a["target_zone"]
        context["entity_b_warning_from_a_distance_m"] = b_from_a["distance_m"]

        a_warned_by_b = a_from_b["active"] == "true"
        b_warned_by_a = b_from_a["active"] == "true"
        # Pair-specific detection: A is treated as detecting B only when A sent
        # a fresh warning to B. A local non-CLEAR zone alone may be another robot
        # or an obstacle, so it is not enough for robot-robot collision reasons.
        a_detecting = b_warned_by_a
        b_detecting = a_warned_by_b
        pair_warning_active = a_warned_by_b or b_warned_by_a
        context["pair_warning_active"] = str(pair_warning_active).lower()

        if a_detecting and b_detecting:
            summary = "both_detecting"
        elif a_detecting and not b_detecting:
            summary = "entity_a_detecting_only"
        elif b_detecting and not a_detecting:
            summary = "entity_b_detecting_only"
        else:
            summary = "neither_detecting"

        if a_warned_by_b and b_warned_by_a:
            summary += "|both_warned"
        elif a_warned_by_b:
            summary += "|entity_a_warned_by_b"
        elif b_warned_by_a:
            summary += "|entity_b_warned_by_a"
        else:
            summary += "|no_pair_warning"
        context["pair_detection_summary"] = summary
        return context
    # ------------------------------------------------------------

    # ---------------- Topic discovery (global) ----------------
    def _refresh_contact_subs(self):
        topics = self.get_topic_names_and_types()
        for topic, types in topics:
            # type check (robust to different string forms)
            if not any(("ros_gz_interfaces/msg/Contacts" in t) or t.endswith("/Contacts") or t.endswith("Contacts")
                       for t in types):
                continue

            if self.contact_topic_substring and (self.contact_topic_substring not in topic):
                continue

            if self.topic_regex:
                try:
                    if re.search(self.topic_regex, topic) is None:
                        continue
                except re.error as e:
                    self.get_logger().error(f"Invalid topic_regex '{self.topic_regex}': {e}")
                    # disable regex filtering to avoid spamming
                    self.topic_regex = ""
                    continue

            if topic in self._subs:
                continue

            self._subs[topic] = self.create_subscription(
                Contacts,
                topic,
                lambda msg, source=topic: self._on_contacts(msg, source_topic=source),
                10,
            )
            self.get_logger().info(f"[{self.label}] subscribed: {topic}")
    # ---------------------------------------------------------

    # ---------------- Classification helpers ----------------
    def _robot_from_entity_name(self, name: str) -> str:
        m = self._robot_re.search(name)
        return m.group(1) if m else ""

    def _classify_bump(self, entity_a: str, entity_b: str) -> Tuple[str, Optional[str], Optional[str]]:
        """
        Returns:
          bump_type: "robot" or "obstacle"
          robot_a, robot_b/other
        """
        ra = self._robot_from_entity_name(entity_a)
        rb = self._robot_from_entity_name(entity_b)

        if ra and rb:
            return "robot", ra, rb

        if ra or rb:
            robot = ra or rb
            other = entity_b if ra else entity_a
            return "obstacle", robot, other

        return "unknown", None, None

    def _cooldown_for_type(self, bump_type: str) -> Duration:
        if bump_type == "robot":
            return self.cooldown_robot
        if bump_type == "obstacle":
            return self.cooldown_obstacle
        return self.cooldown_default
    # --------------------------------------------------------

    def _publish_counts(self, last_with: str, bump_type: str, last_robot: str):
        self.pub_total.publish(UInt32(data=int(self.bump_total)))
        self.pub_robot.publish(UInt32(data=int(self.bump_robot)))
        self.pub_obs.publish(UInt32(data=int(self.bump_obstacle)))
        self.pub_last.publish(String(data=last_with))
        self.pub_last_type.publish(String(data=bump_type))
        self.pub_last_robot.publish(String(data=last_robot))

    def _on_contacts(self, msg: Contacts, source_topic: str = ""):
        now = self.get_clock().now()

        for c in msg.contacts:
            col1 = entity_name(c.collision1)
            col2 = entity_name(c.collision2)

            if self.global_mode:
                bump_type, r1, r2_or_other = self._classify_bump(col1, col2)
                if bump_type == "unknown":
                    continue

                if bump_type == "robot":
                    # key is robot pair (sorted) => counts ONCE per pair
                    r2 = r2_or_other
                    key = tuple(sorted([r1, r2]))
                    entity_a, entity_b = key[0], key[1]
                    last_with = f"{entity_a}<->{entity_b}"
                    last_robot = f"{entity_a},{entity_b}"
                else:
                    # key is (robot, obstacle_entity) => counts ONCE per pair
                    robot = r1
                    other = r2_or_other
                    key = (robot, other)
                    entity_a, entity_b = robot, other
                    last_with = other
                    last_robot = robot

            else:
                # Per-robot mode: only count contacts that involve this namespace.
                # Determine "me" by namespace containment (same heuristic you used).
                ns = self.get_namespace().strip("/")
                is1_me = (ns != "") and (ns in col1)
                is2_me = (ns != "") and (ns in col2)
                if is1_me == is2_me:
                    continue

                me = col1 if is1_me else col2
                other = col2 if is1_me else col1
                bump_type, r_me, r_other = self._classify_bump(me, other)
                if bump_type == "unknown":
                    continue

                key = other  # debounced per "other entity"
                entity_a, entity_b = me, other
                last_with = other
                last_robot = r_me or ns

            # Debounce: new bump only if key not active
            if key not in self.active:
                if bump_type == "robot":
                    self.bump_robot += 1
                elif bump_type == "obstacle":
                    self.bump_obstacle += 1
                else:
                    # shouldn't happen due to filtering, but keep safe
                    continue

                self.bump_total = self.bump_robot + self.bump_obstacle
                self.active[key] = (now, bump_type)

                avg_xyz = mean_xyz(c.positions)
                stamp = msg.header.stamp
                self._append_csv(stamp, bump_type, key, entity_a, entity_b, avg_xyz, source_topic)

                self._publish_counts(last_with=last_with, bump_type=bump_type, last_robot=last_robot)

                self.get_logger().info(
                    f"[{self.label}] bump #{self.bump_total} ({bump_type}) with: {last_with}"
                )
            else:
                # refresh last seen
                self.active[key] = (now, self.active[key][1])

    def _prune(self):
        now = self.get_clock().now()
        to_drop = []
        for key, (last_seen, bump_type) in self.active.items():
            cd = self._cooldown_for_type(bump_type)
            if (now - last_seen) > cd:
                to_drop.append(key)
        for k in to_drop:
            del self.active[k]


def main():
    rclpy.init()
    node = BumpCounter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
