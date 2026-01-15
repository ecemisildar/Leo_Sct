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
from pathlib import Path
from typing import Dict, Tuple, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import UInt32, String
from ros_gz_interfaces.msg import Contacts

# ------------------ Defaults ------------------
DEFAULT_COOLDOWN_SEC = 0.5          # generic cooldown
DEFAULT_COOLDOWN_ROBOT_SEC = 0.8    # optional: slightly longer for robot-robot
DEFAULT_COOLDOWN_OBS_SEC = 0.5      # optional: for robot-obstacle
PRUNE_HZ = 10.0                     # prune inactive contacts at this rate
CONTACT_TOPIC = "contact"           # used only in per-robot mode
LOG_DIR = Path.home() / "ros_bump_logs"
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

        # ---- Identity label for CSV/logging ----
        ns = self.get_namespace().strip("/") or "root"
        self.label = "global" if self.global_mode else ns

        # ---- State ----
        # active: key -> (last_seen_time, bump_type)
        self.active: Dict[Any, Tuple[rclpy.time.Time, str]] = {}

        self.bump_total = 0
        self.bump_robot = 0       # global_mode: robot-robot PAIR events
        self.bump_obstacle = 0

        # ---- Publishers ----
        self.pub_total = self.create_publisher(UInt32, "bump_count", 10)
        self.pub_robot = self.create_publisher(UInt32, "bump_count_robot", 10)
        self.pub_obs = self.create_publisher(UInt32, "bump_count_obstacle", 10)
        self.pub_last = self.create_publisher(String, "last_bump_with", 10)
        self.pub_last_type = self.create_publisher(String, "last_bump_type", 10)

        # ---- Subscriptions ----
        self._subs = {}
        if self.global_mode:
            self._topic_scan_timer = self.create_timer(1.0, self._refresh_contact_subs)
        else:
            self.sub = self.create_subscription(Contacts, CONTACT_TOPIC, self._on_contacts, 10)

        # prune timer
        self.create_timer(1.0 / PRUNE_HZ, self._prune)

        # ---- CSV logging ----
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.csv_path = self._csv_path_for_today()
        self._ensure_csv_header()

        self.get_logger().info(
            f"[{self.label}] global_mode={self.global_mode} "
            f"cooldown(robot)={cooldown_robot_sec:.2f}s cooldown(obs)={cooldown_obstacle_sec:.2f}s "
            f"Logging to {self.csv_path}"
        )

    # ---------------- CSV helpers ----------------
    def _csv_path_for_today(self) -> Path:
        timestr = time.strftime("%H%M%S")
        return LOG_DIR / f"bumps_{self.label}_{timestr}.csv"

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
                    "source_topic"
                ])

    def _append_csv(self, stamp, bump_type: str, key: Any,
                    entity_a: str, entity_b: str, avg_xyz, source_topic: str):
        with self.csv_path.open("a", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                int(stamp.sec), int(stamp.nanosec),
                self.label,
                int(self.bump_total), int(self.bump_robot), int(self.bump_obstacle),
                bump_type,
                str(key),
                entity_a, entity_b,
                f"{avg_xyz[0]:.6f}" if isinstance(avg_xyz[0], float) else "",
                f"{avg_xyz[1]:.6f}" if isinstance(avg_xyz[1], float) else "",
                f"{avg_xyz[2]:.6f}" if isinstance(avg_xyz[2], float) else "",
                source_topic
            ])
    # --------------------------------------------

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

    def _publish_counts(self, last_with: str, bump_type: str):
        self.pub_total.publish(UInt32(data=int(self.bump_total)))
        self.pub_robot.publish(UInt32(data=int(self.bump_robot)))
        self.pub_obs.publish(UInt32(data=int(self.bump_obstacle)))
        self.pub_last.publish(String(data=last_with))
        self.pub_last_type.publish(String(data=bump_type))

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
                else:
                    # key is (robot, obstacle_entity) => counts ONCE per pair
                    robot = r1
                    other = r2_or_other
                    key = (robot, other)
                    entity_a, entity_b = robot, other
                    last_with = other

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

                self._publish_counts(last_with=last_with, bump_type=bump_type)

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
