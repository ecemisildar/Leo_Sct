import os
import csv
import random
import math
import time
from glob import glob
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts

from ament_index_python.packages import get_package_share_directory
from swarm_basics.sct import SCT


@dataclass
class ActionSpec:
    """How to execute a controllable event in the ROS node."""
    linear_x: float = 0.0
    angular_z: float = 0.0
    hold_s: Optional[float] = None          # if None -> use default hold (motion_hold_duration)
    is_full_rotate: bool = False            # special: rotate 180 deg using odom (or timed fallback)


def _wrap_to_pi(a: float) -> float:
    # normalize to (-pi, pi]
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a


def _yaw_from_quat(q) -> float:
    # q is geometry_msgs/Quaternion
    # yaw from quaternion (x,y,z,w)
    x, y, z, w = q.x, q.y, q.z, q.w
    # yaw (Z axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class RobotSupervisor(Node):
    def __init__(self):
        super().__init__("robot_supervisor")

        # -------------------------------
        # Parameters
        # -------------------------------
        self.supervisor_period = float(self.declare_parameter("supervisor_period", 0.1).value)
        self.motion_hold_duration = float(self.declare_parameter("motion_hold_duration", 0.2).value)

        # Full-rotate execution settings
        # Cap at 180 degrees max, even if overridden via parameters.
        self.full_rotate_target_rad = min(
            math.pi,
            float(self.declare_parameter("full_rotate_target_rad", math.pi).value),
        )
        self.full_rotate_omega = float(self.declare_parameter("full_rotate_omega", 1.0).value)  # rad/s
        self.full_rotate_timeout_s = float(self.declare_parameter("full_rotate_timeout_s", 3.5).value)
        self.full_rotate_retrigger_block_s = float(
            self.declare_parameter("full_rotate_retrigger_block_s", 0.6).value
        )
        self.rotate_90_retrigger_block_s = float(
            self.declare_parameter("rotate_90_retrigger_block_s", 0.6).value
        )
        self.post_turn_settle_s = float(
            self.declare_parameter("post_turn_settle_s", 0.3).value
        )

        # Obstacle-front escape bias: block full_rotate briefly after obstacle_front
        self.full_rotate_block_after_obs_front = float(
            self.declare_parameter("full_rotate_block_after_obs_front", 0.8).value
        )  # seconds
        self.block_full_rotate_until = 0.0
        self.front_obstacle_active = False
        self.recovery_back_hold_s = float(
            self.declare_parameter("recovery_back_hold_s", 0.35).value
        )

        # 90-degree rotate settings
        self.rotate_90_target_rad = min(
            math.pi,
            float(self.declare_parameter("rotate_90_target_rad", math.pi / 2.0).value),
        )
        self.rotate_90_omega = float(
            self.declare_parameter("rotate_90_omega", 1.0).value
        )
        self.rotate_90_timeout_s = float(
            self.declare_parameter("rotate_90_timeout_s", 2.2).value
        )

        self.rotate_90_active = False
        self.rotate_90_target = 0.0
        self.rotate_90_accum = 0.0
        self.rotate_90_started_at = 0.0
        self.rotate_90_prev_yaw = 0.0


        # Zone update throttle. Keep low for fast reaction to depth obstacles.
        self.zone_update_min_dt = float(
            self.declare_parameter("zone_update_min_dt", 0.1).value
        )
        self.obstacle_zone_memory_s = float(
            self.declare_parameter("obstacle_zone_memory_s", 0.4).value
        )

        # Random seed (per-robot namespace)
        self.ns = self.get_namespace().strip("/") or "root"
        self.robot_index = self._namespace_index()
        base_seed = int(self.declare_parameter("random_seed", 12345).value)
        robot_seed = base_seed + self.robot_index
        self.rng = random.Random(robot_seed)
        random.seed(robot_seed)
        self.robot_seed = robot_seed
        self.sct_choice_mode = str(
            self.declare_parameter("sct_choice_mode", "random").value
        ).strip().lower()

        self.enabled = bool(self.declare_parameter("enabled", False).value)
        self.static_mode = bool(self.declare_parameter("static", False).value)
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
        self.sct_decision_log_enabled = bool(
            self.declare_parameter("sct_decision_log_enabled", False).value
        )
        self.peer_warning_enabled = bool(
            self.declare_parameter("peer_warning_enabled", True).value
        )
        self.peer_warning_log_enabled = bool(
            self.declare_parameter("peer_warning_log_enabled", False).value
        )
        self.peer_warning_timeout_s = float(
            self.declare_parameter("peer_warning_timeout_s", 0.35).value
        )
        self.results_dir = str(self.declare_parameter("results_dir", "").value).strip()
        self.run_id = str(self.declare_parameter("run_id", "").value).strip()
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
                f"/world/random_world/model/{self.ns}/link/{self.ns}/base_footprint/sensor/contact_sensor/contact",
            ).value
        )

        # -------------------------------
        # Load SCT YAML
        # -------------------------------
        self.config_dir = os.path.join(get_package_share_directory("swarm_basics"), "config")
        self.explicit_yaml_path = str(self.declare_parameter("supervisor_yaml_path", "").value).strip()
        self.current_mission = "explore"
        self.current_yaml_path = ""
        self._load_initial_sct()
        self._last_printed_sup_states: Optional[Tuple[int, ...]] = None

        # -------------------------------
        # State (sensing)
        # -------------------------------
        self.obstacle_zones = ["CLEAR"]
        self.last_non_clear_obstacle_zone = "CLEAR"
        self.last_non_clear_obstacle_zone_time = 0.0
        self.front_obstacle_distance_m = float("inf")
        self.last_front_obstacle_distance_time = 0.0
        self.last_zone_update = 0.0
        self.last_logged_zone = "CLEAR"
        self.peer_warning_zone = "CLEAR"
        self.peer_warning_source = ""
        self.last_peer_warning_time = 0.0
        # Odom / yaw tracking for full-rotate
        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_stamp_sec = 0
        self.odom_stamp_nsec = 0

        # -------------------------------
        # State (actuation)
        # -------------------------------
        self.stop_sent = False
        self.active_event: Optional[str] = None
        self.active_twist = Twist()
        self.motion_until = 0.0

        # Full-rotate mode bookkeeping
        self.full_rotate_active = False
        self.full_rotate_start_yaw = 0.0
        self.full_rotate_accum = 0.0
        self.full_rotate_started_at = 0.0
        self.full_rotate_prev_yaw = 0.0
        self.full_rotate_using_timed_fallback = False
        self.full_rotate_stall_count = 0
        self.last_full_rotate_completed_at = 0.0
        self.last_rotate_90_completed_at = 0.0
        self.turn_settle_until = 0.0
        self.contact_recovery_until = 0.0
        self.last_contact_recovery_started_at = 0.0
        self.contact_recovery_source = ""
        self.cbf_filter_log_path: Optional[Path] = None
        self.sct_decision_log_path: Optional[Path] = None
        self.peer_warning_log_path: Optional[Path] = None
        self._init_cbf_filter_log()
        self._init_sct_decision_log()
        self._init_peer_warning_log()

        # -------------------------------
        # Publishers/Subscribers
        # -------------------------------
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.sub_zone = self.create_subscription(String, "detected_zones", self.zone_callback, 10)
        self.sub_front_obstacle_distance = self.create_subscription(
            Float32,
            "front_obstacle_distance",
            self.front_obstacle_distance_callback,
            10,
        )
        self.sub_odom = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.sub_contact = self.create_subscription(
            Contacts,
            self.contact_topic,
            self.contact_callback,
            10,
        )
        self.sub_peer_warning = self.create_subscription(
            String,
            "peer_warning_zone",
            self.peer_warning_callback,
            10,
        )

        # -------------------------------
        # SCT callbacks for UCEs (data-driven, but still attaches known sensors)
        # -------------------------------
        # Timer + service
        self.timer = self.create_timer(self.supervisor_period, self.timer_callback)
        self.enable_service = self.create_service(SetBool, "enable_supervisor", self.handle_enable_supervisor)
        self.enable_service_explore = self.create_service(
            SetBool,
            "enable_supervisor_explore",
            self.handle_enable_supervisor_explore,
        )

        # -------------------------------
        # Action table (data-driven)
        # -------------------------------
        # Only place you encode motion parameters. No “logic” needs event names elsewhere.
        self.action_table: Dict[str, ActionSpec] = {
            "EV_random_walk": ActionSpec(linear_x=0.0, angular_z=0.0, hold_s=None),  # special handled below
            "EV_move_forward": ActionSpec(linear_x=0.2, angular_z=0.0),
            "EV_move_backward": ActionSpec(linear_x=-0.2, angular_z=0.0, hold_s=self.recovery_back_hold_s),
            "EV_rotate_clockwise": ActionSpec(
                linear_x=0.0,
                angular_z=-self.rotate_90_omega,
                is_full_rotate=False  # we handle separately
            ),
            "EV_rotate_counterclockwise": ActionSpec(
                linear_x=0.0,
                angular_z=self.rotate_90_omega,
                is_full_rotate=False
            ),
            "EV_stop": ActionSpec(linear_x=0.0, angular_z=0.0),
            # full_rotate is executed as an atomic rotation using odom; target is full_rotate_target_rad (≤ π here).
            "EV_full_rotate": ActionSpec(linear_x=0.0, angular_z=self.full_rotate_omega, is_full_rotate=True),
        }

    def _canonical_mission_name(self, mission: str) -> str:
        key = str(mission or "").strip().lower().replace("-", "_").replace(" ", "_")
        if key != "explore":
            key = "explore"
        return key

    def _mission_yaml_candidates(self, mission: str):
        mission_key = self._canonical_mission_name(mission)
        candidates = []
        preferred = os.path.join(self.config_dir, f"{mission_key}_sup_gpt.yaml")
        if os.path.exists(preferred):
            candidates.append(preferred)
        task_files = sorted(
            glob(os.path.join(self.config_dir, f"{mission_key}_sup_gpt_*.yaml")),
            key=os.path.getmtime,
            reverse=True,
        )
        candidates.extend(task_files)
        fallback = os.path.join(self.config_dir, "sup_gpt.yaml")
        if os.path.exists(fallback):
            candidates.append(fallback)

        seen = set()
        unique = []
        for path in candidates:
            if path not in seen:
                unique.append(path)
                seen.add(path)
        return unique

    def _load_sct_from_yaml(self, config_path: str):
        self.sct = SCT(
            config_path,
            random_seed=self.robot_seed,
            choice_mode=self.sct_choice_mode,
        )
        self.ev_name_by_id = {ev_id: ev_name for ev_name, ev_id in self.sct.EV.items()}
        for ev_id in self.sct.EV.values():
            if ev_id not in self.sct.callback:
                self.sct.callback[ev_id] = {
                    "callback": None,
                    "check_input": (lambda _sup_data: False),
                    "sup_data": None,
                }
        self._install_uncontrollable_callbacks()

    def _load_initial_sct(self):
        if self.explicit_yaml_path:
            if not os.path.exists(self.explicit_yaml_path):
                self.get_logger().error(
                    f"Explicit supervisor YAML not found: {self.explicit_yaml_path}"
                )
                raise SystemExit(1)
            config_path = self.explicit_yaml_path
            self._load_sct_from_yaml(config_path)
            self.current_yaml_path = config_path
            self.get_logger().info(
                f"Loaded initial mission '{self.current_mission}' from explicit YAML {os.path.basename(config_path)}"
            )
            return
        paths = self._mission_yaml_candidates(self.current_mission)
        if not paths:
            self.get_logger().error(
                f"No supervisor YAML found in {self.config_dir}. "
                "Expected mission-specific files or sup_gpt.yaml."
            )
            raise SystemExit(1)
        config_path = paths[0]
        self._load_sct_from_yaml(config_path)
        self.current_yaml_path = config_path
        self.get_logger().info(
            f"Loaded initial mission '{self.current_mission}' from {os.path.basename(config_path)}"
        )

    def _switch_mission(self, mission: str) -> Tuple[bool, str]:
        if self.explicit_yaml_path:
            return True, os.path.basename(self.current_yaml_path or self.explicit_yaml_path)
        mission_key = self._canonical_mission_name(mission)
        paths = self._mission_yaml_candidates(mission_key)
        if not paths:
            return False, f"No YAML found for mission '{mission_key}' in {self.config_dir}"
        config_path = paths[0]
        try:
            self._load_sct_from_yaml(config_path)
        except Exception as exc:
            return False, f"Failed to load {os.path.basename(config_path)}: {exc}"
        self.current_mission = mission_key
        self.current_yaml_path = config_path
        self._last_printed_sup_states = None
        self.get_logger().info(
            f"Switched mission to '{mission_key}' using {os.path.basename(config_path)}"
        )
        return True, os.path.basename(config_path)

    def _print_current_state(self):
        states = tuple(int(s) for s in self.sct.sup_current_state)
        if states == self._last_printed_sup_states:
            return
        self._last_printed_sup_states = states
        # print(
        #     f"[robot_supervisor] mission={self.current_mission} current_state={states}",
        #     flush=True,
        # )

    def _set_enabled(self, enable: bool):
        self.enabled = bool(enable)
        self.stop_sent = False
        self._cancel_all_motion()
        if not self.enabled:
            self._publish_stop()

    def _publish_cmd(self, twist: Twist):
        # Testing mode: clamp all outgoing cmd_vel values to zero.
        if self.static_mode:
            self.cmd_pub.publish(Twist())
            return
        safe_twist = self._cbf_filter_twist(twist)
        self.cmd_pub.publish(safe_twist)

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
        self.cbf_filter_log_path = log_dir / f"cbf_filter_{self.ns}.csv"
        if self.cbf_filter_log_path.exists():
            return

        with self.cbf_filter_log_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "wall_time",
                "robot",
                "event",
                "reasons",
                "zones",
                "front_distance_m",
                "distance_age_s",
                "cbf_safety_distance_m",
                "cbf_alpha",
                "requested_linear_x",
                "requested_angular_z",
                "filtered_linear_x",
                "filtered_angular_z",
            ])

    def _init_sct_decision_log(self):
        if not self.sct_decision_log_enabled:
            return

        log_dir = self._run_log_dir()
        log_dir.mkdir(parents=True, exist_ok=True)
        self.sct_decision_log_path = log_dir / f"sct_decisions_{self.ns}.csv"
        if self.sct_decision_log_path.exists():
            return

        with self.sct_decision_log_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "wall_time",
                "robot",
                "selected_event",
                "ce_exists",
                "sct_state",
                "raw_zone",
                "effective_zone",
                "front_distance_m",
                "distance_age_s",
                "path_clear",
                "obstacle_front",
                "obstacle_left",
                "obstacle_right",
                "contact_recovery_active",
                "turn_settle_active",
                "active_event_before",
            ])

    def _init_peer_warning_log(self):
        if not self.peer_warning_log_enabled:
            return

        log_dir = self._run_log_dir()
        log_dir.mkdir(parents=True, exist_ok=True)
        self.peer_warning_log_path = log_dir / f"peer_warnings_{self.ns}.csv"
        if self.peer_warning_log_path.exists():
            return

        with self.peer_warning_log_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "wall_time",
                "odom_stamp_sec",
                "odom_stamp_nsec",
                "receiver",
                "warning_zone",
                "source",
                "source_zone",
                "target_zone",
                "distance_m",
                "bearing_rad",
                "bearing_deg",
                "zone_reason",
                "source_x",
                "source_y",
                "source_yaw",
                "target_x",
                "target_y",
                "target_yaw",
                "raw_zone",
                "effective_zones_after",
                "front_distance_m",
                "distance_age_s",
                "active_event",
                "contact_recovery_active",
                "turn_settle_active",
                "receiver_x",
                "receiver_y",
                "receiver_yaw",
                "raw_message",
            ])

    def _parse_peer_warning_fields(self, message: str) -> Dict[str, str]:
        fields: Dict[str, str] = {}
        for part in message.split(","):
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            fields[key.strip().lower()] = value.strip()
        return fields

    def _log_peer_warning(self, warning_zone: str, raw_message: str):
        if not self.peer_warning_log_enabled or self.peer_warning_log_path is None:
            return

        now = time.time()
        fields = self._parse_peer_warning_fields(raw_message)
        distance = fields.get("distance", "")
        try:
            distance_text = f"{float(distance):.6f}"
        except (TypeError, ValueError):
            distance_text = distance

        effective_zones = self._effective_obstacle_zones()
        distance_age = now - self.last_front_obstacle_distance_time

        with self.peer_warning_log_path.open("a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                f"{now:.6f}",
                int(self.odom_stamp_sec),
                int(self.odom_stamp_nsec),
                self.ns,
                warning_zone,
                fields.get("source", ""),
                fields.get("source_zone", ""),
                fields.get("target_zone", ""),
                distance_text,
                fields.get("bearing_rad", ""),
                fields.get("bearing_deg", ""),
                fields.get("zone_reason", ""),
                fields.get("source_x", ""),
                fields.get("source_y", ""),
                fields.get("source_yaw", ""),
                fields.get("target_x", ""),
                fields.get("target_y", ""),
                fields.get("target_yaw", ""),
                self.obstacle_zones[0],
                "|".join(effective_zones),
                f"{self.front_obstacle_distance_m:.6f}" if math.isfinite(self.front_obstacle_distance_m) else "inf",
                f"{distance_age:.6f}" if math.isfinite(distance_age) else "inf",
                self._current_command_event_label(),
                str(bool(now < self.contact_recovery_until)).lower(),
                str(bool(now < self.turn_settle_until)).lower(),
                f"{self.x:.6f}",
                f"{self.y:.6f}",
                f"{self.yaw:.6f}",
                raw_message,
            ])

    def _snapshot_sct_inputs(self, now: float):
        effective_zones = self._effective_obstacle_zones()
        depth_obstacles = {"LEFT", "RIGHT", "CORNER"}
        return {
            "raw_zone": self.obstacle_zones[0],
            "effective_zone": "|".join(effective_zones),
            "front_distance_m": self.front_obstacle_distance_m,
            "distance_age_s": now - self.last_front_obstacle_distance_time,
            "path_clear": not any(zone in depth_obstacles for zone in effective_zones),
            "obstacle_front": "CORNER" in effective_zones,
            "obstacle_left": "LEFT" in effective_zones,
            "obstacle_right": "RIGHT" in effective_zones,
            "contact_recovery_active": now < self.contact_recovery_until,
            "turn_settle_active": now < self.turn_settle_until,
            "active_event_before": self.active_event or "",
        }

    def _log_sct_decision(self, selected_event: str, ce_exists: bool, snapshot):
        if not self.sct_decision_log_enabled or self.sct_decision_log_path is None:
            return

        with self.sct_decision_log_path.open("a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                f"{time.time():.6f}",
                self.ns,
                selected_event,
                str(bool(ce_exists)).lower(),
                "|".join(str(int(s)) for s in self.sct.sup_current_state),
                snapshot["raw_zone"],
                snapshot["effective_zone"],
                f"{snapshot['front_distance_m']:.6f}" if math.isfinite(snapshot["front_distance_m"]) else "inf",
                f"{snapshot['distance_age_s']:.6f}" if math.isfinite(snapshot["distance_age_s"]) else "inf",
                str(bool(snapshot["path_clear"])).lower(),
                str(bool(snapshot["obstacle_front"])).lower(),
                str(bool(snapshot["obstacle_left"])).lower(),
                str(bool(snapshot["obstacle_right"])).lower(),
                str(bool(snapshot["contact_recovery_active"])).lower(),
                str(bool(snapshot["turn_settle_active"])).lower(),
                snapshot["active_event_before"],
            ])

    def _current_command_event_label(self) -> str:
        if self.contact_recovery_until > time.time():
            return "CONTACT_RECOVERY"
        if self.full_rotate_active:
            return self.active_event or "FULL_ROTATE"
        if self.rotate_90_active:
            return self.active_event or "ROTATE_90"
        return self.active_event or "none"

    def _log_cbf_filter(
        self,
        requested: Twist,
        filtered: Twist,
        effective_zones,
        distance_age: float,
        reasons,
    ):
        if not self.cbf_filter_log_enabled or self.cbf_filter_log_path is None or not reasons:
            return

        with self.cbf_filter_log_path.open("a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                f"{time.time():.6f}",
                self.ns,
                self._current_command_event_label(),
                "|".join(reasons),
                "|".join(effective_zones),
                f"{self.front_obstacle_distance_m:.6f}" if math.isfinite(self.front_obstacle_distance_m) else "inf",
                f"{distance_age:.6f}" if math.isfinite(distance_age) else "inf",
                f"{self.cbf_safety_distance_m:.6f}",
                f"{self.cbf_alpha:.6f}",
                f"{requested.linear.x:.6f}",
                f"{requested.angular.z:.6f}",
                f"{filtered.linear.x:.6f}",
                f"{filtered.angular.z:.6f}",
            ])

    def _cbf_filter_twist(self, twist: Twist) -> Twist:
        """
        One-dimensional CBF safety filter for the robot's front depth ROI.

        h(d) = d - d_safe. For a forward command v, approximate h_dot = -v.
        Enforce h_dot + alpha*h >= 0, which gives v <= alpha*(d-d_safe).
        Backward and rotational commands are left available so the supervisor can
        recover from blocked states.
        """
        safe_twist = Twist()
        safe_twist.linear.x = float(twist.linear.x)
        safe_twist.linear.y = float(twist.linear.y)
        safe_twist.linear.z = float(twist.linear.z)
        safe_twist.angular.x = float(twist.angular.x)
        safe_twist.angular.y = float(twist.angular.y)
        safe_twist.angular.z = float(twist.angular.z)

        if not self.cbf_enabled:
            return safe_twist

        now = time.time()
        effective_zones = self._effective_obstacle_zones()
        zones = set(effective_zones)
        distance_age = now - self.last_front_obstacle_distance_time
        reasons = []

        if self.cbf_zone_stop_forward and safe_twist.linear.x > 0.0 and zones & {"LEFT", "RIGHT", "CORNER"}:
            safe_twist.linear.x = 0.0
            reasons.append("zone_stop_forward")

        if self.cbf_zone_avoid_turning_into_obstacle:
            if "LEFT" in zones and safe_twist.angular.z > 0.0:
                safe_twist.angular.z = 0.0
                reasons.append("turn_left_blocked")
            if "RIGHT" in zones and safe_twist.angular.z < 0.0:
                safe_twist.angular.z = 0.0
                reasons.append("turn_right_blocked")

        if safe_twist.linear.x <= 0.0:
            self._log_cbf_filter(twist, safe_twist, effective_zones, distance_age, reasons)
            return safe_twist

        distance_is_fresh = (
            self.last_front_obstacle_distance_time > 0.0
            and distance_age <= self.cbf_distance_timeout_s
            and math.isfinite(self.front_obstacle_distance_m)
        )

        if not distance_is_fresh:
            if self.cbf_stop_on_stale_distance and effective_zones[0] != "CLEAR":
                safe_twist.linear.x = 0.0
                reasons.append("stale_distance_stop")
                self._log_cbf_filter(twist, safe_twist, effective_zones, distance_age, reasons)
            return safe_twist

        h = self.front_obstacle_distance_m - self.cbf_safety_distance_m
        max_forward_v = max(0.0, self.cbf_alpha * h)
        if safe_twist.linear.x > max_forward_v:
            safe_twist.linear.x = max_forward_v
            reasons.append("distance_cbf_cap")
        self._log_cbf_filter(twist, safe_twist, effective_zones, distance_age, reasons)
        return safe_twist

    # -------------------------------
    # Subscriptions
    # -------------------------------
    def odom_callback(self, msg: Odometry):
        self.have_odom = True
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self.odom_stamp_sec = int(msg.header.stamp.sec)
        self.odom_stamp_nsec = int(msg.header.stamp.nanosec)

    def zone_callback(self, msg: String):
        now = time.time()
        if now - self.last_zone_update < self.zone_update_min_dt:
            return
        self.last_zone_update = now

        z = msg.data.strip().upper()
        # accept only known tokens
        if z in {"LEFT", "RIGHT", "CORNER", "CLEAR"}:
            self.obstacle_zones = [z]
        else:
            self.obstacle_zones = ["CLEAR"]

        zone = self.obstacle_zones[0]
        if zone != "CLEAR":
            self.last_non_clear_obstacle_zone = zone
            self.last_non_clear_obstacle_zone_time = now
        if zone != self.last_logged_zone:
            self.get_logger().info(f"Detected zone changed to {zone}")
            self.last_logged_zone = zone

    def peer_warning_callback(self, msg: String):
        if not self.peer_warning_enabled:
            return

        token = msg.data.strip().upper().split(",", 1)[0].split(":", 1)[0].strip()
        if token not in {"LEFT", "RIGHT", "CORNER", "CLEAR"}:
            return

        self.peer_warning_zone = token
        self.peer_warning_source = msg.data
        self.last_peer_warning_time = time.time()
        self._log_peer_warning(token, msg.data)

    def _effective_obstacle_zones(self):
        zones = []
        if self.obstacle_zones[0] != "CLEAR":
            zones.extend(self.obstacle_zones)
        elif (
            self.last_non_clear_obstacle_zone != "CLEAR"
            and (time.time() - self.last_non_clear_obstacle_zone_time) <= self.obstacle_zone_memory_s
        ):
            zones.append(self.last_non_clear_obstacle_zone)

        if (
            self.peer_warning_enabled
            and self.peer_warning_zone != "CLEAR"
            and (time.time() - self.last_peer_warning_time) <= self.peer_warning_timeout_s
        ):
            zones.append(self.peer_warning_zone)

        if not zones:
            return ["CLEAR"]

        unique_zones = []
        for zone in zones:
            if zone not in unique_zones:
                unique_zones.append(zone)
        return unique_zones

    def front_obstacle_distance_callback(self, msg: Float32):
        distance = float(msg.data)
        self.front_obstacle_distance_m = distance if math.isfinite(distance) else float("inf")
        self.last_front_obstacle_distance_time = time.time()

    def contact_callback(self, msg: Contacts):
        if not self.contact_recovery_enabled or not self.enabled:
            return
        if not msg.contacts:
            return

        now = time.time()
        if (now - self.last_contact_recovery_started_at) < self.contact_recovery_retrigger_block_s:
            return

        self.last_contact_recovery_started_at = now
        self.contact_recovery_until = max(
            self.contact_recovery_until,
            now + self.contact_recovery_duration_s,
        )
        self.contact_recovery_source = self._contact_summary(msg)
        self.full_rotate_active = False
        self.rotate_90_active = False
        self.active_event = None
        self.motion_until = 0.0
        self.get_logger().info(
            "CONTACT RECOVERY activated "
            f"(source={self.contact_recovery_source}, duration_s={self.contact_recovery_duration_s:.2f})"
        )

    def _contact_summary(self, msg: Contacts) -> str:
        names = []
        for contact in msg.contacts[:3]:
            for collision in (contact.collision1, contact.collision2):
                name = getattr(collision, "name", str(collision))
                if self.ns not in name:
                    names.append(name)
        return ",".join(names[:3]) or "local_contact"

    # -------------------------------
    # SCT input check functions (uncontrollables)
    # -------------------------------
    def clear_path_check(self, sup_data):
        depth_obstacles = {"LEFT", "RIGHT", "CORNER"}
        return not any(zone in depth_obstacles for zone in self._effective_obstacle_zones())

    def middle_check(self, sup_data):
        hit = "CORNER" in self._effective_obstacle_zones()
        if hit and not self.front_obstacle_active:
            now = time.time()
            # If we're in a front-obstacle situation, block full_rotate for ~1 tick
            self.block_full_rotate_until = max(
                self.block_full_rotate_until,
                now + self.full_rotate_block_after_obs_front,
            )
        self.front_obstacle_active = hit
        return hit

    def left_check(self, sup_data):
        return "LEFT" in self._effective_obstacle_zones()

    def right_check(self, sup_data):
        return "RIGHT" in self._effective_obstacle_zones()

    def _install_uncontrollable_callbacks(self):
        # Attach callbacks only for events that exist in current supervisor YAML.
        def add(ev: str, fn):
            key = f"EV_{ev}"
            if key in self.sct.EV:
                self.sct.add_callback(self.sct.EV[key], None, fn, None)

        add("obstacle_front", self.middle_check)
        add("path_clear", self.clear_path_check)
        add("obstacle_left", self.left_check)
        add("obstacle_right", self.right_check)

    # -------------------------------
    # Enable service
    # -------------------------------
    def handle_enable_supervisor(self, request, response):
        # Backward-compatible entry point: keep current mission, only toggle enabled state.
        self._set_enabled(request.data)
        if self.enabled:
            response.message = (
                f"Supervisor enabled (mission={self.current_mission}, yaml={os.path.basename(self.current_yaml_path)})."
            )
        else:
            response.message = "Supervisor disabled."
        response.success = True
        return response

    def handle_enable_supervisor_explore(self, request, response):
        if bool(request.data):
            ok, detail = self._switch_mission("explore")
            if not ok:
                response.success = False
                response.message = detail
                return response
        self._set_enabled(request.data)
        response.success = True
        if self.enabled:
            response.message = f"Supervisor enabled for explore ({detail})."
        else:
            response.message = "Supervisor disabled."
        return response

    def _namespace_index(self) -> int:
        if self.ns.startswith("robot_"):
            try:
                return int(self.ns.split("_")[-1])
            except ValueError:
                return 0
        return 0

    # -------------------------------
    # Motion execution
    # -------------------------------
    def _publish_stop(self):
        self.active_twist = Twist()
        self._publish_cmd(self.active_twist)

    def _publish_contact_recovery_cmd(self):
        twist = Twist()
        linear_x = self.contact_recovery_linear_x
        turn_sign = 1.0 if (self.robot_index % 2 == 0) else -1.0
        zones = self._effective_obstacle_zones()

        if "CORNER" in zones:
            linear_x = min(linear_x, 0.0)
        elif "LEFT" in zones:
            turn_sign = -1.0
        elif "RIGHT" in zones:
            turn_sign = 1.0

        twist.linear.x = linear_x
        twist.angular.z = turn_sign * abs(self.contact_recovery_angular_z)
        self.active_twist = twist
        self._publish_cmd(self.active_twist)

    def _cancel_all_motion(self):
        self.active_event = None
        self.motion_until = 0.0
        self.active_twist = Twist()
        self.full_rotate_active = False
        self.full_rotate_accum = 0.0
        self.full_rotate_started_at = 0.0
        self.rotate_90_active = False
        self.rotate_90_accum = 0.0
        self.rotate_90_started_at = 0.0
        self.rotate_90_prev_yaw = 0.0
        self.contact_recovery_until = 0.0
        self.turn_settle_until = 0.0

    def _enter_post_turn_settle(self, now: float):
        # Give sensing callbacks a short window to catch up before asking SCT
        # for another controllable event. This prevents turn retriggers when the
        # supervisor period is faster than obstacle updates.
        self.turn_settle_until = max(self.turn_settle_until, now + self.post_turn_settle_s)
        self.active_event = None
        self.motion_until = 0.0
        self._publish_stop()

    def _start_full_rotate(self, omega: float):
        # Start a 180° rotation that persists beyond the 0.2s pulse.
        self.full_rotate_active = True
        self.full_rotate_started_at = time.time()
        self.full_rotate_accum = 0.0
        self.full_rotate_using_timed_fallback = False
        self.full_rotate_stall_count = 0

        if self.have_odom:
            self.full_rotate_start_yaw = self.yaw
            self.full_rotate_prev_yaw = self.yaw
        else:
            # fallback: timed rotation
            # duration = target / omega
            dur = abs(self.full_rotate_target_rad / max(1e-6, abs(omega)))
            self.motion_until = time.time() + min(dur, self.full_rotate_timeout_s)

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = omega
        self.active_twist = twist
        self.get_logger().info(
            "Starting FULL ROTATE "
            f"(omega={omega:.3f}, have_odom={self.have_odom}, "
            f"target_rad={self.full_rotate_target_rad:.3f}, timeout_s={self.full_rotate_timeout_s:.3f})"
        )
        self._publish_cmd(self.active_twist)

    def _update_full_rotate(self) -> bool:
        """
        Returns True if full rotate completed, False otherwise.
        """
        now = time.time()
        if (now - self.full_rotate_started_at) > self.full_rotate_timeout_s:
            # timeout safety
            self.get_logger().info("FULL ROTATE completed by timeout")
            return True

        if not self.have_odom or self.full_rotate_using_timed_fallback:
            # timed fallback uses motion_until
            done = now >= self.motion_until
            if done:
                self.get_logger().info("FULL ROTATE completed by timed fallback")
            return done

        # integrate yaw delta each tick (and keep publishing until done)
        dy = _wrap_to_pi(self.yaw - self.full_rotate_prev_yaw)
        self.full_rotate_accum += abs(dy)
        self.full_rotate_prev_yaw = self.yaw

        if abs(dy) < 1e-3:
            self.full_rotate_stall_count += 1
        else:
            self.full_rotate_stall_count = 0

        if self.full_rotate_stall_count >= 5:
            remaining = max(0.0, self.full_rotate_target_rad - self.full_rotate_accum)
            omega = max(1e-6, abs(self.active_twist.angular.z))
            dur = min(remaining / omega, self.full_rotate_timeout_s)
            self.motion_until = now + dur
            self.full_rotate_using_timed_fallback = True
            self.get_logger().info(
                "FULL ROTATE switching to timed fallback "
                f"(accum_rad={self.full_rotate_accum:.3f}, remaining_rad={remaining:.3f}, "
                f"stall_count={self.full_rotate_stall_count}, fallback_s={dur:.3f})"
            )

        done = self.full_rotate_accum >= self.full_rotate_target_rad
        if done:
            self.get_logger().info(
                f"FULL ROTATE completed by odom (accum_rad={self.full_rotate_accum:.3f})"
            )
        return done

    def _start_rotate_90(self, omega: float):
        self.rotate_90_active = True
        self.rotate_90_started_at = time.time()
        self.rotate_90_accum = 0.0

        if self.have_odom:
            self.rotate_90_prev_yaw = self.yaw
        else:
            dur = abs(self.rotate_90_target_rad / max(1e-6, abs(omega)))
            self.motion_until = time.time() + min(dur, self.rotate_90_timeout_s)

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = omega
        self.active_twist = twist
        self._publish_cmd(self.active_twist)

    def _update_rotate_90(self) -> bool:
        now = time.time()

        if (now - self.rotate_90_started_at) > self.rotate_90_timeout_s:
            return True

        if not self.have_odom:
            return now >= self.motion_until

        dy = _wrap_to_pi(self.yaw - self.rotate_90_prev_yaw)
        self.rotate_90_accum += abs(dy)
        self.rotate_90_prev_yaw = self.yaw

        return self.rotate_90_accum >= self.rotate_90_target_rad
    
    

    def publish_twist_for_event(self, ev_name: str):
        spec = self.action_table.get(ev_name)

        # Unknown controllable -> stop (safe)
        if spec is None:
            self.active_event = None
            self._publish_stop()
            return

        # random walk is special (stochastic each time it fires)
        if ev_name == "EV_random_walk":
            twist = Twist()
            twist.linear.x = self.rng.uniform(0.1, 0.4)
            hold = self.motion_hold_duration
            if self.rng.random() < 0.6:
                twist.angular.z = 0.0
            else:
                turn_angle = self.rng.uniform(-math.pi, math.pi)
                twist.angular.z = math.copysign(
                    self.rng.uniform(0.35, 0.9),
                    turn_angle,
                )
                hold = min(
                    abs(turn_angle) / max(1e-6, abs(twist.angular.z)),
                    1.4,
                )
            self.active_event = ev_name
            self.active_twist = twist
            self.motion_until = time.time() + hold
            self._publish_cmd(self.active_twist)
            return

        if ev_name in ("EV_rotate_clockwise", "EV_rotate_counterclockwise"):
            now = time.time()
            if (now - self.last_rotate_90_completed_at) < self.rotate_90_retrigger_block_s:
                self.get_logger().info("rotate_90 blocked by recent completion; stopping this tick")
                self.active_event = None
                self.motion_until = 0.0
                self._publish_stop()
                return
            self.active_event = ev_name
            self._start_rotate_90(spec.angular_z)
            return

        if spec.is_full_rotate:
            now = time.time()
            if (now - self.last_full_rotate_completed_at) < self.full_rotate_retrigger_block_s:
                self.get_logger().info("full_rotate blocked by recent completion; stopping this tick")
                self.active_event = None
                self.motion_until = 0.0
                self._publish_stop()
                return
            # Cooldown: prevent repeated "scan in place" when we're stuck in obs_front
            if now < self.block_full_rotate_until:
                self.get_logger().info("full_rotate blocked by cooldown; stopping this tick")
                self.active_event = None
                self.motion_until = 0.0
                self._publish_stop()
                return
            self.active_event = ev_name
            self._start_full_rotate(spec.angular_z)
            return

        # Normal pulse action
        twist = Twist()
        twist.linear.x = float(spec.linear_x)
        twist.angular.z = float(spec.angular_z)

        hold = self.motion_hold_duration if spec.hold_s is None else float(spec.hold_s)

        self.active_event = ev_name
        self.active_twist = twist
        self.motion_until = time.time() + hold
        self._publish_cmd(self.active_twist)

    # -------------------------------
    # Supervisor tick
    # -------------------------------
    def timer_callback(self):
        if not self.enabled:
            if not self.stop_sent:
                self._cancel_all_motion()
                self._publish_stop()
                self.stop_sent = True
            return

        self.stop_sent = False
        now = time.time()

        if now < self.contact_recovery_until:
            self._publish_contact_recovery_cmd()
            return

        # If we’re in the middle of a true full_rotate, keep executing until complete.
        if self.full_rotate_active:
            self.get_logger().info(
                "FULL ROTATE active "
                f"(accum_rad={self.full_rotate_accum:.3f}, "
                f"timed_fallback={self.full_rotate_using_timed_fallback}, "
                f"stall_count={self.full_rotate_stall_count})"
            )
            self._publish_cmd(self.active_twist)
            if self._update_full_rotate():
                # stop rotation and resume supervisor next tick
                self.full_rotate_active = False
                self.last_full_rotate_completed_at = now
                self._enter_post_turn_settle(now)
                self.get_logger().info(
                    "FULL ROTATE stopped; supervisor will select next event next tick"
                )
            return

        # If doing a 90-degree rotate
        if self.rotate_90_active:
            self._publish_cmd(self.active_twist)
            if self._update_rotate_90():
                self.rotate_90_active = False
                self.last_rotate_90_completed_at = now
                self._enter_post_turn_settle(now)
            return

        if now < self.turn_settle_until:
            self._publish_stop()
            return

        # Normal pulse-hold: keep publishing until hold expires
        if self.active_event and now < self.motion_until:
            self._publish_cmd(self.active_twist)
            return

        # Otherwise: pick next event from SCT
        self.active_event = None
        self.sct.input_buffer = []
        sct_input_snapshot = self._snapshot_sct_inputs(now)
        ce_exists, ce = self.sct.run_step()
        self._print_current_state()
        if not ce_exists:
            # No controllable enabled -> stop
            self._log_sct_decision("none", False, sct_input_snapshot)
            self._publish_stop()
            return

        ev_name = self.ev_name_by_id.get(int(ce))
        if ev_name is None:
            self._log_sct_decision(f"unknown:{int(ce)}", True, sct_input_snapshot)
            self._publish_stop()
            return

        self._log_sct_decision(ev_name, True, sct_input_snapshot)
        self.get_logger().info(f"Selected controllable event: {ev_name}")
        self.publish_twist_for_event(ev_name)


def main(args=None):
    rclpy.init(args=args)
    node = RobotSupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
