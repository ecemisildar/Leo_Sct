import os
import random
import math
import re
import time
from glob import glob
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry

from ament_index_python import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory

try:
    from .sct import SCT
except ImportError:
    from leo_real.sct import SCT


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
        super().__init__("robot_supervisor_rgb")

        # -------------------------------
        # Parameters
        # -------------------------------
        self.supervisor_period = float(self.declare_parameter("supervisor_period", 0.5).value)
        self.motion_hold_duration = float(self.declare_parameter("motion_hold_duration", 0.2).value)

        # Full-rotate execution settings
        # Cap at 180 degrees max, even if overridden via parameters.
        self.full_rotate_target_rad = min(
            math.pi,
            float(self.declare_parameter("full_rotate_target_rad", math.pi).value),
        )
        self.full_rotate_omega = float(self.declare_parameter("full_rotate_omega", 2.0).value)  # rad/s
        self.full_rotate_timeout_s = float(self.declare_parameter("full_rotate_timeout_s", 6.0).value)
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
            self.declare_parameter("rotate_90_omega", 1.5).value
        )
        self.rotate_90_timeout_s = float(
            self.declare_parameter("rotate_90_timeout_s", 4.0).value
        )

        self.rotate_90_active = False
        self.rotate_90_target = 0.0
        self.rotate_90_accum = 0.0
        self.rotate_90_started_at = 0.0
        self.rotate_90_prev_yaw = 0.0


        # Zone update throttle.
        self.zone_update_min_dt = float(
            self.declare_parameter("zone_update_min_dt", 0.1).value
        )
        self.green_centering_gain = float(
            self.declare_parameter("green_centering_gain", 0.8).value
        )
        self.green_centering_max_angular = abs(
            float(self.declare_parameter("green_centering_max_angular", 0.45).value)
        )
        self.green_centering_deadband = abs(
            float(self.declare_parameter("green_centering_deadband", 0.08).value)
        )
        self.green_offset_timeout_s = float(
            self.declare_parameter("green_offset_timeout_s", 0.5).value
        )
        self.green_follow_linear_x = float(
            self.declare_parameter("green_follow_linear_x", 0.06).value
        )
        self.green_follow_centered_deadband = abs(
            float(self.declare_parameter("green_follow_centered_deadband", 0.18).value)
        )

        # Random seed (per-robot namespace)
        self.ns = self.get_namespace().strip("/") or "root"
        base_seed = int(self.declare_parameter("random_seed", 12345).value)
        self.rng = random.Random(base_seed + self._namespace_index())

        self.enabled = bool(self.declare_parameter("enabled", False).value)
        self.static_mode = bool(self.declare_parameter("static", False).value)
        self.routine_logging_enabled = bool(
            self.declare_parameter("routine_logging_enabled", False).value
        )
        

        # -------------------------------
        # Load SCT YAML
        # -------------------------------
        self.config_dir = self._resolve_config_dir()
        self.explicit_yaml_path = str(self.declare_parameter("supervisor_yaml_path", "").value).strip()
        self.current_mission = "explore"
        self.current_yaml_path = ""
        self._load_initial_sct()
        self._last_printed_sup_states: Optional[Tuple[int, ...]] = None

        # -------------------------------
        # State (sensing)
        # -------------------------------
        self.obstacle_zones = ["CLEAR"]
        self.perception_tokens = {"CLEAR"}
        self.last_zone_update = 0.0
        self.last_logged_zone = "CLEAR"
        self.green_logged_active = False
        self.green_center_offset: Optional[float] = None
        self.green_center_offset_time = 0.0
    

        # Odom / yaw tracking for full-rotate
        self.have_odom = False
        self.yaw = 0.0

        # -------------------------------
        # State (actuation)
        # -------------------------------
        self.stop_sent = False
        self.active_event: Optional[str] = None
        self.active_twist = Twist()
        self.motion_until = 0.0
        self.last_perception_signature: Optional[Tuple[Tuple[str, ...], Tuple[str, ...]]] = None
        self.last_selected_controllable_id: Optional[int] = None
        self.last_executed_controllable: Optional[str] = None

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

        # -------------------------------
        # Publishers/Subscribers
        # -------------------------------
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.sub_zone = self.create_subscription(String, "detected_zones", self.zone_callback, 10)
        self.sub_green_center_offset = self.create_subscription(
            Float32, "green_center_offset", self.green_center_offset_callback, 10
        )
        self.sub_odom = self.create_subscription(Odometry, "odom", self.odom_callback, 10)

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
            # "EV_random_walk": ActionSpec(linear_x=0.0, angular_z=0.0, hold_s=None),  # special handled below
            "EV_move_forward": ActionSpec(linear_x=0.2, angular_z=0.0),
            "EV_move_backward": ActionSpec(linear_x=-0.5, angular_z=0.0, hold_s=self.recovery_back_hold_s),
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
            "EV_go_to_green": ActionSpec(linear_x=0.16, angular_z=0.0),
            "EV_escape_purple": ActionSpec(linear_x=-0.2, angular_z=0.6, hold_s=self.recovery_back_hold_s),
            # full_rotate is executed as an atomic rotation using odom; target is full_rotate_target_rad (≤ π here).
            "EV_full_rotate": ActionSpec(linear_x=0.0, angular_z=self.full_rotate_omega, is_full_rotate=True),
        }

    def _routine_info(self, message: str):
        if self.routine_logging_enabled:
            self.get_logger().info(message)

    def _perception_signature(self) -> Tuple[Tuple[str, ...], Tuple[str, ...]]:
        return (tuple(sorted(self.perception_tokens)), tuple(self.obstacle_zones))

    def _log_triggered_events(self):
        event_ids = getattr(self.sct, "last_triggered_events", [])
        if not event_ids:
            return
        event_names = [
            self.ev_name_by_id.get(int(ev_id), f"EV_{int(ev_id)}")
            for ev_id in event_ids
        ]
        self.get_logger().info(f"Triggered events: {', '.join(event_names)}")

    def _canonical_mission_name(self, mission: str) -> str:
        key = str(mission or "").strip().lower().replace("-", "_").replace(" ", "_")
        if key != "explore":
            key = "explore"
        return key

    def _resolve_config_dir(self) -> str:
        try:
            return os.path.join(get_package_share_directory("leo_real"), "config")
        except PackageNotFoundError:
            # Fallback for running directly from the source tree before install.
            return os.path.join(os.path.dirname(os.path.dirname(__file__)), "config")

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
        self.sct = SCT(config_path)
        self.ev_name_by_id = {ev_id: ev_name for ev_name, ev_id in self.sct.EV.items()}
        self.enabled_color_tokens = set()
        if "EV_green_detected" in self.sct.EV:
            self.enabled_color_tokens.add("GREEN")
        if "EV_purple_detected" in self.sct.EV:
            self.enabled_color_tokens.add("purple")
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
            self._routine_info(
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
        self._routine_info(
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
        self._routine_info(
            f"Switched mission to '{mission_key}' using {os.path.basename(config_path)}"
        )
        return True, os.path.basename(config_path)

    def _print_current_state(self):
        states = tuple(int(s) for s in self.sct.sup_current_state)
        if states == self._last_printed_sup_states:
            return
        self._last_printed_sup_states = states

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
        safe_twist = Twist()
        safe_twist.linear.x = float(twist.linear.x)
        safe_twist.linear.y = float(twist.linear.y)
        safe_twist.linear.z = float(twist.linear.z)
        safe_twist.angular.x = float(twist.angular.x)
        safe_twist.angular.y = float(twist.angular.y)
        safe_twist.angular.z = float(twist.angular.z)
        self.cmd_pub.publish(safe_twist)

    # -------------------------------
    # Subscriptions
    # -------------------------------
    def odom_callback(self, msg: Odometry):
        self.have_odom = True
        self.yaw = _yaw_from_quat(msg.pose.pose.orientation)

    def green_center_offset_callback(self, msg: Float32):
        value = float(msg.data)
        if math.isfinite(value):
            self.green_center_offset = max(-1.0, min(1.0, value))
            self.green_center_offset_time = time.time()
        else:
            self.green_center_offset = None
            self.green_center_offset_time = 0.0

    def zone_callback(self, msg: String):
        now = time.time()
        if now - self.last_zone_update < self.zone_update_min_dt:
            return
        self.last_zone_update = now

        tokens = self._filter_zone_tokens_for_current_sct(self._parse_zone_tokens(msg.data))
        self.perception_tokens = tokens

        obstacle_tokens = tokens & {"LEFT", "RIGHT", "CORNER"}
        if obstacle_tokens:
            ordered_obstacles = [name for name in ("CORNER", "LEFT", "RIGHT") if name in obstacle_tokens]
            self.obstacle_zones = ordered_obstacles
        else:
            self.obstacle_zones = ["CLEAR"]

        green_active = "GREEN" in tokens
        if green_active and not self.green_logged_active:
            self.get_logger().info("Green detected")
        self.green_logged_active = green_active

   
    def _parse_zone_tokens(self, text: str) -> set[str]:
        aliases = {
            "FRONT": "CORNER",
            "MIDDLE": "CORNER",
            "OBSTACLE_FRONT": "CORNER",
            "OBSTACLE_LEFT": "LEFT",
            "OBSTACLE_RIGHT": "RIGHT",
            "GREEN_DETECTED": "GREEN",
            "GREEN_AREA": "GREEN",
            "PURPLE": "purple",
            "PURPLE_DETECTED": "purple",
            "PURPLE_AREA": "purple",
        }
        valid = {"CLEAR", "LEFT", "RIGHT", "CORNER", "GREEN", "purple"}
        raw_tokens = re.split(r"[\s,;/|]+", str(text).strip().upper())
        tokens = set()
        for raw in raw_tokens:
            if not raw:
                continue
            token = aliases.get(raw, raw)
            if token in valid:
                tokens.add(token)

        if not tokens:
            return {"CLEAR"}

        if tokens - {"CLEAR"}:
            tokens.discard("CLEAR")
        return tokens

    def _filter_zone_tokens_for_current_sct(self, tokens: set[str]) -> set[str]:
        filtered = set(tokens)
        for color in ("GREEN", "purple"):
            if color in filtered and color not in getattr(self, "enabled_color_tokens", set()):
                filtered.discard(color)
        if not filtered:
            return {"CLEAR"}
        if filtered - {"CLEAR"}:
            filtered.discard("CLEAR")
        return filtered

    # -------------------------------
    # SCT input check functions (uncontrollables)
    # -------------------------------
    def clear_path_check(self, sup_data):
        if "GREEN" in self.perception_tokens:
            return False
        depth_obstacles = {"LEFT", "RIGHT", "CORNER"}
        return not any(zone in depth_obstacles for zone in self.obstacle_zones)

    def middle_check(self, sup_data):
        hit = "CORNER" in self.obstacle_zones
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
        return "LEFT" in self.obstacle_zones

    def right_check(self, sup_data):
        return "RIGHT" in self.obstacle_zones

    def green_detected_check(self, sup_data):
        return "GREEN" in self.perception_tokens

    def purple_detected_check(self, sup_data):
        return "purple" in self.perception_tokens


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
        add("green_detected", self.green_detected_check)
        add("purple_detected", self.purple_detected_check)

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
        self._routine_info(
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
            self._routine_info("FULL ROTATE completed by timeout")
            return True

        if not self.have_odom or self.full_rotate_using_timed_fallback:
            # timed fallback uses motion_until
            done = now >= self.motion_until
            if done:
                self._routine_info("FULL ROTATE completed by timed fallback")
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
            self._routine_info(
                "FULL ROTATE switching to timed fallback "
                f"(accum_rad={self.full_rotate_accum:.3f}, remaining_rad={remaining:.3f}, "
                f"stall_count={self.full_rotate_stall_count}, fallback_s={dur:.3f})"
            )

        done = self.full_rotate_accum >= self.full_rotate_target_rad
        if done:
            self._routine_info(
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

    def _green_centering_angular_z(self) -> float:
        if self.green_center_offset is None:
            return 0.0
        if (time.time() - self.green_center_offset_time) > self.green_offset_timeout_s:
            return 0.0

        offset = self.green_center_offset
        if abs(offset) < self.green_centering_deadband:
            return 0.0

        angular_z = -self.green_centering_gain * offset
        return max(
            -self.green_centering_max_angular,
            min(self.green_centering_max_angular, angular_z),
        )

    def _green_follow_linear_x(self) -> float:
        if self.green_center_offset is None:
            return 0.0
        if (time.time() - self.green_center_offset_time) > self.green_offset_timeout_s:
            return 0.0
        if abs(self.green_center_offset) > self.green_follow_centered_deadband:
            return 0.0
        return max(0.0, self.green_follow_linear_x)

    def _hold_green_centered(self):
        twist = Twist()
        twist.linear.x = self._green_follow_linear_x()
        twist.angular.z = self._green_centering_angular_z()
        self.active_event = "EV_go_to_green"
        self.active_twist = twist
        self.motion_until = time.time() + self.motion_hold_duration
        self._publish_cmd(self.active_twist)
    
    

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
            twist.linear.x = self.rng.uniform(0.1, 0.2)
            twist.angular.z = self.rng.uniform(-1.0, 1.0)
            self.active_event = ev_name
            self.active_twist = twist
            self.motion_until = time.time() + self.motion_hold_duration
            self._publish_cmd(self.active_twist)
            return

        if ev_name in ("EV_rotate_clockwise", "EV_rotate_counterclockwise"):
            now = time.time()
            if (now - self.last_rotate_90_completed_at) < self.rotate_90_retrigger_block_s:
                self._routine_info("rotate_90 blocked by recent completion; stopping this tick")
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
                self._routine_info("full_rotate blocked by recent completion; stopping this tick")
                self.active_event = None
                self.motion_until = 0.0
                self._publish_stop()
                return
            # Cooldown: prevent repeated "scan in place" when we're stuck in obs_front
            if now < self.block_full_rotate_until:
                self._routine_info("full_rotate blocked by cooldown; stopping this tick")
                self.active_event = None
                self.motion_until = 0.0
                self._publish_stop()
                return
            self.active_event = ev_name
            self._start_full_rotate(spec.angular_z)
            return

        # Normal pulse action
        if ev_name == "EV_move_backward":
            if self.last_executed_controllable == "EV_move_backward":
                self.get_logger().info("EV_move_backward blocked because it was already executed last")
                self.active_event = None
                self.motion_until = 0.0
                self._publish_stop()
                return

        twist = Twist()
        twist.linear.x = float(spec.linear_x)
        twist.angular.z = float(spec.angular_z)
        if ev_name == "EV_go_to_green":
            twist.angular.z = self._green_centering_angular_z()

        hold = self.motion_hold_duration if spec.hold_s is None else float(spec.hold_s)

        self.active_event = ev_name
        self.active_twist = twist
        self.motion_until = time.time() + hold
        self.last_executed_controllable = ev_name
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
        

        # If we’re in the middle of a true full_rotate, keep executing until complete.
        if self.full_rotate_active:
            self._routine_info(
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
                self._routine_info(
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

        if "GREEN" in self.perception_tokens:
            self.get_logger().info("Triggered events: EV_green_detected, EV_go_to_green")
            self._hold_green_centered()
            return

        # Normal pulse-hold: keep publishing until hold expires
        if self.active_event and now < self.motion_until:
            self._publish_cmd(self.active_twist)
            return

        # Otherwise: pick next event from SCT
        self.active_event = None
        self.sct.input_buffer = []
        perception_signature = self._perception_signature()
        if perception_signature == self.last_perception_signature:
            self.sct.preferred_controllable_event = self.last_selected_controllable_id
        else:
            self.sct.preferred_controllable_event = None
        ce_exists, ce = self.sct.run_step()
        self._log_triggered_events()
        self._print_current_state()
        if not ce_exists:
            # No controllable enabled -> stop
            self._publish_stop()
            return

        ev_name = self.ev_name_by_id.get(int(ce))
        if ev_name is None:
            self._publish_stop()
            return

        self.last_perception_signature = perception_signature
        self.last_selected_controllable_id = int(ce)
        self._routine_info(f"Selected controllable event: {ev_name}")
        self.publish_twist_for_event(ev_name)


def main(args=None):
    rclpy.init(args=args)
    node = RobotSupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
