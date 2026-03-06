import os
import random
import math
import time
from glob import glob
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry

from ament_index_python.packages import get_package_share_directory
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
        super().__init__("robot_supervisor")

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

        # Obstacle-front escape bias: block full_rotate briefly after obstacle_front
        self.full_rotate_block_after_obs_front = float(
            self.declare_parameter("full_rotate_block_after_obs_front", 0.8).value
        )  # seconds
        self.block_full_rotate_until = 0.0
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




        # Throttle zone updates to match supervisor cadence
        self.zone_update_min_dt = self.supervisor_period

        # Random seed (per-robot namespace)
        self.ns = self.get_namespace().strip("/") or "root"
        base_seed = int(self.declare_parameter("random_seed", 12345).value)
        self.rng = random.Random(base_seed + self._namespace_index())

        self.enabled = bool(self.declare_parameter("enabled", False).value)
        self.static_mode = bool(self.declare_parameter("static", False).value)
        self.aruco_follow_enabled = True
        self.aruco_follow_linear_x = 0.15
        self.aruco_follow_angular_z = 0.6
        self.aruco_stop_distance_m = 0.3
        self.aruco_hold_timeout_s = 1.0
        self.override_mode = str(self.declare_parameter("override_mode", "auto").value).strip().lower()
        if self.override_mode not in {"auto", "stop", "forward"}:
            self.get_logger().warn(
                f"Invalid override_mode='{self.override_mode}', falling back to 'auto'."
            )
            self.override_mode = "auto"

        # -------------------------------
        # Load SCT YAML
        # -------------------------------
        self.config_dir = os.path.join(get_package_share_directory("leo_real"), "config")
        self.current_mission = "explore"
        self.current_yaml_path = ""
        self._load_initial_sct()
        self._last_printed_sup_states: Optional[Tuple[int, ...]] = None

        # -------------------------------
        # State (sensing)
        # -------------------------------
        self.obstacle_zones = ["CLEAR"]
        self.last_zone_update = 0.0
        self.aruco_detected = False
        self.aruco_direction = "NONE"
        self.aruco_distance_m = float("inf")
        self.last_aruco_seen_time = 0.0

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

        # Full-rotate mode bookkeeping
        self.full_rotate_active = False
        self.full_rotate_start_yaw = 0.0
        self.full_rotate_accum = 0.0
        self.full_rotate_started_at = 0.0
        self.full_rotate_prev_yaw = 0.0

        # -------------------------------
        # Publishers/Subscribers
        # -------------------------------
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.sub_zone = self.create_subscription(String, "detected_zones", self.zone_callback, 10)
        self.sub_aruco_detected = self.create_subscription(Bool, "aruco_id1_detected", self.aruco_detected_callback, 10)
        self.sub_aruco_direction = self.create_subscription(String, "aruco_id1_direction", self.aruco_direction_callback, 10)
        self.sub_aruco_distance = self.create_subscription(Float32, "aruco_id1_distance", self.aruco_distance_callback, 10)
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
        self.override_stop_service = self.create_service(
            SetBool,
            "override_stop",
            self.handle_override_stop,
        )
        self.override_forward_service = self.create_service(
            SetBool,
            "override_forward",
            self.handle_override_forward,
        )

        # -------------------------------
        # Action table (data-driven)
        # -------------------------------
        # Only place you encode motion parameters. No “logic” needs event names elsewhere.
        self.action_table: Dict[str, ActionSpec] = {
            "EV_random_walk": ActionSpec(linear_x=0.0, angular_z=0.0, hold_s=None),  # special handled below
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
        self.sct = SCT(config_path)
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
        print(
            f"[robot_supervisor] mission={self.current_mission} current_state={states}",
            flush=True,
        )

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
        self.cmd_pub.publish(twist)

    # -------------------------------
    # Subscriptions
    # -------------------------------
    def odom_callback(self, msg: Odometry):
        self.have_odom = True
        self.yaw = _yaw_from_quat(msg.pose.pose.orientation)

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

    def aruco_detected_callback(self, msg: Bool):
        self.aruco_detected = bool(msg.data)
        if self.aruco_detected:
            self.last_aruco_seen_time = time.time()
        else:
            self.aruco_direction = "NONE"

    def aruco_direction_callback(self, msg: String):
        direction = msg.data.strip().upper()
        if direction in {"LEFT", "CENTER", "RIGHT", "NONE"}:
            self.aruco_direction = direction
        else:
            self.aruco_direction = "NONE"

    def aruco_distance_callback(self, msg: Float32):
        d = float(msg.data)
        if math.isfinite(d):
            self.aruco_distance_m = d
        else:
            self.aruco_distance_m = float("inf")

    # -------------------------------
    # SCT input check functions (uncontrollables)
    # -------------------------------
    def clear_path_check(self, sup_data):
        depth_obstacles = {"LEFT", "RIGHT", "CORNER"}
        return not any(zone in depth_obstacles for zone in self.obstacle_zones)

    def middle_check(self, sup_data):
        hit = "CORNER" in self.obstacle_zones
        if hit:
            now = time.time()
            # If we're in a front-obstacle situation, block full_rotate for ~1 tick
            self.block_full_rotate_until = max(
                self.block_full_rotate_until,
                now + self.full_rotate_block_after_obs_front,
            )
        return hit

    def left_check(self, sup_data):
        return "LEFT" in self.obstacle_zones

    def right_check(self, sup_data):
        return "RIGHT" in self.obstacle_zones

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

    def _set_override_mode(self, mode: str):
        mode = str(mode).strip().lower()
        if mode not in {"auto", "stop", "forward"}:
            mode = "auto"
        if mode != self.override_mode:
            self.override_mode = mode
            self._cancel_all_motion()
            self.stop_sent = False
            self.get_logger().info(f"Override mode set to '{self.override_mode}'")

    def handle_override_stop(self, request, response):
        if bool(request.data):
            self._set_override_mode("stop")
            response.message = "Override enabled: stop."
        else:
            if self.override_mode == "stop":
                self._set_override_mode("auto")
            response.message = "Stop override disabled."
        response.success = True
        return response

    def handle_override_forward(self, request, response):
        if bool(request.data):
            self._set_override_mode("forward")
            response.message = "Override enabled: forward."
        else:
            if self.override_mode == "forward":
                self._set_override_mode("auto")
            response.message = "Forward override disabled."
        response.success = True
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

    def _publish_forward_override(self):
        spec = self.action_table.get("EV_move_forward")
        twist = Twist()
        if spec is not None:
            twist.linear.x = float(spec.linear_x)
            twist.angular.z = float(spec.angular_z)
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        self.active_event = "EV_move_forward"
        self.active_twist = twist
        self.motion_until = 0.0
        self._publish_cmd(self.active_twist)

    def _publish_aruco_follow_cmd(self):
        twist = Twist()
        if self.aruco_direction == "LEFT":
            twist.angular.z = abs(self.aruco_follow_angular_z)
        elif self.aruco_direction == "RIGHT":
            twist.angular.z = -abs(self.aruco_follow_angular_z)
        elif self.aruco_direction == "CENTER":
            twist.linear.x = max(0.0, self.aruco_follow_linear_x)
        else:
            # Detection true but direction unknown -> stop for safety.
            twist = Twist()
        self.active_event = None
        self.active_twist = twist
        self.motion_until = 0.0
        self._publish_cmd(self.active_twist)

    def _aruco_control_active(self) -> bool:
        if self.aruco_detected:
            return True
        if self.last_aruco_seen_time <= 0.0:
            return False
        return (time.time() - self.last_aruco_seen_time) <= self.aruco_hold_timeout_s

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

    def _start_full_rotate(self, omega: float):
        # Start a 180° rotation that persists beyond the 0.2s pulse.
        self.full_rotate_active = True
        self.full_rotate_started_at = time.time()
        self.full_rotate_accum = 0.0

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
        self._publish_cmd(self.active_twist)

    def _update_full_rotate(self) -> bool:
        """
        Returns True if full rotate completed, False otherwise.
        """
        now = time.time()
        if (now - self.full_rotate_started_at) > self.full_rotate_timeout_s:
            # timeout safety
            return True

        if not self.have_odom:
            # timed fallback uses motion_until
            return now >= self.motion_until

        # integrate yaw delta each tick (and keep publishing until done)
        dy = _wrap_to_pi(self.yaw - self.full_rotate_prev_yaw)
        self.full_rotate_accum += abs(dy)
        self.full_rotate_prev_yaw = self.yaw

        return self.full_rotate_accum >= self.full_rotate_target_rad

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
            twist.linear.x = self.rng.uniform(0.1, 0.2)
            twist.angular.z = self.rng.uniform(-1.0, 1.0)
            self.active_event = ev_name
            self.active_twist = twist
            self.motion_until = time.time() + self.motion_hold_duration
            self._publish_cmd(self.active_twist)
            return

        if ev_name in ("EV_rotate_clockwise", "EV_rotate_counterclockwise"):
            self.active_event = ev_name
            self._start_rotate_90(spec.angular_z)
            return

        if spec.is_full_rotate:
            # Cooldown: prevent repeated "scan in place" when we're stuck in obs_front
            if time.time() < self.block_full_rotate_until:
                self.get_logger().info("full_rotate blocked by cooldown; publishing stop this tick")
                self.active_event = "EV_rotate_clockwise"
                self._start_rotate_90(-self.rotate_90_omega)
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

        # Manual override states supersede SCT choices.
        if self.override_mode == "stop":
            self._cancel_all_motion()
            self._publish_stop()
            return
        if self.override_mode == "forward":
            self._cancel_all_motion()
            self._publish_forward_override()
            return

        if self.aruco_follow_enabled and self._aruco_control_active():
            self._cancel_all_motion()
            if self.aruco_distance_m <= self.aruco_stop_distance_m:
                self._publish_stop()
                return
            if self.aruco_detected:
                self._publish_aruco_follow_cmd()
                return
            # Hold control briefly when marker flickers to avoid SCT takeover oscillation.
            self._publish_stop()
            return

        # If we’re in the middle of a true full_rotate, keep executing until complete.
        if self.full_rotate_active:
            self._publish_cmd(self.active_twist)
            if self._update_full_rotate():
                # stop rotation and resume supervisor next tick
                self.full_rotate_active = False
                self.active_event = None
                self.motion_until = 0.0
                self._publish_stop()
            return

        # If doing a 90-degree rotate
        if self.rotate_90_active:
            self._publish_cmd(self.active_twist)
            if self._update_rotate_90():
                self.rotate_90_active = False
                self.active_event = None
                self.motion_until = 0.0
                self._publish_stop()
            return
            

        # Normal pulse-hold: keep publishing until hold expires
        if self.active_event and now < self.motion_until:
            self._publish_cmd(self.active_twist)
            return

        # Otherwise: pick next event from SCT
        self.active_event = None
        self.sct.input_buffer = []
        ce_exists, ce = self.sct.run_step()
        self._print_current_state()
        if not ce_exists:
            # No controllable enabled -> stop
            self._publish_stop()
            return

        ev_name = self.ev_name_by_id.get(int(ce))
        if ev_name is None:
            self._publish_stop()
            return

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
