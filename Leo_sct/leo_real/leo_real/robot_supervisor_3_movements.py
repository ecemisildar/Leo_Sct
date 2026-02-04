import os
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2
import numpy as np
from leo_real.sct import SCT
from ament_index_python.packages import get_package_share_directory
import time
import math

from nav_msgs.msg import Odometry
# from math import atan2, sqrt, pi


class RobotSupervisor(Node):
    def __init__(self):
        super().__init__('robot_supervisor_3_movements')

        # Load SCT YAML
        config_path = os.path.join(
            get_package_share_directory('leo_real'),
            'config',
            'sup_gpt.yaml'
        )
        if not os.path.exists(config_path):
            self.get_logger().error(f"YAML file not found: {config_path}")
            rclpy.shutdown()
            return

        self.supervisor_period = self.declare_parameter(
            'supervisor_period', 0.5
        ).value
        self.zone_update_min_dt = self.declare_parameter(
            'zone_update_min_dt', self.supervisor_period
        ).value
        # Ensure zone throttling matches the cmd_vel timer cadence
        self.zone_update_min_dt = self.supervisor_period
        self.motion_hold_duration = self.declare_parameter(
            'motion_hold_duration', 0.2
        ).value
        self.marker_stop_distance = self.declare_parameter(
            'marker_stop_distance', 0.5
        ).value
        self.marker_distance_timeout = self.declare_parameter(
            'marker_distance_timeout', 0.6
        ).value
        self.marker_lost_timeout = self.declare_parameter(
            'marker_lost_timeout', 0.8
        ).value

        self.sct = SCT(config_path)
        self.target_offset = 0.0
        self.target_distance = float('inf')
        self.obstacle_zones = []
        self.enabled = self.declare_parameter('enabled', False).value
        self.stop_sent = False
        self.last_zone_update = 0.0
        self.motion_until = 0.0
        self.active_event = None
        self.active_twist = Twist()
        self.marker_seen = False
        self.marker_distance = float('inf')
        self.last_marker_distance_time = 0.0
        self.last_marker_lost_time = 0.0
        self.ns = self.get_namespace().strip('/') or 'root'
        base_seed = int(self.declare_parameter('random_seed', 12345).value)
        self.rng = random.Random(base_seed + self._namespace_index())

        # Ensure all events have a callback entry
        for ev_name, ev_id in self.sct.EV.items():
            if ev_id not in self.sct.callback:
                self.sct.callback[ev_id] = {"callback": None}

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber
        self.sub = self.create_subscription(String, 'detected_zones', self.zone_callback, 10)
        self.marker_seen_sub = self.create_subscription(Bool, 'marker_seen', self.marker_seen_callback, 10)
        self.marker_lost_sub = self.create_subscription(Bool, 'marker_lost', self.marker_lost_callback, 10)
        self.marker_distance_sub = self.create_subscription(Float32, 'marker_distance', self.marker_distance_callback, 10)


        # SCT callbacks for UCEs
        if 'EV_obstacle_front' in self.sct.EV:
            self.sct.add_callback(self.sct.EV['EV_obstacle_front'],None,self.middle_check,None) # EV_S0
        if 'EV_path_clear' in self.sct.EV:
            self.sct.add_callback(self.sct.EV['EV_path_clear'],None,self.clear_path_check,None) # EV_S1
        if 'EV_obstacle_left' in self.sct.EV:
            self.sct.add_callback(self.sct.EV['EV_obstacle_left'],None,self.left_check,None)    # EV_S2
        if 'EV_obstacle_right' in self.sct.EV:
            self.sct.add_callback(self.sct.EV['EV_obstacle_right'],None,self.right_check,None) # EV_S3
        if 'EV_marker_seen' in self.sct.EV:
            self.sct.add_callback(self.sct.EV['EV_marker_seen'],None,self.marker_check,None)
        if 'EV_marker_lost' in self.sct.EV:
            self.sct.add_callback(self.sct.EV['EV_marker_lost'],None,self.marker_lost_check,None)
        if 'EV_marker_close' in self.sct.EV:
            self.sct.add_callback(self.sct.EV['EV_marker_close'],None,self.marker_close_check,None)

        # Patch make_transition to log supervisor transitions
        original_make_transition = self.sct.make_transition
        def logged_make_transition(ev):
            ev_name = list(self.sct.EV.keys())[ev]
            # self.get_logger().info(f"--- Applying event: {ev_name} (index {ev}) ---")
            old_states = self.sct.sup_current_state.copy()
            original_make_transition(ev)

        self.sct.make_transition = logged_make_transition

        self.timer = self.create_timer(self.supervisor_period, self.timer_callback)
        self.enable_service = self.create_service(
            SetBool,
            'enable_supervisor',
            self.handle_enable_supervisor
        )


    def zone_callback(self, msg):
        now = time.time()
        if now - self.last_zone_update < self.zone_update_min_dt:
            return
        self.last_zone_update = now

        z = msg.data.strip().upper()
        # accept only known tokens
        if z in {"LEFT", "RIGHT", "CORNER", "CLEAR", "marker"}:
            self.obstacle_zones = [z]   # keep your existing list-based checks working
        else:
            self.obstacle_zones = ["CLEAR"]

    def marker_seen_callback(self, msg):
        seen = bool(msg.data)
        if seen and not self.marker_seen:
            self.get_logger().info("marker_seen event triggered")
        
        self.marker_seen = seen
        if seen:
            self.last_marker_lost_time = 0.0

    def marker_lost_callback(self, msg):
        if bool(msg.data):
            self.last_marker_lost_time = time.time()

    def marker_distance_callback(self, msg):
        if math.isfinite(msg.data):
            self.marker_distance = float(msg.data)
            self.last_marker_distance_time = time.time()
        else:
            self.marker_distance = float('inf')




    # -------------------------------
    # SCT input check functions
    # -------------------------------
    def clear_path_check(self, sup_data):
        depth_obstacles = {'LEFT', 'RIGHT', 'CORNER'}
        return not any(zone in depth_obstacles for zone in self.obstacle_zones)

    def middle_check(self, sup_data):
        return 'CORNER' in self.obstacle_zones
            
    def left_check(self, sup_data):
        return 'LEFT' in self.obstacle_zones
           
    def right_check(self, sup_data):
        return 'RIGHT' in self.obstacle_zones     

    def marker_check(self, sup_data):
        return self.marker_seen

    def marker_close_check(self, sup_data):
        if not self.marker_seen:
            return False
        if not math.isfinite(self.marker_distance):
            return False
        if time.time() - self.last_marker_distance_time > self.marker_distance_timeout:
            return False
        return self.marker_distance <= self.marker_stop_distance

    def marker_lost_check(self, sup_data):
        if self.marker_seen:
            return False
        if self.last_marker_lost_time <= 0.0:
            return False
        return (time.time() - self.last_marker_lost_time) <= self.marker_lost_timeout


    # -------------------------------
    # Robot motion
    # -------------------------------
    def publish_twist(self, ev_name: str):
        twist = Twist()

        if ev_name == "EV_random_walk": # EV_V1
            # self.get_logger().info("Supervisor decision: RANDOM WALK")
            twist.linear.x = self.rng.uniform(0.1, 0.2)
            twist.angular.z = self.rng.uniform(-1.0, 1.0)

        elif ev_name == "EV_full_rotate": # EV_V0
            # self.get_logger().info("Supervisor decision: FULL ROTATE")
            twist.linear.x = 0.0
            twist.angular.z = 2.0

        elif ev_name == "EV_rotate_clockwise": # EV_V2    
            # self.get_logger().info("Supervisor decision: CW")
            twist.linear.x = 0.0
            twist.angular.z = -0.5

        elif ev_name == "EV_rotate_counterclockwise":  # EV_V3  
            # self.get_logger().info("Supervisor decision: CCW")
            twist.linear.x = 0.0  
            twist.angular.z = 0.5
        
        elif ev_name == "EV_move_forward":   # EV_V4  
            # self.get_logger().info("Supervisor decision: FORWARD")
            twist.linear.x = 0.2
            twist.angular.z = 0.0  

        elif ev_name == "EV_move_backward":  # EV_V5   
            # self.get_logger().info("Supervisor decision: BACKWARD")
            twist.linear.x = -0.5
            twist.angular.z = 0.0  

        elif ev_name == "EV_move_to_marker":  # EV_V5   
            # self.get_logger().info("Supervisor decision: MOVE TO marker")
            twist.linear.x = 0.2
            twist.angular.z = 0.0         

        elif ev_name == "EV_stop":
            # self.get_logger().info("Supervisor decision: STOP")
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        else:
            # Unknown or no event -> stop for safety
            twist.linear.x = 0.0

        self.active_event = ev_name
        self.active_twist = twist
        self.motion_until = time.time() + self.motion_hold_duration
        self.cmd_pub.publish(self.active_twist)

    def handle_enable_supervisor(self, request, response):
        self.enabled = bool(request.data)
        self.stop_sent = False
        if not self.enabled:
            self.active_event = None
            self.motion_until = 0.0
            self.active_twist = Twist()
            self.cmd_pub.publish(self.active_twist)
            response.message = "Supervisor disabled."
        else:
            response.message = "Supervisor enabled."
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
    # Timer callback
    # -------------------------------
    def timer_callback(self):
        if not self.enabled:
            if not self.stop_sent:
                self.active_event = None
                self.motion_until = 0.0
                self.active_twist = Twist()
                self.cmd_pub.publish(self.active_twist)
                self.stop_sent = True
            return
        now = time.time()
        # Continue executing the current command until its hold time expires
        if self.active_event and now < self.motion_until:
            self.cmd_pub.publish(self.active_twist)
            return

        self.active_event = None
        # Run supervisor logic
        self.sct.input_buffer = []
        ce_exists, ce = self.sct.run_step()

        if ce_exists:
            event_name = [name for name, val in self.sct.EV.items() if val == ce][0]
            # self.get_logger().info(f"Controllable event chosen: {event_name}")
            self.publish_twist(event_name)

   

def main(args=None):
    rclpy.init(args=args)
    node = RobotSupervisor()
    rclpy.spin(node)

    # cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
