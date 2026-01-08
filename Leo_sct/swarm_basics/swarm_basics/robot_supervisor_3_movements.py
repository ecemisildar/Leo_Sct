import os
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from swarm_basics.sct import SCT
from ament_index_python.packages import get_package_share_directory
import time, random, math

from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
# from math import atan2, sqrt, pi


class RobotSupervisor(Node):
    def __init__(self):
        super().__init__('robot_supervisor_3_movements')

        # Load SCT YAML
        config_path = os.path.join(
            get_package_share_directory('swarm_basics'),
            'config',
            'sup_gpt.yaml'
        )
        if not os.path.exists(config_path):
            self.get_logger().error(f"YAML file not found: {config_path}")
            rclpy.shutdown()
            return

        self.supervisor_period = self.declare_parameter(
            'supervisor_period', 0.3
        ).value
        self.zone_update_min_dt = self.declare_parameter(
            'zone_update_min_dt', self.supervisor_period
        ).value
        # Ensure zone throttling matches the cmd_vel timer cadence
        self.zone_update_min_dt = self.supervisor_period
        self.motion_hold_duration = self.declare_parameter(
            'motion_hold_duration', 0.6
        ).value
        self.neighbor_timeout = self.declare_parameter(
            'neighbor_timeout', 1.0
        ).value
        self.separation_radius = self.declare_parameter(
            'separation_radius', 1.2
        ).value
        self.separation_gain = self.declare_parameter(
            'separation_gain', 0.8
        ).value

        self.sct = SCT(config_path)
        self.target_offset = 0.0
        self.target_distance = float('inf')
        self.obstacle_zones = []
        self.last_zone_update = 0.0
        self.motion_until = 0.0
        self.active_event = None
        self.active_twist = Twist()
        self.ns = self.get_namespace().strip('/') or 'root'
        self.self_pose = None  # (x, y, yaw, t)
        self.neighbors = {}    # ns -> (x, y, t)

        # Ensure all events have a callback entry
        for ev_name, ev_id in self.sct.EV.items():
            if ev_id not in self.sct.callback:
                self.sct.callback[ev_id] = {"callback": None}

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber
        self.sub = self.create_subscription(String, 'detected_zones', self.zone_callback, 10)
        self.create_subscription(
            TFMessage,
            '/world/random_world/dynamic_pose/info',
            self.tf_callback,
            10,
        )


        # SCT callbacks for UCEs
        self.sct.add_callback(self.sct.EV['EV_obstacle_front'],None,self.middle_check,None) # EV_S0
        self.sct.add_callback(self.sct.EV['EV_path_clear'],None,self.clear_path_check,None) # EV_S1
        self.sct.add_callback(self.sct.EV['EV_obstacle_left'],None,self.left_check,None)    # EV_S2
        self.sct.add_callback(self.sct.EV['EV_obstacle_right'],None,self.right_check,None) # EV_S3
        # self.sct.add_callback(self.sct.EV['EV_sense_crowd'],None,self.crowd_check,None) # EV_S4

        # self.sct.add_callback(self.sct.EV['EV_S4'],None,self.red_check,None)
        # self.sct.add_callback(self.sct.EV['EV_S5'],None,self.blue_check,None)

        # Patch make_transition to log supervisor transitions
        original_make_transition = self.sct.make_transition
        def logged_make_transition(ev):
            ev_name = list(self.sct.EV.keys())[ev]
            # self.get_logger().info(f"--- Applying event: {ev_name} (index {ev}) ---")
            old_states = self.sct.sup_current_state.copy()
            original_make_transition(ev)

        self.sct.make_transition = logged_make_transition

        self.timer = self.create_timer(self.supervisor_period, self.timer_callback)


    def zone_callback(self, msg):
        now = time.time()
        if now - self.last_zone_update < self.zone_update_min_dt:
            return
        self.last_zone_update = now
        self.obstacle_zones = [z.strip() for z in msg.data.split(',') if z.strip()]
        self.get_logger().info(f'ZONE: {self.obstacle_zones}')

        for z in self.obstacle_zones:
            parts = z.split(",")
            if len(parts) == 3:
                try:
                    self.target_offset = float(parts[1])
                    self.target_distance = float(parts[2])
                    break
                except ValueError:
                    continue



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

    # def crowd_check(self, sup_data):
    #     sensed = random.random() < 0.05
    #     self.get_logger().info(f"crowd_sense invoked -> {'trigger' if sensed else 'idle'}")
    #     return sensed


    # def red_check(self, sup_data):
    #     return 'RED' in self.obstacle_zones

    # def blue_check(self, sup_data):
    #     return 'BLUE' in self.obstacle_zones            


    # -------------------------------
    # Robot motion
    # -------------------------------
    def publish_twist(self, ev_name: str):
        twist = Twist()

        # if ev_name == "EV_random_walk": # EV_V1      
        #     self.get_logger().info("Supervisor decision: RANDOM WALK")
        #     twist.linear.x = random.uniform(0.1, 1.0)
        #     twist.angular.z = random.uniform(-1.0, 1.0)
                
        if ev_name == "EV_full_rotate": # EV_V0      
            self.get_logger().info("Supervisor decision: FULL ROTATE")
            twist.linear.x = 0.0
            twist.angular.z = 2.0
            time.sleep(0.1) 

        elif ev_name == "EV_rotate_clockwise": # EV_V2    
            self.get_logger().info("Supervisor decision: CW")
            twist.linear.x = 0.0
            twist.angular.z = -0.5

        elif ev_name == "EV_rotate_counterclockwise":  # EV_V3  
            self.get_logger().info("Supervisor decision: CCW")
            twist.linear.x = 0.0  
            twist.angular.z = 0.5
        
        elif ev_name == "EV_move_forward":   # EV_V4  
            self.get_logger().info("Supervisor decision: FORWARD")
            twist.linear.x = 0.5
            twist.angular.z = 0.0  

        elif ev_name == "EV_move_backward":  # EV_V5   
            self.get_logger().info("Supervisor decision: BACKWARD")
            twist.linear.x = -0.5
            twist.angular.z = 0.0      

        # elif ev_name == "EV_slow_down":  # EV_V6   
        #     self.get_logger().info("Supervisor decision: SLOW DOWN")
        #     twist.linear.x = 0.1
        #     twist.angular.z = 0.0     

        # elif ev_name == "EV_speed_up":  # EV_V7   
        #     self.get_logger().info("Supervisor decision: SPEED UP")
        #     twist.linear.x = 1.0
        #     twist.angular.z = 0.0           


        # elif ev_name == "EV_V4": # RED
        #     if self.target_distance > 1.0 and self.target_distance < 0.5:
        #         twist.linear.x = 0.2
        #         twist.angular.z = -1.0 * self.target_offset
        #         self.get_logger().info(f"Going toward: offset={self.target_offset:.2f}, dist={self.target_distance:.2f}")

        # elif ev_name == "EV_V5": # BLUE  
        #         twist.linear.x = 0.0
        #         twist.angular.z = -1.0 * abs(self.target_offset)
        #         self.get_logger().info(f"Escaping: offset={self.target_offset:.2f}, dist={self.target_distance:.2f}")
           
        else:
            # Unknown or no event -> stop for safety
            twist.linear.x = 0.0
            # self.get_logger().debug(f"Unknown or None event: {ev_name}")

        self.active_event = ev_name
        self.active_twist = twist
        self.motion_until = time.time() + self.motion_hold_duration
        self.cmd_pub.publish(self.active_twist)


    # -------------------------------
    # Timer callback
    # -------------------------------
    def timer_callback(self):
        # Continue executing the current command until its hold time expires
        now = time.time()
        if self.active_event and now < self.motion_until:
            self.cmd_pub.publish(self.active_twist)
            return

        # Neighbor avoidance using shared TF (no extra sensors)
        if self.avoid_neighbors():
            return

        self.active_event = None
        # Run supervisor logic
        self.sct.input_buffer = []
        ce_exists, ce = self.sct.run_step()

        if ce_exists:
            event_name = [name for name, val in self.sct.EV.items() if val == ce][0]
            # self.get_logger().info(f"Controllable event chosen: {event_name}")
            self.publish_twist(event_name)

    def tf_callback(self, msg: TFMessage):
        """Track self and neighbor poses from the global TF stream."""
        now = time.time()
        for t in msg.transforms:
            name = t.child_frame_id
            if not name.startswith('robot_') or '/' in name:
                continue

            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

            if name == self.ns:
                self.self_pose = (x, y, yaw, now)
            else:
                self.neighbors[name] = (x, y, now)

    def avoid_neighbors(self) -> bool:
        """Steer or slow down when another robot is too close."""
        if not self.self_pose:
            return False

        now = time.time()
        # Drop stale neighbor data
        self.neighbors = {
            k: v for k, v in self.neighbors.items() if now - v[2] <= self.neighbor_timeout
        }
        if not self.neighbors:
            return False

        sx, sy, syaw, _ = self.self_pose
        closest = None
        closest_d = float('inf')
        for ns, (nx, ny, _) in self.neighbors.items():
            d = math.hypot(nx - sx, ny - sy)
            if d < closest_d:
                closest_d = d
                closest = (ns, nx, ny)

        if closest is None or closest_d > self.separation_radius:
            return False

        _, nx, ny = closest
        angle_to_neighbor = math.atan2(ny - sy, nx - sx)
        heading_error = math.atan2(
            math.sin(angle_to_neighbor - syaw),
            math.cos(angle_to_neighbor - syaw)
        )

        twist = Twist()
        # Turn away from the neighbor; back up if very close
        twist.angular.z = -self.separation_gain * math.copysign(1.0, heading_error)
        twist.linear.x = -0.2 if closest_d < (self.separation_radius * 0.7) else 0.0

        self.active_event = "EV_neighbor_avoid"
        self.active_twist = twist
        self.motion_until = now + self.motion_hold_duration
        self.cmd_pub.publish(self.active_twist)
        return True
   

def main(args=None):
    rclpy.init(args=args)
    node = RobotSupervisor()
    rclpy.spin(node)

    # cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
