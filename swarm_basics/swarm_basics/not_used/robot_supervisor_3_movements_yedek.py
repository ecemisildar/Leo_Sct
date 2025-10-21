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
# from math import atan2, sqrt, pi


class RobotSupervisor(Node):
    def __init__(self):
        super().__init__('robot_supervisor_3_movements')

        # Load SCT YAML
        config_path = os.path.join(
            get_package_share_directory('swarm_basics'),
            'config',
            'supervisor2.yaml'
        )
        if not os.path.exists(config_path):
            self.get_logger().error(f"YAML file not found: {config_path}")
            rclpy.shutdown()
            return

        self.sct = SCT(config_path)
        self.bridge = CvBridge()
        self.obstacle = False
        self.min_front_distance = float("inf")
        self.obstacle_zones = []

        # Levy Walk
        self.levy_state = "choosing"
        self.levy_heading = 0.0
        self.levy_remaining = 0.0
        self.dt = 0.1  # loop period, adjust if you know your timer rate


        # Ensure all events have a callback entry
        for ev_name, ev_id in self.sct.EV.items():
            if ev_id not in self.sct.callback:
                self.sct.callback[ev_id] = {"callback": None}

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Depth camera subscriber
        # self.depth_sub = self.create_subscription(
        #     Image,
        #     "depth_camera/depth_image",
        #     self.depth_callback,
        #     10
        # )

        # SCT callbacks for UCEs
        self.sct.add_callback(self.sct.EV['EV_S0'],None,self.middle_check,None)
        self.sct.add_callback(self.sct.EV['EV_S1'],None,self.clear_path_check,None)
        self.sct.add_callback(self.sct.EV['EV_S2'],None,self.left_check,None)
        self.sct.add_callback(self.sct.EV['EV_S3'],None,self.right_check,None)
        self.sct.add_callback(self.sct.EV['EV_S4'],None,self.red_check,None)
        self.sct.add_callback(self.sct.EV['EV_S5'],None,self.blue_check,None)

        # Patch make_transition to log supervisor transitions
        original_make_transition = self.sct.make_transition
        def logged_make_transition(ev):
            ev_name = list(self.sct.EV.keys())[ev]
            # self.get_logger().info(f"--- Applying event: {ev_name} (index {ev}) ---")
            old_states = self.sct.sup_current_state.copy()
            original_make_transition(ev)

        self.sct.make_transition = logged_make_transition

        self.timer = self.create_timer(0.1, self.timer_callback)



    # Levy Step Function 
    def levy_step(self, mu=1.5, min_d=0.2, max_d=3.0):
        u = np.random.uniform(0,1)
        step = min_d * (1 - u)  ** (-1 / (mu - 1))
        return float(np.clip(step, min_d, max_d))

    # -------------------------------
    # Depth / obstacle
    # -------------------------------
    # def depth_callback(self, msg: Image):
    #     try:
    #         # Convert ROS Image to CV2
    #         color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    #         depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
    #         self.obstacle_zones = []

    #         height, width = depth_image.shape

    #         # --- Optional: crop bottom 10-20% to remove floor ---
    #         roi_bottom = int(height * 0.9)
    #         depth_image = depth_image[0:roi_bottom, :]
    #         color_image = color_image[0:roi_bottom, :]

    #         # --- 1. Split image into 3 main sections ---
    #         w1 = width // 2
    #         left_section = depth_image[:, 0:w1]
    #         right_section = depth_image[:, w1:width]

    #         # --- Helper: block-based min to detect small objects ---
    #         def find_section_min(section, block_size=4, percentile=5):
    #             min_vals = []
    #             h, w = section.shape
    #             for x in range(0, w, block_size):
    #                 block = section[:, x:x+block_size]
    #                 valid = block[(block > 0.1) & np.isfinite(block)]
    #                 if valid.size > 0:
    #                     # Use percentile for robustness
    #                     min_vals.append(np.percentile(valid, percentile))
    #             return min(min_vals) if min_vals else float("inf")

    #         # Compute min distances per section
    #         min_dist_left = find_section_min(left_section)
    #         min_dist_right = find_section_min(right_section)

    #         self.min_distances = {
    #             'left': min_dist_left,
    #             'right': min_dist_right
    #         }

    #         # --- 2. Determine closest obstacle zone with corner fallback ---
    #         OBSTACLE_THRESHOLD = 0.5  
    #         EPSILON = 0.05 # threshold for considering L ≈ R
    #         min_distance_overall = min(self.min_distances.values())

    #         if min_distance_overall < OBSTACLE_THRESHOLD:
    #             # Check if left and right are very close → obstacle in corner
    #             if abs(min_dist_left - min_dist_right) < EPSILON:
    #                 self.closest_obstacle_zone = "corner"
    #                 # Assign corner distance as the min of left/right for logging
    #                 self.min_distances['corner'] = min(min_dist_left, min_dist_right)
    #                 self.obstacle_zones.append("CORNER")
    #             else:
    #                 # Otherwise, pick the section with the absolute min distance
    #                 closest_zone = min(self.min_distances, key=self.min_distances.get)
    #                 self.closest_obstacle_zone = closest_zone.upper()
    #                 self.obstacle_zones.append(closest_zone.upper())


    #          # --- COLOR DETECTION (RED / BLUE) ---
    #         hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

    #         # Red mask (two hue ranges)
    #         lower_red1 = np.array([0, 100, 100])
    #         upper_red1 = np.array([10, 255, 255])
    #         lower_red2 = np.array([160, 100, 100])
    #         upper_red2 = np.array([179, 255, 255])
    #         red_mask = cv2.bitwise_or(
    #             cv2.inRange(hsv, lower_red1, upper_red1),
    #             cv2.inRange(hsv, lower_red2, upper_red2)
    #         )

    #         # Blue mask
    #         lower_blue = np.array([100, 150, 50])
    #         upper_blue = np.array([130, 255, 255])
    #         blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    #         def detect_color_objects(mask, color_name, bgr_color):
    #             contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #             for c in contours:
    #                 area = cv2.contourArea(c)
    #                 if area > 400:  # ignore small blobs
    #                     x, y, w, h = cv2.boundingRect(c)
    #                     cx, cy = x + w // 2, y + h // 2
    #                     if 0 <= cy < depth_image.shape[0] and 0 <= cx < depth_image.shape[1]:
    #                         depth_value = depth_image[cy, cx]
    #                     else:
    #                         depth_value = float("nan")

    #                     # Add detected color as a "zone" if close enough
    #                     COLOR_OBSTACLE_THRESHOLD = 1.0  # meters
    #                     if np.isfinite(depth_value) and depth_value < COLOR_OBSTACLE_THRESHOLD:
    #                         self.obstacle_zones.append(color_name.upper())

    #                     self.get_logger().info(
    #                         f"Detected {color_name} object at ({cx}, {cy}), depth={depth_value:.2f} m"
    #                     )

    #                     # Optional: draw debug overlays
    #                     cv2.rectangle(color_image, (x, y), (x + w, y + h), bgr_color, 2)
    #                     cv2.putText(color_image, f"{color_name} {depth_value:.2f}m",
    #                                 (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr_color, 1, cv2.LINE_AA)

    #         # Run detections
    #         detect_color_objects(red_mask, "RED", (0, 0, 255))
    #         detect_color_objects(blue_mask, "BLUE", (255, 0, 0))
    #             # self.get_logger().info(
    #             #     f"Dists: L:{min_dist_left:.2f}m | R:{min_dist_right:.2f}m. "
    #             #     f"Obstacles in: {', '.join(self.obstacle_zones) if self.obstacle_zones else 'NONE'}"
    #             # )

    #         # --- 3. Visualization ---
    #         # display_image = np.nan_to_num(depth_image, nan=0.0, posinf=10.0, neginf=0.0)
    #         # clipped_image = np.clip(display_image, 0.1, 10.0)
    #         # display_norm = cv2.normalize(clipped_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    #         # depth_colormap = cv2.applyColorMap(display_norm, cv2.COLORMAP_JET)

    #         # cv2.line(depth_colormap, (w1, 0), (w1, roi_bottom), (255, 255, 255), 2)
    #         # cv2.imshow("Depth Camera (2-Way Split)", depth_colormap)
    #         # cv2.waitKey(1)

    #     except Exception as e:
    #         self.get_logger().warn(f"Depth callback error: {e}")

    # -------------------------------
    # SCT input check functions
    # -------------------------------
    def clear_path_check(self, sup_data):
        # self.get_logger().info(f"Obstacle: {self.obstacle}")
        return not self.obstacle_zones

    def middle_check(self, sup_data):
        return 'CORNER' in self.obstacle_zones
            
    def left_check(self, sup_data):
        return 'LEFT' in self.obstacle_zones
           
    def right_check(self, sup_data):
        return 'RIGHT' in self.obstacle_zones

    def red_check(self, sup_data):
        return 'RED' in self.obstacle_zones

    def blue_check(self, sup_data):
        return 'BLUE' in self.obstacle_zones            


    # -------------------------------
    # Robot motion
    # -------------------------------
    def publish_twist(self, ev_name: str):
        twist = Twist()

        if ev_name == "EV_V1":     
            # self.get_logger().info("Supervisor decision: LEVY WALK")
            twist.linear.x = random.uniform(0.1, 1.0)
            twist.angular.z = random.uniform(-1.0, 1.0)

            # if self.levy_state == "choosing":
            #     self.levy_heading = random.uniform(-math.pi, math.pi)
            #     self.levy_remaining = self.levy_step()
            #     self.levy_state = "moving"

            # elif self.levy_state == "moving":
            #     twist.linear.x = 0.3
            #     twist.angular.z = 0.0

            #     # Decrease remaining distance based on travel
            #     self.levy_remaining -= twist.linear.x * self.dt

            #     # Occasionally inject small angular noise (to avoid straight-line lock)
            #     twist.angular.z = random.uniform(-0.05, 0.05)

            #     # If reached target or obstacle triggers another event, reset
            #     if self.levy_remaining <= 0:
            #         self.levy_state = "choosing"    
                
                
        elif ev_name == "EV_V0":       
            # self.get_logger().info("Supervisor decision: CORNER")
            twist.linear.x = 0.0
            twist.angular.z = 1.0
            time.sleep(0.3) 

        elif ev_name == "EV_V2":     
            # self.get_logger().info("Supervisor decision: CW")
            twist.linear.x = 0.0
            twist.angular.z = -0.5

        elif ev_name == "EV_V3":    
            # self.get_logger().info("Supervisor decision: CCW")
            twist.linear.x = 0.0  
            twist.angular.z = 0.5

        elif ev_name == "EV_V4":    
            # self.get_logger().info("Supervisor decision: GO")
            twist.linear.x = 0.0  
            twist.angular.z = 

        elif ev_name == "EV_V5":    
            # self.get_logger().info("Supervisor decision: TURN BACK")
            twist.linear.x = 0.0  
            twist.angular.z =         

        else:
            # Unknown or no event -> stop for safety
            twist.linear.x = 0.0
            # self.get_logger().debug(f"Unknown or None event: {ev_name}")

        self.cmd_pub.publish(twist)


    # -------------------------------
    # Timer callback
    # -------------------------------
    def timer_callback(self):
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
