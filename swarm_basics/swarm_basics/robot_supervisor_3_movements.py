import os
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from swarm_basics.sct import SCT
from ament_index_python.packages import get_package_share_directory


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

        # Ensure all events have a callback entry
        for ev_name, ev_id in self.sct.EV.items():
            if ev_id not in self.sct.callback:
                self.sct.callback[ev_id] = {"callback": None}

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Depth camera subscriber
        self.depth_sub = self.create_subscription(
            Image,
            "depth_camera/depth_image",
            self.depth_callback,
            10
        )

        # SCT callbacks for UCEs
        self.sct.add_callback(
            self.sct.EV['EV_S0'],
            None,
            self.middle_check,
            None
        )
        self.sct.add_callback(
            self.sct.EV['EV_S1'],
            None,
            self.clear_path_check,
            None
        )

        self.sct.add_callback(
            self.sct.EV['EV_S2'],
            None,
            self.left_check,
            None
        )
        self.sct.add_callback(
            self.sct.EV['EV_S3'],
            None,
            self.right_check,
            None
        )

        # Patch make_transition to log supervisor transitions
        original_make_transition = self.sct.make_transition
        def logged_make_transition(ev):
            ev_name = list(self.sct.EV.keys())[ev]
            # self.get_logger().info(f"--- Applying event: {ev_name} (index {ev}) ---")
            old_states = self.sct.sup_current_state.copy()
            original_make_transition(ev)
            # for i, (old, new) in enumerate(zip(old_states, self.sct.sup_current_state)):
                # if old != new:
                #     # self.get_logger().info(f"Supervisor {i} transitioned: {old} -> {new}")
                # else:
                    # self.get_logger().info(f"Supervisor {i} stayed at state: {new}")
        self.sct.make_transition = logged_make_transition

        self.timer = self.create_timer(0.1, self.timer_callback)

        
    # -------------------------------
    # Depth / obstacle
    # -------------------------------
    def depth_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            
            self.obstacle_zones = []
            # --- 1. SEGMENTATION AND DISTANCE CALCULATION ---
            
            width = cv_image.shape[1] # 640
            
            # Define the column boundaries for 3 equal parts
            w1 = width // 3  # ~213
            w2 = 2 * w1      # ~426
            
            # 1. Front-Left Section (Columns 0 to w1-1)
            left_section = cv_image[:, 0:w1]
            
            # 2. Front Section (Columns w1 to w2-1)
            center_section = cv_image[:, w1:w2]
            
            # 3. Front-Right Section (Columns w2 to 639)
            right_section = cv_image[:, w2:width]
            
            # Function to find the minimum *valid* distance in a section
            def find_min_distance(section):
                # Filter out 0 (near clipping), NaN, and Inf (far clipping)
                valid = section[(section > 0.1) & (np.isfinite(section))]
                return float(np.nanmax(valid)) if valid.size > 0 else float("inf")

            # Calculate minimum distance for each section
            min_dist_left = find_min_distance(left_section)
            min_dist_center = find_min_distance(center_section)
            min_dist_right = find_min_distance(right_section)
            
            # Store results for logic/planning node
            self.min_distances = {
                'left': min_dist_left,
                'center': min_dist_center,
                'right': min_dist_right
            }

            # Determine which zone has an obstacle (e.g., closer than 0.65m)
            OBSTACLE_THRESHOLD = 2.0
            
            
            # 1. Find the overall minimum distance
            all_distances = self.min_distances
            min_distance_overall = min(all_distances.values())
            
            # 2. Check if the overall closest object is actually an "obstacle"
            if min_distance_overall < OBSTACLE_THRESHOLD:
                
                # 3. Determine which zone holds this overall minimum distance
                if min_distance_overall == min_dist_center:
                    # Center is the closest/highest priority
                    self.closest_obstacle_zone = 'CENTER'
                    self.obstacle_zones.append("CENTER") # For logging/general check
                elif min_distance_overall == min_dist_left:
                    # Left is the closest
                    self.closest_obstacle_zone = 'LEFT'
                    self.obstacle_zones.append("LEFT")
                elif min_distance_overall == min_dist_right:
                    # Right is the closest
                    self.closest_obstacle_zone = 'RIGHT'
                    self.obstacle_zones.append("RIGHT")

            self.get_logger().info(
                f"Dists: L:{min_dist_left:.2f}m | C:{min_dist_center:.2f}m | R:{min_dist_right:.2f}m. Obstacles in: {', '.join(self.obstacle_zones) if self.obstacle_zones else 'NONE'}"
            )

            # --- 2. VISUALIZATION (Adding section lines) ---
            
            # Normalization and color mapping (using 10m max as before)
            display_image = np.nan_to_num(cv_image, nan=0.0, posinf=10.0, neginf=0.0)
            clipped_image = np.clip(display_image, 0.1, 10.0)
            display_norm = cv2.normalize(clipped_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_colormap = cv2.applyColorMap(display_norm, cv2.COLORMAP_JET)
            
            # Draw section lines
            cv2.line(depth_colormap, (w1, 0), (w1, 480), (255, 255, 255), 2) # White line for Left/Center split
            cv2.line(depth_colormap, (w2, 0), (w2, 480), (255, 255, 255), 2) # White line for Center/Right split
            
            cv2.imshow("Depth Camera (3-Way Split)", depth_colormap)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().warn(f"Depth callback error: {e}")

    # -------------------------------
    # SCT input check functions
    # -------------------------------
    def clear_path_check(self, sup_data):
        # self.get_logger().info(f"Obstacle: {self.obstacle}")
        return not self.obstacle_zones

    def middle_check(self, sup_data):
        return 'CENTER' in self.obstacle_zones
            
    def left_check(self, sup_data):
        return 'LEFT' in self.obstacle_zones
           
    def right_check(self, sup_data):
        return 'RIGHT' in self.obstacle_zones
            
    
       # -------------------------------
    # Robot motion
    # -------------------------------
    def publish_twist(self, ev_name: str):
        twist = Twist()

        if ev_name == "EV_V0":       
            self.get_logger().info("Supervisor decision: STOP")
            twist.linear.x = 0.1
            twist.angular.z = 0.3

        elif ev_name == "EV_V1":     
            self.get_logger().info("Supervisor decision: FORWARD")
            twist.linear.x = 0.5

        elif ev_name == "EV_V2":     
            self.get_logger().info("Supervisor decision: CW")
            twist.linear.x = 0.1
            twist.angular.z = -0.5

        elif ev_name == "EV_V3":    
            self.get_logger().info("Supervisor decision: CCW")
            twist.linear.x = 0.1  
            twist.angular.z = 0.5

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
        else:
            # No controllable event enabled -> stop
            # self.get_logger().info("No controllable event, stopping.")
            self.publish_twist("EV_V0")

        


def main(args=None):
    rclpy.init(args=args)
    node = RobotSupervisor()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
