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
        super().__init__('robot_supervisor')

        # Load SCT YAML
        config_path = os.path.join(
            get_package_share_directory('swarm_basics'),
            'config',
            'supervisor.yaml'
        )
        if not os.path.exists(config_path):
            self.get_logger().error(f"YAML file not found: {config_path}")
            rclpy.shutdown()
            return

        self.sct = SCT(config_path)
        self.bridge = CvBridge()
        self.obstacle = False
        self.min_front_distance = float("inf")
        self.sim_started = False

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
            self.obstacle_check,
            None
        )
        self.sct.add_callback(
            self.sct.EV['EV_S1'],
            None,
            self.clear_path_check,
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
            # Convert ROS Image message to OpenCV image
            # '32FC1' is critical here, indicating the data is 32-bit floats (meters)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            
            # --- 1. CALCULATE DISTANCES ---
            
            # Select the center column of the depth image
            center_col = cv_image[:, cv_image.shape[1] // 2]
            
            # Filter for valid, finite depth measurements (removes 0, NaN, and Inf)
            # We also need to filter out very small values (close to 0) which can be noise/padding
            valid_distances = center_col[(center_col > 0.0) & (np.isfinite(center_col))]
            
            # Calculate minimum distance from the valid set
            # min_front_distance will be 'inf' if no valid data is found (e.g., aiming at an open sky)
            # self.min_front_distance = float(np.nanmin(valid_distances)) if valid_distances.size > 0 else float("inf")
            
            # Calculate maximum distance from the valid set (to check for the 10m range)
            self.max_front_distance = float(np.nanmax(valid_distances)) if valid_distances.size > 0 else float("inf")

            # Set obstacle flag based on a configurable threshold (e.g., 0.65m)
            self.obstacle = self.max_front_distance < 1.0
            
            self.get_logger().info(
                f"Min dist (SDF check): {self.max_front_distance:.3f}m | Obstacle: {self.obstacle}"
            )

            # --- 2. VISUALIZATION ---
            
            # Replace non-finite values (NaN, Inf) with 0.0 for visualization
            display_image = np.nan_to_num(cv_image, nan=0.0, posinf=0.0, neginf=0.0)
            
            # **Crucial Improvement:** Normalize the image to a known range (0 to 10 meters)
            # This makes the visualization consistent and the color map meaningful.
            NORM_MIN = 0.1  # Near clip plane
            NORM_MAX = 10.0 # Far clip plane (based on your SDF setting)
            
            # Clip values to the normalization range to avoid extreme colors
            clipped_image = np.clip(display_image, NORM_MIN, NORM_MAX)
            
            # Normalize the clipped, finite image to 0-255 scale
            display_norm = cv2.normalize(clipped_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # Apply a color map
            depth_colormap = cv2.applyColorMap(display_norm, cv2.COLORMAP_JET)
            
            # Draw a red line showing the center column analyzed
            cv2.line(depth_colormap, 
                    (cv_image.shape[1] // 2, 0), 
                    (cv_image.shape[1] // 2, cv_image.shape[0]), 
                    (0, 0, 255), 1)

            cv2.imshow("Depth Camera (Normalized 0.1m - 10m)", depth_colormap)
            cv2.waitKey(1)

        except Exception as e:
            # Note: If valid_distances is empty and you use np.nanmin/nanmax, this code is robust.
            # This catch is mainly for bridge/CV errors.
            self.get_logger().warn(f"Depth callback error: {e}")

    # -------------------------------
    # SCT input check functions
    # -------------------------------
    def clear_path_check(self, sup_data):
        # self.get_logger().info(f"Obstacle: {self.obstacle}")
        return not self.obstacle

    def obstacle_check(self, sup_data):
        # self.get_logger().info(f"Obstacle: {self.obstacle}")
        return self.obstacle

       # -------------------------------
    # Robot motion
    # -------------------------------
    def publish_twist(self, ev_name: str):
        twist = Twist()

        if ev_name == "EV_V1":       # Move forward
            # self.get_logger().info("Supervisor decision: MOVE")
            twist.linear.x = 1.0

        elif ev_name == "EV_V0":     # Stop
            # self.get_logger().info("Supervisor decision: STOP")
            twist.linear.x = 0.0

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
