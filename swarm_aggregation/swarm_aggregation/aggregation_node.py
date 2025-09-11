import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# from swarm_aggregation.sct_wrapper import SCTWrapper
import os
from ament_index_python.packages import get_package_share_directory
from swarm_aggregation.sct import SCT


class AggregationNode(Node):
    def __init__(self):
        super().__init__('aggregation_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        yaml_path = os.path.join(get_package_share_directory('swarm_aggregation'), 'config', 'supervisor.yaml')

        self.supervisor = SCT(yaml_path)

        # Register callbacks for uncontrollable events EV_S0 and EV_S1
        self.supervisor.add_callback(self.supervisor.EV['EV_S0'], None, self.check_s0, None)
        self.supervisor.add_callback(self.supervisor.EV['EV_S1'], None, self.check_s1, None)
        self.supervisor.add_callback(self.supervisor.EV['EV_V0'], None, None, None)
        self.supervisor.add_callback(self.supervisor.EV['EV_V1'], None, None, None)



        self.subscription = self.create_subscription(Image,'camera/image_raw',self.image_callback, qos_profile)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.bridge = CvBridge()


        # Run supervisor at 10 Hz
        self.timer = self.create_timer(0.5, self.run)
        self.robot_detected = False 

        self.callback = {}
        self.input_buffer = None # Clear content after timestep


    def image_callback(self, msg):
        # self.get_logger().info("Image Callback!")
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # cv2.imshow("Camera Feed", cv_image)
        # cv2.waitKey(1)  # Needed for OpenCV GUI
        
        # Detect yellow color (top of the robot)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Optional: ignore small regions
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.robot_detected = any(cv2.contourArea(c) > 100 for c in contours)

    def check_s1(self, sup_data):
        return self.robot_detected

    def check_s0(self, sup_data):
        return not self.robot_detected
   
    def run(self):
        self.supervisor.input_buffer = []
        # Run one step of the supervisor
        ce_exists, ce = self.supervisor.run_step()
        
        # The supervisor will check its callbacks and add the appropriate uncontrollable event (EV_S0 or EV_S1)
        # to its input buffer before calculating the next controllable event.

        # If a controllable event is returned by the supervisor, publish the corresponding Twist message
        if ce_exists:
            event_name = [name for name, val in self.supervisor.EV.items() if val == ce][0]
            self.publish_twist(event_name)
        else:
            self.get_logger().info("No controllable event found.")
            # Optionally stop the robot if no event is enabled.
            # self.publish_twist(None) 


                
       
    

    def publish_twist(self, ev_name):
        if ev_name == "EV_V1":
            self.get_logger().info("Robot Detected! Moving with V1")
            # Rotate clockwise on the spot
            vl, vr = 1.0, -1.0
        elif ev_name == "EV_V0":
            self.get_logger().info("No Robot! Moving with V0")
            # Move backward in a circular path
            vl, vr = -0.7, 1.0
        else:
            self.get_logger().warn(f"Unknown velocity event: {ev_name}")
            return    

        # Convert wheel velocities to Twist
        twist = Twist()
        wheel_base = 0.5  # adjust to your robot
        twist.linear.x = 10*(vl + vr) / 2.0
        twist.angular.z = 0.1*(vr - vl) / wheel_base
        self.publisher.publish(twist)
        

def main(args=None):
    rclpy.init(args=args)
    node = AggregationNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()        