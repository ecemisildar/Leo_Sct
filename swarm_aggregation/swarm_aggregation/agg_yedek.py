import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class AggregationNode(Node):
    def __init__(self):
        super().__init__('aggregation_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.subscription = self.create_subscription(Image,'camera/image_raw',self.image_callback, qos_profile)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.bridge = CvBridge()

        # Timer to keep publishing at a fixed rate
        self.timer = self.create_timer(0.1, self.publish_twist)  # 10 Hz
        self.robot_detected = False 

    def image_callback(self, msg):
        
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

    def publish_twist(self):
        if self.robot_detected:
            self.get_logger().info("Robot Detected!")
            # Rotate clockwise on the spot
            vl, vr = 1.0, -1.0
        else:
            self.get_logger().info("No Robot!")
            # Move backward in a circular path
            vl, vr = -0.7, 1.0

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