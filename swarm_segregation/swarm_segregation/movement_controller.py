import time
from geometry_msgs.msg import Twist

class MovementController:
    def __init__(self, publisher, logger):
        """
        publisher: rclpy pub for cmd_vel
        """
        self.pub = publisher
        self.logger = logger

    def move_forward(self):
        twist = Twist()
        self.logger.info("Moving forward")
        twist.linear.x = 0.8
        self.pub.publish(twist)
        time.sleep(0.5)

    def stop(self):
        twist = Twist()
        self.logger.info("Stopping")
        twist.linear.x = 0.0
        self.pub.publish(twist)
        time.sleep(0.5)

    def turn_cw(self):
        twist = Twist()
        self.logger.info("Turning CW")
        twist.angular.z = -0.8
        self.pub.publish(twist)
        time.sleep(0.5)

    def turn_ccw(self):
        twist = Twist()
        self.logger.info("Turning CCW")
        twist.angular.z = 0.8
        self.pub.publish(twist)
        time.sleep(0.5)
  