import time
from geometry_msgs.msg import Twist

class MovementController:
    def __init__(self, publisher, logger):
        self.pub = publisher
        self.logger = logger
        self.current_twist = Twist()
        self.action_end_time = 0.0  # timestamp until which action is locked
        self.action_duration = 0

    def set_twist(self, twist: Twist, duration: float = 1.0):
        """Set a new twist command and hold it for duration seconds"""
        self.last_twist = twist
        self.action_start_time = time.time()
        self.action_duration = duration
        self.pub.publish(twist)
        self.logger.info(f"Publishing twist: lin={twist.linear.x}, ang={twist.angular.z}")

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0 
        self.set_twist(twist)
        self.logger.info("Moving forward")

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0 
        self.set_twist(twist)
        self.logger.info("Stopping")

    def turn_cw(self):
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = -0.5
        self.set_twist(twist)
        self.logger.info("Turning CW")

    def turn_ccw(self):
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.5
        self.set_twist(twist)
        self.logger.info("Turning CCW")

    def publish_last(self):
        """Keep publishing the last twist until duration expires"""
        if time.time() - self.action_start_time < self.action_duration:
            self.pub.publish(self.last_twist)

