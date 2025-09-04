import time
from geometry_msgs.msg import Twist

class MovementController:
    def __init__(self, publisher, logger):
        self.pub = publisher
        self.logger = logger
        self.twist = Twist()
        self.current_action = "stop"

    def set_action(self, action: str):
        self.current_action = action

    def publish(self):
        # update twist depending on current action
        if self.current_action == "forward":
            self.logger.info("FORWARD")
            self.twist.linear.x = 0.5
            self.twist.angular.z = 0.0
        elif self.current_action == "cw":
            self.logger.info("CW")
            self.twist.linear.x = 0.0
            self.twist.angular.z = -0.05
        elif self.current_action == "ccw":
            self.logger.info("CCW")
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.05
        elif self.current_action == "stop":  
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

        self.pub.publish(self.twist)


