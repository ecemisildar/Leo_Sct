# random_walk.py
import random
from geometry_msgs.msg import Twist

def random_walk(publisher, logger):
    """Perform a random walk step and publish Twist message."""
    twist = Twist()
    twist.linear.x = random.uniform(0.0, 0.2)   # forward speed
    twist.angular.z = random.uniform(-1.0, 1.0) # turning rate

    publisher.publish(twist)
    # logger.info("Executing random walk...")
