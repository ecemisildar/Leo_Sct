import numpy as np
from nav_msgs.msg import Odometry

class PositionTracker:
    def __init__(self, spawn_x, spawn_y, logger):
        self.spawn_x = spawn_x
        self.spawn_y = spawn_y
        self.logger = logger

        self.red_leader_pos   = np.array([0.0, 0.0])
        self.green_leader_pos = np.array([3.0, 3.0])
        self.blue_leader_pos  = np.array([6.0, 0.0])

        self.dist2red = float("inf")
        self.dist2green = float("inf")
        self.dist2blue = float("inf")

    def odom_callback(self, msg: Odometry):
        global_x = self.spawn_x + msg.pose.pose.position.x
        global_y = self.spawn_y + msg.pose.pose.position.y

        follower_pos = np.array([global_x, global_y])

        self.dist2red = np.linalg.norm(self.red_leader_pos - follower_pos)
        self.dist2green = np.linalg.norm(self.green_leader_pos - follower_pos)
        self.dist2blue = np.linalg.norm(self.blue_leader_pos - follower_pos)

        self.logger.info(f"Distances: dist_red={self.dist2red}, dist_green={self.dist2green}, dist_blue={self.dist2blue}")
    

    
