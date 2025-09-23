import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from tf_transformations import euler_from_quaternion
from cv_bridge import CvBridge


class RobotCircleController(Node):
    def __init__(self):
        super().__init__("robot_circle_controller")

        # --- Circle formation parameters ---
        self.circle_radius = 2.0  # meters
        self.total_robots = 6  # total robots in formation

        # --- Potential fields parameters ---
        self.K_attractive = 0.5  # Gain for attractive force
        self.K_repulsive = 1.0  # Gain for repulsive force
        self.safe_distance = 0.5  # meters

        # --- Spawn offsets for local odom -> global frame ---
        # Note: In a real-world scenario, you would use a localization system
        # like AMCL to get a global pose. These offsets are a stand-in for a
        # global coordinate frame in a simulation.
        self.robot_offsets = {
            0: (0.0, 0.0),
            1: (3.0, 0.0),
            2: (0.0, 3.0),
            3: (3.0, 3.0),
            4: (5.0, 0.0),
            5: (0.0, 5.0),
        }

        # --- Determine robot index from namespace ---
        ns = self.get_namespace().strip("/")
        try:
            self.robot_index = int(ns.split("_")[-1])
        except ValueError:
            self.get_logger().error(f"Could not parse robot index from namespace: {ns}. Defaulting to 0.")
            self.robot_index = 0

        # Apply spawn offset for this robot
        self.x_offset, self.y_offset = self.robot_offsets.get(self.robot_index, (0.0, 0.0))

        # Angle offset for even spacing around the circle
        self.angle_offset = 2 * math.pi * self.robot_index / self.total_robots

        # --- ROS interfaces ---
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        
        # Subscribe to other robots' odometry for repulsion calculation
        self.robot_positions = {}
        for i in range(self.total_robots):
            if i == self.robot_index:
                continue
            self.create_subscription(
                Odometry,
                f"/robot_{i}/odom",
                lambda msg, robot_id=i: self.other_odom_callback(msg, robot_id),
                10,
            )

        self.depth_sub = self.create_subscription(Image, "depth_camera/depth_image", self.depth_callback, 10)
        self.bridge = CvBridge()
        self.min_front_distance = float("inf")

        self.get_logger().info(
            f"{ns}: Circle controller started "
            f"(index={self.robot_index}, of {self.total_robots}, radius={self.circle_radius})"
        )

    def other_odom_callback(self, msg: Odometry, robot_id: int):
        """Callback to store other robots' positions."""
        x = msg.pose.pose.position.x + self.robot_offsets.get(robot_id, (0.0, 0.0))[0]
        y = msg.pose.pose.position.y + self.robot_offsets.get(robot_id, (0.0, 0.0))[1]
        self.robot_positions[robot_id] = (x, y)

    def depth_callback(self, msg: Image):
        """Callback to process depth camera data for obstacle avoidance."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            center_col = cv_image[:, cv_image.shape[1] // 2]
            valid = center_col[np.isfinite(center_col)]
            self.min_front_distance = float(np.nanmin(valid)) if valid.size > 0 else float("inf")
        except Exception as e:
            self.get_logger().warn(f"Depth processing failed: {e}")

    def odom_callback(self, msg: Odometry):
        """Main control loop that runs with each new odometry message."""
        # --- Current global position using local odom + spawn offset ---
        x = msg.pose.pose.position.x + self.x_offset
        y = msg.pose.pose.position.y + self.y_offset

        # --- Orientation ---
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ])

        # --- Target position on circle centered at origin ---
        # Compute circle center from spawn offsets
        center_x = sum(x_off for x_off, _ in self.robot_offsets.values()) / self.total_robots
        center_y = sum(y_off for _, y_off in self.robot_offsets.values()) / self.total_robots

        # Target point for this robot
        target_x = center_x + self.circle_radius * math.cos(self.angle_offset)
        target_y = center_y + self.circle_radius * math.sin(self.angle_offset)

        # --- Compute attractive force ($F_{att}$) ---
        # A simple linear attractive force proportional to the distance to the target.
        vec_to_target = np.array([target_x - x, target_y - y])
        F_att = self.K_attractive * vec_to_target

        # --- Compute repulsive force ($F_{rep}$) ---
        F_rep = np.array([0.0, 0.0])

        # Repulsion from other robots
        for other_id, other_pos in self.robot_positions.items():
            dx = x - other_pos[0]
            dy = y - other_pos[1]
            dist = math.hypot(dx, dy)

            if 0 < dist < self.safe_distance:
                # Repulsion vector points away from the other robot
                rep_vector = np.array([dx, dy]) / dist
                # The magnitude of the force is high when close, fading to zero
                magnitude = self.K_repulsive * ((1.0 / dist) - (1.0 / self.safe_distance))
                F_rep += magnitude * rep_vector

        # Repulsion from obstacles (using depth sensor)
        if self.min_front_distance < self.safe_distance:
            # The repulsion vector points in the opposite direction of the obstacle
            rep_vector_obs = np.array([math.cos(yaw), math.sin(yaw)])
            magnitude_obs = self.K_repulsive * ((1.0 / self.min_front_distance) - (1.0 / self.safe_distance))
            F_rep += -magnitude_obs * rep_vector_obs

        # --- Compute total force and normalize for control ---
        F_total = F_att + F_rep
        
        # --- Velocity control ---
        cmd = Twist()

        # Calculate the robot's current heading vector
        robot_heading = np.array([math.cos(yaw), math.sin(yaw)])
        
        # Linear velocity is the component of the total force in the robot's heading direction
        linear_velocity = np.dot(F_total, robot_heading)
        cmd.linear.x = np.clip(linear_velocity, 0.0, 0.5) # Clamp the value to a max speed

        # Calculate angular speed to align with the total force vector
        target_yaw = math.atan2(F_total[1], F_total[0])
        yaw_error = target_yaw - yaw

        # Normalize yaw error to [-pi, pi]
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi

        # Proportional control for angular velocity
        cmd.angular.z = np.clip(0.5 * yaw_error, -1.0, 1.0)

        # If the robot is close to its target AND there's no strong repulsive force, stop
        distance_to_target = math.hypot(target_x - x, target_y - y)
        if distance_to_target < 0.05 and np.linalg.norm(F_rep) < 0.1:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RobotCircleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
