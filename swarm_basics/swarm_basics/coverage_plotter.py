import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from collections import defaultdict
import json
import os
import math
from ament_index_python.packages import get_package_share_directory


class CoveragePlotter(Node):
    def __init__(self):
        super().__init__('coverage_plotter')

        # === CONFIGURATION ===
        self.robot_namespaces = [f"robot_{i}" for i in range(10)]
        self.save_path = "/home/ecem/ros2_ws/src/swarm_basics/config/coverage_results.png"

        self.json_file = os.path.join(
            get_package_share_directory('swarm_basics'),
            'config',
            'cylinder_positions.json'
        )

        # === LOAD WAYPOINTS ===
        if os.path.exists(self.json_file):
            with open(self.json_file, "r") as f:
                self.waypoints = json.load(f)
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from JSON file.")
        else:
            self.get_logger().error("Could not find waypoints JSON file!")
            self.waypoints = []

        # === ROBOT TRAJECTORIES & VISITED WAYPOINTS ===
        self.trajectories = defaultdict(list)
        self.visited = set()  # store indices of visited waypoints
        self.reach_threshold = 0.3  # meters

        # === SUBSCRIPTIONS ===
        for ns in self.robot_namespaces:
            self.create_subscription(
                Odometry,
                f"/{ns}/odom",
                lambda msg, ns=ns: self.odom_callback(msg, ns),
                10,
            )

        # === PLOT SETUP ===
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_title("Multi-Robot Coverage")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect("equal")

        # Fixed boundaries
        self.ax.set_xlim(-5, 25)
        self.ax.set_ylim(-10, 10)

        # Plot waypoints once
        if self.waypoints:
            wp_x, wp_y = zip(*self.waypoints)
            self.ax.scatter(wp_x, wp_y, marker='x', color='red', s=80, label='Waypoints')
            for i, (x, y) in enumerate(self.waypoints):
                self.ax.text(x + 0.3, y + 0.3, f"WP{i+1}", color='darkred', fontsize=8)

        plt.ion()
        plt.show()

        # === TIMER FOR UPDATES ===
        self.timer = self.create_timer(0.5, self.update_plot)

    def odom_callback(self, msg, ns):
        """Store trajectory points and check for waypoint visits."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.trajectories[ns].append((x, y))

        # Check if this robot reached any waypoint
        for idx, (wx, wy) in enumerate(self.waypoints):
            if idx not in self.visited:
                dist = math.sqrt((x - wx)**2 + (y - wy)**2)
                if dist < self.reach_threshold:
                    self.visited.add(idx)
                    self.get_logger().info(
                        f"{ns} reached waypoint {idx + 1} at ({wx:.2f}, {wy:.2f})"
                    )

    def update_plot(self):
        """Redraw robot trajectories and update reached waypoint info."""
        self.ax.clear()
        self.ax.set_title("Multi-Robot Coverage")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect("equal")
        self.ax.set_xlim(-5, 25)
        self.ax.set_ylim(-10, 10)

        # Replot waypoints (visited vs unvisited)
        if self.waypoints:
            wp_x, wp_y = zip(*self.waypoints)
            # Color code: red = unvisited, green = visited
            for i, (x, y) in enumerate(self.waypoints):
                color = 'green' if i in self.visited else 'red'
                self.ax.scatter(x, y, marker='x', color=color, s=80)
                self.ax.text(x + 0.3, y + 0.3, f"WP{i+1}", color=color, fontsize=8)

        # Plot robot trajectories
        for ns, traj in self.trajectories.items():
            if len(traj) > 1:
                xs, ys = zip(*traj)
                self.ax.plot(xs, ys, label=ns)

        # Add summary info
        self.ax.legend(loc="upper right", fontsize="small")
        self.ax.text(
            0.05, 0.95,
            f"Reached {len(self.visited)}/{len(self.waypoints)} waypoints",
            transform=self.ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            bbox=dict(facecolor='white', alpha=0.6, edgecolor='none')
        )

        plt.draw()
        plt.pause(0.01)

    def save_final_plot(self):
        """Save the final plot on shutdown."""
        self.get_logger().info(f"Saving final coverage map to {self.save_path}")
        self.ax.clear()
        self.ax.set_title("Final Coverage Map")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect("equal")
        self.ax.set_xlim(-5, 25)
        self.ax.set_ylim(-10, 10)

        # Replot waypoints and trajectories
        if self.waypoints:
            for i, (x, y) in enumerate(self.waypoints):
                color = 'green' if i in self.visited else 'red'
                self.ax.scatter(x, y, marker='x', color=color, s=80)
                self.ax.text(x + 0.3, y + 0.3, f"WP{i+1}", color=color, fontsize=8)

        for ns, traj in self.trajectories.items():
            if len(traj) > 1:
                xs, ys = zip(*traj)
                self.ax.plot(xs, ys, label=ns)

        self.ax.legend(loc="upper right", fontsize="small")
        self.ax.text(
            0.05, 0.95,
            f"Reached {len(self.visited)}/{len(self.waypoints)} waypoints",
            transform=self.ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            bbox=dict(facecolor='white', alpha=0.6, edgecolor='none')
        )

        self.fig.savefig(self.save_path)
        self.get_logger().info(
            f"Final plot saved successfully. {len(self.visited)}/{len(self.waypoints)} waypoints reached."
        )


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_final_plot()
        node.get_logger().info("Shutting down coverage plotter.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
