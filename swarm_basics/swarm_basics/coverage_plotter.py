import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from collections import defaultdict


class CoveragePlotter(Node):
    def __init__(self):
        super().__init__('coverage_plotter')

        # === CONFIGURATION ===
        self.robot_namespaces = [f"robot_{i}" for i in range(10)]
        self.save_path = "/home/ecem/ros2_ws/src/swarm_basics/config/coverage_results.png"

        # === GRID SETUP ===
        self.env_min = -8
        self.env_max = 8
        self.grid_size = 1.0  # 1x1 m cells

        # Generate cell coordinates (bottom-left corner of each square)
        self.cells = []
        x_coords = [i for i in range(self.env_min, self.env_max)]
        y_coords = [j for j in range(self.env_min, self.env_max)]
        for x in x_coords:
            for y in y_coords:
                self.cells.append((x, y))

        self.visited = set()
        self.reach_threshold = self.grid_size / 2  # half cell width

        # === ROBOT TRAJECTORIES ===
        self.trajectories = defaultdict(list)

        # === SUBSCRIPTIONS ===
        for ns in self.robot_namespaces:
            self.create_subscription(
                Odometry,
                f"/{ns}/odom",
                lambda msg, ns=ns: self.odom_callback(msg, ns),
                10,
            )  

        # === PLOT SETUP ===
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_title("Multi-Robot Coverage")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect("equal")
        self.ax.set_xlim(self.env_min, self.env_max)
        self.ax.set_ylim(self.env_min, self.env_max)

        plt.ion()
        plt.show()

        # === TIMER FOR UPDATES ===
        self.timer = self.create_timer(0.5, self.update_plot)

    def odom_callback(self, msg, ns):
        """Store trajectory points and mark cells visited."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.trajectories[ns].append((x, y))
        self.get_logger().info(f"{ns}: x={x:.2f}, y={y:.2f}")

        # Check which cell the robot is in
        for idx, (cx, cy) in enumerate(self.cells):
            if idx not in self.visited:
                if cx <= x < cx + self.grid_size and cy <= y < cy + self.grid_size:
                    self.visited.add(idx)
                    self.get_logger().info(f"{ns} visited cell {idx} at ({cx},{cy})")

    def update_plot(self):
        """Redraw robot trajectories and visited cells."""
        self.ax.clear()
        self.ax.set_title("Multi-Robot Coverage")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect("equal")
        self.ax.set_xlim(self.env_min, self.env_max)
        self.ax.set_ylim(self.env_min, self.env_max)

        # Plot grid cells as squares
        for idx, (cx, cy) in enumerate(self.cells):
            color = 'green' if idx in self.visited else 'red'
            rect = Rectangle((cx, cy), self.grid_size, self.grid_size, facecolor=color, edgecolor='black', alpha=0.5)
            self.ax.add_patch(rect)

        # Plot robot trajectories
        for ns, traj in self.trajectories.items():
            if len(traj) > 1:
                xs, ys = zip(*traj)
                self.ax.plot(xs, ys, label=ns)

        # Add summary info
        self.ax.legend(loc="upper right", fontsize="small")
        self.ax.text(
            0.05, 0.95,
            f"Visited {len(self.visited)}/{len(self.cells)} cells",
            transform=self.ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            bbox=dict(facecolor='white', alpha=0.6, edgecolor='none')
        )
        plt.draw()
        plt.pause(0.01)

    def save_final_plot(self):
        """Save the final coverage map."""
        self.ax.clear()
        self.ax.set_title("Final Coverage Map")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect("equal")
        self.ax.set_xlim(self.env_min, self.env_max)
        self.ax.set_ylim(self.env_min, self.env_max)

        # Plot cells as squares
        for idx, (cx, cy) in enumerate(self.cells):
            color = 'green' if idx in self.visited else 'red'
            rect = Rectangle((cx, cy), self.grid_size, self.grid_size, facecolor=color, edgecolor='black', alpha=0.5)
            self.ax.add_patch(rect)

        # Plot trajectories
        for ns, traj in self.trajectories.items():
            if len(traj) > 1:
                xs, ys = zip(*traj)
                self.ax.plot(xs, ys, label=ns)

        self.ax.legend(loc="upper right", fontsize="small")
        self.ax.text(
            0.05, 0.95,
            f"Visited {len(self.visited)}/{len(self.cells)} cells",
            transform=self.ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            bbox=dict(facecolor='white', alpha=0.6, edgecolor='none')
        )

        self.fig.savefig(self.save_path)
        self.get_logger().info(
            f"Final plot saved. {len(self.visited)}/{len(self.cells)} cells visited."
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
