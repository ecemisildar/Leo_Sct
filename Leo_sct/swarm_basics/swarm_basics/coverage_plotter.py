import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import UInt32
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from collections import defaultdict
from pathlib import Path
import csv
import time
import math
import xml.etree.ElementTree as ET


class CoveragePlotter(Node):
    def __init__(self):
        super().__init__('coverage_plotter')

        # === CONFIGURATION ===
        self.robot_namespaces = [f"robot_{i}" for i in range(10)]
        # Persist runs to the source tree so results are easy to find
        package_root = Path.home() / "ros2_ws/src/Leo_sct/swarm_basics"
        self.results_dir = Path(
            self.declare_parameter(
                'results_dir',
                str(package_root / "results")
            ).value
        )
        self.run_id = self.declare_parameter(
            'run_id',
            time.strftime("run_%Y%m%d_%H%M%S")
        ).value
        self.run_duration = float(self.declare_parameter(
            'run_duration', 1000.0
        ).value)
        self.obstacle_occupancy_threshold = float(self.declare_parameter(
            'obstacle_occupancy_threshold', 0.4
        ).value)
        self.world_sdf = Path(self.declare_parameter(
            'world_sdf',
            str(package_root / "worlds" / "random_world.sdf")
        ).value)
        self.run_dir = self.results_dir / str(self.run_id)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.metrics_save_path = self.run_dir / "coverage_vs_time.png"
        self._saved = False
        self._wall_start = time.time()

        # === GRID SETUP ===
        self.env_min = -7
        self.env_max = 7
        self.grid_size = 1.0  # 1x1 m cells
        self.cells = [(x, y) for x in range(self.env_min, self.env_max)
                              for y in range(self.env_min, self.env_max)]
        self.visited = set()
        self.blocked = self._compute_blocked_cells()

        # === ROBOT TRAJECTORIES ===
        self.trajectories = defaultdict(list)
        self.robot_colors = {
            "robot_0": "tab:blue",
            "robot_1": "tab:orange",
            "robot_2": "tab:green",
            "robot_3": "tab:red",
            "robot_4": "tab:purple",
            "robot_5": "tab:brown",
            "robot_6": "tab:pink",
            "robot_7": "tab:gray",
            "robot_8": "gold",
            "robot_9": "tab:cyan",
        }

        # === COLLISION TRACKING & COVERAGE METRICS ===
        self.start_time = self.get_clock().now()
        self.coverage_history = []          # (time_s, coverage_pct)
        self.collision_history = [(0.0, 0)] # (time_s, total_collisions)
        self.collision_counts = {ns: 0 for ns in self.robot_namespaces}
        self.total_collisions = 0

        # === SUBSCRIPTION ===
        # We only need one subscription for all robots (global positions)
        self.create_subscription(
            TFMessage,
            '/world/random_world/dynamic_pose/info',
            self.pose_callback,
            10,
        )
        # Collision counters per robot
        for ns in self.robot_namespaces:
            self.create_subscription(
                UInt32,
                f'/{ns}/bump_count',
                lambda msg, ns=ns: self.bump_callback(ns, msg),
                10,
            )

        # === PLOT SETUP (separate figures) ===
        self.fig_map, self.ax_map = plt.subplots(figsize=(8, 8))
        self.fig_metrics, self.ax_metrics = plt.subplots(figsize=(10, 5))
        self.ax_metrics_right = self.ax_metrics.twinx()
        self._setup_map_axes(self.ax_map)
        self.ax_metrics.set_title("Coverage & Collisions vs Time")
        self.ax_metrics.set_xlabel("Time (s)")
        self.ax_metrics.set_ylabel("Coverage (%)")
        self.ax_metrics_right.set_ylabel("Collisions (cumulative)")
        plt.ion()
        plt.show(block=False)

        # === TIMER FOR UPDATES ===
        self.timer = self.create_timer(0.5, self.update_plot)
        self.run_timer = self.create_timer(1.0, self.check_run_timeout)

    def pose_callback(self, msg: TFMessage):
        """Handle poses from /world/.../dynamic_pose/info (TFMessage)."""
        for t in msg.transforms:
            name = t.child_frame_id
            if not name.startswith('robot_'):
                continue  # skip non-robot entities
            if '/' in name:  # skip any sublink like robot_0/base_link
                continue

            # extract position (global)
            x = t.transform.translation.x
            y = t.transform.translation.y

            self.trajectories[name].append((x, y))
            # self.get_logger().info(f"{name}: x={x:.2f}, y={y:.2f}")

            # check which grid cell the robot visited
            for idx, (cx, cy) in enumerate(self.cells):
                if idx not in self.visited and idx not in self.blocked:
                    if cx <= x < cx + self.grid_size and cy <= y < cy + self.grid_size:
                        self.visited.add(idx)
                        self.get_logger().info(f"{name} visited cell {idx} at ({cx},{cy})")
                        break

    def bump_callback(self, namespace: str, msg: UInt32):
        """Track collision count updates from each robot."""
        new_val = int(msg.data)
        current = self.collision_counts.get(namespace, 0)

        # handle resets (e.g., node restart)
        if new_val < current:
            self.collision_counts[namespace] = new_val
            return

        if new_val == current:
            return

        delta = new_val - current
        self.collision_counts[namespace] = new_val
        self.total_collisions += delta

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.collision_history.append((elapsed, self.total_collisions))
        self.get_logger().info(
            f"{namespace} collisions: {new_val} (total={self.total_collisions})"
        )

    def _setup_map_axes(self, ax):
        ax.set_title("Multi-Robot Global Coverage")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_aspect("equal")
        ax.set_xlim(self.env_min, self.env_max)
        ax.set_ylim(self.env_min, self.env_max)

    def update_plot(self, skip_pause: bool = False):
        """Redraw robot trajectories and visited cells."""
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        free_cells = max(1, (len(self.cells) - len(self.blocked)))
        coverage_pct = (len(self.visited) / free_cells) * 100.0
        self.coverage_history.append((elapsed, coverage_pct))

        # Map figure
        self.ax_map.clear()
        self._setup_map_axes(self.ax_map)

        # Plot grid cells
        for idx, (cx, cy) in enumerate(self.cells):
            if idx in self.blocked:
                color = 'blue'
            else:
                color = 'green' if idx in self.visited else 'red'
            rect = Rectangle((cx, cy), self.grid_size, self.grid_size,
                             facecolor=color, edgecolor='black', alpha=0.3)
            self.ax_map.add_patch(rect)

        # Plot trajectories
        for ns in self.robot_namespaces:
            traj = self.trajectories.get(ns, [])
            if len(traj) > 1:
                xs, ys = zip(*traj)
                self.ax_map.plot(
                    xs, ys,
                    label=ns,
                    color=self.robot_colors.get(ns),
                )

        self.ax_map.legend(loc="upper right", fontsize="small")
        self.ax_map.text(
            0.05, 0.95,
            f"Visited {len(self.visited)}/{len(self.cells) - len(self.blocked)} free cells\n"
            f"Collisions: {self.total_collisions}",
            transform=self.ax_map.transAxes,
            fontsize=10,
            verticalalignment='top',
            bbox=dict(facecolor='white', alpha=0.6, edgecolor='none')
        )

        # Metrics figure
        self.ax_metrics.clear()
        self.ax_metrics_right.clear()
        self.ax_metrics.set_title("Coverage & Collisions vs Time")
        self.ax_metrics.set_xlabel("Time (s)")
        self.ax_metrics.set_ylabel("Coverage (%)")
        self.ax_metrics.set_ylim(0, 105)
        self.ax_metrics.grid(True, linestyle='--', alpha=0.4)
        self.ax_metrics_right.set_ylabel("Collisions (cumulative)")

        if self.coverage_history:
            t_cov, cov = zip(*self.coverage_history)
            self.ax_metrics.plot(t_cov, cov, color='navy', label='Coverage %')

        if self.collision_history:
            t_col, col = zip(*self.collision_history)
            self.ax_metrics_right.step(
                t_col, col, where='post', color='crimson', label='Collisions'
            )

        # Combined legend
        handles, labels = [], []
        for ax in (self.ax_metrics, self.ax_metrics_right):
            h, l = ax.get_legend_handles_labels()
            handles += h
            labels += l
        if handles:
            self.ax_metrics.legend(handles, labels, loc="upper left", fontsize="small")

        if not skip_pause:
            self.fig_map.canvas.draw_idle()
            self.fig_metrics.canvas.draw_idle()
            plt.pause(0.01)

    def save_final_plot(self):
        """Save final coverage plot."""
        if self._saved:
            return
        self._saved = True
        self.update_plot(skip_pause=True)
        self.fig_metrics.savefig(self.metrics_save_path)
        self._save_coverage_map()
        self._write_timeseries_csvs()
        self.get_logger().info(
            f"Final plot saved. {len(self.visited)}/{len(self.cells) - len(self.blocked)} free cells visited. "
            f"Total collisions: {self.total_collisions}."
        )

    def _save_coverage_map(self):
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_title("Final Global Coverage Map")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_aspect("equal")
        ax.set_xlim(self.env_min, self.env_max)
        ax.set_ylim(self.env_min, self.env_max)

        for idx, (cx, cy) in enumerate(self.cells):
            if idx in self.blocked:
                color = 'blue'
            else:
                color = 'green' if idx in self.visited else 'red'
            rect = Rectangle((cx, cy), self.grid_size, self.grid_size,
                             facecolor=color, edgecolor='black', alpha=0.3)
            ax.add_patch(rect)

        for ns in self.robot_namespaces:
            traj = self.trajectories.get(ns, [])
            if len(traj) > 1:
                xs, ys = zip(*traj)
                ax.plot(
                    xs, ys,
                    label=ns,
                    color=self.robot_colors.get(ns),
                )

        ax.legend(loc="upper right", fontsize="small")
        ax.text(
            0.05, 0.95,
            f"Visited {len(self.visited)}/{len(self.cells) - len(self.blocked)} free cells",
            transform=ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            bbox=dict(facecolor='white', alpha=0.6, edgecolor='none')
        )

        map_path = self.run_dir / "coverage_map.png"
        fig.savefig(map_path)
        plt.close(fig)

    def _write_timeseries_csvs(self):
        coverage_path = self.run_dir / "coverage_timeseries.csv"
        collisions_path = self.run_dir / "collision_timeseries.csv"

        with coverage_path.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_s', 'coverage_pct'])
            for t, cov in self.coverage_history:
                writer.writerow([f"{t:.3f}", f"{cov:.3f}"])

        with collisions_path.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_s', 'collisions'])
            for t, col in self.collision_history:
                writer.writerow([f"{t:.3f}", int(col)])

    def _compute_blocked_cells(self):
        obstacles = self._load_obstacle_rectangles()
        if not obstacles:
            return set()

        blocked = set()
        cell_area = self.grid_size * self.grid_size
        for idx, (cx, cy) in enumerate(self.cells):
            cell_min_x = cx
            cell_min_y = cy
            cell_max_x = cx + self.grid_size
            cell_max_y = cy + self.grid_size
            for obs in obstacles:
                overlap = self._overlap_area_rect_cell(
                    obs, cell_min_x, cell_min_y, cell_max_x, cell_max_y
                )
                if (overlap / cell_area) >= self.obstacle_occupancy_threshold:
                    blocked.add(idx)
                    break
        return blocked

    def _load_obstacle_rectangles(self):
        if not self.world_sdf.exists():
            self.get_logger().warn(f"SDF not found: {self.world_sdf}")
            return []

        try:
            root = ET.parse(self.world_sdf).getroot()
        except ET.ParseError as exc:
            self.get_logger().warn(f"Failed to parse SDF: {exc}")
            return []

        world = root.find("world")
        if world is None:
            return []

        obstacles = []
        for model in world.findall("model"):
            name = model.get("name", "")
            if name == "ground_plane":
                continue

            model_pose = self._parse_pose(model.findtext("pose"))
            for link in model.findall("link"):
                link_pose = self._parse_pose(link.findtext("pose"))
                for collision in link.findall("collision"):
                    collision_pose = self._parse_pose(collision.findtext("pose"))
                    size_text = collision.findtext("geometry/box/size")
                    if not size_text:
                        continue

                    sx, sy, _ = (float(v) for v in size_text.split())
                    x = model_pose[0] + link_pose[0] + collision_pose[0]
                    y = model_pose[1] + link_pose[1] + collision_pose[1]
                    yaw = model_pose[5] + link_pose[5] + collision_pose[5]
                    obstacles.append((x, y, sx, sy, yaw))
        return obstacles

    def _parse_pose(self, pose_text):
        if not pose_text:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        vals = [float(v) for v in pose_text.split()]
        while len(vals) < 6:
            vals.append(0.0)
        return tuple(vals[:6])

    def _rect_corners(self, cx, cy, sx, sy, yaw):
        hx = sx / 2.0
        hy = sy / 2.0
        c = math.cos(yaw)
        s = math.sin(yaw)
        corners = []
        for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
            x = cx + (dx * c - dy * s)
            y = cy + (dx * s + dy * c)
            corners.append((x, y))
        return corners

    def _clip_polygon(self, poly, edge_fn, intersect_fn):
        if not poly:
            return []
        output = []
        prev = poly[-1]
        prev_inside = edge_fn(prev)
        for curr in poly:
            curr_inside = edge_fn(curr)
            if curr_inside:
                if not prev_inside:
                    output.append(intersect_fn(prev, curr))
                output.append(curr)
            elif prev_inside:
                output.append(intersect_fn(prev, curr))
            prev, prev_inside = curr, curr_inside
        return output

    def _polygon_area(self, poly):
        if len(poly) < 3:
            return 0.0
        area = 0.0
        for i in range(len(poly)):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % len(poly)]
            area += x1 * y2 - x2 * y1
        return abs(area) * 0.5

    def _overlap_area_rect_cell(self, obstacle, min_x, min_y, max_x, max_y):
        cx, cy, sx, sy, yaw = obstacle
        poly = self._rect_corners(cx, cy, sx, sy, yaw)

        def clip_left(p):
            return p[0] >= min_x

        def clip_right(p):
            return p[0] <= max_x

        def clip_bottom(p):
            return p[1] >= min_y

        def clip_top(p):
            return p[1] <= max_y

        def intersect_x(p1, p2, xk):
            x1, y1 = p1
            x2, y2 = p2
            if x1 == x2:
                return (xk, y1)
            t = (xk - x1) / (x2 - x1)
            return (xk, y1 + t * (y2 - y1))

        def intersect_y(p1, p2, yk):
            x1, y1 = p1
            x2, y2 = p2
            if y1 == y2:
                return (x1, yk)
            t = (yk - y1) / (y2 - y1)
            return (x1 + t * (x2 - x1), yk)

        poly = self._clip_polygon(poly, clip_left, lambda p1, p2: intersect_x(p1, p2, min_x))
        poly = self._clip_polygon(poly, clip_right, lambda p1, p2: intersect_x(p1, p2, max_x))
        poly = self._clip_polygon(poly, clip_bottom, lambda p1, p2: intersect_y(p1, p2, min_y))
        poly = self._clip_polygon(poly, clip_top, lambda p1, p2: intersect_y(p1, p2, max_y))
        return self._polygon_area(poly)

    def check_run_timeout(self):
        if self.run_duration <= 0:
            return
        elapsed = time.time() - self._wall_start
        if elapsed >= self.run_duration:
            self.get_logger().info(
                f"Run duration reached ({self.run_duration}s). Saving results."
            )
            self.save_final_plot()
            rclpy.shutdown()


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
