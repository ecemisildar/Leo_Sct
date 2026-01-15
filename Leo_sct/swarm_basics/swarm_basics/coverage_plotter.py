#!/usr/bin/env python3
import os
import time
import math
import csv
import traceback
import xml.etree.ElementTree as ET
from pathlib import Path
from collections import defaultdict

import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import matplotlib
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


class CoveragePlotter(Node):
    """
    - Tracks visited grid cells from Gazebo poses (TFMessage)
    - Saves results to results/<run_id>/
    """

    def __init__(self):
        super().__init__("coverage_plotter")

        # fixed settings (no args)
        self.robot_namespaces = [f"robot_{i}" for i in range(10)]
        package_root = Path.home() / "ros2_ws/src/Leo_sct/swarm_basics"

        self.results_dir = package_root / "results"
        self.run_id = time.strftime("run_%Y%m%d_%H%M%S")
        self.run_duration = 500.0

        self.enable_live_plot = False
        self.obstacle_occupancy_threshold = 0.4
        self.world_sdf = package_root / "worlds" / "random_world.sdf"

        self.run_dir = self.results_dir / self.run_id
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.metrics_save_path = self.run_dir / "coverage_vs_time.png"
        self.map_save_path = self.run_dir / "coverage_map.png"
        self.coverage_csv_path = self.run_dir / "coverage_timeseries.csv"
        self.status_path = self.run_dir / "SAVE_STATUS.txt"
        self.error_path = self.run_dir / "SAVE_ERROR.txt"

        self._saved_ok = False
        self._saving_now = False
        self._wall_start = time.time()

        # grid
        self.env_min = -7
        self.env_max = 7
        self.grid_size = 1.0
        self.cells = [(x, y) for x in range(self.env_min, self.env_max)
                              for y in range(self.env_min, self.env_max)]
        self.visited = set()
        self.blocked = self._compute_blocked_cells()

        # trajectories + metrics
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

        self.start_time = self.get_clock().now()
        self.coverage_history = []

        # --- subscriptions ---
        pose_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            TFMessage,
            "/world/random_world/dynamic_pose/info",
            self.pose_callback,
            pose_qos,
        )


        # plot setup
        self.fig_map, self.ax_map = plt.subplots(figsize=(8, 8))
        self.fig_metrics, self.ax_metrics = plt.subplots(figsize=(10, 5))
        self._setup_map_axes()
        self._setup_metrics_axes()

        if self.enable_live_plot and os.environ.get("DISPLAY"):
            plt.ion()
            plt.show(block=False)

        # timers
        self.timer_plot = self.create_timer(0.5, self._on_plot_timer)
        self.timer_timeout = self.create_timer(1.0, self._on_timeout_timer)

        self._write_status(
            "Node started\n"
            f"run_dir: {self.run_dir}\n"
        )

    def pose_callback(self, msg: TFMessage):
        for t in msg.transforms:
            name = t.child_frame_id
            if not name.startswith("robot_"):
                continue
            if "/" in name:
                continue

            x = t.transform.translation.x
            y = t.transform.translation.y
            self.trajectories[name].append((x, y))

            for idx, (cx, cy) in enumerate(self.cells):
                if idx in self.visited or idx in self.blocked:
                    continue
                if cx <= x < cx + self.grid_size and cy <= y < cy + self.grid_size:
                    self.visited.add(idx)
                    break

    # timers
    def _on_plot_timer(self):
        if self._saving_now:
            return
        self._update_plots(live_pause=bool(self.enable_live_plot and os.environ.get("DISPLAY")))

    def _on_timeout_timer(self):
        elapsed = time.time() - self._wall_start
        if elapsed >= self.run_duration:
            self._write_status(f"Timeout reached ({self.run_duration}s). Saving + shutdown.\n")
            self.shutdown_and_save()

    # plotting
    def _setup_map_axes(self):
        self.ax_map.set_title("Multi-Robot Global Coverage")
        self.ax_map.set_xlabel("X (m)")
        self.ax_map.set_ylabel("Y (m)")
        self.ax_map.set_aspect("equal")
        self.ax_map.set_xlim(self.env_min, self.env_max)
        self.ax_map.set_ylim(self.env_min, self.env_max)

    def _setup_metrics_axes(self):
        self.ax_metrics.set_title("Coverage vs Time")
        self.ax_metrics.set_xlabel("Time (s)")
        self.ax_metrics.set_ylabel("Coverage (%)")

    def _update_plots(self, live_pause: bool):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        free_cells = max(1, (len(self.cells) - len(self.blocked)))
        coverage_pct = (len(self.visited) / free_cells) * 100.0
        self.coverage_history.append((elapsed, coverage_pct))

        # map
        self.ax_map.clear()
        self._setup_map_axes()

        for idx, (cx, cy) in enumerate(self.cells):
            if idx in self.blocked:
                color = "blue"
            else:
                color = "green" if idx in self.visited else "red"
            rect = Rectangle((cx, cy), self.grid_size, self.grid_size,
                             facecolor=color, edgecolor="black", alpha=0.3)
            self.ax_map.add_patch(rect)

        for ns in self.robot_namespaces:
            traj = self.trajectories.get(ns, [])
            if len(traj) > 1:
                xs, ys = zip(*traj)
                self.ax_map.plot(xs, ys, label=ns, color=self.robot_colors.get(ns))

        self.ax_map.legend(loc="upper right", fontsize="small")
        self.ax_map.text(
            0.05, 0.95,
            f"Visited {len(self.visited)}/{len(self.cells) - len(self.blocked)} free cells",
            transform=self.ax_map.transAxes,
            fontsize=10,
            verticalalignment="top",
            bbox=dict(facecolor="white", alpha=0.6, edgecolor="none"),
        )

        # metrics
        self.ax_metrics.clear()

        self.ax_metrics.set_title("Coverage vs Time")
        self.ax_metrics.set_xlabel("Time (s)")
        self.ax_metrics.set_ylabel("Coverage (%)")
        self.ax_metrics.set_ylim(0, 105)
        self.ax_metrics.grid(True, linestyle="--", alpha=0.4)

        if self.coverage_history:
            t_cov, cov = zip(*self.coverage_history)
            self.ax_metrics.plot(t_cov, cov, label="Coverage %")


        handles, labels = self.ax_metrics.get_legend_handles_labels()
        if handles:
            self.ax_metrics.legend(handles, labels, loc="upper left", fontsize="small")

        self.fig_map.canvas.draw_idle()
        self.fig_metrics.canvas.draw_idle()
        if live_pause:
            plt.pause(0.01)

    # saving
    def shutdown_and_save(self):
        self._saving_now = True
        try:
            try:
                self.timer_plot.cancel()
                self.timer_timeout.cancel()
            except Exception:
                pass

            self._update_plots(live_pause=False)
            self.save_all_artifacts()
        finally:
            self._saving_now = False
            rclpy.shutdown()

    def save_all_artifacts(self):
        if self._saved_ok:
            return
        try:
            self._write_status("Saving started...\n")

            self.fig_metrics.tight_layout()
            self.fig_metrics.canvas.draw()
            self.fig_map.tight_layout()
            self.fig_map.canvas.draw()

            self.fig_metrics.savefig(self.metrics_save_path, dpi=200, bbox_inches="tight")
            self._save_final_map_png()
            self._write_timeseries_csvs()

            self._saved_ok = True
            self._write_status("Saving OK\n")
        except Exception:
            self.error_path.write_text(traceback.format_exc(), encoding="utf-8")
            self._write_status("Saving FAILED (see SAVE_ERROR.txt)\n")

    def _save_final_map_png(self):
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_title("Final Global Coverage Map")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_aspect("equal")
        ax.set_xlim(self.env_min, self.env_max)
        ax.set_ylim(self.env_min, self.env_max)

        for idx, (cx, cy) in enumerate(self.cells):
            if idx in self.blocked:
                color = "blue"
            else:
                color = "green" if idx in self.visited else "red"
            rect = Rectangle((cx, cy), self.grid_size, self.grid_size,
                             facecolor=color, edgecolor="black", alpha=0.3)
            ax.add_patch(rect)

        for ns in self.robot_namespaces:
            traj = self.trajectories.get(ns, [])
            if len(traj) > 1:
                xs, ys = zip(*traj)
                ax.plot(xs, ys, label=ns, color=self.robot_colors.get(ns))

        ax.legend(loc="upper right", fontsize="small")
        ax.text(
            0.05, 0.95,
            f"Visited {len(self.visited)}/{len(self.cells) - len(self.blocked)} free cells",
            transform=ax.transAxes,
            fontsize=10,
            verticalalignment="top",
            bbox=dict(facecolor="white", alpha=0.6, edgecolor="none"),
        )

        fig.tight_layout()
        fig.savefig(self.map_save_path, dpi=200, bbox_inches="tight")
        plt.close(fig)

    def _write_timeseries_csvs(self):
        with self.coverage_csv_path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "coverage_pct"])
            for t, cov in self.coverage_history:
                w.writerow([f"{t:.3f}", f"{cov:.3f}"])

    def _write_status(self, text: str):
        with self.status_path.open("a", encoding="utf-8") as f:
            f.write(text)

    # obstacles
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
                overlap = self._overlap_area_rect_cell(obs, cell_min_x, cell_min_y, cell_max_x, cell_max_y)
                if (overlap / cell_area) >= self.obstacle_occupancy_threshold:
                    blocked.add(idx)
                    break
        return blocked

    def _load_obstacle_rectangles(self):
        if not self.world_sdf.exists():
            self._write_status(f"WARNING: SDF not found: {self.world_sdf}\n")
            return []
        try:
            root = ET.parse(self.world_sdf).getroot()
        except ET.ParseError as exc:
            self._write_status(f"WARNING: Failed to parse SDF: {exc}\n")
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

        def clip_left(p):   return p[0] >= min_x
        def clip_right(p):  return p[0] <= max_x
        def clip_bottom(p): return p[1] >= min_y
        def clip_top(p):    return p[1] <= max_y

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

        poly = self._clip_polygon(poly, clip_left,   lambda a, b: intersect_x(a, b, min_x))
        poly = self._clip_polygon(poly, clip_right,  lambda a, b: intersect_x(a, b, max_x))
        poly = self._clip_polygon(poly, clip_bottom, lambda a, b: intersect_y(a, b, min_y))
        poly = self._clip_polygon(poly, clip_top,    lambda a, b: intersect_y(a, b, max_y))
        return self._polygon_area(poly)


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._write_status("KeyboardInterrupt received. Saving + shutdown.\n")
        node.shutdown_and_save()
    except BaseException:
        node.error_path.write_text(traceback.format_exc(), encoding="utf-8")
        node._write_status("BaseException received. Attempting save in finally.\n")
    finally:
        try:
            if not node._saved_ok:
                node.save_all_artifacts()
        except Exception:
            node.error_path.write_text(traceback.format_exc(), encoding="utf-8")
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
