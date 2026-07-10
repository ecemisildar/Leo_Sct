#!/usr/bin/env python3
import time
import math
import csv
import shutil
import traceback
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

class CoverageCounter(Node):
    """
    - Tracks visited grid cells from Gazebo poses (TFMessage)
    - Saves results into a timestamped run folder
    - Logs paths + visited cells to CSV for offline analysis
    - Copies run metadata (active YAML + prompt text) into the same folder
    - Does not render or save plots (handled by offline scripts)
    """

    def __init__(self):
        super().__init__("coverage_counter")

        # fixed settings (no args)
        package_root = Path(get_package_share_directory("swarm_basics"))

        results_dir_default = package_root / "results_exp"
        self.results_root = Path(
            str(self.declare_parameter("results_dir", str(results_dir_default)).value)
        )
        run_id_param = str(self.declare_parameter("run_id", "").value).strip()
        self.run_id = run_id_param or time.strftime("run_%Y%m%d_%H%M%S")
        self.metadata_yaml_path_raw = str(
            self.declare_parameter("metadata_yaml_path", "").value
        ).strip()
        self.metadata_yaml_path = Path(self.metadata_yaml_path_raw) if self.metadata_yaml_path_raw else None
        self.prompt_file_path_raw = str(
            self.declare_parameter("prompt_file_path", "").value
        ).strip()
        self.prompt_file_path = Path(self.prompt_file_path_raw) if self.prompt_file_path_raw else None
        self.prompt_text = str(self.declare_parameter("prompt_text", "").value)
        self.run_duration = float(self.declare_parameter("run_duration", 200.0).value)
        self.flush_interval_sec = float(self.declare_parameter("flush_interval_sec", 3.0).value)
        self.flush_max_rows = int(self.declare_parameter("flush_max_rows", 2000).value)
        launch_parameter_names = (
            "sim_world",
            "headless",
            "auto_start",
            "robot_ns",
            "run_duration",
            "total_robots",
            "spawn_layout",
            "spawn_seed",
            "random_seed",
            "sct_choice_mode",
            "controller_mode",
            "peer_warning_enabled",
            "auto_start_supervisor",
            "results_dir",
            "metadata_yaml_path",
            "prompt_text",
            "prompt_file_path",
        )
        self.launch_arguments = {
            name: str(self.declare_parameter(f"launch_{name}", "").value)
            for name in launch_parameter_names
        }

        self.obstacle_occupancy_threshold = 0.4
        self.circle_obstacle_occupancy_threshold = 0.05
        sim_world = self.launch_arguments.get("sim_world", "").strip()
        self.world_sdf = (
            Path(sim_world)
            if sim_world
            else package_root / "worlds" / "random_world.sdf"
        )

        self.results_dir = self.results_root / self.run_id
        self.results_dir.mkdir(parents=True, exist_ok=True)

        self.coverage_csv_path = self.results_dir / "coverage_timeseries.csv"
        self.paths_csv_path = self.results_dir / "coverage_paths.csv"
        self.visited_cells_csv_path = self.results_dir / "coverage_visited_cells.csv"
        self.status_path = self.results_dir / "SAVE_STATUS.txt"
        self.error_path = self.results_dir / "SAVE_ERROR.txt"
        self.prompt_txt_path = self.results_dir / "prompt.txt"

        self._saved_ok = False
        self._saving_now = False
        self._wall_start = time.time()
        # grid
        self.env_min = -5
        self.env_max = 5
        self.grid_size = 1.0
        self.num_cells_y = int((self.env_max - self.env_min) / self.grid_size)
        self.cells = [(x, y) for x in range(self.env_min, self.env_max)
                              for y in range(self.env_min, self.env_max)]
        self.visited = set()
        self.blocked = self._compute_blocked_cells()

        # metrics

        self.start_time = self.get_clock().now()
        self.coverage_history = []
        self._ensure_paths_csv_header()
        self._ensure_cells_csv_header()
        self._path_rows = []
        self._cell_rows = []

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

        self.pose_interval_sec = 0.2
        self._last_pose_time = self.get_clock().now()

        # timers
        self.timer_plot = self.create_timer(0.5, self._on_metrics_timer)
        self.timer_timeout = self.create_timer(1.0, self._on_timeout_timer)
        self.timer_flush = self.create_timer(self.flush_interval_sec, self._flush_buffers)

        self._write_status(
            "Node started\n"
            f"run_id: {self.run_id}\n"
            f"results_dir: {self.results_dir}\n"
            f"metadata_yaml: {self.metadata_yaml_path}\n"
            f"prompt_file: {self.prompt_file_path}\n"
            "launch_arguments:\n"
            + "".join(
                f"  {name}: {value}\n"
                for name, value in self.launch_arguments.items()
            )
        )
        self._save_run_metadata()

    def pose_callback(self, msg: TFMessage):
        now = self.get_clock().now()
        if (now - self._last_pose_time).nanoseconds < int(self.pose_interval_sec * 1e9):
            return
        self._last_pose_time = now

        for t in msg.transforms:
            name = t.child_frame_id
            if not name.startswith("robot_"):
                continue
            if "/" in name:
                continue

            x = t.transform.translation.x
            y = t.transform.translation.y
            self._append_path_row(name, x, y)

            ix = int(math.floor((x - self.env_min) / self.grid_size))
            iy = int(math.floor((y - self.env_min) / self.grid_size))
            if 0 <= ix < self.num_cells_y and 0 <= iy < self.num_cells_y:
                idx = ix * self.num_cells_y + iy
                if idx not in self.visited and idx not in self.blocked:
                    cx = self.env_min + ix * self.grid_size
                    cy = self.env_min + iy * self.grid_size
                    self.visited.add(idx)
                    self._append_visited_cell_row(name, idx, cx, cy)

    # timers
    def _on_metrics_timer(self):
        if self._saving_now:
            return
        self._update_metrics()

    def _on_timeout_timer(self):
        elapsed = time.time() - self._wall_start
        if elapsed >= self.run_duration:
            self._write_status(f"Timeout reached ({self.run_duration}s). Saving + shutdown.\n")
            self.shutdown_and_save()

    def _update_metrics(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        free_cells = max(1, (len(self.cells) - len(self.blocked)))
        coverage_pct = (len(self.visited) / free_cells) * 100.0
        self.coverage_history.append((elapsed, coverage_pct))

    # saving
    def shutdown_and_save(self):
        self._saving_now = True
        try:
            try:
                self.timer_plot.cancel()
                self.timer_timeout.cancel()
            except Exception:
                pass

            self._update_metrics()
            self.save_all_artifacts()
        finally:
            self._saving_now = False
            rclpy.shutdown()

    def save_all_artifacts(self):
        if self._saved_ok:
            return
        try:
            self._write_status("Saving started...\n")

            self._flush_buffers(force=True)
            self._write_timeseries_csvs()

            self._saved_ok = True
            self._write_status("Saving OK\n")
        except Exception:
            self.error_path.write_text(traceback.format_exc(), encoding="utf-8")
            self._write_status("Saving FAILED (see SAVE_ERROR.txt)\n")

    def _write_timeseries_csvs(self):
        with self.coverage_csv_path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "coverage_pct"])
            for t, cov in self.coverage_history:
                w.writerow([f"{t:.3f}", f"{cov:.3f}"])

    def _ensure_paths_csv_header(self):
        if not self.paths_csv_path.exists():
            with self.paths_csv_path.open("w", newline="") as f:
                w = csv.writer(f)
                w.writerow([
                    "stamp_sec", "stamp_nsec",
                    "robot",
                    "x", "y"
                ])

    def _ensure_cells_csv_header(self):
        if not self.visited_cells_csv_path.exists():
            with self.visited_cells_csv_path.open("w", newline="") as f:
                w = csv.writer(f)
                w.writerow([
                    "stamp_sec", "stamp_nsec",
                    "robot",
                    "cell_index",
                    "cell_min_x", "cell_min_y",
                    "cell_center_x", "cell_center_y"
                ])

    def _append_path_row(self, robot: str, x: float, y: float):
        stamp = self.get_clock().now().to_msg()
        self._path_rows.append([
            int(stamp.sec), int(stamp.nanosec),
            robot,
            f"{x:.3f}", f"{y:.3f}"
        ])
        if len(self._path_rows) >= self.flush_max_rows:
            self._flush_buffers()

    def _append_visited_cell_row(self, robot: str, idx: int, cx: float, cy: float):
        stamp = self.get_clock().now().to_msg()
        center_x = cx + (self.grid_size * 0.5)
        center_y = cy + (self.grid_size * 0.5)
        self._cell_rows.append([
            int(stamp.sec), int(stamp.nanosec),
            robot,
            int(idx),
            f"{cx:.3f}", f"{cy:.3f}",
            f"{center_x:.3f}", f"{center_y:.3f}"
        ])
        if len(self._cell_rows) >= self.flush_max_rows:
            self._flush_buffers()

    def _flush_buffers(self, force: bool = False):
        if not self._path_rows and not self._cell_rows:
            return
        if not force and self._saving_now:
            return
        if self._path_rows:
            with self.paths_csv_path.open("a", newline="") as f:
                w = csv.writer(f)
                w.writerows(self._path_rows)
            self._path_rows.clear()
        if self._cell_rows:
            with self.visited_cells_csv_path.open("a", newline="") as f:
                w = csv.writer(f)
                w.writerows(self._cell_rows)
            self._cell_rows.clear()

    def _write_status(self, text: str):
        with self.status_path.open("a", encoding="utf-8") as f:
            f.write(text)

    def _save_run_metadata(self):
        if self.prompt_file_path is not None and self.prompt_file_path.exists():
            prompt_contents = self.prompt_file_path.read_text(encoding="utf-8")
        else:
            if self.prompt_file_path is not None and not self.prompt_file_path.exists():
                self._write_status(f"WARNING: Prompt file not found: {self.prompt_file_path}\n")
            prompt_contents = self.prompt_text
        self.prompt_txt_path.write_text(prompt_contents, encoding="utf-8")
        if self.metadata_yaml_path is None:
            self._write_status("WARNING: No metadata_yaml_path provided.\n")
            return
        if not self.metadata_yaml_path.exists():
            self._write_status(f"WARNING: YAML not found: {self.metadata_yaml_path}\n")
            return
        try:
            shutil.copy2(self.metadata_yaml_path, self.results_dir / self.metadata_yaml_path.name)
        except Exception as exc:
            self._write_status(f"WARNING: Failed to copy YAML: {exc}\n")

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
                overlap = self._overlap_area_obstacle_cell(
                    obs, cell_min_x, cell_min_y, cell_max_x, cell_max_y
                )
                threshold = (
                    self.circle_obstacle_occupancy_threshold
                    if obs[0] == "circle"
                    else self.obstacle_occupancy_threshold
                )
                if (overlap / cell_area) >= threshold:
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
            if name == "ground_plane" or name.endswith("_wall"):
                continue
            model_pose = self._parse_pose(model.findtext("pose"))
            for link in model.findall("link"):
                link_pose = self._parse_pose(link.findtext("pose"))
                for collision in link.findall("collision"):
                    collision_pose = self._parse_pose(collision.findtext("pose"))
                    x = model_pose[0] + link_pose[0] + collision_pose[0]
                    y = model_pose[1] + link_pose[1] + collision_pose[1]
                    yaw = model_pose[5] + link_pose[5] + collision_pose[5]

                    size_text = collision.findtext("geometry/box/size")
                    if size_text:
                        sx, sy, _ = (float(v) for v in size_text.split())
                        obstacles.append(("box", x, y, sx, sy, yaw))
                        continue
                    radius_text = collision.findtext("geometry/cylinder/radius")
                    if radius_text:
                        obstacles.append(("circle", x, y, float(radius_text)))
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

    def _overlap_area_obstacle_cell(self, obstacle, min_x, min_y, max_x, max_y):
        if obstacle[0] == "circle":
            return self._overlap_area_circle_cell(obstacle, min_x, min_y, max_x, max_y)
        return self._overlap_area_rect_cell(obstacle, min_x, min_y, max_x, max_y)

    def _overlap_area_rect_cell(self, obstacle, min_x, min_y, max_x, max_y):
        _, cx, cy, sx, sy, yaw = obstacle
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

    def _overlap_area_circle_cell(self, obstacle, min_x, min_y, max_x, max_y):
        _, cx, cy, radius = obstacle
        samples_per_axis = 10
        inside = 0
        step_x = (max_x - min_x) / samples_per_axis
        step_y = (max_y - min_y) / samples_per_axis
        radius_sq = radius * radius
        for ix in range(samples_per_axis):
            px = min_x + (ix + 0.5) * step_x
            for iy in range(samples_per_axis):
                py = min_y + (iy + 0.5) * step_y
                if (px - cx) ** 2 + (py - cy) ** 2 <= radius_sq:
                    inside += 1
        return (inside / (samples_per_axis * samples_per_axis)) * (
            (max_x - min_x) * (max_y - min_y)
        )


def main(args=None):
    rclpy.init(args=args)
    node = CoverageCounter()
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
