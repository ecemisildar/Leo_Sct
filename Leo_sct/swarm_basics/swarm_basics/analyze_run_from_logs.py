#!/usr/bin/env python3
import argparse
import csv
import math
import os
from pathlib import Path

import matplotlib
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
RESULTS_DIRS = [
    PACKAGE_ROOT / "results_exp",
    PACKAGE_ROOT / "results",
]
WORLD_SDF = PACKAGE_ROOT / "swarm_basics" / "worlds" / "random_world.sdf"

ENV_MIN = -5
ENV_MAX = 5
GRID_SIZE = 1.0
OBSTACLE_OCCUPANCY_THRESHOLD = 0.4
def pick_latest_run_dir(results_dirs: list[Path]) -> tuple[str, Path, Path]:
    run_dirs = []
    legacy_runs = []
    for results_dir in results_dirs:
        if not results_dir.exists():
            continue
        run_dirs.extend(
            path for path in results_dir.rglob("run_*")
            if path.is_dir() and (path / "coverage_timeseries.csv").exists()
        )
        legacy_runs.extend(results_dir.glob("run_*_coverage_timeseries.csv"))

    if run_dirs:
        run_dirs.sort(key=lambda p: p.stat().st_mtime)
        run_dir = run_dirs[-1]
        return run_dir.name, run_dir, run_dir.parent

    if not legacy_runs:
        joined = ", ".join(str(path) for path in results_dirs)
        raise SystemExit(
            f"No run folders or legacy run_*_coverage_timeseries.csv files found under {joined}"
        )
    legacy_runs.sort(key=lambda p: p.stat().st_mtime)
    latest = legacy_runs[-1]
    run_id = latest.name.removesuffix("_coverage_timeseries.csv")
    return run_id, latest.parent, latest.parent


def pick_matching_bump_file(run_dir: Path, run_id: str) -> Path | None:
    if run_dir.exists():
        run_bumps = sorted(run_dir.glob("bumps_*.csv"), key=lambda p: p.stat().st_mtime)
        if run_bumps:
            return run_bumps[-1]

    parent_dir = run_dir.parent if run_dir.name.startswith("run_") else run_dir
    if not parent_dir.exists():
        return None
    run_hms = run_id.rsplit("_", 1)[-1]
    files = []
    for path in parent_dir.glob("bumps_*.csv"):
        stem = path.stem
        suffix = stem.rsplit("_", 1)[-1]
        if len(suffix) != 6 or not suffix.isdigit():
            continue
        files.append((abs(int(suffix) - int(run_hms)), path))
    if not files:
        return None
    files.sort(key=lambda item: item[0])
    return files[0][1] if files[0][0] <= 2 else None


def iter_run_dirs(results_dirs: list[Path]) -> list[Path]:
    run_dirs = []
    for results_dir in results_dirs:
        if not results_dir.exists():
            continue
        run_dirs.extend(
            path for path in results_dir.rglob("run_*")
            if path.is_dir() and (path / "coverage_timeseries.csv").exists()
        )
    return sorted(set(run_dirs))


def read_bump_rows(path: Path):
    """
    Reads bump log rows as dicts.
    Autodetect delimiter (TSV vs CSV).
    Requires at least: stamp_sec, stamp_nsec, bump_type, total_index (or can still work without).
    """
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        rows = [row for row in reader if row]
    return rows


def read_coverage_timeseries(path: Path):
    times, covs = [], []
    with path.open("r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            t = row.get("time_s", "")
            c = row.get("coverage_pct", "")
            if not t or not c:
                continue
            try:
                times.append(float(t))
                covs.append(float(c))
            except ValueError:
                continue
    return times, covs


def read_visited_cells(path: Path):
    visited = set()
    with path.open("r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            idx = row.get("cell_index", "")
            if idx == "":
                continue
            try:
                visited.add(int(idx))
            except ValueError:
                continue
    return visited


def read_robot_paths(path: Path):
    paths = {}
    with path.open("r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            robot = (row.get("robot") or "").strip()
            x = row.get("x", "")
            y = row.get("y", "")
            if not robot or x == "" or y == "":
                continue
            try:
                px = float(x)
                py = float(y)
            except ValueError:
                continue
            paths.setdefault(robot, []).append((px, py))
    return paths


def build_cells(env_min, env_max):
    return [(x, y) for x in range(env_min, env_max) for y in range(env_min, env_max)]


def parse_pose(pose_text):
    if not pose_text:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    vals = [float(v) for v in pose_text.split()]
    while len(vals) < 6:
        vals.append(0.0)
    return tuple(vals[:6])


def load_obstacle_rectangles(world_sdf: Path):
    if not world_sdf.exists():
        return []
    try:
        root = ET.parse(world_sdf).getroot()
    except ET.ParseError:
        return []
    world = root.find("world")
    if world is None:
        return []
    obstacles = []
    for model in world.findall("model"):
        name = model.get("name", "")
        if name == "ground_plane":
            continue
        model_pose = parse_pose(model.findtext("pose"))
        for link in model.findall("link"):
            link_pose = parse_pose(link.findtext("pose"))
            for collision in link.findall("collision"):
                collision_pose = parse_pose(collision.findtext("pose"))
                size_text = collision.findtext("geometry/box/size")
                if not size_text:
                    continue
                sx, sy, _ = (float(v) for v in size_text.split())
                x = model_pose[0] + link_pose[0] + collision_pose[0]
                y = model_pose[1] + link_pose[1] + collision_pose[1]
                yaw = model_pose[5] + link_pose[5] + collision_pose[5]
                obstacles.append((x, y, sx, sy, yaw))
    return obstacles


def rect_corners(cx, cy, sx, sy, yaw):
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


def clip_polygon(poly, edge_fn, intersect_fn):
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


def polygon_area(poly):
    if len(poly) < 3:
        return 0.0
    area = 0.0
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        area += x1 * y2 - x2 * y1
    return abs(area) * 0.5


def overlap_area_rect_cell(obstacle, min_x, min_y, max_x, max_y):
    cx, cy, sx, sy, yaw = obstacle
    poly = rect_corners(cx, cy, sx, sy, yaw)

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

    poly = clip_polygon(poly, clip_left,   lambda a, b: intersect_x(a, b, min_x))
    poly = clip_polygon(poly, clip_right,  lambda a, b: intersect_x(a, b, max_x))
    poly = clip_polygon(poly, clip_bottom, lambda a, b: intersect_y(a, b, min_y))
    poly = clip_polygon(poly, clip_top,    lambda a, b: intersect_y(a, b, max_y))
    return polygon_area(poly)


def compute_blocked_cells(cells, obstacles, grid_size, occupancy_threshold):
    if not obstacles:
        return set()
    blocked = set()
    cell_area = grid_size * grid_size
    for idx, (cx, cy) in enumerate(cells):
        cell_min_x = cx
        cell_min_y = cy
        cell_max_x = cx + grid_size
        cell_max_y = cy + grid_size
        for obs in obstacles:
            overlap = overlap_area_rect_cell(obs, cell_min_x, cell_min_y, cell_max_x, cell_max_y)
            if (overlap / cell_area) >= occupancy_threshold:
                blocked.add(idx)
                break
    return blocked


def plot_coverage_map(cells, visited, blocked, paths, out_png: Path):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlabel("X (m)", fontsize=16)
    ax.set_ylabel("Y (m)", fontsize=16)
    ax.set_aspect("equal")
    ax.set_xlim(ENV_MIN, ENV_MAX)
    ax.set_ylim(ENV_MIN, ENV_MAX)
    fig.subplots_adjust(right=0.78)

    for idx, (cx, cy) in enumerate(cells):
        if idx in blocked:
            color = "blue"
        else:
            color = "green" if idx in visited else "red"
        rect = plt.Rectangle(
            (cx, cy), GRID_SIZE, GRID_SIZE,
            facecolor=color, edgecolor="black", alpha=0.3
        )
        ax.add_patch(rect)

    for robot, pts in sorted(paths.items()):
        if len(pts) < 2:
            continue
        xs, ys = zip(*pts)
        ax.plot(xs, ys, label=robot, linewidth=1.0)

    if paths:
        ax.legend(
            loc="center left",
            bbox_to_anchor=(1.02, 0.5),
            borderaxespad=0.0,
            fontsize=12,
        )

    ax.text(
        0.05, 0.95,
        f"Visited {len(visited)}/{len(cells) - len(blocked)} free cells",
        transform=ax.transAxes,
        fontsize=16,
        verticalalignment="top",
        bbox=dict(facecolor="white", alpha=0.6, edgecolor="none"),
    )

    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_collisions(rows, cov_times, covs, out_png: Path):
    t = []
    rob = []
    obs = []
    if not rows and cov_times:
        t = list(cov_times)
        rob = [0] * len(t)
        obs = [0] * len(t)
    else:
        for row in rows:
            try:
                sec = int(row.get("stamp_sec", ""))
                nsec = int(row.get("stamp_nsec", "0"))
                t.append(float(sec) + float(nsec) * 1e-9)
            except Exception:
                continue
            try:
                rob.append(int(row.get("robot_index", "0")))
                obs.append(int(row.get("obstacle_index", "0")))
            except Exception:
                rob.append(rob[-1] if rob else 0)
                obs.append(obs[-1] if obs else 0)

    if not t:
        return False

    if t[0] > 0.0:
        t = [0.0] + t
        rob = [0] + rob
        obs = [0] + obs

    if cov_times:
        end_t = cov_times[-1]
        if t[-1] < end_t:
            t.append(end_t)
            rob.append(rob[-1])
            obs.append(obs[-1])

    fig, ax = plt.subplots(figsize=(9, 4))
    ax_cov = ax.twinx()

    ax.plot(t, rob, label="Robot collisions")
    ax.plot(t, obs, label="Obstacle collisions")
    ax.set_xlabel("Time (s)", fontsize=16)
    ax.set_ylabel("Collisions (cumulative)", fontsize=16)
    ax.set_xlim(left=0.0)
    ax.set_ylim(bottom=0.0)
    ax.grid(True, linestyle="--", alpha=0.4)

    if cov_times and covs:
        ax_cov.plot(cov_times, covs, color="tab:green", label="Coverage (%)")
        ax_cov.set_ylabel("Coverage (%)", fontsize=16)
        ax_cov.set_ylim(0, 105)

    handles, labels = [], []
    for axis in (ax, ax_cov):
        h, l = axis.get_legend_handles_labels()
        handles += h
        labels += l
    if handles:
        ax.legend(handles, labels, loc="upper left", fontsize=12)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return True


def analyze_run(run_id: str, run_dir: Path, root_dir: Path) -> bool:
    visited_csv = run_dir / "coverage_visited_cells.csv"
    if not visited_csv.exists():
        visited_csv = root_dir / f"{run_id}_coverage_visited_cells.csv"
    if not visited_csv.exists():
        print(f"[WARN] Missing {visited_csv}")
        return False

    visited = read_visited_cells(visited_csv)
    if not visited:
        print(f"[WARN] Empty visited cells in {visited_csv}")
        return False

    paths_csv = run_dir / "coverage_paths.csv"
    if not paths_csv.exists():
        paths_csv = root_dir / f"{run_id}_coverage_paths.csv"
    paths = read_robot_paths(paths_csv) if paths_csv.exists() else {}

    cells = build_cells(ENV_MIN, ENV_MAX)
    obstacles = load_obstacle_rectangles(WORLD_SDF)
    blocked = compute_blocked_cells(
        cells, obstacles, GRID_SIZE, OBSTACLE_OCCUPANCY_THRESHOLD
    )

    map_out = run_dir / "coverage_map_offline.png"
    plot_coverage_map(cells, visited, blocked, paths, map_out)

    cov_csv = run_dir / "coverage_timeseries.csv"
    if not cov_csv.exists():
        cov_csv = root_dir / f"{run_id}_coverage_timeseries.csv"
    cov_times, covs = ([], [])
    if cov_csv.exists():
        cov_times, covs = read_coverage_timeseries(cov_csv)

    bump_file = pick_matching_bump_file(run_dir, run_id)
    if bump_file is None:
        print(f"[WARN] No bump file found for {run_id}. Plotting zeros.")
        rows = []
    else:
        rows = read_bump_rows(bump_file)
        if not rows:
            print(f"[WARN] Bump file empty: {bump_file}. Plotting zeros.")

    out_png = run_dir / "collisions_vs_time_offline.png"
    if not plot_collisions(rows, cov_times, covs, out_png):
        print(f"[WARN] Could not plot collisions for {run_id}")
        return False

    print(f"[OK] {run_id}")
    print(f"  coverage_map: {map_out}")
    if bump_file is not None:
        print(f"  bump_file: {bump_file}")
    print(f"  collisions_plot: {out_png}")
    return True


def parse_args():
    parser = argparse.ArgumentParser(description="Generate offline plots for run folders.")
    parser.add_argument(
        "--results-dir",
        action="append",
        dest="results_dirs",
        help="Search root containing run_* folders. Can be passed multiple times.",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Analyze every run_* folder found under the given results dirs.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    results_dirs = [Path(p).resolve() for p in args.results_dirs] if args.results_dirs else RESULTS_DIRS

    if args.all:
        run_dirs = iter_run_dirs(results_dirs)
        if not run_dirs:
            joined = ", ".join(str(path) for path in results_dirs)
            raise SystemExit(f"No run folders found under {joined}")

        ok = 0
        for run_dir in run_dirs:
            if analyze_run(run_dir.name, run_dir, run_dir.parent):
                ok += 1
        print(f"[DONE] analyzed {ok}/{len(run_dirs)} runs")
        return

    run_id, run_dir, root_dir = pick_latest_run_dir(results_dirs)
    analyze_run(run_id, run_dir, root_dir)


if __name__ == "__main__":
    main()
