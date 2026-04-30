#!/usr/bin/env python3
from __future__ import annotations

import csv
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

if not matplotlib.get_backend().lower().startswith("agg"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


REPO_ROOT = Path(__file__).resolve().parent
WORLD_SDF = REPO_ROOT / "swarm_basics" / "worlds" / "random_world.sdf"
OUT_PNG = REPO_ROOT / "LATEX FIGURES" / "model_random_run_visited_cells_grid.png"
ENV_MIN = -5
ENV_MAX = 5
GRID_SIZE = 1.0
FONT_SIZE = 12

SELECTIONS = [
    ("gpt-4.1 best 1", "gpt-4.1/prompt_3/run_4/trial_1/run_20260428_231810", "p3-r4"),
    ("gpt-4.1 best 2", "gpt-4.1/prompt_1/run_1/trial_1/run_20260428_203118", "p1-r1"),
    ("gpt-4.1 worst 1", "gpt-4.1/prompt_1/run_5/trial_1/run_20260428_230259", "p1-r5"),
    ("gpt-4.1 worst 2", "gpt-4.1/prompt_4/run_1/trial_1/run_20260428_204628", "p4-r1"),
    ("gpt-5.4 best 1", "gpt-5.4/prompt_2/run_1/trial_1/run_20260428_193358", "p2-r1"),
    ("gpt-5.4 best 2", "gpt-5.4/prompt_4/run_3/trial_2/run_20260429_060210", "p4-r3"),
    ("gpt-5.4 worst 1", "gpt-5.4/prompt_3/run_4/trial_1/run_20260428_235838", "p3-r4"),
    ("gpt-5.4 worst 2", "gpt-5.4/prompt_4/run_2/trial_1/run_20260428_215847", "p4-r2"),
]


def parse_pose(text: str | None) -> tuple[float, float, float, float, float, float]:
    if not text:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    values = [float(value) for value in text.split()]
    values.extend([0.0] * (6 - len(values)))
    return tuple(values[:6])


def load_obstacle_shapes(world_sdf: Path) -> list[dict[str, float | str]]:
    if not world_sdf.exists():
        return []
    root = ET.parse(world_sdf).getroot()
    world = root.find("world")
    if world is None:
        return []

    obstacles: list[dict[str, float | str]] = []
    for model in world.findall("model"):
        model_name = model.get("name", "")
        if model_name == "ground_plane" or model_name.endswith("_wall"):
            continue
        model_pose = parse_pose(model.findtext("pose"))
        for link in model.findall("link"):
            link_pose = parse_pose(link.findtext("pose"))
            for collision in link.findall("collision"):
                collision_pose = parse_pose(collision.findtext("pose"))
                x = model_pose[0] + link_pose[0] + collision_pose[0]
                y = model_pose[1] + link_pose[1] + collision_pose[1]
                yaw = model_pose[5] + link_pose[5] + collision_pose[5]
                size_text = collision.findtext("geometry/box/size")
                if size_text:
                    sx, sy, _ = (float(value) for value in size_text.split())
                    obstacles.append({"shape": "box", "x": x, "y": y, "sx": sx, "sy": sy, "yaw": yaw})
                    continue
                radius_text = collision.findtext("geometry/cylinder/radius")
                if radius_text:
                    obstacles.append({"shape": "cylinder", "x": x, "y": y, "radius": float(radius_text)})
    return obstacles


def rect_corners(cx: float, cy: float, sx: float, sy: float, yaw: float) -> list[tuple[float, float]]:
    half_x = sx / 2.0
    half_y = sy / 2.0
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return [
        (cx + (dx * cos_yaw - dy * sin_yaw), cy + (dx * sin_yaw + dy * cos_yaw))
        for dx, dy in ((-half_x, -half_y), (half_x, -half_y), (half_x, half_y), (-half_x, half_y))
    ]


def empty_grid_counts() -> list[list[int]]:
    width = int((ENV_MAX - ENV_MIN) / GRID_SIZE)
    return [[0 for _ in range(width)] for _ in range(width)]


def read_path_grid_counts(paths_csv: Path) -> list[list[int]]:
    counts = empty_grid_counts()
    with paths_csv.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                x = float(row["x"])
                y = float(row["y"])
            except (KeyError, ValueError):
                continue
            if not (ENV_MIN <= x < ENV_MAX and ENV_MIN <= y < ENV_MAX):
                continue
            col = int(math.floor((x - ENV_MIN) / GRID_SIZE))
            row_idx = int(math.floor((y - ENV_MIN) / GRID_SIZE))
            counts[row_idx][col] += 1
    return counts


def read_final_coverage(coverage_csv: Path) -> float:
    final = None
    with coverage_csv.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            final = row
    if final is None:
        return 0.0
    return float(final.get("coverage_pct", final.get("coverage", 0.0)))


def read_total_collisions(bump_csv: Path) -> int:
    total = 0
    with bump_csv.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                total = int(row.get("total_index", total))
            except ValueError:
                continue
    return total


def draw_panel(ax: plt.Axes, run_dir: Path, title: str, short_label: str, obstacles: list[dict[str, float | str]]) -> None:
    counts = read_path_grid_counts(run_dir / "coverage_paths.csv")
    for row_idx, row in enumerate(counts):
        for col_idx, value in enumerate(row):
            x = ENV_MIN + col_idx * GRID_SIZE
            y = ENV_MIN + row_idx * GRID_SIZE
            rect = plt.Rectangle(
                (x, y),
                GRID_SIZE,
                GRID_SIZE,
                facecolor="white",
                edgecolor="#d9d9d9",
                linewidth=0.8,
            )
            if value > 0:
                rect.set_facecolor("#79b878")
                rect.set_edgecolor("#79b878")
                rect.set_alpha(0.70)
            ax.add_patch(rect)

    for obstacle in obstacles:
        if obstacle["shape"] == "box":
            patch = Polygon(
                rect_corners(
                    float(obstacle["x"]),
                    float(obstacle["y"]),
                    float(obstacle["sx"]),
                    float(obstacle["sy"]),
                    float(obstacle["yaw"]),
                ),
                closed=True,
                facecolor="#7b2cbf",
                edgecolor="#5a189a",
                linewidth=1.0,
                alpha=0.75,
                zorder=4,
            )
        else:
            patch = plt.Circle(
                (float(obstacle["x"]), float(obstacle["y"])),
                float(obstacle["radius"]),
                facecolor="#7b2cbf",
                edgecolor="#5a189a",
                linewidth=1.0,
                alpha=0.75,
                zorder=4,
            )
        ax.add_patch(patch)

    coverage = round(read_final_coverage(run_dir / "coverage_timeseries.csv"))
    bump_files = sorted(run_dir.glob("bumps_global_*.csv"))
    collisions = read_total_collisions(bump_files[-1]) if bump_files else 0
    ax.set_xlim(ENV_MIN, ENV_MAX)
    ax.set_ylim(ENV_MIN, ENV_MAX)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.text(
        0.5,
        -0.12,
        f"{title}\n{short_label}  cov={coverage}%  col={collisions}",
        transform=ax.transAxes,
        ha="center",
        va="top",
        fontsize=FONT_SIZE,
    )


def main() -> None:
    obstacles = load_obstacle_shapes(WORLD_SDF)
    fig, axes = plt.subplots(2, 4, figsize=(16, 9.0))
    for ax, (title, rel_run_dir, short_label) in zip(axes.flat, SELECTIONS):
        draw_panel(ax, REPO_ROOT / "LAST_PROMPTS" / rel_run_dir, title, short_label, obstacles)

    visited_patch = plt.Rectangle((0, 0), 1, 1, facecolor="#79b878", edgecolor="black", linewidth=1.0, alpha=0.70)
    obstacle_patch = plt.Rectangle((0, 0), 1, 1, facecolor="#7b2cbf", edgecolor="#5a189a", linewidth=1.0, alpha=0.75)
    fig.legend(
        [visited_patch, obstacle_patch],
        ["visited", "obstacle"],
        loc="lower center",
        ncol=2,
        frameon=False,
        fontsize=FONT_SIZE,
    )
    fig.subplots_adjust(left=0.02, right=0.99, top=0.98, bottom=0.12, wspace=0.20, hspace=0.34)
    fig.savefig(OUT_PNG, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] wrote {OUT_PNG}")


if __name__ == "__main__":
    main()
