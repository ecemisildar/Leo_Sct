#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

if not matplotlib.get_backend().lower().startswith("agg"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt


REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_ANALYSIS_ROOT = REPO_ROOT / "new_videos_analysis_out" / "window_trim10s_len180s"
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "new_videos_analysis_out"
PLOT_FONT_SIZE = 12


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create box plots for final coverage and collision counts across analyzed videos."
    )
    parser.add_argument("--analysis-root", type=Path, default=DEFAULT_ANALYSIS_ROOT)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--tag", default="new_videos_trim10s_len180s")
    return parser.parse_args()


def read_final_coverage(coverage_csv: Path) -> tuple[float, int]:
    final_row: dict[str, str] | None = None
    with coverage_csv.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            final_row = row
    if final_row is None:
        raise RuntimeError(f"No coverage rows found in {coverage_csv}")
    return float(final_row["coverage_pct"]), int(final_row["visited_cells"])


def read_final_collisions(bumps_csv: Path) -> tuple[int, int, int]:
    robot_collisions = 0
    obstacle_collisions = 0
    total_collisions = 0
    with bumps_csv.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            total_collisions += 1
            bump_type = row.get("bump_type", "")
            if bump_type == "robot":
                robot_collisions += 1
            elif bump_type == "obstacle":
                obstacle_collisions += 1
    return total_collisions, robot_collisions, obstacle_collisions


def collect_metrics(analysis_root: Path) -> list[dict[str, str | int | float]]:
    rows: list[dict[str, str | int | float]] = []
    for video_dir in sorted(path for path in analysis_root.iterdir() if path.is_dir()):
        coverage_csv = video_dir / "coverage_timeseries.csv"
        bump_files = sorted(video_dir.glob("bumps_global_*.csv"))
        if not coverage_csv.exists() or not bump_files:
            continue
        final_coverage, visited_cells = read_final_coverage(coverage_csv)
        total_collisions, robot_collisions, obstacle_collisions = read_final_collisions(bump_files[-1])
        rows.append(
            {
                "video": video_dir.name,
                "coverage_pct": final_coverage,
                "visited_cells": visited_cells,
                "total_collisions": total_collisions,
                "robot_collisions": robot_collisions,
                "obstacle_collisions": obstacle_collisions,
                "bumps_csv": str(bump_files[-1]),
            }
        )
    return rows


def write_metrics_csv(rows: list[dict[str, str | int | float]], out_csv: Path) -> None:
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "video",
        "coverage_pct",
        "visited_cells",
        "total_collisions",
        "robot_collisions",
        "obstacle_collisions",
        "bumps_csv",
    ]
    with out_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def style_axes(ax: plt.Axes) -> None:
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(axis="y", linestyle="--", alpha=0.25)
    ax.xaxis.label.set_size(PLOT_FONT_SIZE)
    ax.yaxis.label.set_size(PLOT_FONT_SIZE)
    ax.tick_params(axis="both", labelsize=PLOT_FONT_SIZE)


def plot_boxplots(rows: list[dict[str, str | int | float]], out_png: Path) -> None:
    coverage = [float(row["coverage_pct"]) for row in rows]
    total = [int(row["total_collisions"]) for row in rows]
    robot = [int(row["robot_collisions"]) for row in rows]
    obstacle = [int(row["obstacle_collisions"]) for row in rows]

    fig, (ax_cov, ax_col) = plt.subplots(1, 2, figsize=(10.5, 4.8))

    ax_cov.boxplot(
        [coverage],
        labels=["Coverage"],
        patch_artist=True,
        boxprops={"facecolor": "#dce8d8", "edgecolor": "#4f6f52"},
        medianprops={"color": "#2f4f32", "linewidth": 2.0},
        whiskerprops={"color": "#4f6f52"},
        capprops={"color": "#4f6f52"},
    )
    ax_cov.scatter([1] * len(coverage), coverage, color="#4f6f52", s=24, alpha=0.75, zorder=3)
    ax_cov.set_ylabel("Coverage (%)")
    ax_cov.set_ylim(0.0, 100.0)
    style_axes(ax_cov)

    ax_col.boxplot(
        [total, robot, obstacle],
        labels=["Total", "Robot", "Obstacle"],
        patch_artist=True,
        boxprops={"facecolor": "#dce3ee", "edgecolor": "#4c78a8"},
        medianprops={"color": "#244b72", "linewidth": 2.0},
        whiskerprops={"color": "#4c78a8"},
        capprops={"color": "#4c78a8"},
    )
    for x_pos, values in enumerate([total, robot, obstacle], start=1):
        ax_col.scatter([x_pos] * len(values), values, color="#4c78a8", s=24, alpha=0.75, zorder=3)
    ax_col.set_ylabel("Collisions")
    ax_col.set_ylim(bottom=0.0)
    style_axes(ax_col)

    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    args = parse_args()
    rows = collect_metrics(args.analysis_root.resolve())
    if not rows:
        raise SystemExit(f"No complete video analyses found under {args.analysis_root}")

    output_root = args.output_root.resolve()
    metrics_csv = output_root / f"boxplot_metrics_{args.tag}.csv"
    boxplot_png = output_root / f"coverage_collisions_boxplot_{args.tag}.png"
    write_metrics_csv(rows, metrics_csv)
    plot_boxplots(rows, boxplot_png)

    print(f"[OK] videos: {len(rows)}")
    print(f"[OK] metrics_csv: {metrics_csv}")
    print(f"[OK] boxplot_png: {boxplot_png}")


if __name__ == "__main__":
    main()
