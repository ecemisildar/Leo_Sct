#!/usr/bin/env python3
import argparse
import csv
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def _read_rows(csv_path: Path):
    with csv_path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    for row in rows:
        row["time_s"] = float(row.get("stamp_sec", 0.0)) + float(row.get("stamp_nsec", 0.0)) * 1e-9
    return rows


def _barh(ax, counter: Counter, title: str, color: str):
    if not counter:
        ax.text(0.5, 0.5, "no data", ha="center", va="center")
        ax.set_title(title)
        return
    items = sorted(counter.items(), key=lambda item: item[1])
    labels = [item[0] for item in items]
    values = [item[1] for item in items]
    ax.barh(labels, values, color=color)
    ax.set_title(title)
    ax.set_xlabel("count")
    for idx, value in enumerate(values):
        ax.text(value + 0.05, idx, str(value), va="center", fontsize=9)


def plot_bump_reasons(csv_path: Path, output_path: Path):
    rows = _read_rows(csv_path)
    obstacle_rows = [row for row in rows if row.get("bump_type") == "obstacle"]
    robot_rows = [row for row in rows if row.get("bump_type") == "robot"]

    summary_counts = Counter(row.get("pair_detection_summary") or "missing" for row in rows)
    obstacle_summary_counts = Counter(row.get("pair_detection_summary") or "missing" for row in obstacle_rows)
    robot_summary_counts = Counter(row.get("pair_detection_summary") or "missing" for row in robot_rows)
    entity_a_dirs = Counter(row.get("entity_a_bump_direction") or "unknown" for row in rows)
    entity_b_dirs = Counter(row.get("entity_b_bump_direction") or "none/obstacle" for row in rows)

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle(f"Bump reasons summary: {csv_path.parent.name}", fontsize=15, fontweight="bold")

    _barh(axes[0, 0], summary_counts, "All bump detection summaries", "#4c78a8")
    _barh(axes[0, 1], obstacle_summary_counts or robot_summary_counts, "Obstacle summaries", "#f58518")

    direction_order = ["front", "left", "right", "back", "unknown", "none/obstacle"]
    labels = [label for label in direction_order if entity_a_dirs.get(label, 0) or entity_b_dirs.get(label, 0)]
    a_values = [entity_a_dirs.get(label, 0) for label in labels]
    b_values = [entity_b_dirs.get(label, 0) for label in labels]
    x_values = list(range(len(labels)))
    axes[1, 0].bar([x - 0.18 for x in x_values], a_values, width=0.36, label="entity_a", color="#54a24b")
    axes[1, 0].bar([x + 0.18 for x in x_values], b_values, width=0.36, label="entity_b", color="#e45756")
    axes[1, 0].set_xticks(x_values, labels, rotation=25, ha="right")
    axes[1, 0].set_title("Bump direction by CSV entity side")
    axes[1, 0].set_ylabel("count")
    axes[1, 0].legend()

    y_positions = {"obstacle": 0, "robot": 1}
    colors = {"obstacle": "#f58518", "robot": "#4c78a8"}
    for row in rows:
        bump_type = row.get("bump_type", "")
        y = y_positions.get(bump_type, 0.5)
        axes[1, 1].scatter(row["time_s"], y, s=70, color=colors.get(bump_type, "gray"))
        label = (row.get("pair_detection_summary") or "").replace("robot_obstacle_", "")
        label = label.replace("|no_pair_warning", "")
        axes[1, 1].text(row["time_s"], y + 0.06, label, rotation=45, ha="left", va="bottom", fontsize=7)
    axes[1, 1].set_yticks([0, 1], ["obstacle", "robot"])
    axes[1, 1].set_xlabel("simulation time (s)")
    axes[1, 1].set_title("Bumps over time")
    axes[1, 1].grid(axis="x", alpha=0.25)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)
    return rows, summary_counts


def parse_args():
    parser = argparse.ArgumentParser(description="Plot bump reason summaries from a bumps_global CSV.")
    parser.add_argument("csv_path", type=Path, help="Path to bumps_global_*.csv")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output PNG path. Defaults to bump_reasons_summary.png beside the CSV.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    output = args.output or args.csv_path.with_name("bump_reasons_summary.png")
    rows, summary_counts = plot_bump_reasons(args.csv_path, output)
    print(output)
    print(f"total={len(rows)} summaries={dict(summary_counts)}")


if __name__ == "__main__":
    main()
