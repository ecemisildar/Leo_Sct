#!/usr/bin/env python3
import argparse
import csv
import os
import math
import random
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import cm, colors
from matplotlib.patches import Polygon


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RESULTS_DIR = PROJECT_ROOT / "results_exp" / "explore"
DEFAULT_OUT_DIR = DEFAULT_RESULTS_DIR / "comparisons"
DEFAULT_CATEGORIES = ["long", "medium", "short"]
WORLD_SDF = PROJECT_ROOT / "swarm_basics" / "worlds" / "random_world.sdf"
ENV_MIN = -5
ENV_MAX = 5
GRID_SIZE = 1.0
CATEGORY_DIR_CANDIDATES = {
    "long": ["long"],
    "medium": ["medium"],
    "short": ["short", "vague"],
    "vague": ["short", "vague"],
}


def format_coverage_tag(min_coverage: float) -> str:
    if float(min_coverage).is_integer():
        return f"mincov_{int(min_coverage)}"
    return f"mincov_{str(min_coverage).replace('.', 'p')}"


def format_scenario_label(scenario: str) -> str:
    return scenario.replace("_", " ")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compare prompt categories across trials and plot per-trial results."
    )
    parser.add_argument(
        "--results-dir",
        default=str(DEFAULT_RESULTS_DIR),
        help="Root directory containing category folders such as long/medium/short.",
    )
    parser.add_argument(
        "--out-dir",
        default=str(DEFAULT_OUT_DIR),
        help="Directory where plots and summary CSV will be written.",
    )
    parser.add_argument(
        "--categories",
        nargs="+",
        default=DEFAULT_CATEGORIES,
        help="Prompt categories to compare.",
    )
    parser.add_argument(
        "--scenario",
        default="middle_circle",
        help="Scenario folder to include, e.g. middle_circle or spread.",
    )
    parser.add_argument(
        "--min-coverage",
        type=float,
        default=10.0,
        help="Success threshold for maximum achieved coverage percentage. Success means strictly greater than this value.",
    )
    parser.add_argument(
        "--runs",
        nargs="+",
        default=None,
        help="Optional run directories to include, e.g. run_1 run_2 run_3.",
    )
    return parser.parse_args()


def read_max_coverage(coverage_csv: Path) -> float | None:
    max_coverage = None
    with coverage_csv.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            value = row.get("coverage_pct", "")
            if value == "":
                continue
            try:
                coverage = float(value)
            except ValueError:
                continue
            max_coverage = coverage if max_coverage is None else max(max_coverage, coverage)
    return max_coverage


def read_total_collisions(bump_csv: Path | None) -> int:
    if bump_csv is None or not bump_csv.exists():
        return 0
    last = 0
    with bump_csv.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            value = row.get("total_index", "")
            if value == "":
                continue
            try:
                last = int(value)
            except ValueError:
                continue
    return last


def find_bump_csv(run_dir: Path) -> Path | None:
    bump_files = sorted(run_dir.glob("bumps_*.csv"))
    return bump_files[-1] if bump_files else None


def collect_rows(
    results_dir: Path,
    categories: list[str],
    scenario: str,
    runs_filter: set[str] | None = None,
) -> list[dict]:
    rows = []
    for category in categories:
        candidates = CATEGORY_DIR_CANDIDATES.get(category, [category])
        category_dir = next((results_dir / name for name in candidates if (results_dir / name).exists()), None)
        if category_dir is None:
            continue
        if not category_dir.exists():
            continue
        for prompt_dir in sorted(category_dir.glob("prompt_*")):
            for run_dir in sorted(prompt_dir.glob("run_*")):
                if runs_filter is not None and run_dir.name not in runs_filter:
                    continue
                scenario_dir = run_dir / scenario
                if not scenario_dir.exists():
                    continue
                for trial_dir in sorted(scenario_dir.glob("trial_*")):
                    run_folders = sorted(
                        p for p in trial_dir.glob("run_*")
                        if p.is_dir() and (p / "coverage_timeseries.csv").exists()
                    )
                    if not run_folders:
                        continue
                    run_folder = run_folders[-1]
                    coverage_csv = run_folder / "coverage_timeseries.csv"
                    coverage = read_max_coverage(coverage_csv)
                    if coverage is None:
                        continue
                    bump_csv = find_bump_csv(run_folder)
                    rows.append(
                        {
                            "category": category,
                            "prompt": prompt_dir.name,
                            "run": run_dir.name,
                            "scenario": scenario,
                            "trial": trial_dir.name,
                            "run_folder": run_folder.name,
                            "coverage_pct": coverage,
                            "collisions_total": read_total_collisions(bump_csv),
                            "coverage_csv": str(coverage_csv),
                            "paths_csv": str(run_folder / "coverage_paths.csv"),
                            "bump_csv": str(bump_csv) if bump_csv else "",
                        }
                    )
    return rows


def write_summary_csv(rows: list[dict], out_csv: Path):
    fieldnames = [
        "category",
        "prompt",
        "run",
        "scenario",
        "trial",
        "run_folder",
        "coverage_pct",
        "collisions_total",
        "coverage_csv",
        "paths_csv",
        "bump_csv",
    ]
    with out_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def filter_rows_by_coverage(rows: list[dict], min_coverage: float) -> tuple[list[dict], list[dict]]:
    kept = [row for row in rows if row["coverage_pct"] > min_coverage]
    discarded = [row for row in rows if row["coverage_pct"] <= min_coverage]
    return kept, discarded


def write_discarded_summary(
    all_rows: list[dict],
    discarded_rows: list[dict],
    categories: list[str],
    min_coverage: float,
    out_csv: Path,
):
    fieldnames = [
        "category",
        "total_trials",
        "discarded_trials",
        "kept_trials",
        "discarded_pct",
        "min_coverage",
    ]
    with out_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for category in categories:
            total = sum(1 for row in all_rows if row["category"] == category)
            discarded = sum(1 for row in discarded_rows if row["category"] == category)
            kept = total - discarded
            discarded_pct = (100.0 * discarded / total) if total else 0.0
            writer.writerow(
                {
                    "category": category,
                    "total_trials": total,
                    "discarded_trials": discarded,
                    "kept_trials": kept,
                    "discarded_pct": f"{discarded_pct:.2f}",
                    "min_coverage": f"{min_coverage:.1f}",
                }
            )


def plot_success_counts(
    all_rows: list[dict],
    discarded_rows: list[dict],
    categories: list[str],
    min_coverage: float,
    out_png: Path | None = None,
    ax=None,
):
    color_map = {
        "long": "#1f77b4",
        "medium": "#ff7f0e",
        "short": "#2ca02c",
    }
    totals = [sum(1 for row in all_rows if row["category"] == category) for category in categories]
    discarded = [sum(1 for row in discarded_rows if row["category"] == category) for category in categories]
    kept = [total - dropped for total, dropped in zip(totals, discarded)]

    if ax is None:
        fig, ax = plt.subplots(figsize=(9, 5))
    else:
        fig = ax.figure
    positions = list(range(len(categories)))
    success_bars = ax.bar(
        positions,
        kept,
        color=[color_map.get(category, "#999999") for category in categories],
        alpha=0.8,
        width=0.6,
        label=f"Max coverage > {min_coverage:.1f}%",
    )
    discarded_bars = ax.bar(
        positions,
        discarded,
        bottom=kept,
        color="#c7c7c7",
        alpha=0.9,
        width=0.6,
        label=f"Coverage <= {min_coverage:.1f}%",
    )
    ax.set_xticks(positions)
    ax.set_xticklabels(categories, fontsize=12)
    ax.set_ylabel("Trials", fontsize=13)
    ax.set_xlabel("Prompt Category", fontsize=13)
    ax.grid(True, axis="y", linestyle="--", alpha=0.35)
    for success_bar, discarded_bar, total, passed, dropped in zip(success_bars, discarded_bars, totals, kept, discarded):
        pct = (100.0 * passed / total) if total else 0.0
        label = f"{pct:.1f}%"
        ax.text(
            success_bar.get_x() + success_bar.get_width() / 2.0,
            success_bar.get_height(),
            label,
            ha="center",
            va="bottom",
            fontsize=11,
        )

    if out_png is not None:
        fig.tight_layout()
        fig.savefig(out_png, dpi=200, bbox_inches="tight")
        plt.close(fig)


def plot_metric(
    rows: list[dict],
    categories: list[str],
    metric: str,
    ylabel: str,
    out_png: Path | None = None,
    ax=None,
):
    color_map = {
        "long": "#1f77b4",
        "medium": "#ff7f0e",
        "short": "#2ca02c",
    }
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 6))
    else:
        fig = ax.figure

    positions = list(range(len(categories)))
    grouped = []
    for category in categories:
        values = [row[metric] for row in rows if row["category"] == category]
        grouped.append(values)

    box = ax.boxplot(
        grouped,
        positions=positions,
        widths=0.5,
        patch_artist=True,
        showfliers=False,
        medianprops={"color": "black", "linewidth": 1.5},
    )

    for patch, category in zip(box["boxes"], categories):
        patch.set_facecolor(color_map.get(category, "#999999"))
        patch.set_alpha(0.25)
        patch.set_edgecolor(color_map.get(category, "#999999"))

    for idx, category in enumerate(categories):
        category_rows = [row for row in rows if row["category"] == category]
        offsets = []
        if category_rows:
            # Deterministic spread so each trial is visible without randomness.
            count = len(category_rows)
            if count == 1:
                offsets = [0.0]
            else:
                step = 0.36 / (count - 1)
                offsets = [-0.18 + step * i for i in range(count)]
        for offset, row in zip(offsets, category_rows):
            ax.scatter(
                idx + offset,
                row[metric],
                color=color_map.get(category, "#999999"),
                edgecolors="black",
                linewidths=0.5,
                s=45,
                zorder=3,
            )

    ax.set_xticks(positions)
    ax.set_xticklabels(categories, fontsize=12)
    ax.set_ylabel(ylabel, fontsize=13)
    ax.set_xlabel("Prompt Category", fontsize=13)
    ax.grid(True, axis="y", linestyle="--", alpha=0.35)
    if out_png is not None:
        fig.tight_layout()
        fig.savefig(out_png, dpi=200, bbox_inches="tight")
        plt.close(fig)


def parse_pose(pose_text: str | None) -> tuple[float, float, float, float, float, float]:
    if not pose_text:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    values = [float(value) for value in pose_text.split()]
    while len(values) < 6:
        values.append(0.0)
    return tuple(values[:6])


def load_obstacle_shapes(world_sdf: Path) -> list[dict]:
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
        model_name = model.get("name", "")
        if model_name == "ground_plane" or model_name.endswith("_wall"):
            continue
        model_pose = parse_pose(model.findtext("pose"))
        for link in model.findall("link"):
            link_pose = parse_pose(link.findtext("pose"))
            for collision in link.findall("collision"):
                collision_pose = parse_pose(collision.findtext("pose"))
                size_text = collision.findtext("geometry/box/size")
                x = model_pose[0] + link_pose[0] + collision_pose[0]
                y = model_pose[1] + link_pose[1] + collision_pose[1]
                yaw = model_pose[5] + link_pose[5] + collision_pose[5]
                if size_text:
                    sx, sy, _ = (float(value) for value in size_text.split())
                    obstacles.append(
                        {
                            "shape": "box",
                            "x": x,
                            "y": y,
                            "sx": sx,
                            "sy": sy,
                            "yaw": yaw,
                        }
                    )
                    continue

                radius_text = collision.findtext("geometry/cylinder/radius")
                if radius_text:
                    obstacles.append(
                        {
                            "shape": "cylinder",
                            "x": x,
                            "y": y,
                            "radius": float(radius_text),
                        }
                    )
    return obstacles


def rect_corners(cx: float, cy: float, sx: float, sy: float, yaw: float) -> list[tuple[float, float]]:
    half_x = sx / 2.0
    half_y = sy / 2.0
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    corners = []
    for dx, dy in ((-half_x, -half_y), (half_x, -half_y), (half_x, half_y), (-half_x, half_y)):
        x = cx + (dx * cos_yaw - dy * sin_yaw)
        y = cy + (dx * sin_yaw + dy * cos_yaw)
        corners.append((x, y))
    return corners


def empty_grid_counts() -> list[list[int]]:
    width = int((ENV_MAX - ENV_MIN) / GRID_SIZE)
    return [[0 for _ in range(width)] for _ in range(width)]


def read_path_grid_counts(paths_csv: Path) -> list[list[int]]:
    counts = empty_grid_counts()
    if not paths_csv.exists():
        return counts

    with paths_csv.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            x_text = row.get("x", "")
            y_text = row.get("y", "")
            if x_text == "" or y_text == "":
                continue
            try:
                x = float(x_text)
                y = float(y_text)
            except ValueError:
                continue
            if not (ENV_MIN <= x < ENV_MAX and ENV_MIN <= y < ENV_MAX):
                continue
            col = int(math.floor((x - ENV_MIN) / GRID_SIZE))
            row_idx = int(math.floor((y - ENV_MIN) / GRID_SIZE))
            counts[row_idx][col] += 1
    return counts


def merge_grid_counts(all_counts: list[list[list[int]]]) -> list[list[int]]:
    merged = empty_grid_counts()
    for counts in all_counts:
        for row_idx, row in enumerate(counts):
            for col_idx, value in enumerate(row):
                merged[row_idx][col_idx] += value
    return merged


def choose_sample_row(rows: list[dict], category: str, scenario: str) -> dict | None:
    if not rows:
        return None
    rng = random.Random(f"{category}:{scenario}")
    return rng.choice(rows)


def draw_category_heatmap(
    ax,
    category: str,
    scenario: str,
    counts: list[list[int]],
    obstacles: list[dict],
    sample_counts: list[list[int]] | None = None,
):
    scenario_label = format_scenario_label(scenario)
    max_count = max((max(row) for row in counts), default=0)
    cmap = cm.get_cmap("Greens")
    norm = colors.Normalize(vmin=1, vmax=max_count if max_count > 0 else 1)

    for row_idx, row in enumerate(counts):
        for col_idx, value in enumerate(row):
            x = ENV_MIN + col_idx * GRID_SIZE
            y = ENV_MIN + row_idx * GRID_SIZE
            facecolor = "white" if value == 0 else cmap(norm(value))
            rect = plt.Rectangle(
                (x, y),
                GRID_SIZE,
                GRID_SIZE,
                facecolor=facecolor,
                edgecolor="#d9d9d9",
                linewidth=0.8,
            )
            ax.add_patch(rect)

    if sample_counts is not None:
        for row_idx, row in enumerate(sample_counts):
            for col_idx, value in enumerate(row):
                if value <= 0:
                    continue
                x = ENV_MIN + col_idx * GRID_SIZE
                y = ENV_MIN + row_idx * GRID_SIZE
                rect = plt.Rectangle(
                    (x, y),
                    GRID_SIZE,
                    GRID_SIZE,
                    facecolor="#d62828",
                    edgecolor="#9d0208",
                    linewidth=1.2,
                    alpha=0.35,
                    zorder=3,
                )
                ax.add_patch(rect)

    for obstacle in obstacles:
        if obstacle["shape"] == "box":
            patch = Polygon(
                rect_corners(
                    obstacle["x"],
                    obstacle["y"],
                    obstacle["sx"],
                    obstacle["sy"],
                    obstacle["yaw"],
                ),
                closed=True,
                facecolor="#7b2cbf",
                edgecolor="#5a189a",
                linewidth=1.0,
                alpha=0.9,
                zorder=4,
            )
        else:
            patch = plt.Circle(
                (obstacle["x"], obstacle["y"]),
                obstacle["radius"],
                facecolor="#7b2cbf",
                edgecolor="#5a189a",
                linewidth=1.0,
                alpha=0.9,
                zorder=4,
            )
        ax.add_patch(patch)

    ax.set_xlim(ENV_MIN, ENV_MAX)
    ax.set_ylim(ENV_MIN, ENV_MAX)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)", fontsize=13)
    ax.set_ylabel("Y (m)", fontsize=13)
    ax.set_title(f"{category} {scenario_label} visit heatmap", fontsize=14)
    if sample_counts is not None:
        ax.text(
            0.02,
            0.98,
            "Red overlay: one random run",
            transform=ax.transAxes,
            ha="left",
            va="top",
            fontsize=10,
            color="#9d0208",
            bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.8, "pad": 2.5},
            zorder=6,
        )

    if max_count > 0:
        sm = cm.ScalarMappable(norm=norm, cmap=cmap)
        sm.set_array([])
        cbar = ax.figure.colorbar(sm, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label("Visit count", fontsize=12)


def plot_category_heatmap(
    category: str,
    scenario: str,
    counts: list[list[int]],
    obstacles: list[dict],
    sample_counts: list[list[int]] | None,
    out_png: Path,
):
    fig, ax = plt.subplots(figsize=(8, 8))
    draw_category_heatmap(ax, category, scenario, counts, obstacles, sample_counts=sample_counts)

    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)


def write_category_heatmaps(rows: list[dict], categories: list[str], scenario: str, out_dir: Path):
    obstacles = load_obstacle_shapes(WORLD_SDF)
    for category in categories:
        category_rows = [row for row in rows if row["category"] == category]
        if not category_rows:
            continue
        sample_row = choose_sample_row(category_rows, category, scenario)
        merged_counts = merge_grid_counts(
            [read_path_grid_counts(Path(row["paths_csv"])) for row in category_rows]
        )
        sample_counts = read_path_grid_counts(Path(sample_row["paths_csv"])) if sample_row else None
        out_png = out_dir / f"category_heatmap_{category}_{scenario}.png"
        plot_category_heatmap(category, scenario, merged_counts, obstacles, sample_counts, out_png)
        print(f"[OK] wrote {out_png}")


def write_combined_heatmap_grid(
    results_dir: Path,
    categories: list[str],
    scenarios: list[str],
    out_dir: Path,
    runs_filter: set[str] | None = None,
):
    obstacles = load_obstacle_shapes(WORLD_SDF)
    fig, axes = plt.subplots(len(scenarios), len(categories), figsize=(5 * len(categories), 5 * len(scenarios)))

    if len(scenarios) == 1 and len(categories) == 1:
        axes = [[axes]]
    elif len(scenarios) == 1:
        axes = [axes]
    elif len(categories) == 1:
        axes = [[ax] for ax in axes]

    for row_idx, scenario in enumerate(scenarios):
        scenario_rows = collect_rows(results_dir, categories, scenario, runs_filter=runs_filter)
        for col_idx, category in enumerate(categories):
            ax = axes[row_idx][col_idx]
            category_rows = [row for row in scenario_rows if row["category"] == category]
            sample_row = choose_sample_row(category_rows, category, scenario)
            merged_counts = merge_grid_counts(
                [read_path_grid_counts(Path(row["paths_csv"])) for row in category_rows]
            )
            sample_counts = read_path_grid_counts(Path(sample_row["paths_csv"])) if sample_row else None
            draw_category_heatmap(ax, category, scenario, merged_counts, obstacles, sample_counts=sample_counts)

    fig.tight_layout()
    out_png = out_dir / "category_heatmap_grid.png"
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] wrote {out_png}")


def write_combined_prompt_comparison_grid(
    results_dir: Path,
    categories: list[str],
    scenarios: list[str],
    out_dir: Path,
    runs_filter: set[str] | None = None,
):
    fig, axes = plt.subplots(len(scenarios), 2, figsize=(12, 9))

    for row_idx, scenario in enumerate(scenarios):
        scenario_label = format_scenario_label(scenario)
        all_rows = collect_rows(results_dir, categories, scenario, runs_filter=runs_filter)

        plot_metric(
            all_rows,
            categories,
            "coverage_pct",
            "Max Coverage Achieved (%)",
            ax=axes[row_idx][0],
        )
        axes[row_idx][0].set_title(f"{scenario_label} coverage", fontsize=14)

        plot_metric(
            all_rows,
            categories,
            "collisions_total",
            "Total Collisions",
            ax=axes[row_idx][1],
        )
        axes[row_idx][1].set_title(f"{scenario_label} collisions", fontsize=14)

    fig.tight_layout()
    out_png = out_dir / "prompt_comparison_grid.png"
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] wrote {out_png}")


def write_combined_success_grid(
    results_dir: Path,
    categories: list[str],
    scenarios: list[str],
    min_coverage: float,
    coverage_tag: str,
    out_dir: Path,
    runs_filter: set[str] | None = None,
):
    color_map = {
        "long": "#1f77b4",
        "medium": "#ff7f0e",
        "short": "#2ca02c",
    }
    scenario_stats = {}
    for scenario in scenarios:
        all_rows = collect_rows(results_dir, categories, scenario, runs_filter=runs_filter)
        _, discarded_rows = filter_rows_by_coverage(all_rows, min_coverage)
        scenario_stats[scenario] = {
            category: (
                sum(1 for row in all_rows if row["category"] == category)
                - sum(1 for row in discarded_rows if row["category"] == category),
                sum(1 for row in discarded_rows if row["category"] == category),
                sum(1 for row in all_rows if row["category"] == category),
            )
            for category in categories
        }

    fig, ax = plt.subplots(figsize=(12, 6))
    positions = list(range(len(categories)))
    width = 0.24
    offsets = [-width / 1.5, width / 1.5] if len(scenarios) == 2 else [0.0]

    for scenario, offset in zip(scenarios, offsets):
        kept_values = [scenario_stats[scenario][category][0] for category in categories]
        discarded_values = [scenario_stats[scenario][category][1] for category in categories]
        totals = [scenario_stats[scenario][category][2] for category in categories]
        success_bars = ax.bar(
            [pos + offset for pos in positions],
            kept_values,
            width=width,
            color=[color_map.get(category, "#999999") for category in categories],
            alpha=0.85 if scenario == scenarios[0] else 0.45,
        )
        discarded_bars = ax.bar(
            [pos + offset for pos in positions],
            discarded_values,
            width=width,
            bottom=kept_values,
            color="#c7c7c7",
            alpha=0.9 if scenario == scenarios[0] else 0.6,
        )
        for bar, top_bar, kept, dropped, total in zip(success_bars, discarded_bars, kept_values, discarded_values, totals):
            pct = (100.0 * kept / total) if total else 0.0
            ax.text(
                bar.get_x() + bar.get_width() / 2.0,
                bar.get_height(),
                f"{pct:.1f}%",
                ha="center",
                va="bottom",
                fontsize=10,
            )

    ax.set_xticks(positions)
    ax.set_xticklabels(categories, fontsize=11)
    ax.set_ylabel("Trials", fontsize=13)
    ax.set_xlabel("Prompt Category", fontsize=13)
    ax.grid(True, axis="y", linestyle="--", alpha=0.35)
    fig.tight_layout()
    out_png = out_dir / f"prompt_comparison_success_grid_{coverage_tag}.png"
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] wrote {out_png}")


def main():
    args = parse_args()
    results_dir = Path(args.results_dir).resolve()
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    runs_filter = set(args.runs) if args.runs else None

    all_rows = collect_rows(results_dir, args.categories, args.scenario, runs_filter=runs_filter)
    if not all_rows:
        raise SystemExit(
            f"No matching runs found under {results_dir} for categories={args.categories} "
            f"and scenario={args.scenario}"
        )
    success_rows, discarded_rows = filter_rows_by_coverage(all_rows, args.min_coverage)
    dropped = len(discarded_rows)

    coverage_tag = format_coverage_tag(args.min_coverage)
    summary_csv = out_dir / f"prompt_comparison_{args.scenario}.csv"
    discarded_csv = out_dir / f"prompt_discarded_summary_{args.scenario}_{coverage_tag}.csv"
    coverage_png = out_dir / f"prompt_comparison_coverage_{args.scenario}.png"
    collisions_png = out_dir / f"prompt_comparison_collisions_{args.scenario}.png"
    success_png = out_dir / f"prompt_comparison_success_{args.scenario}_{coverage_tag}.png"

    write_summary_csv(all_rows, summary_csv)
    write_discarded_summary(all_rows, discarded_rows, args.categories, args.min_coverage, discarded_csv)
    plot_metric(all_rows, args.categories, "coverage_pct", "Max Coverage Achieved (%)", coverage_png)
    plot_metric(all_rows, args.categories, "collisions_total", "Total Collisions", collisions_png)
    plot_success_counts(all_rows, discarded_rows, args.categories, args.min_coverage, success_png)
    write_category_heatmaps(all_rows, args.categories, args.scenario, out_dir)
    write_combined_heatmap_grid(
        results_dir,
        args.categories,
        ["middle_circle", "spread"],
        out_dir,
        runs_filter=runs_filter,
    )
    write_combined_prompt_comparison_grid(
        results_dir,
        args.categories,
        ["middle_circle", "spread"],
        out_dir,
        runs_filter=runs_filter,
    )
    write_combined_success_grid(
        results_dir,
        args.categories,
        ["middle_circle", "spread"],
        args.min_coverage,
        coverage_tag,
        out_dir,
        runs_filter=runs_filter,
    )

    print(f"[OK] wrote {summary_csv}")
    print(f"[OK] wrote {discarded_csv}")
    print(f"[OK] wrote {coverage_png}")
    print(f"[OK] wrote {collisions_png}")
    print(f"[OK] wrote {success_png}")
    print(
        f"[DONE] plotted {len(all_rows)} trial samples for scenario={args.scenario}; "
        f"{len(success_rows)} are successes and {dropped} are non-successes at max coverage > {args.min_coverage:.1f}%"
    )


if __name__ == "__main__":
    main()
