#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

if not matplotlib.get_backend().lower().startswith("agg"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


DEFAULT_CATEGORIES = ["long", "medium", "short"]
DEFAULT_SCENARIOS = ["middle_circle", "spread"]
MODEL_ORDER = ["gpt41", "gpt54"]
MODEL_LABELS = {
    "gpt41": "GPT-4.1",
    "gpt54": "GPT-5.4",
}
MODEL_COLORS = {
    "gpt41": "#4c78a8",
    "gpt54": "#e45756",
}
DISCARDED_COLOR = "#c7c7c7"
PLOT_FONT_SIZE = 14
WORLD_SDF = Path(__file__).resolve().parents[1] / "worlds" / "random_world.sdf"
ENV_MIN = -5
ENV_MAX = 5
GRID_SIZE = 1.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plot GPT-4.1 and GPT-5.4 on the same figures for coverage, collisions, "
            "and success by prompt category."
        )
    )
    parser.add_argument(
        "--gpt41-dir",
        type=Path,
        default=Path("results_exp/explore/gpt41/comparisons"),
        help="Directory containing GPT-4.1 prompt comparison CSV outputs.",
    )
    parser.add_argument(
        "--gpt54-dir",
        type=Path,
        default=Path("results_exp/explore_gpt54_repeat3_once/gpt54/comparisons"),
        help="Directory containing GPT-5.4 prompt comparison CSV outputs.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("results_exp/model_comparisons/gpt41_vs_gpt54"),
        help="Directory where combined figures will be written.",
    )
    parser.add_argument(
        "--categories",
        nargs="+",
        default=DEFAULT_CATEGORIES,
        help="Prompt categories to include.",
    )
    parser.add_argument(
        "--scenarios",
        nargs="+",
        default=DEFAULT_SCENARIOS,
        help="Scenarios to include.",
    )
    parser.add_argument(
        "--min-coverage",
        type=float,
        default=10.0,
        help="Success threshold for maximum achieved coverage. Success means strictly greater than this value.",
    )
    return parser.parse_args()


def format_coverage_tag(min_coverage: float) -> str:
    if float(min_coverage).is_integer():
        return f"mincov_{int(min_coverage)}"
    return f"mincov_{str(min_coverage).replace('.', 'p')}"


def format_scenario_label(scenario: str) -> str:
    return scenario.replace("_", " ")


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

    with paths_csv.open("r", newline="", encoding="utf-8") as handle:
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


def choose_extreme_row(rows: list[dict], mode: str, min_coverage: float) -> dict | None:
    if mode == "best":
        qualifying_rows = [row for row in rows if row["coverage_pct"] >= min_coverage]
    elif mode == "worst":
        qualifying_rows = [row for row in rows if row["coverage_pct"] > min_coverage]
    else:
        raise ValueError(f"Unsupported mode: {mode}")
    if not qualifying_rows:
        return None
    if mode == "best":
        return max(qualifying_rows, key=lambda row: (row["coverage_pct"], -row["collisions_total"]))
    if mode == "worst":
        return min(qualifying_rows, key=lambda row: (row["coverage_pct"], row["collisions_total"]))
    raise ValueError(f"Unsupported mode: {mode}")


def read_summary_csv(path: Path) -> list[dict]:
    rows: list[dict] = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                row["coverage_pct"] = float(row["coverage_pct"])
                row["collisions_total"] = float(row["collisions_total"])
            except (KeyError, TypeError, ValueError):
                continue
            rows.append(row)
    return rows


def load_model_rows(root: Path, scenario: str) -> list[dict]:
    csv_path = root / f"prompt_comparison_{scenario}.csv"
    if not csv_path.exists():
        raise SystemExit(f"Missing comparison CSV: {csv_path}")
    return read_summary_csv(csv_path)


def grouped_values(rows_by_model: dict[str, list[dict]], categories: list[str], metric: str) -> list[list[float]]:
    grouped: list[list[float]] = []
    for category in categories:
        for model_name in MODEL_ORDER:
            values = [row[metric] for row in rows_by_model[model_name] if row["category"] == category]
            grouped.append(values)
    return grouped


def plot_grouped_boxplot(ax, rows_by_model: dict[str, list[dict]], categories: list[str], metric: str, ylabel: str):
    positions: list[float] = []
    data: list[list[float]] = []
    colors: list[str] = []
    base_gap = 1.4
    offsets = {"gpt41": -0.22, "gpt54": 0.22}

    for idx, category in enumerate(categories):
        base = idx * base_gap
        for model_name in MODEL_ORDER:
            positions.append(base + offsets[model_name])
            values = [row[metric] for row in rows_by_model[model_name] if row["category"] == category]
            data.append(values)
            colors.append(MODEL_COLORS[model_name])

    box = ax.boxplot(
        data,
        positions=positions,
        widths=0.34,
        patch_artist=True,
        showfliers=False,
        medianprops={"color": "black", "linewidth": 1.5},
    )

    for patch, color in zip(box["boxes"], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.3)
        patch.set_edgecolor(color)

    for position, values, color in zip(positions, data, colors):
        if not values:
            continue
        if len(values) == 1:
            x_positions = [position]
        else:
            step = 0.22 / (len(values) - 1)
            x_positions = [position - 0.11 + step * i for i in range(len(values))]
        for x_pos, value in zip(x_positions, values):
            ax.scatter(
                x_pos,
                value,
                color=color,
                edgecolors="black",
                linewidths=0.5,
                s=36,
                zorder=3,
            )

    ax.set_xticks([idx * base_gap for idx in range(len(categories))])
    ax.set_xticklabels(categories, fontsize=PLOT_FONT_SIZE)
    ax.set_ylabel(ylabel, fontsize=PLOT_FONT_SIZE)
    ax.set_xlabel("Prompt Category", fontsize=PLOT_FONT_SIZE)
    ax.tick_params(axis="y", labelsize=PLOT_FONT_SIZE)
    ax.grid(True, axis="y", linestyle="--", alpha=0.35)


def plot_success(ax, rows_by_model: dict[str, list[dict]], categories: list[str], min_coverage: float):
    base_gap = 1.4
    width = 0.34
    offsets = {"gpt41": -0.22, "gpt54": 0.22}

    for idx, category in enumerate(categories):
        base = idx * base_gap
        for model_name in MODEL_ORDER:
            rows = [row for row in rows_by_model[model_name] if row["category"] == category]
            kept = sum(1 for row in rows if row["coverage_pct"] > min_coverage)
            dropped = sum(1 for row in rows if row["coverage_pct"] <= min_coverage)
            total = len(rows)
            x_pos = base + offsets[model_name]

            ax.bar(
                x_pos,
                kept,
                width=width,
                color=MODEL_COLORS[model_name],
                alpha=0.85,
            )
            ax.bar(
                x_pos,
                dropped,
                width=width,
                bottom=kept,
                color=DISCARDED_COLOR,
                alpha=0.9,
            )
            pct = (100.0 * kept / total) if total else 0.0
            ax.text(
                x_pos,
                kept,
                f"{pct:.1f}%",
                ha="center",
                va="bottom",
                fontsize=10,
            )

    ax.set_xticks([idx * base_gap for idx in range(len(categories))])
    ax.set_xticklabels(categories, fontsize=PLOT_FONT_SIZE)
    ax.set_ylabel("Trials", fontsize=PLOT_FONT_SIZE)
    ax.set_xlabel("Prompt Category", fontsize=PLOT_FONT_SIZE)
    ax.tick_params(axis="y", labelsize=PLOT_FONT_SIZE)
    ax.grid(True, axis="y", linestyle="--", alpha=0.35)
    ax.set_title(f"Success: max coverage > {min_coverage:.1f}%", fontsize=PLOT_FONT_SIZE)


def model_legend_handles(include_discarded: bool = True):
    handles = [
        plt.Line2D([0], [0], color=MODEL_COLORS["gpt41"], lw=6, alpha=0.8, label=MODEL_LABELS["gpt41"]),
        plt.Line2D([0], [0], color=MODEL_COLORS["gpt54"], lw=6, alpha=0.8, label=MODEL_LABELS["gpt54"]),
    ]
    if include_discarded:
        handles.append(
            plt.Line2D([0], [0], color=DISCARDED_COLOR, lw=6, alpha=0.9, label="Non-success")
        )
    return handles


def add_model_legend(ax):
    ax.legend(handles=model_legend_handles(), loc="upper right", fontsize=PLOT_FONT_SIZE)


def draw_visited_cells(ax, counts: list[list[int]], obstacles: list[dict], title: str):
    for row_idx, row in enumerate(counts):
        for col_idx, value in enumerate(row):
            x = ENV_MIN + col_idx * GRID_SIZE
            y = ENV_MIN + row_idx * GRID_SIZE
            rect = plt.Rectangle(
                (x, y),
                GRID_SIZE,
                GRID_SIZE,
                facecolor=MODEL_COLORS["gpt41"] if False else "white",
                edgecolor="#d9d9d9",
                linewidth=0.8,
            )
            if value > 0:
                rect.set_facecolor("#d62828")
                rect.set_edgecolor("#9d0208")
                rect.set_alpha(0.45)
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
    ax.set_xlabel("X (m)", fontsize=11)
    ax.set_ylabel("Y (m)", fontsize=11)
    ax.set_title(title, fontsize=12)


def write_random_run_visited_cells_grid(
    scenario_rows: dict[str, dict[str, list[dict]]],
    scenarios: list[str],
    min_coverage: float,
    out_dir: Path,
):
    obstacles = load_obstacle_shapes(WORLD_SDF)
    grid_columns = [(mode, model_name) for mode in ("best", "worst") for model_name in MODEL_ORDER]
    fig, axes = plt.subplots(len(scenarios), len(grid_columns), figsize=(20, 5.2 * len(scenarios)))
    if len(scenarios) == 1:
        axes = [axes]

    for row_idx, scenario in enumerate(scenarios):
        scenario_label = format_scenario_label(scenario)
        for col_idx, (mode, model_name) in enumerate(grid_columns):
            ax = axes[row_idx][col_idx]
            rows = scenario_rows[scenario][model_name]
            sample_row = choose_extreme_row(rows, mode, min_coverage)
            if sample_row is None:
                ax.set_axis_off()
                continue
            counts = read_path_grid_counts(Path(sample_row["paths_csv"]))
            title = f"{MODEL_LABELS[model_name]} {mode.capitalize()}"
            draw_visited_cells(ax, counts, obstacles, title)
            ax.text(
                0.5,
                -0.14,
                f"{sample_row['category']} prompt: {sample_row['coverage_pct']:.1f}% max coverage",
                transform=ax.transAxes,
                ha="center",
                va="top",
                fontsize=9,
            )
            if col_idx == 0:
                ax.text(
                    -0.28,
                    0.5,
                    scenario_label,
                    transform=ax.transAxes,
                    rotation=90,
                    ha="center",
                    va="center",
                    fontsize=13,
                    fontweight="bold",
                )

    fig.subplots_adjust(left=0.08, right=0.98, top=0.96, bottom=0.12, wspace=0.2, hspace=0.34)
    out_png = out_dir / "model_random_run_visited_cells_grid.png"
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] wrote {out_png}")


def write_scenario_figure(
    rows_by_model: dict[str, list[dict]],
    scenario: str,
    categories: list[str],
    min_coverage: float,
    out_dir: Path,
):
    scenario_label = format_scenario_label(scenario)
    fig, axes = plt.subplots(1, 3, figsize=(17, 5.5))

    plot_grouped_boxplot(axes[0], rows_by_model, categories, "coverage_pct", "Max Coverage Achieved (%)")
    axes[0].set_title(f"{scenario_label} coverage", fontsize=PLOT_FONT_SIZE)

    plot_grouped_boxplot(axes[1], rows_by_model, categories, "collisions_total", "Total Collisions")
    axes[1].set_title(f"{scenario_label} collisions", fontsize=PLOT_FONT_SIZE)

    plot_success(axes[2], rows_by_model, categories, min_coverage)
    axes[2].set_title(f"{scenario_label} success", fontsize=PLOT_FONT_SIZE)
    add_model_legend(axes[2])

    fig.tight_layout()
    out_png = out_dir / f"model_comparison_{scenario}.png"
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] wrote {out_png}")


def write_combined_grid(
    scenario_rows: dict[str, dict[str, list[dict]]],
    scenarios: list[str],
    categories: list[str],
    min_coverage: float,
    out_dir: Path,
):
    fig, axes = plt.subplots(len(scenarios), 3, figsize=(17, 5.5 * len(scenarios)))
    if len(scenarios) == 1:
        axes = [axes]

    for row_idx, scenario in enumerate(scenarios):
        rows_by_model = scenario_rows[scenario]
        scenario_label = format_scenario_label(scenario)

        plot_grouped_boxplot(axes[row_idx][0], rows_by_model, categories, "coverage_pct", "Max Coverage Achieved (%)")
        axes[row_idx][0].set_title(f"{scenario_label} coverage", fontsize=PLOT_FONT_SIZE)

        plot_grouped_boxplot(axes[row_idx][1], rows_by_model, categories, "collisions_total", "Total Collisions")
        axes[row_idx][1].set_title(f"{scenario_label} collisions", fontsize=PLOT_FONT_SIZE)

        plot_success(axes[row_idx][2], rows_by_model, categories, min_coverage)
        axes[row_idx][2].set_title(f"{scenario_label} success", fontsize=PLOT_FONT_SIZE)
        axes[row_idx][0].text(
            -0.34,
            0.5,
            scenario_label,
            transform=axes[row_idx][0].transAxes,
            rotation=90,
            ha="center",
            va="center",
            fontsize=PLOT_FONT_SIZE,
            fontweight="bold",
        )

    fig.legend(
        handles=model_legend_handles(include_discarded=False),
        loc="upper center",
        ncol=2,
        fontsize=PLOT_FONT_SIZE,
        frameon=False,
        bbox_to_anchor=(0.5, 0.995),
    )
    fig.subplots_adjust(left=0.1, right=0.98, top=0.9, bottom=0.08, wspace=0.28, hspace=0.28)
    out_png = out_dir / "model_comparison_grid.png"
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] wrote {out_png}")


def main():
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    scenario_rows: dict[str, dict[str, list[dict]]] = {}
    for scenario in args.scenarios:
        rows_by_model = {
            "gpt41": load_model_rows(args.gpt41_dir, scenario),
            "gpt54": load_model_rows(args.gpt54_dir, scenario),
        }
        scenario_rows[scenario] = rows_by_model
        write_scenario_figure(rows_by_model, scenario, args.categories, args.min_coverage, args.out_dir)

    write_combined_grid(scenario_rows, args.scenarios, args.categories, args.min_coverage, args.out_dir)
    write_random_run_visited_cells_grid(scenario_rows, args.scenarios, args.min_coverage, args.out_dir)

    coverage_tag = format_coverage_tag(args.min_coverage)
    print(
        f"[DONE] wrote combined GPT-4.1 vs GPT-5.4 figures for scenarios={args.scenarios} "
        f"at success threshold max coverage > {args.min_coverage:.1f}% ({coverage_tag})"
    )


if __name__ == "__main__":
    main()
