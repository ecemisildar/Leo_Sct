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
DEFAULT_INPUT_CSV = (
    REPO_ROOT
    / "LAST_PROMPTS"
    / "random_safe_selected_controllers"
    / "pareto_analysis"
    / "pareto_runs.csv"
)
DEFAULT_ANALYSIS_OUTPUT = (
    REPO_ROOT
    / "LAST_PROMPTS"
    / "random_safe_selected_controllers"
    / "pareto_analysis"
    / "group_boxplots.png"
)
DEFAULT_LATEX_OUTPUT = REPO_ROOT / "LATEX FIGURES" / "group_boxplots.png"
PLOT_FONT_SIZE = 12

FRONTIER_ORDER = [
    ("gpt-4.1", "prompt_4_run_5"),
    ("gpt-4.1", "prompt_2_run_5"),
    ("gpt-4.1", "prompt_2_run_4"),
    ("gpt-4.1", "prompt_1_run_2"),
    ("gpt-5.4", "prompt_1_run_5"),
    ("gpt-5.4", "prompt_3_run_1"),
    ("gpt-5.4", "prompt_4_run_3"),
    ("gpt-5.4", "prompt_2_run_1"),
]

SERIES_COLORS = {
    ("gpt-4.1", "prompt_1_run_2"): "#1f77b4",
    ("gpt-4.1", "prompt_2_run_4"): "#17becf",
    ("gpt-4.1", "prompt_2_run_5"): "#bcbd22",
    ("gpt-5.4", "prompt_1_run_5"): "#d62728",
    ("gpt-5.4", "prompt_3_run_1"): "#2ca02c",
    ("gpt-5.4", "prompt_4_run_3"): "#9467bd",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Regenerate random-safe grouped boxplots in merged Pareto-frontier order."
    )
    parser.add_argument("--input-csv", type=Path, default=DEFAULT_INPUT_CSV)
    parser.add_argument("--analysis-output", type=Path, default=DEFAULT_ANALYSIS_OUTPUT)
    parser.add_argument("--latex-output", type=Path, default=DEFAULT_LATEX_OUTPUT)
    return parser.parse_args()


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def format_controller_label(controller_set: str) -> str:
    parts = controller_set.split("_")
    if len(parts) >= 4 and parts[0] == "prompt" and parts[2] == "run":
        return f"p{parts[1]}_r{parts[3]}"
    return controller_set


def format_series_label(model: str, controller_set: str) -> str:
    short_model = model.replace("gpt-", "")
    return f"{format_controller_label(controller_set)}\n{short_model}"


def style_axis(ax: plt.Axes, ylabel: str) -> None:
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.set_ylabel(ylabel, fontsize=PLOT_FONT_SIZE)
    ax.tick_params(axis="both", labelsize=PLOT_FONT_SIZE)
    ax.grid(True, axis="y", linestyle="--", linewidth=0.5, alpha=0.35)


def values_for(
    rows: list[dict[str, str]],
    model: str,
    controller_set: str,
    field: str,
) -> list[float]:
    return [
        float(row["collisions_total"]) / (float(row["last_time_s"]) / 60.0)
        if field == "collisions_per_min"
        else float(row[field])
        for row in rows
        if row["model"] == model and row["controller_set"] == controller_set
    ]


def plot(rows: list[dict[str, str]], output_paths: list[Path]) -> None:
    ordered_groups = [
        group
        for group in FRONTIER_ORDER
        if any(row["model"] == group[0] and row["controller_set"] == group[1] for row in rows)
    ]
    if not ordered_groups:
        raise SystemExit("No matching controller groups found in input CSV.")

    labels = [format_series_label(model, controller) for model, controller in ordered_groups]
    colors = [SERIES_COLORS.get(group, "#7f7f7f") for group in ordered_groups]

    fig, axes = plt.subplots(1, 2, figsize=(14, 6.5))
    metrics = [
        ("coverage_pct", "Coverage (%)"),
        ("collisions_per_min", "Collisions per minute"),
    ]

    for ax, (field, ylabel) in zip(axes, metrics):
        data = [values_for(rows, model, controller, field) for model, controller in ordered_groups]
        box = ax.boxplot(
            data,
            patch_artist=True,
            showfliers=False,
            medianprops={"color": "black", "linewidth": 1.5},
        )
        for patch, color in zip(box["boxes"], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.32)
            patch.set_edgecolor(color)
        for whisker in box["whiskers"]:
            whisker.set_linewidth(1.0)
        for cap in box["caps"]:
            cap.set_linewidth(1.0)

        ax.set_xticks(range(1, len(labels) + 1))
        ax.set_xticklabels(labels, fontsize=PLOT_FONT_SIZE)
        style_axis(ax, ylabel)

    fig.tight_layout()
    for output_path in output_paths:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=200)
        print(f"[OK] wrote {output_path}")
    plt.close(fig)


def main() -> None:
    args = parse_args()
    rows = read_rows(args.input_csv.resolve())
    if not rows:
        raise SystemExit(f"No rows found in {args.input_csv}")
    plot(rows, [args.analysis_output.resolve(), args.latex_output.resolve()])


if __name__ == "__main__":
    main()
