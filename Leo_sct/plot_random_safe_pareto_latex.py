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
DEFAULT_OUTPUT_PNG = REPO_ROOT / "LATEX FIGURES" / "pareto_coverage_vs_collisions.png"
PLOT_FONT_SIZE = 12

SERIES_STYLES = {
    ("gpt-4.1", "prompt_1_run_2"): ("#1f77b4", "o"),
    ("gpt-4.1", "prompt_2_run_4"): ("#17becf", "v"),
    ("gpt-4.1", "prompt_2_run_5"): ("#bcbd22", "X"),
    ("gpt-5.4", "prompt_1_run_5"): ("#d62728", "s"),
    ("gpt-5.4", "prompt_3_run_1"): ("#2ca02c", "^"),
    ("gpt-5.4", "prompt_4_run_3"): ("#9467bd", "D"),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Regenerate the random-safe Pareto scatter plot for LaTeX."
    )
    parser.add_argument("--input-csv", type=Path, default=DEFAULT_INPUT_CSV)
    parser.add_argument("--output-png", type=Path, default=DEFAULT_OUTPUT_PNG)
    return parser.parse_args()


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def style_axis(ax: plt.Axes) -> None:
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.xaxis.label.set_size(PLOT_FONT_SIZE)
    ax.yaxis.label.set_size(PLOT_FONT_SIZE)
    ax.tick_params(axis="both", labelsize=PLOT_FONT_SIZE)
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.35)


def format_controller_label(controller_set: str) -> str:
    parts = controller_set.split("_")
    if len(parts) >= 4 and parts[0] == "prompt" and parts[2] == "run":
        return f"p{parts[1]}_r{parts[3]}"
    return controller_set


def format_series_label(model: str, controller_set: str) -> str:
    return f"{model}/{format_controller_label(controller_set)}"


def plot(rows: list[dict[str, str]], out_png: Path) -> None:
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.0, 5.2))

    for model, controller_set in SERIES_STYLES:
        subset = [
            row
            for row in rows
            if row["controller_set"] == controller_set and row["model"] == model
        ]
        if not subset:
            continue
        color, marker = SERIES_STYLES[(model, controller_set)]
        ax.scatter(
            [float(row["collisions_total"]) for row in subset],
            [float(row["coverage_pct"]) for row in subset],
            c=color,
            marker=marker,
            s=80,
            label=format_series_label(model, controller_set),
            alpha=0.85,
            edgecolors="black",
            linewidths=0.5,
        )

    ax.set_xlabel("Total collision count")
    ax.set_ylabel("Coverage (%)")
    style_axis(ax)
    ax.legend(frameon=False, fontsize=PLOT_FONT_SIZE)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    args = parse_args()
    rows = read_rows(args.input_csv.resolve())
    if not rows:
        raise SystemExit(f"No rows found in {args.input_csv}")
    plot(rows, args.output_png.resolve())
    print(f"[OK] wrote {args.output_png.resolve()}")


if __name__ == "__main__":
    main()
