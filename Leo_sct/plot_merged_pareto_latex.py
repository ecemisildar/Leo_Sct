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
DEFAULT_INPUT_CSV = REPO_ROOT / "LAST_PROMPTS" / "pareto_plots" / "merged_pareto_runs.csv"
DEFAULT_OUTPUT_PNG = REPO_ROOT / "LATEX FIGURES" / "merged_pareto_coverage_vs_collisions.png"
PLOT_FONT_SIZE = 12


MODEL_COLORS = {
    "gpt-4.1": "#4c78a8",
    "gpt-5.4": "#e45756",
}
MODEL_LABELS = {
    "gpt-4.1": "GPT-4.1",
    "gpt-5.4": "GPT-5.4",
}
PROMPT_MARKERS = {
    "prompt_1": "o",
    "prompt_2": "s",
    "prompt_3": "^",
    "prompt_4": "D",
}
PROMPT_LABELS = {
    "prompt_1": "p1",
    "prompt_2": "p2",
    "prompt_3": "p3",
    "prompt_4": "p4",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Regenerate the merged Pareto plot for LaTeX figures."
    )
    parser.add_argument("--input-csv", type=Path, default=DEFAULT_INPUT_CSV)
    parser.add_argument("--output-png", type=Path, default=DEFAULT_OUTPUT_PNG)
    return parser.parse_args()


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def is_true(value: str) -> bool:
    return value.strip().lower() == "true"


def style_axis(ax: plt.Axes) -> None:
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.xaxis.label.set_size(PLOT_FONT_SIZE)
    ax.yaxis.label.set_size(PLOT_FONT_SIZE)
    ax.tick_params(axis="both", labelsize=PLOT_FONT_SIZE)
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.35)


def plot(rows: list[dict[str, str]], out_png: Path) -> None:
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.0, 5.2))

    for model in sorted({row["model"] for row in rows}):
        for prompt in sorted({row["prompt"] for row in rows}):
            model_rows = [
                row for row in rows if row["model"] == model and row["prompt"] == prompt
            ]
            if not model_rows:
                continue
            ax.scatter(
                [int(row["collisions_total"]) for row in model_rows],
                [float(row["coverage_pct"]) for row in model_rows],
                c=MODEL_COLORS.get(model, "#7f7f7f"),
                marker=PROMPT_MARKERS.get(prompt, "o"),
                s=62,
                alpha=0.85,
                edgecolors="black",
                linewidths=0.45,
            )

    for model in sorted({row["model"] for row in rows}):
        ax.scatter(
            [],
            [],
            c=MODEL_COLORS.get(model, "#7f7f7f"),
            marker="o",
            s=62,
            label=MODEL_LABELS.get(model, model),
            alpha=0.85,
            edgecolors="black",
            linewidths=0.45,
        )

    for prompt in sorted({row["prompt"] for row in rows}):
        ax.scatter(
            [],
            [],
            c="#8c8c8c",
            marker=PROMPT_MARKERS.get(prompt, "o"),
            s=62,
            label=PROMPT_LABELS.get(prompt, prompt),
            alpha=0.85,
            edgecolors="black",
            linewidths=0.45,
        )

    pareto_rows = [row for row in rows if is_true(row.get("pareto_optimal_merged", ""))]
    if pareto_rows:
        pareto_rows = sorted(
            pareto_rows,
            key=lambda row: (int(row["collisions_total"]), float(row["coverage_pct"])),
        )
        ax.plot(
            [int(row["collisions_total"]) for row in pareto_rows],
            [float(row["coverage_pct"]) for row in pareto_rows],
            color="black",
            linewidth=1.4,
            label="Pareto frontier",
        )

    ax.set_xlabel("Total collision count")
    ax.set_ylabel("Coverage (%)")
    y_min, y_max = ax.get_ylim()
    ax.set_yticks([tick for tick in ax.get_yticks() if tick >= 20.0])
    ax.set_ylim(y_min, y_max)
    ax.set_xticks(range(0, 201, 20))
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
