#!/usr/bin/env python3
import argparse
import csv
import os
from pathlib import Path

import matplotlib
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RESULTS_DIR = PROJECT_ROOT / "results_exp" / "explore"
DEFAULT_OUT_DIR = DEFAULT_RESULTS_DIR / "comparisons"
DEFAULT_CATEGORIES = ["long", "medium", "vague"]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compare prompt categories across trials and plot per-trial results."
    )
    parser.add_argument(
        "--results-dir",
        default=str(DEFAULT_RESULTS_DIR),
        help="Root directory containing category folders such as long/medium/vague.",
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
    return parser.parse_args()


def read_last_coverage(coverage_csv: Path) -> float | None:
    last = None
    with coverage_csv.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            value = row.get("coverage_pct", "")
            if value == "":
                continue
            try:
                last = float(value)
            except ValueError:
                continue
    return last


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


def collect_rows(results_dir: Path, categories: list[str], scenario: str) -> list[dict]:
    rows = []
    for category in categories:
        category_dir = results_dir / category
        if not category_dir.exists():
            continue
        for prompt_dir in sorted(category_dir.glob("prompt_*")):
            for run_dir in sorted(prompt_dir.glob("run_*")):
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
                    coverage = read_last_coverage(coverage_csv)
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
        "bump_csv",
    ]
    with out_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def plot_metric(rows: list[dict], categories: list[str], metric: str, ylabel: str, out_png: Path):
    color_map = {
        "long": "#1f77b4",
        "medium": "#ff7f0e",
        "vague": "#2ca02c",
    }
    fig, ax = plt.subplots(figsize=(10, 6))

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
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)


def main():
    args = parse_args()
    results_dir = Path(args.results_dir).resolve()
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = collect_rows(results_dir, args.categories, args.scenario)
    if not rows:
        raise SystemExit(
            f"No matching runs found under {results_dir} for categories={args.categories} "
            f"and scenario={args.scenario}"
        )

    summary_csv = out_dir / f"prompt_comparison_{args.scenario}.csv"
    coverage_png = out_dir / f"prompt_comparison_coverage_{args.scenario}.png"
    collisions_png = out_dir / f"prompt_comparison_collisions_{args.scenario}.png"

    write_summary_csv(rows, summary_csv)
    plot_metric(rows, args.categories, "coverage_pct", "Final Coverage (%)", coverage_png)
    plot_metric(rows, args.categories, "collisions_total", "Total Collisions", collisions_png)

    print(f"[OK] wrote {summary_csv}")
    print(f"[OK] wrote {coverage_png}")
    print(f"[OK] wrote {collisions_png}")
    print(f"[DONE] plotted {len(rows)} trial samples for scenario={args.scenario}")


if __name__ == "__main__":
    main()
