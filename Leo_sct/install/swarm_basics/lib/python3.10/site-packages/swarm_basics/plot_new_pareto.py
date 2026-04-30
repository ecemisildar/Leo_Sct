#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path

import matplotlib

if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RESULTS_DIR = (
    PROJECT_ROOT / "new prompt runs" / "four_prompt_run" / "explore" / "new"
)


def read_max_coverage(path: Path) -> float | None:
    max_coverage: float | None = None
    with path.open("r", newline="", encoding="utf-8") as handle:
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


def read_total_collisions(path: Path | None) -> int:
    if path is None or not path.exists():
        return 0
    total = 0
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            value = row.get("total_index", "")
            if value == "":
                continue
            try:
                total = int(value)
            except ValueError:
                continue
    return total


def find_bump_csv(run_dir: Path) -> Path | None:
    bump_files = sorted(run_dir.glob("bumps_*.csv"))
    return bump_files[-1] if bump_files else None


def collect_runs(results_dir: Path, layout: str) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for prompt_dir in sorted(results_dir.glob("prompt_*")):
        if not prompt_dir.is_dir():
            continue
        for run_dir in sorted(prompt_dir.glob("run_*")):
            if not run_dir.is_dir():
                continue
            for trial_dir in sorted((run_dir / layout).glob("trial_*")):
                for sim_dir in sorted(trial_dir.glob("run_*")):
                    coverage_csv = sim_dir / "coverage_timeseries.csv"
                    if not coverage_csv.exists():
                        continue
                    coverage = read_max_coverage(coverage_csv)
                    if coverage is None:
                        continue
                    bump_csv = find_bump_csv(sim_dir)
                    rows.append(
                        {
                            "prompt": prompt_dir.name,
                            "run": run_dir.name,
                            "layout": layout,
                            "trial": trial_dir.name,
                            "sim_run": sim_dir.name,
                            "coverage_pct": coverage,
                            "collisions_total": read_total_collisions(bump_csv),
                            "sim_dir": str(sim_dir),
                            "coverage_csv": str(coverage_csv),
                            "bump_csv": str(bump_csv) if bump_csv else "",
                        }
                    )
    return rows


def is_pareto_optimal(row: dict[str, object], rows: list[dict[str, object]]) -> bool:
    coverage = float(row["coverage_pct"])
    collisions = int(row["collisions_total"])
    for other in rows:
        if other is row:
            continue
        other_coverage = float(other["coverage_pct"])
        other_collisions = int(other["collisions_total"])
        dominates = (
            other_coverage >= coverage
            and other_collisions <= collisions
            and (other_coverage > coverage or other_collisions < collisions)
        )
        if dominates:
            return False
    return True


def add_pareto_flags(rows: list[dict[str, object]]) -> None:
    for row in rows:
        row["pareto_optimal"] = is_pareto_optimal(row, rows)


def write_csv(rows: list[dict[str, object]], out_csv: Path) -> None:
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "prompt",
        "run",
        "layout",
        "trial",
        "sim_run",
        "coverage_pct",
        "collisions_total",
        "pareto_optimal",
        "sim_dir",
        "coverage_csv",
        "bump_csv",
    ]
    with out_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def plot_pareto(rows: list[dict[str, object]], out_png: Path) -> None:
    out_png.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(10, 6))
    colors = {
        "run_1": "#1f77b4",
        "run_2": "#d62728",
        "run_3": "#2ca02c",
    }
    markers = {"run_1": "o", "run_2": "s", "run_3": "^"}

    for run_name in sorted({str(row["run"]) for row in rows}):
        subset = [row for row in rows if row["run"] == run_name]
        if not subset:
            continue
        ax.scatter(
            [int(row["collisions_total"]) for row in subset],
            [float(row["coverage_pct"]) for row in subset],
            c=colors.get(run_name, "#7f7f7f"),
            marker=markers.get(run_name, "o"),
            s=80,
            label=run_name,
            alpha=0.85,
            edgecolors="black",
            linewidths=0.5,
        )

    pareto_rows = [row for row in rows if row["pareto_optimal"]]
    if pareto_rows:
        pareto_sorted = sorted(
            pareto_rows,
            key=lambda row: (int(row["collisions_total"]), float(row["coverage_pct"])),
        )
        ax.plot(
            [int(row["collisions_total"]) for row in pareto_sorted],
            [float(row["coverage_pct"]) for row in pareto_sorted],
            color="black",
            linewidth=1.5,
            label="Pareto frontier",
        )

    for row in rows:
        label = f"{row['prompt'].replace('prompt_', 'p')}-{row['run'].replace('run_', 'r')}"
        if row["trial"] != "trial_1":
            label += f"-{row['trial'].replace('trial_', 't')}"
        ax.annotate(
            label,
            (int(row["collisions_total"]), float(row["coverage_pct"])),
            textcoords="offset points",
            xytext=(5, 4),
            fontsize=8,
        )

    ax.set_xlabel("Total collisions")
    ax.set_ylabel("Coverage (%)")
    ax.set_title("Pareto Plot: Coverage vs Collisions")
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_png, dpi=200)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a Pareto plot for saved prompt simulation runs."
    )
    parser.add_argument("--results-dir", default=str(DEFAULT_RESULTS_DIR))
    parser.add_argument("--layout", default="spread")
    parser.add_argument(
        "--out-dir",
        default="",
        help="Output directory. Defaults to <results-dir>/pareto_analysis.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    results_dir = Path(args.results_dir).resolve()
    out_dir = Path(args.out_dir).resolve() if args.out_dir else results_dir / "pareto_analysis"

    rows = collect_runs(results_dir, args.layout)
    if not rows:
        raise SystemExit(f"No run_1/run_2 simulation outputs found under {results_dir}")

    rows.sort(key=lambda row: (str(row["prompt"]), str(row["run"]), str(row["trial"])))
    add_pareto_flags(rows)

    out_csv = out_dir / "pareto_runs.csv"
    out_png = out_dir / "pareto_coverage_vs_collisions.png"
    write_csv(rows, out_csv)
    plot_pareto(rows, out_png)

    print(f"[OK] wrote {out_csv}")
    print(f"[OK] wrote {out_png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
