#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path
import matplotlib.pyplot as plt


def read_last_value(csv_path, value_key):
    last = None
    with csv_path.open('r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            last = row.get(value_key)
    if last is None:
        return None
    return float(last)


def main():
    parser = argparse.ArgumentParser(description="Aggregate coverage/collision results.")
    parser.add_argument(
        "--results_dir",
        default=str(Path(__file__).resolve().parents[1] / "results"),
        help="Directory containing run_* folders",
    )
    parser.add_argument(
        "--out",
        default="summary_boxplots.png",
        help="Output filename inside results_dir",
    )
    args = parser.parse_args()

    results_dir = Path(args.results_dir)
    run_dirs = sorted([p for p in results_dir.iterdir() if p.is_dir()])
    if not run_dirs:
        raise SystemExit(f"No run directories found in {results_dir}")

    coverages = []
    collisions = []
    labels = []
    for run_dir in run_dirs:
        cov_csv = run_dir / "coverage_timeseries.csv"
        col_csv = run_dir / "collision_timeseries.csv"
        if not cov_csv.exists() or not col_csv.exists():
            continue

        cov_last = read_last_value(cov_csv, "coverage_pct")
        col_last = read_last_value(col_csv, "collisions")
        if cov_last is None or col_last is None:
            continue

        coverages.append(cov_last)
        collisions.append(col_last)
        labels.append(run_dir.name)

    if not coverages:
        raise SystemExit("No valid run data found (missing CSVs).")

    fig, (ax_cov, ax_col) = plt.subplots(1, 2, figsize=(10, 5))
    ax_cov.boxplot(coverages, labels=["Final coverage %"])
    ax_cov.set_title("Coverage (final)")
    ax_cov.set_ylabel("Coverage (%)")
    ax_cov.grid(True, linestyle="--", alpha=0.4)

    ax_col.boxplot(collisions, labels=["Total collisions"])
    ax_col.set_title("Collisions (final)")
    ax_col.set_ylabel("Count")
    ax_col.grid(True, linestyle="--", alpha=0.4)

    fig.suptitle(f"Runs: {', '.join(labels)}")
    out_path = results_dir / args.out
    fig.tight_layout()
    fig.savefig(out_path)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    main()
