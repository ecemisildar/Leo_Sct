#!/usr/bin/env python3
import argparse
import csv
import statistics
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


def read_last_value(csv_path: Path, preferred_key: str, fallback_key: str = None):
    """
    Return the last non-empty numeric value from a CSV column.
    Tries preferred_key first; if missing/empty and fallback_key is provided, tries that.
    """
    last = None
    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            v = row.get(preferred_key, "")
            if (v is None or v == "") and fallback_key:
                v = row.get(fallback_key, "")
            if v is None or v == "":
                continue
            last = v
    if last is None:
        return None
    try:
        return float(last)
    except ValueError:
        return None


def pick_runs(run_dirs, window: int, max_runs: int):
    """
    Pick most recent runs:
      - sort by name (works if run folders are timestamped or monotonically increasing)
      - take the last `window`
      - if max_runs > 0, take the last `max_runs` of that
    """
    run_dirs = sorted([p for p in run_dirs if p.is_dir()])
    if window > 0:
        run_dirs = run_dirs[-window:]
    if max_runs and max_runs > 0:
        run_dirs = run_dirs[-max_runs:]
    return run_dirs


def main():
    parser = argparse.ArgumentParser(description="Aggregate coverage/collision results.")
    parser.add_argument(
        "--results_dir",
        default=str(Path(__file__).resolve().parents[1] / "results"),
        help="Directory containing baselines/ and llm_gen/ folders",
    )
    parser.add_argument(
        "--out",
        default="summary_boxplots.png",
        help="Output filename inside results_dir",
    )
    parser.add_argument(
        "--max_runs",
        type=int,
        default=0,
        help="Number of most recent runs to include PER GROUP (0 = use window size)",
    )

    # If you always want exactly N runs per group, edit these two constants:
    parser.add_argument("--baseline_window", type=int, default=10)
    parser.add_argument("--llm_window", type=int, default=10)

    # Collision column preference:
    # - With the new GLOBAL bump counter (pair events), write collisions_total and read it here.
    # - fallback to old 'collisions' if needed.
    parser.add_argument("--collision_key", default="collisions_total")
    parser.add_argument("--collision_fallback_key", default="collisions")

    args = parser.parse_args()

    results_dir = Path(args.results_dir)
    baseline_root = results_dir / "baselines"
    llm_root = results_dir / "llm_gen"
    if not baseline_root.exists() or not llm_root.exists():
        raise SystemExit(f"Expected baselines/ and llm_gen/ folders under {results_dir}")

    baseline_dirs_all = list(baseline_root.iterdir())
    llm_dirs_all = list(llm_root.iterdir())

    baseline_dirs = pick_runs(baseline_dirs_all, args.baseline_window, args.max_runs)
    llm_dirs = pick_runs(llm_dirs_all, args.llm_window, args.max_runs)

    if len(baseline_dirs) < (args.max_runs if args.max_runs > 0 else args.baseline_window):
        raise SystemExit(f"Not enough baseline runs in {baseline_root} (found {len(baseline_dirs)}).")
    if len(llm_dirs) < (args.max_runs if args.max_runs > 0 else args.llm_window):
        raise SystemExit(f"Not enough llm runs in {llm_root} (found {len(llm_dirs)}).")

    def _load_runs(run_list: list) -> tuple:
        cov_vals = []
        col_vals = []
        names = []
        for run_dir in run_list:
            cov_csv = run_dir / "coverage_timeseries.csv"
            col_csv = run_dir / "collision_timeseries.csv"
            if not cov_csv.exists() or not col_csv.exists():
                raise SystemExit(f"Missing CSVs in {run_dir}")

            cov_last = read_last_value(cov_csv, "coverage_pct")

            col_last = read_last_value(col_csv, args.collision_key, args.collision_fallback_key)

            if cov_last is None or col_last is None:
                raise SystemExit(
                    f"Missing/invalid data in CSVs for {run_dir}\n"
                    f"  coverage key: coverage_pct\n"
                    f"  collision keys tried: {args.collision_key}, {args.collision_fallback_key}"
                )

            cov_vals.append(cov_last)
            col_vals.append(col_last)
            names.append(run_dir.name)
        return cov_vals, col_vals, names

    cov_baseline, col_baseline, baseline_names = _load_runs(baseline_dirs)
    cov_llm, col_llm, llm_names = _load_runs(llm_dirs)

    labels = baseline_names + llm_names

    # Stats (keep separate per group)
    cov_baseline_mean = statistics.mean(cov_baseline)
    cov_llm_mean = statistics.mean(cov_llm)
    cov_baseline_std = statistics.pstdev(cov_baseline) if len(cov_baseline) > 1 else 0.0
    cov_llm_std = statistics.pstdev(cov_llm) if len(cov_llm) > 1 else 0.0

    col_baseline_mean = statistics.mean(col_baseline)
    col_llm_mean = statistics.mean(col_llm)
    col_baseline_std = statistics.pstdev(col_baseline) if len(col_baseline) > 1 else 0.0
    col_llm_std = statistics.pstdev(col_llm) if len(col_llm) > 1 else 0.0

    print("Included runs (baseline):")
    for n, c, k in zip(baseline_names, cov_baseline, col_baseline):
        print(f"  {n}: coverage={c:.3f}%  collisions={k:.1f}")

    print("Included runs (llm):")
    for n, c, k in zip(llm_names, cov_llm, col_llm):
        print(f"  {n}: coverage={c:.3f}%  collisions={k:.1f}")

    print("\nSummary (mean ± std):")
    print(f"  Baseline: coverage={cov_baseline_mean:.3f}±{cov_baseline_std:.3f}  "
          f"collisions={col_baseline_mean:.2f}±{col_baseline_std:.2f}")
    print(f"  LLM:      coverage={cov_llm_mean:.3f}±{cov_llm_std:.3f}  "
          f"collisions={col_llm_mean:.2f}±{col_llm_std:.2f}")

    # ---------- Plot ----------
    fig, ax_cov = plt.subplots(1, 1, figsize=(9, 6))
    ax_col = ax_cov.twinx()

    positions = [1, 2]
    offset = 0.18
    cov_positions = [p - offset for p in positions]
    col_positions = [p + offset for p in positions]

    common_props = {
        "patch_artist": True,
        "showmeans": False,
        "showfliers": False,
        "meanline": False,
        "medianprops": {"color": "black", "linewidth": 1.5},
        "boxprops": {"facecolor": "none", "edgecolor": "black"},
        "whiskerprops": {"color": "black"},
        "capprops": {"color": "black"},
    }

    cov_plot = ax_cov.boxplot(
        [cov_baseline, cov_llm],
        positions=cov_positions,
        widths=0.3,
        labels=["", ""],
        **common_props,
    )
    ax_cov.set_xticks([])
    ax_cov.set_ylabel("Coverage (%)", fontsize=18)
    ax_cov.grid(True, linestyle="--", alpha=0.4)

    col_plot = ax_col.boxplot(
        [col_baseline, col_llm],
        positions=col_positions,
        widths=0.3,
        labels=["", ""],
        **common_props,
    )
    ax_col.set_ylabel("Collisions (count)", fontsize=18)
    ax_cov.set_xlim(0.5, 2.5)

    def _color_boxplot(plot, colors):
        for patch, color in zip(plot["boxes"], colors):
            patch.set_edgecolor(color)
        line_colors = [colors[0], colors[0], colors[1], colors[1]]
        for key in ("whiskers", "caps"):
            for line, color in zip(plot[key], line_colors):
                line.set_color(color)
        for line, color in zip(plot["medians"], colors):
            line.set_color(color)

    # baseline blue, llm red (same as your original intent)
    for plot in (cov_plot, col_plot):
        _color_boxplot(plot, ["blue", "red"])

    legend_handles = [
        Line2D([0], [0], color="blue", lw=2, label="Baseline"),
        Line2D([0], [0], color="red", lw=2, label="LLM-generated"),
    ]
    ax_cov.legend(handles=legend_handles, loc="upper right", fontsize=16)
    ax_cov.tick_params(axis="y", labelsize=16)
    ax_col.tick_params(axis="y", labelsize=16)

    out_path = results_dir / args.out
    fig.tight_layout()
    fig.savefig(out_path)
    print(f"\nSaved {out_path}")


if __name__ == "__main__":
    main()
