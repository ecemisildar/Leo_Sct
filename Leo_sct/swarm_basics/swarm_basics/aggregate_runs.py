#!/usr/bin/env python3
import argparse
import csv
import statistics
from pathlib import Path
from datetime import datetime
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


def _parse_run_start(run_id: str):
    try:
        return datetime.strptime(run_id.replace("run_", ""), "%Y%m%d_%H%M%S")
    except Exception:
        return None


def _list_bump_files_for_window(bump_dir: Path, label: str, start_epoch: float, end_epoch: float):
    files = []
    for p in bump_dir.glob(f"bumps_{label}_*.csv"):
        try:
            mtime = p.stat().st_mtime
        except OSError:
            continue
        if (start_epoch - 3600) <= mtime <= (end_epoch + 3600):
            files.append(p)
    return sorted(files)


def _read_bump_events(paths):
    events = []
    for path in paths:
        with path.open("r", encoding="utf-8", newline="") as f:
            reader = csv.DictReader(f)
            required = ["stamp_sec", "stamp_nsec", "bump_type"]
            for k in required:
                if k not in reader.fieldnames:
                    raise SystemExit(f"Missing column '{k}' in {path}. Header={reader.fieldnames}")
            for row in reader:
                try:
                    sec = int(row["stamp_sec"])
                    nsec = int(row["stamp_nsec"])
                    t = float(sec) + float(nsec) * 1e-9
                except Exception:
                    continue
                bump_type = row.get("bump_type", "").strip().lower()
                events.append((t, bump_type))
    events.sort(key=lambda x: x[0])
    return events


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

    args = parser.parse_args()

    results_dir = Path(args.results_dir)
    baseline_root = results_dir / "baselines"
    llm_root = results_dir / "llm_gen"
    if not baseline_root.exists() or not llm_root.exists():
        raise SystemExit(f"Expected baselines/ and llm_gen/ folders under {results_dir}")
    bump_log_dir = Path.home() / "ros_bump_logs"
    bump_label = "global"

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
            if not cov_csv.exists():
                raise SystemExit(f"Missing CSVs in {run_dir}")
            cov_last = read_last_value(cov_csv, "coverage_pct")
            duration = read_last_value(cov_csv, "time_s")
            if cov_last is None or duration is None:
                raise SystemExit(
                    f"Missing/invalid data in CSVs for {run_dir}\n"
                    f"  coverage key: coverage_pct\n"
                    f"  time key: time_s"
                )

            run_start_dt = _parse_run_start(run_dir.name)
            if run_start_dt is None:
                raise SystemExit(f"Could not parse run start time from {run_dir.name}")
            run_start_epoch = run_start_dt.timestamp()
            run_end_epoch = run_start_epoch + float(duration)
            bump_files = _list_bump_files_for_window(
                bump_log_dir, bump_label, run_start_epoch, run_end_epoch
            )
            if not bump_files:
                raise SystemExit(f"No bump logs found near {run_dir.name} in {bump_log_dir}")
            events = _read_bump_events(bump_files)
            col_last = sum(
                1
                for t, bump_type in events
                if run_start_epoch - 0.5 <= t <= run_end_epoch + 0.5
                and bump_type in {"robot", "obstacle"}
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
    fig, ax_cov = plt.subplots(1, 1, figsize=(7, 5))
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
    ax_cov.set_title("Coverage & Collisions")
    ax_cov.set_xticks([])
    ax_cov.set_ylabel("Coverage (%)")
    ax_cov.grid(True, linestyle="--", alpha=0.4)

    col_plot = ax_col.boxplot(
        [col_baseline, col_llm],
        positions=col_positions,
        widths=0.3,
        labels=["", ""],
        **common_props,
    )
    ax_col.set_ylabel("Collisions (count)")
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
    ax_cov.legend(handles=legend_handles, loc="upper right", fontsize="small")

    fig.suptitle(f"Baseline runs: {len(baseline_dirs)} | LLM runs: {len(llm_dirs)}")
    out_path = results_dir / args.out
    fig.tight_layout()
    fig.savefig(out_path)
    print(f"\nSaved {out_path}")


if __name__ == "__main__":
    main()
