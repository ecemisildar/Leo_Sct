#!/usr/bin/env python3
import csv
import os
from pathlib import Path

import matplotlib
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ----------------------------
# CONFIG (no args)
# ----------------------------
PACKAGE_ROOT = Path.home() / "ros2_ws/src/Leo_sct/swarm_basics"
RESULTS_DIR = PACKAGE_ROOT / "results"
BUMP_LOG_DIR = Path.home() / "ros_bump_logs"
BUMP_LABEL = "global"  # bumps_global_*.csv  (TSV in your case)

# Optional offline “episode debouncing”
MIN_RECOUNT_SEC_ROBOT = 0.0   # e.g. 8.0
MIN_RECOUNT_SEC_OBS   = 0.0   # e.g. 1.0
# ----------------------------


def pick_latest_run_dir(results_dir: Path) -> Path:
    runs = [p for p in results_dir.iterdir() if p.is_dir() and p.name.startswith("run_")]
    if not runs:
        raise SystemExit(f"No run_* folders found under {results_dir}")
    runs.sort(key=lambda p: p.stat().st_mtime)
    return runs[-1]


def read_coverage_timeseries(path: Path):
    times, covs = [], []
    with path.open("r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            t = row.get("time_s", "")
            c = row.get("coverage_pct", "")
            if not t or not c:
                continue
            times.append(float(t))
            covs.append(float(c))
    return times, covs


def pick_latest_bump_file(bump_dir: Path, label: str) -> Path | None:
    if not bump_dir.exists():
        return None
    files = sorted(bump_dir.glob(f"bumps_{label}_*.csv"), key=lambda p: p.stat().st_mtime)
    return files[-1] if files else None


def detect_delimiter(header_line: str) -> str:
    # Your logs are TSV, but let’s autodetect
    if "\t" in header_line and header_line.count("\t") >= header_line.count(","):
        return "\t"
    return ","


def read_bump_rows(path: Path):
    """
    Reads bump log rows as dicts.
    Autodetect delimiter (TSV vs CSV).
    Requires at least: stamp_sec, stamp_nsec, bump_type, total_index (or can still work without).
    """
    with path.open("r", encoding="utf-8", newline="") as f:
        header = f.readline()
        if not header:
            return [], None
        delim = detect_delimiter(header)
        fieldnames = [h.strip() for h in header.rstrip("\n").split(delim)]
        reader = csv.DictReader(f, fieldnames=fieldnames, delimiter=delim)
        rows = []
        for row in reader:
            # skip empty
            if not row:
                continue
            # skip if this is an accidental repeated header line
            if (row.get(fieldnames[0]) or "").strip() == fieldnames[0]:
                continue
            rows.append(row)
    return rows, fieldnames


def split_into_segments(rows):
    """
    The bump file contains multiple runs appended.
    We split into segments using 'total_index' resets/decreases.
    If total_index is missing, fallback to stamp going backwards.

    Returns: list of segments, each is list of parsed events:
      (t_sec_float, bump_type, key)
    """
    segments = []
    current = []

    prev_total_idx = None
    prev_t = None

    def flush():
        nonlocal current
        if current:
            segments.append(current)
            current = []

    for row in rows:
        # parse time
        try:
            sec = int(row.get("stamp_sec", ""))
            nsec = int(row.get("stamp_nsec", "0"))
            t = float(sec) + float(nsec) * 1e-9
        except Exception:
            continue

        bump_type = (row.get("bump_type") or "").strip().lower()
        key = (row.get("key") or "").strip()

        # determine if new segment starts
        new_segment = False

        total_idx_raw = row.get("total_index", None)
        if total_idx_raw is not None and total_idx_raw != "":
            try:
                total_idx = int(total_idx_raw)
            except Exception:
                total_idx = None

            if total_idx is not None and prev_total_idx is not None:
                # reset or backward => new run in same file
                if total_idx <= 1 and prev_total_idx > 1:
                    new_segment = True
                elif total_idx < prev_total_idx:
                    new_segment = True
            prev_total_idx = total_idx
        else:
            # fallback: time went backwards a lot => new segment
            if prev_t is not None and (t + 0.001) < prev_t:
                new_segment = True

        if new_segment:
            flush()

        current.append((t, bump_type, key))
        prev_t = t

    flush()
    return segments


def debounce_events(events, min_robot, min_obs):
    if min_robot <= 0.0 and min_obs <= 0.0:
        return events
    last = {}
    out = []
    for t, typ, key in events:
        thr = min_robot if typ == "robot" else min_obs
        if thr <= 0.0 or not key:
            out.append((t, typ, key))
            continue
        prev = last.get((typ, key))
        if prev is None or (t - prev) >= thr:
            out.append((t, typ, key))
            last[(typ, key)] = t
    return out


def build_collision_timeseries(coverage_times, events_abs):
    """
    coverage_times: [0..T] (relative)
    events_abs: [(t_abs, typ, key)] where t_abs is "sim seconds" like 41.88, 97.89, ...
    We do NOT shift time; we treat bump timestamps as already in the same time base as coverage plotter.
    """
    robot = 0
    obs = 0
    i = 0
    out = []

    events_abs = sorted(events_abs, key=lambda x: x[0])

    for t in coverage_times:
        while i < len(events_abs) and events_abs[i][0] <= t + 1e-9:
            _, typ, _ = events_abs[i]
            if typ == "robot":
                robot += 1
            elif typ == "obstacle":
                obs += 1
            i += 1
        out.append((t, robot + obs, robot, obs))
    return out


def save_collision_csv(path: Path, rows):
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "collisions_total", "collisions_robot", "collisions_obstacle"])
        for t, tot, rob, obs in rows:
            w.writerow([f"{t:.3f}", int(tot), int(rob), int(obs)])


def plot_coverage(times, covs, out_png: Path, title: str):
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(times, covs)
    ax.set_title(title)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Coverage (%)")
    ax.set_ylim(0, 105)
    ax.grid(True, linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_collisions(rows, out_png: Path, title: str):
    t = [r[0] for r in rows]
    rob = [r[2] for r in rows]
    obs = [r[3] for r in rows]

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.step(t, rob, where="post", label="Robot collisions")
    ax.step(t, obs, where="post", label="Obstacle collisions")
    ax.set_title(title)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Collisions (cumulative)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="upper left", fontsize="small")
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_combined(times, covs, rows, out_png: Path, title: str):
    t = [r[0] for r in rows]
    rob = [r[2] for r in rows]
    obs = [r[3] for r in rows]

    fig, ax_cov = plt.subplots(figsize=(10, 4.8))
    ax_col = ax_cov.twinx()

    ax_cov.plot(times, covs, label="Coverage (%)")
    ax_cov.set_xlabel("Time (s)")
    ax_cov.set_ylabel("Coverage (%)")
    ax_cov.set_ylim(0, 105)
    ax_cov.grid(True, linestyle="--", alpha=0.4)

    ax_col.step(t, rob, where="post", label="Robot collisions")
    ax_col.step(t, obs, where="post", label="Obstacle collisions")
    ax_col.set_ylabel("Collisions (cumulative)")

    handles, labels = [], []
    for ax in (ax_cov, ax_col):
        h, l = ax.get_legend_handles_labels()
        handles += h
        labels += l
    if handles:
        ax_cov.legend(handles, labels, loc="upper left", fontsize="small")

    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)


def main():
    run_dir = pick_latest_run_dir(RESULTS_DIR)
    cov_csv = run_dir / "coverage_timeseries.csv"
    if not cov_csv.exists():
        raise SystemExit(f"Missing {cov_csv}")

    cov_t, cov = read_coverage_timeseries(cov_csv)
    if not cov_t:
        raise SystemExit(f"Empty coverage timeseries in {cov_csv}")

    bump_file = pick_latest_bump_file(BUMP_LOG_DIR, BUMP_LABEL)
    if bump_file is None:
        print(f"[WARN] No bump file found in {BUMP_LOG_DIR} (bumps_{BUMP_LABEL}_*.csv). Plotting coverage only.")
        plot_coverage(cov_t, cov, run_dir / "coverage_vs_time_offline.png", "Coverage vs Time")
        return

    rows, fieldnames = read_bump_rows(bump_file)
    if not rows:
        print(f"[WARN] Bump file empty: {bump_file}. Plotting coverage only.")
        plot_coverage(cov_t, cov, run_dir / "coverage_vs_time_offline.png", "Coverage vs Time")
        return

    segments = split_into_segments(rows)
    if not segments:
        print(f"[WARN] Could not split bump log into segments: {bump_file}")
        plot_coverage(cov_t, cov, run_dir / "coverage_vs_time_offline.png", "Coverage vs Time")
        return

    # Use the LAST segment = most recent run appended to this bump file
    events = segments[-1]

    # optional debouncing
    events = debounce_events(events, MIN_RECOUNT_SEC_ROBOT, MIN_RECOUNT_SEC_OBS)

    # Build time series sampled at coverage timestamps
    collision_rows = build_collision_timeseries(cov_t, events)

    # Save collision_timeseries.csv in the run folder (so your aggregator works)
    out_collision_csv = run_dir / "collision_timeseries.csv"
    save_collision_csv(out_collision_csv, collision_rows)

    # Plots
    plot_coverage(cov_t, cov, run_dir / "coverage_vs_time_offline.png", "Coverage vs Time")
    plot_collisions(collision_rows, run_dir / "collisions_vs_time_offline.png", "Collisions vs Time")
    plot_combined(cov_t, cov, collision_rows, run_dir / "coverage_and_collisions_offline.png", "Coverage & Collisions")

    # Quick console sanity
    final_tot = collision_rows[-1][1]
    final_rob = collision_rows[-1][2]
    final_obs = collision_rows[-1][3]
    print(f"[OK] {run_dir.name}")
    print(f"  bump_file: {bump_file}")
    print(f"  segments: {len(segments)} (using last)")
    print(f"  final collisions: total={final_tot} robot={final_rob} obstacle={final_obs}")
    print(f"  wrote: {out_collision_csv}")


if __name__ == "__main__":
    main()
