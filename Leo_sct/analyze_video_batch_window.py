#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import shutil
import subprocess
import sys
from pathlib import Path

import matplotlib
import numpy as np

if not matplotlib.get_backend().lower().startswith("agg"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt


REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_VIDEO_ROOT = REPO_ROOT / "video_analysis_out"
DEFAULT_ANALYSIS_ROOT = DEFAULT_VIDEO_ROOT / "window_trim5s_len150s"
DEFAULT_OUTPUT_TAG = "all18_trim5s_len150s"
DEFAULT_MODELS = ("gpt41", "gpt54")
DEFAULT_CONTROLLERS = ("1", "2", "3")
DEFAULT_STEP_S = 1.0 / 15.0
MODEL_COLORS = {
    "gpt41": "#4c78a8",
    "gpt54": "#e45756",
}
MODEL_LABELS = {
    "gpt41": "GPT-4.1",
    "gpt54": "GPT-5.4",
}
HEATMAP_CMAP = "YlGnBu"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Trim a fixed window from all controller videos, analyze each clip, "
            "and regenerate aggregate collision/coverage and heatmap outputs."
        )
    )
    parser.add_argument("--video-root", type=Path, default=DEFAULT_VIDEO_ROOT)
    parser.add_argument("--analysis-root", type=Path, default=DEFAULT_ANALYSIS_ROOT)
    parser.add_argument("--output-tag", default=DEFAULT_OUTPUT_TAG)
    parser.add_argument("--trim-start", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=150.0)
    parser.add_argument("--models", nargs="+", default=list(DEFAULT_MODELS))
    parser.add_argument("--controllers", nargs="+", default=list(DEFAULT_CONTROLLERS))
    parser.add_argument(
        "--force",
        action="store_true",
        help="Rebuild trimmed clips and rerun analysis even when outputs already exist.",
    )
    return parser.parse_args()


def sanitize_stem(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", name).strip("_")


def discover_videos(video_root: Path, models: list[str], controllers: list[str]) -> list[dict]:
    videos: list[dict] = []
    for model in models:
        for controller in controllers:
            folder = video_root / model / controller
            if not folder.exists():
                continue
            for video_path in sorted(folder.glob("*.mp4")):
                videos.append(
                    {
                        "model": model,
                        "controller": controller,
                        "video_path": video_path.resolve(),
                    }
                )
    return videos


def run_command(cmd: list[str]) -> None:
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            "Command failed:\n"
            + " ".join(cmd)
            + "\n\nstdout:\n"
            + result.stdout
            + "\n\nstderr:\n"
            + result.stderr
        )


def trim_video(input_path: Path, output_path: Path, trim_start: float, duration: float, force: bool) -> None:
    if output_path.exists() and not force:
        return
    output_path.parent.mkdir(parents=True, exist_ok=True)
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg is required but was not found in PATH.")
    cmd = [
        ffmpeg,
        "-y",
        "-ss",
        f"{trim_start:.3f}",
        "-t",
        f"{duration:.3f}",
        "-i",
        str(input_path),
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "veryfast",
        "-crf",
        "18",
        "-pix_fmt",
        "yuv420p",
        str(output_path),
    ]
    run_command(cmd)


def analyze_trimmed_video(trimmed_video: Path, analysis_dir: Path, force: bool) -> None:
    summary_path = analysis_dir / "summary.json"
    if summary_path.exists() and not force:
        return
    analysis_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable,
        str(REPO_ROOT / "analyze_real_robot_video.py"),
        "--video",
        str(trimmed_video),
        "--detect-gray-rectangle",
        "--output-dir",
        str(analysis_dir),
    ]
    run_command(cmd)


def read_coverage_series(path: Path, max_time: float) -> tuple[list[float], list[float]]:
    times: list[float] = []
    values: list[float] = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                time_s = float(row["time_s"])
                coverage_pct = float(row["coverage_pct"])
            except (KeyError, TypeError, ValueError):
                continue
            if time_s > max_time + 1e-9:
                break
            times.append(time_s)
            values.append(coverage_pct)
    return times, values


def read_collision_series(path: Path, max_time: float) -> tuple[list[float], list[float], list[float]]:
    times: list[float] = []
    robot_values: list[float] = []
    obstacle_values: list[float] = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                sec = float(row["stamp_sec"])
                nsec = float(row.get("stamp_nsec", "0"))
                robot_idx = float(row.get("robot_index", "0"))
                obstacle_idx = float(row.get("obstacle_index", "0"))
            except (KeyError, TypeError, ValueError):
                continue
            time_s = sec + nsec * 1e-9
            if time_s > max_time + 1e-9:
                break
            times.append(time_s)
            robot_values.append(robot_idx)
            obstacle_values.append(obstacle_idx)
    return times, robot_values, obstacle_values


def read_heatmap_counts(path: Path) -> dict[int, int]:
    counts: dict[int, int] = {}
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                cell_index = int(row["cell_index"])
            except (KeyError, TypeError, ValueError):
                continue
            counts[cell_index] = counts.get(cell_index, 0) + 1
    return counts


def step_sample(times: list[float], values: list[float], sample_times: np.ndarray) -> np.ndarray:
    if not times or not values:
        return np.zeros_like(sample_times, dtype=float)
    sampled = np.zeros_like(sample_times, dtype=float)
    idx = 0
    current_value = values[0]
    for sample_idx, sample_t in enumerate(sample_times):
        while idx + 1 < len(times) and times[idx + 1] <= sample_t + 1e-9:
            idx += 1
            current_value = values[idx]
        sampled[sample_idx] = current_value
    return sampled


def write_collisions_coverage_csv(
    sample_times: np.ndarray,
    robot_matrix: np.ndarray,
    obstacle_matrix: np.ndarray,
    coverage_matrix: np.ndarray,
    per_model_matrices: dict[str, dict[str, np.ndarray]],
    out_csv: Path,
) -> None:
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        header = [
            "time_s",
            "robot_collisions_mean",
            "robot_collisions_min",
            "robot_collisions_max",
            "obstacle_collisions_mean",
            "obstacle_collisions_min",
            "obstacle_collisions_max",
            "coverage_mean_pct",
            "coverage_min_pct",
            "coverage_max_pct",
        ]
        for model_name in sorted(per_model_matrices):
            header.extend(
                [
                    f"{model_name}_robot_collisions_mean",
                    f"{model_name}_robot_collisions_min",
                    f"{model_name}_robot_collisions_max",
                    f"{model_name}_obstacle_collisions_mean",
                    f"{model_name}_obstacle_collisions_min",
                    f"{model_name}_obstacle_collisions_max",
                    f"{model_name}_coverage_mean_pct",
                    f"{model_name}_coverage_min_pct",
                    f"{model_name}_coverage_max_pct",
                ]
            )
        writer.writerow(header)
        for idx, time_s in enumerate(sample_times):
            row = [
                f"{time_s:.3f}",
                f"{robot_matrix[:, idx].mean():.3f}",
                f"{robot_matrix[:, idx].min():.3f}",
                f"{robot_matrix[:, idx].max():.3f}",
                f"{obstacle_matrix[:, idx].mean():.3f}",
                f"{obstacle_matrix[:, idx].min():.3f}",
                f"{obstacle_matrix[:, idx].max():.3f}",
                f"{coverage_matrix[:, idx].mean():.3f}",
                f"{coverage_matrix[:, idx].min():.3f}",
                f"{coverage_matrix[:, idx].max():.3f}",
            ]
            for model_name in sorted(per_model_matrices):
                model_robot = per_model_matrices[model_name]["robot"]
                model_obstacle = per_model_matrices[model_name]["obstacle"]
                model_coverage = per_model_matrices[model_name]["coverage"]
                row.extend(
                    [
                        f"{model_robot[:, idx].mean():.3f}",
                        f"{model_robot[:, idx].min():.3f}",
                        f"{model_robot[:, idx].max():.3f}",
                        f"{model_obstacle[:, idx].mean():.3f}",
                        f"{model_obstacle[:, idx].min():.3f}",
                        f"{model_obstacle[:, idx].max():.3f}",
                        f"{model_coverage[:, idx].mean():.3f}",
                        f"{model_coverage[:, idx].min():.3f}",
                        f"{model_coverage[:, idx].max():.3f}",
                    ]
                )
            writer.writerow(row)


def plot_collisions_coverage(
    sample_times: np.ndarray,
    robot_matrix: np.ndarray,
    obstacle_matrix: np.ndarray,
    coverage_matrix: np.ndarray,
    per_model_matrices: dict[str, dict[str, np.ndarray]],
    out_png: Path,
) -> None:
    fig, (ax_robot, ax_obstacle, ax_cov) = plt.subplots(
        1,
        3,
        figsize=(16.5, 4.8),
        sharex=True,
    )

    robot_mean = robot_matrix.mean(axis=0)
    robot_min = robot_matrix.min(axis=0)
    robot_max = robot_matrix.max(axis=0)
    obstacle_mean = obstacle_matrix.mean(axis=0)
    obstacle_min = obstacle_matrix.min(axis=0)
    obstacle_max = obstacle_matrix.max(axis=0)
    coverage_mean = coverage_matrix.mean(axis=0)
    coverage_min = coverage_matrix.min(axis=0)
    coverage_max = coverage_matrix.max(axis=0)

    ax_robot.fill_between(sample_times, robot_min, robot_max, color="#b8b8b8", alpha=0.10)
    ax_obstacle.fill_between(sample_times, obstacle_min, obstacle_max, color="#c7c7c7", alpha=0.08)
    ax_cov.fill_between(sample_times, coverage_min, coverage_max, color="#c8d2c8", alpha=0.16)
    ax_robot.plot(
        sample_times,
        robot_mean,
        color="#666666",
        linewidth=1.5,
        alpha=0.85,
        label="All",
    )
    ax_obstacle.plot(
        sample_times,
        obstacle_mean,
        color="#7a7a7a",
        linewidth=1.5,
        alpha=0.85,
        label="All",
    )
    ax_cov.plot(
        sample_times,
        coverage_mean,
        color="#6d7f6d",
        linewidth=1.6,
        alpha=0.90,
        label="All",
    )

    for model_name in sorted(per_model_matrices):
        color = MODEL_COLORS.get(model_name, None)
        model_label = MODEL_LABELS.get(model_name, model_name)
        model_robot = per_model_matrices[model_name]["robot"]
        model_obstacle = per_model_matrices[model_name]["obstacle"]
        model_coverage = per_model_matrices[model_name]["coverage"]

        model_robot_mean = model_robot.mean(axis=0)
        model_robot_min = model_robot.min(axis=0)
        model_robot_max = model_robot.max(axis=0)
        model_obstacle_mean = model_obstacle.mean(axis=0)
        model_obstacle_min = model_obstacle.min(axis=0)
        model_obstacle_max = model_obstacle.max(axis=0)
        model_coverage_mean = model_coverage.mean(axis=0)
        model_coverage_min = model_coverage.min(axis=0)
        model_coverage_max = model_coverage.max(axis=0)

        ax_robot.fill_between(sample_times, model_robot_min, model_robot_max, color=color, alpha=0.12)
        ax_obstacle.fill_between(sample_times, model_obstacle_min, model_obstacle_max, color=color, alpha=0.07)
        ax_cov.fill_between(sample_times, model_coverage_min, model_coverage_max, color=color, alpha=0.08)
        ax_robot.plot(
            sample_times,
            model_robot_mean,
            color=color,
            linewidth=2.3,
            label=model_label,
        )
        ax_obstacle.plot(
            sample_times,
            model_obstacle_mean,
            color=color,
            linewidth=2.3,
            label=model_label,
        )
        ax_cov.plot(
            sample_times,
            model_coverage_mean,
            color=color,
            linewidth=2.3,
            label=model_label,
        )

    ax_robot.set_xlabel("Time (s)")
    ax_obstacle.set_xlabel("Time (s)")
    ax_cov.set_xlabel("Time (s)")
    ax_robot.set_ylabel("Robot Collisions")
    ax_obstacle.set_ylabel("Obstacle Collisions")
    ax_cov.set_ylabel("Coverage (%)")
    ax_robot.set_xlim(0.0, float(sample_times[-1]))
    ax_robot.set_ylim(bottom=0.0)
    ax_obstacle.set_ylim(bottom=0.0)
    ax_cov.set_ylim(0.0, 100.0)
    ax_robot.grid(True, linestyle="--", alpha=0.30)
    ax_obstacle.grid(True, linestyle="--", alpha=0.30)
    ax_cov.grid(True, linestyle="--", alpha=0.30)
    ax_robot.legend(loc="upper left", frameon=False)

    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)


def write_heatmap_csv(cell_counts: dict[int, int], out_csv: Path, rows: int = 20, cols: int = 30) -> None:
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["row", "col", "cell_index", "cell_min_x", "cell_min_y", "coverage_count"])
        for row in range(rows):
            for col in range(cols):
                cell_index = row * cols + col
                writer.writerow(
                    [
                        row,
                        col,
                        cell_index,
                        f"{col * 0.1:.1f}",
                        f"{row * 0.1:.1f}",
                        cell_counts.get(cell_index, 0),
                    ]
                )


def counts_to_grid(cell_counts: dict[int, int], rows: int = 20, cols: int = 30) -> np.ndarray:
    grid = np.zeros((rows, cols), dtype=float)
    for cell_index, count in cell_counts.items():
        row = cell_index // cols
        col = cell_index % cols
        if 0 <= row < rows and 0 <= col < cols:
            grid[row, col] = count
    return grid


def plot_heatmap(
    cell_counts: dict[int, int],
    per_model_cell_counts: dict[str, dict[int, int]],
    out_png: Path,
    rows: int = 20,
    cols: int = 30,
) -> None:
    ordered_models = [model_name for model_name in DEFAULT_MODELS if model_name in per_model_cell_counts]
    if not ordered_models:
        ordered_models = sorted(per_model_cell_counts)

    if len(ordered_models) < 2:
        grid = counts_to_grid(cell_counts, rows=rows, cols=cols)
        fig, ax = plt.subplots(figsize=(10, 6))
        image = ax.imshow(grid, cmap=HEATMAP_CMAP, origin="lower", aspect="auto")
        ax.set_xlabel("Arena X cell")
        ax.set_ylabel("Arena Y cell")
        ax.set_title("Coverage")
        cbar = fig.colorbar(image, ax=ax)
        cbar.set_label("Visits")
        fig.tight_layout()
        fig.savefig(out_png, dpi=200, bbox_inches="tight")
        plt.close(fig)
        return

    grids = [counts_to_grid(per_model_cell_counts[model_name], rows=rows, cols=cols) for model_name in ordered_models]
    vmax = max(float(grid.max()) for grid in grids) if grids else 1.0
    vmax = max(vmax, 1.0)

    fig, axes = plt.subplots(
        1,
        len(ordered_models),
        figsize=(12.5, 5.8),
        sharex=True,
        sharey=True,
        constrained_layout=True,
    )
    if len(ordered_models) == 1:
        axes = [axes]

    image = None
    for ax, model_name, grid in zip(axes, ordered_models, grids):
        image = ax.imshow(grid, cmap=HEATMAP_CMAP, origin="lower", aspect="auto", vmin=0.0, vmax=vmax)
        ax.set_xlabel("Arena X cell")
        ax.set_title(f"{MODEL_LABELS.get(model_name, model_name)}")
    axes[0].set_ylabel("Arena Y cell")

    cbar = fig.colorbar(image, ax=axes, shrink=0.92)
    cbar.set_label("Visits")
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)


def build_aggregate_outputs(manifest: list[dict[str, str]], duration: float, output_dir: Path, output_tag: str) -> dict[str, str]:
    sample_times = np.arange(0.0, duration + 1e-9, DEFAULT_STEP_S)
    coverage_series: list[np.ndarray] = []
    robot_series: list[np.ndarray] = []
    obstacle_series: list[np.ndarray] = []
    cell_counts: dict[int, int] = {}
    per_model_cell_counts: dict[str, dict[int, int]] = {}
    model_series: dict[str, dict[str, list[np.ndarray]]] = {}

    for item in manifest:
        model_name = item["model"]
        analysis_dir = Path(item["analysis_dir"])
        coverage_csv = analysis_dir / "coverage_timeseries.csv"
        visited_csv = analysis_dir / "coverage_visited_cells.csv"
        bump_files = sorted(analysis_dir.glob("bumps_global_*.csv"))
        if not coverage_csv.exists() or not visited_csv.exists():
            raise RuntimeError(f"Missing analysis outputs in {analysis_dir}")

        cov_times, cov_values = read_coverage_series(coverage_csv, duration)
        coverage_sampled = step_sample(cov_times, cov_values, sample_times)
        coverage_series.append(coverage_sampled)

        if bump_files:
            col_times, robot_values, obstacle_values = read_collision_series(bump_files[-1], duration)
            robot_sampled = step_sample(col_times, robot_values, sample_times)
            obstacle_sampled = step_sample(col_times, obstacle_values, sample_times)
        else:
            robot_sampled = np.zeros_like(sample_times, dtype=float)
            obstacle_sampled = np.zeros_like(sample_times, dtype=float)

        robot_series.append(robot_sampled)
        obstacle_series.append(obstacle_sampled)
        model_series.setdefault(model_name, {"robot": [], "obstacle": [], "coverage": []})
        model_series[model_name]["robot"].append(robot_sampled)
        model_series[model_name]["obstacle"].append(obstacle_sampled)
        model_series[model_name]["coverage"].append(coverage_sampled)
        per_model_cell_counts.setdefault(model_name, {})

        for cell_index, count in read_heatmap_counts(visited_csv).items():
            cell_counts[cell_index] = cell_counts.get(cell_index, 0) + count
            per_model_counts = per_model_cell_counts[model_name]
            per_model_counts[cell_index] = per_model_counts.get(cell_index, 0) + count

    coverage_matrix = np.vstack(coverage_series)
    robot_matrix = np.vstack(robot_series)
    obstacle_matrix = np.vstack(obstacle_series)
    per_model_matrices = {
        model_name: {
            key: np.vstack(series_list)
            for key, series_list in series_dict.items()
        }
        for model_name, series_dict in model_series.items()
    }

    collisions_csv = output_dir / f"collisions_coverage_average_{output_tag}.csv"
    collisions_png = output_dir / f"collisions_coverage_average_{output_tag}.png"
    heatmap_csv = output_dir / f"coverage_heatmap_{output_tag}.csv"
    heatmap_png = output_dir / f"coverage_heatmap_{output_tag}.png"

    write_collisions_coverage_csv(
        sample_times,
        robot_matrix,
        obstacle_matrix,
        coverage_matrix,
        per_model_matrices,
        collisions_csv,
    )
    plot_collisions_coverage(
        sample_times,
        robot_matrix,
        obstacle_matrix,
        coverage_matrix,
        per_model_matrices,
        collisions_png,
    )
    write_heatmap_csv(cell_counts, heatmap_csv)
    plot_heatmap(cell_counts, per_model_cell_counts, heatmap_png)

    return {
        "collisions_coverage_average_csv": str(collisions_csv),
        "collisions_coverage_average_png": str(collisions_png),
        "coverage_heatmap_csv": str(heatmap_csv),
        "coverage_heatmap_png": str(heatmap_png),
    }


def publish_per_video_outputs(
    manifest: list[dict[str, str]],
    video_root: Path,
    trim_start: float,
    duration: float,
) -> None:
    start_tag = f"{int(trim_start)}s" if float(trim_start).is_integer() else f"{trim_start:g}s"
    end_time = trim_start + duration
    end_tag = f"{int(end_time)}s" if float(end_time).is_integer() else f"{end_time:g}s"
    window_tag = f"{start_tag}_to_{end_tag}"

    file_map = {
        "coverage_map_plot.png": f"coverage_map_plot_{window_tag}.png",
        "coverage_map_video.png": f"coverage_map_video_{window_tag}.png",
        "collisions_vs_time_video.png": f"collisions_vs_time_video_{window_tag}.png",
        "summary.json": f"summary_{window_tag}.json",
    }

    for item in manifest:
        model = item["model"]
        controller = item["controller"]
        source_video = Path(item["source_video"])
        analysis_dir = Path(item["analysis_dir"])
        dest_dir = video_root / model / controller
        base_name = sanitize_stem(source_video.stem)

        for src_name, dst_suffix in file_map.items():
            src_path = analysis_dir / src_name
            if not src_path.exists():
                continue
            dst_path = dest_dir / f"{base_name}_{dst_suffix}"
            shutil.copy2(src_path, dst_path)


def main() -> None:
    args = parse_args()
    videos = discover_videos(args.video_root.resolve(), args.models, args.controllers)
    if not videos:
        raise SystemExit(f"No videos found under {args.video_root}")

    analysis_dirs: list[Path] = []
    manifest: list[dict[str, str]] = []

    for item in videos:
        video_path = item["video_path"]
        model = item["model"]
        controller = item["controller"]
        safe_stem = sanitize_stem(video_path.stem)
        analysis_dir = (args.analysis_root / model / controller / safe_stem).resolve()
        trimmed_path = analysis_dir / f"{safe_stem}_{args.output_tag}.mp4"

        trim_video(video_path, trimmed_path, args.trim_start, args.duration, args.force)
        analyze_trimmed_video(trimmed_path, analysis_dir, args.force)

        analysis_dirs.append(analysis_dir)
        manifest.append(
            {
                "model": model,
                "controller": controller,
                "source_video": str(video_path),
                "trimmed_video": str(trimmed_path),
                "analysis_dir": str(analysis_dir),
            }
        )

    aggregate_outputs = build_aggregate_outputs(
        manifest=manifest,
        duration=args.duration,
        output_dir=args.video_root.resolve(),
        output_tag=args.output_tag,
    )
    publish_per_video_outputs(
        manifest=manifest,
        video_root=args.video_root.resolve(),
        trim_start=args.trim_start,
        duration=args.duration,
    )

    manifest_path = args.video_root.resolve() / f"analysis_manifest_{args.output_tag}.json"
    manifest_path.write_text(
        json.dumps(
            {
                "trim_start_s": args.trim_start,
                "duration_s": args.duration,
                "video_count": len(manifest),
                "videos": manifest,
                "aggregate_outputs": aggregate_outputs,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    print(f"[OK] analyzed {len(manifest)} videos")
    print(f"[OK] manifest: {manifest_path}")
    for key, value in aggregate_outputs.items():
        print(f"[OK] {key}: {value}")


if __name__ == "__main__":
    main()
