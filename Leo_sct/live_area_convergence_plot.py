#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import re
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np


REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_VIDEO = REPO_ROOT / "new_videos" / "2026-04-30 15-21-20.mp4"
DEFAULT_ANALYSIS_ROOT = REPO_ROOT / "new_videos_analysis_out" / "window_trim10s_len180s"
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "new_videos_analysis_out" / "live_area_convergence"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Create a synchronized live-style video showing arena coverage area "
            "converging over time."
        )
    )
    parser.add_argument("--video", type=Path, default=DEFAULT_VIDEO)
    parser.add_argument(
        "--analysis-dir",
        type=Path,
        help="Directory containing coverage_timeseries.csv and summary.json.",
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--output-name", default="area_convergence_live.mp4")
    parser.add_argument(
        "--output-fps",
        type=float,
        default=15.0,
        help="Frame rate of the generated live plot video.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1280,
        help="Output video width in pixels.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=720,
        help="Output video height in pixels.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the live plot while generating it. Requires a GUI session.",
    )
    parser.add_argument(
        "--crop-video-right-at-white",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Crop the video panel at the first sustained white strip on the right side.",
    )
    parser.add_argument(
        "--force-analysis",
        action="store_true",
        help="Rerun video analysis even if a cached coverage CSV exists.",
    )
    return parser.parse_args()


def load_summary(path: Path) -> dict:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def sanitize_stem(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", name).strip("_")


def resolve_analysis_dir(video_path: Path, requested: Path | None) -> Path:
    if requested is not None:
        return requested.expanduser().resolve()
    return DEFAULT_ANALYSIS_ROOT / sanitize_stem(video_path.stem)


def ensure_analysis(video_path: Path, analysis_dir: Path, force: bool) -> None:
    coverage_csv = analysis_dir / "coverage_timeseries.csv"
    summary_path = analysis_dir / "summary.json"
    if coverage_csv.exists() and summary_path.exists() and not force:
        return

    analysis_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable,
        str(REPO_ROOT / "analyze_real_robot_video.py"),
        "--video",
        str(video_path),
        "--detect-gray-rectangle",
        "--use-video-top-corners",
        "--output-dir",
        str(analysis_dir),
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise SystemExit(
            "Video analysis failed.\n"
            f"Command: {' '.join(cmd)}\n\n"
            f"stdout:\n{result.stdout}\n\nstderr:\n{result.stderr}"
        )


def read_coverage(path: Path, grid_size_m: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    times: list[float] = []
    percentages: list[float] = []
    areas_m2: list[float] = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                time_s = float(row["time_s"])
                coverage_pct = float(row["coverage_pct"])
                visited_cells = int(row["visited_cells"])
            except (KeyError, TypeError, ValueError):
                continue
            times.append(time_s)
            percentages.append(coverage_pct)
            areas_m2.append(visited_cells * grid_size_m * grid_size_m)

    if not times:
        raise SystemExit(f"No coverage samples found in {path}")
    return (
        np.asarray(times, dtype=np.float32),
        np.asarray(percentages, dtype=np.float32),
        np.asarray(areas_m2, dtype=np.float32),
    )


def value_at_time(times: np.ndarray, values: np.ndarray, time_s: float) -> float:
    idx = int(np.searchsorted(times, time_s, side="right") - 1)
    idx = max(0, min(idx, len(values) - 1))
    return float(values[idx])


def fit_into(image: np.ndarray, width: int, height: int) -> np.ndarray:
    canvas = np.full((height, width, 3), 245, dtype=np.uint8)
    image_h, image_w = image.shape[:2]
    scale = min(width / image_w, height / image_h)
    scaled_w = max(1, int(round(image_w * scale)))
    scaled_h = max(1, int(round(image_h * scale)))
    resized = cv2.resize(image, (scaled_w, scaled_h), interpolation=cv2.INTER_AREA)
    x0 = (width - scaled_w) // 2
    y0 = (height - scaled_h) // 2
    canvas[y0 : y0 + scaled_h, x0 : x0 + scaled_w] = resized
    return canvas


def detect_right_white_start(frame: np.ndarray) -> int:
    b, g, r = cv2.split(frame)
    white = (b > 210) & (g > 210) & (r > 210)
    column_ratios = white.mean(axis=0)
    rolling_window = 21
    rolling = np.convolve(
        column_ratios,
        np.ones(rolling_window, dtype=np.float32) / rolling_window,
        mode="same",
    )
    min_x = int(frame.shape[1] * 0.55)
    candidates = np.where((np.arange(frame.shape[1]) >= min_x) & (rolling > 0.2))[0]
    if len(candidates) == 0:
        return frame.shape[1]
    return max(1, int(candidates[0]))


def draw_text(
    canvas: np.ndarray,
    text: str,
    origin: tuple[int, int],
    scale: float = 0.8,
    color: tuple[int, int, int] = (35, 35, 35),
    thickness: int = 2,
) -> None:
    cv2.putText(
        canvas,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def draw_plot(
    width: int,
    height: int,
    times: np.ndarray,
    percentages: np.ndarray,
    areas_m2: np.ndarray,
    now_s: float,
    arena_area_m2: float,
) -> np.ndarray:
    canvas = np.full((height, width, 3), 255, dtype=np.uint8)
    margin_l, margin_r, margin_t, margin_b = 90, 40, 92, 92
    plot_x0 = margin_l
    plot_y0 = margin_t
    plot_x1 = width - margin_r
    plot_y1 = height - margin_b
    plot_w = plot_x1 - plot_x0
    plot_h = plot_y1 - plot_y0

    final_pct = float(percentages[-1])
    current_pct = value_at_time(times, percentages, now_s)
    max_time = float(times[-1])

    draw_text(canvas, "Coverage area convergence", (34, 42), 1.05, (25, 25, 25), 2)

    cv2.rectangle(canvas, (plot_x0, plot_y0), (plot_x1, plot_y1), (40, 40, 40), 1)
    for tick in range(0, 101, 20):
        y = int(round(plot_y1 - (tick / 100.0) * plot_h))
        cv2.line(canvas, (plot_x0, y), (plot_x1, y), (225, 225, 225), 1)
        draw_text(canvas, str(tick), (plot_x0 - 52, y + 7), 0.48, (80, 80, 80), 1)

    for frac in np.linspace(0.0, 1.0, 6):
        x = int(round(plot_x0 + frac * plot_w))
        cv2.line(canvas, (x, plot_y0), (x, plot_y1), (236, 236, 236), 1)
        label = f"{frac * max_time:.0f}"
        draw_text(canvas, label, (x - 14, plot_y1 + 34), 0.46, (80, 80, 80), 1)

    visible = times <= now_s + 1e-6
    if not np.any(visible):
        visible[0] = True
    xs = plot_x0 + (times[visible] / max_time) * plot_w
    ys = plot_y1 - (percentages[visible] / 100.0) * plot_h
    points = np.column_stack([xs, ys]).round().astype(np.int32)
    if len(points) >= 2:
        cv2.polylines(canvas, [points], False, (34, 139, 34), 3, cv2.LINE_AA)
    current_x = int(round(plot_x0 + min(now_s, max_time) / max_time * plot_w))
    current_y = int(round(plot_y1 - (current_pct / 100.0) * plot_h))
    cv2.line(canvas, (current_x, plot_y0), (current_x, plot_y1), (120, 120, 120), 1)
    cv2.circle(canvas, (current_x, current_y), 6, (34, 139, 34), -1, cv2.LINE_AA)

    final_y = int(round(plot_y1 - (final_pct / 100.0) * plot_h))
    cv2.line(canvas, (plot_x0, final_y), (plot_x1, final_y), (170, 170, 170), 1)
    draw_text(canvas, f"final {final_pct:.1f}%", (plot_x1 - 138, final_y - 10), 0.5, (80, 80, 80), 1)
    draw_text(canvas, "Time (s)", (plot_x0 + plot_w // 2 - 42, height - 24), 0.58, (35, 35, 35), 1)
    draw_text(canvas, "Coverage (%)", (16, plot_y0 - 18), 0.55, (35, 35, 35), 1)
    return canvas


def create_live_video(
    source_video: Path,
    output_path: Path,
    times: np.ndarray,
    percentages: np.ndarray,
    areas_m2: np.ndarray,
    arena_area_m2: float,
    output_fps: float,
    output_size: tuple[int, int],
    show: bool,
    crop_video_right_at_white: bool,
) -> None:
    capture = cv2.VideoCapture(str(source_video))
    if not capture.isOpened():
        raise SystemExit(f"Could not open video for rendering: {source_video}")

    source_fps = capture.get(cv2.CAP_PROP_FPS)
    if not source_fps or np.isnan(source_fps):
        source_fps = 30.0
    ok, first_frame = capture.read()
    if not ok:
        raise SystemExit(f"Could not read first frame from video: {source_video}")
    crop_right_x = (
        detect_right_white_start(first_frame)
        if crop_video_right_at_white
        else first_frame.shape[1]
    )
    capture.set(cv2.CAP_PROP_POS_FRAMES, 0)

    output_w, output_h = output_size
    left_w = int(output_w * 0.55)
    right_w = output_w - left_w
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(output_path), fourcc, output_fps, (output_w, output_h))
    if not writer.isOpened():
        raise SystemExit(f"Could not open output video writer: {output_path}")

    frame_index = 0
    rendered = 0
    source_step = max(1, int(round(source_fps / output_fps)))
    max_time = float(times[-1])

    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame_index % source_step != 0:
            frame_index += 1
            continue

        time_s = frame_index / source_fps
        if time_s > max_time + (0.5 / output_fps):
            break

        frame = frame[:, :crop_right_x]
        video_panel = fit_into(frame, left_w, output_h)
        plot_panel = draw_plot(
            right_w,
            output_h,
            times,
            percentages,
            areas_m2,
            min(time_s, max_time),
            arena_area_m2,
        )
        combined = np.hstack([video_panel, plot_panel])
        writer.write(combined)
        rendered += 1

        if show:
            cv2.imshow("Area convergence", combined)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        frame_index += 1

    capture.release()
    writer.release()
    if show:
        cv2.destroyAllWindows()
    if rendered == 0:
        raise SystemExit("No frames were rendered.")


def main() -> None:
    args = parse_args()
    video_path = args.video.expanduser().resolve()
    if not video_path.exists():
        raise SystemExit(f"Video not found: {video_path}")

    analysis_dir = resolve_analysis_dir(video_path, args.analysis_dir)
    ensure_analysis(video_path, analysis_dir, args.force_analysis)
    summary = load_summary(analysis_dir / "summary.json")
    grid_size_m = float(summary.get("grid_size_m", 0.1))
    arena_area_m2 = float(summary.get("arena_width_m", 3.0)) * float(
        summary.get("arena_height_m", 2.0)
    )
    coverage_csv = analysis_dir / "coverage_timeseries.csv"
    times, percentages, areas_m2 = read_coverage(coverage_csv, grid_size_m)

    source_video = Path(summary.get("video", video_path)).expanduser().resolve()
    output_path = args.output_dir.expanduser().resolve() / args.output_name
    create_live_video(
        source_video=source_video,
        output_path=output_path,
        times=times,
        percentages=percentages,
        areas_m2=areas_m2,
        arena_area_m2=arena_area_m2,
        output_fps=args.output_fps,
        output_size=(args.width, args.height),
        show=args.show,
        crop_video_right_at_white=args.crop_video_right_at_white,
    )

    print(f"[OK] source video: {source_video}")
    print(f"[OK] coverage CSV: {coverage_csv}")
    print(f"[OK] live convergence video: {output_path}")
    print(f"[OK] final coverage: {percentages[-1]:.1f}% ({areas_m2[-1]:.2f}/{arena_area_m2:.2f} m2)")


if __name__ == "__main__":
    main()
