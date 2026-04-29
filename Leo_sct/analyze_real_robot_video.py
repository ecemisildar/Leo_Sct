#!/usr/bin/env python3
"""
Analyze a ceiling-mounted video of real robots moving in a 3 x 2 m arena.

Outputs:
- coverage_timeseries.csv
- coverage_paths.csv
- coverage_visited_cells.csv
- bumps_global_<time>.csv
- coverage_map_video.png
- summary.json

Example:
  python3 analyze_real_robot_video.py \
      --video /path/to/video.mp4 \
      --crop-left 180 \
      --arena-corners 312,188 3652,210 3678,2478 298,2460 \
      --output-dir /tmp/run_analysis

Or detect a gray arena rectangle automatically:
  python3 analyze_real_robot_video.py \
      --video /path/to/video.mp4 \
      --detect-gray-rectangle \
      --output-dir /tmp/run_analysis
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import matplotlib
import numpy as np
if not matplotlib.get_backend().lower().startswith("agg"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt

cv2.setNumThreads(1)
if hasattr(cv2, "ocl"):
    cv2.ocl.setUseOpenCL(False)


DEFAULT_ARENA_WIDTH_M = 3.0
DEFAULT_ARENA_HEIGHT_M = 2.0
DEFAULT_PIXELS_PER_METER = 250
DEFAULT_GRID_SIZE_M = 0.10
DEFAULT_ROBOT_WIDTH_M = 0.45
DEFAULT_ROBOT_HEIGHT_M = 0.45
DEFAULT_MARKER_TO_FRONT_M = 0.15
DEFAULT_MARKER_TO_BACK_M = 0.30
DEFAULT_MAX_ASSIGNMENT_M = 0.30
DEFAULT_BUMP_COOLDOWN_SEC = 0.8
DEFAULT_WALL_BUMP_MARGIN_M = 0.04
DEFAULT_WALL_RELEASE_MARGIN_M = 0.08
DEFAULT_WALL_DETECTION_PADDING_M = 0.15
DEFAULT_MIN_CONTOUR_AREA = 150
DEFAULT_MAX_CONTOUR_AREA = 60000
DEFAULT_BACKGROUND_HISTORY = 400
DEFAULT_WARMUP_FRAMES = 0
DEFAULT_FRAME_STEP = 2
DEFAULT_MAX_MISSED_FRAMES = 12
DEFAULT_ARUCO_DICT = "DICT_4X4_50"
DEFAULT_ARUCO_IDS = [0, 1, 2]
ROBOT_COLORS_BGR = [
    (255, 99, 71),
    (60, 179, 113),
    (65, 105, 225),
    (255, 191, 0),
    (186, 85, 211),
    (64, 224, 208),
]
ROBOT_COLORS_MPL = [
    "#e76f51",
    "#2a9d8f",
    "#3a86ff",
    "#f4a261",
    "#9d4edd",
    "#2ec4b6",
]


@dataclass
class Track:
    track_id: int
    centroid_px: tuple[float, float]
    centroid_m: tuple[float, float]
    last_frame_index: int
    heading_xy: tuple[float, float] = (0.0, -1.0)
    missed_frames: int = 0
    age_frames: int = 1


@dataclass
class BumpState:
    active_pairs: dict[str, float] = field(default_factory=dict)
    last_event_time: dict[str, float] = field(default_factory=dict)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Estimate arena coverage and bump events from an overhead robot video."
    )
    parser.add_argument("--video", required=True, help="Path to the input video file.")
    parser.add_argument(
        "--arena-corners",
        nargs=4,
        metavar=("TL", "TR", "BR", "BL"),
        help="Arena corners in image pixels as x,y ordered top-left top-right bottom-right bottom-left.",
    )
    parser.add_argument(
        "--output-dir",
        help="Directory for CSV and summary outputs. Defaults to <video-stem>_video_analysis.",
    )
    parser.add_argument("--crop-left", type=int, default=0, help="Crop this many pixels from the left side.")
    parser.add_argument("--crop-right", type=int, default=0, help="Crop this many pixels from the right side.")
    parser.add_argument("--crop-top", type=int, default=0, help="Crop this many pixels from the top side.")
    parser.add_argument("--crop-bottom", type=int, default=0, help="Crop this many pixels from the bottom side.")
    parser.add_argument(
        "--detect-gray-rectangle",
        action="store_true",
        help="Automatically detect the main gray arena rectangle from the first cropped frame.",
    )
    parser.add_argument(
        "--gray-min",
        type=int,
        default=50,
        help="Minimum grayscale value for gray rectangle detection.",
    )
    parser.add_argument(
        "--gray-max",
        type=int,
        default=210,
        help="Maximum grayscale value for gray rectangle detection.",
    )
    parser.add_argument(
        "--gray-max-channel-diff",
        type=int,
        default=18,
        help="Maximum allowed BGR channel spread for a pixel to count as gray.",
    )
    parser.add_argument(
        "--min-rectangle-area-ratio",
        type=float,
        default=0.15,
        help="Minimum image area ratio for the detected gray rectangle.",
    )
    parser.add_argument(
        "--gray-border-margin-px",
        type=int,
        default=20,
        help="Reject gray-rectangle candidates that touch the image border within this margin.",
    )
    parser.add_argument("--arena-width-m", type=float, default=DEFAULT_ARENA_WIDTH_M)
    parser.add_argument("--arena-height-m", type=float, default=DEFAULT_ARENA_HEIGHT_M)
    parser.add_argument("--pixels-per-meter", type=int, default=DEFAULT_PIXELS_PER_METER)
    parser.add_argument("--grid-size-m", type=float, default=DEFAULT_GRID_SIZE_M)
    parser.add_argument(
        "--robot-width-m",
        type=float,
        default=DEFAULT_ROBOT_WIDTH_M,
        help="Robot footprint width in meters. Default assumes 45 cm.",
    )
    parser.add_argument(
        "--robot-height-m",
        type=float,
        default=DEFAULT_ROBOT_HEIGHT_M,
        help="Robot footprint height in meters. Default assumes 45 cm.",
    )
    parser.add_argument("--frame-step", type=int, default=DEFAULT_FRAME_STEP)
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=DEFAULT_WARMUP_FRAMES,
        help="Frames to skip before analysis. Default is 0 for ArUco-based tracking.",
    )
    parser.add_argument("--background-history", type=int, default=DEFAULT_BACKGROUND_HISTORY)
    parser.add_argument("--min-contour-area", type=int, default=DEFAULT_MIN_CONTOUR_AREA)
    parser.add_argument("--max-contour-area", type=int, default=DEFAULT_MAX_CONTOUR_AREA)
    parser.add_argument("--max-assignment-distance-m", type=float, default=DEFAULT_MAX_ASSIGNMENT_M)
    parser.add_argument("--max-missed-frames", type=int, default=DEFAULT_MAX_MISSED_FRAMES)
    parser.add_argument("--bump-cooldown-sec", type=float, default=DEFAULT_BUMP_COOLDOWN_SEC)
    parser.add_argument("--wall-bump-margin-m", type=float, default=DEFAULT_WALL_BUMP_MARGIN_M)
    parser.add_argument("--wall-release-margin-m", type=float, default=DEFAULT_WALL_RELEASE_MARGIN_M)
    parser.add_argument(
        "--save-debug-video",
        action="store_true",
        help="Write a rectified debug video with detections, tracks, and bump overlays.",
    )
    return parser.parse_args()


def parse_corner(text: str) -> tuple[float, float]:
    x_text, y_text = text.split(",", 1)
    return (float(x_text), float(y_text))


def validate_args(args: argparse.Namespace) -> None:
    if not args.detect_gray_rectangle and not args.arena_corners:
        raise SystemExit(
            "Provide --arena-corners TL TR BR BL, or use --detect-gray-rectangle."
        )
    if args.robot_width_m <= 0.0 or args.robot_height_m <= 0.0:
        raise SystemExit("--robot-width-m and --robot-height-m must be positive.")


def apply_crop(frame: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    height, width = frame.shape[:2]
    left = max(0, args.crop_left)
    right = max(0, args.crop_right)
    top = max(0, args.crop_top)
    bottom = max(0, args.crop_bottom)
    x0 = left
    x1 = width - right
    y0 = top
    y1 = height - bottom
    if x0 >= x1 or y0 >= y1:
        raise SystemExit(
            f"Invalid crop: left={left} right={right} top={top} bottom={bottom} for frame {width}x{height}"
        )
    return frame[y0:y1, x0:x1]


def adjust_corners_for_crop(
    arena_corners: list[tuple[float, float]],
    args: argparse.Namespace,
) -> list[tuple[float, float]]:
    return [
        (x - max(0, args.crop_left), y - max(0, args.crop_top))
        for x, y in arena_corners
    ]


def order_corners(points: np.ndarray) -> list[tuple[float, float]]:
    pts = np.asarray(points, dtype=np.float32)
    sums = pts.sum(axis=1)
    diffs = pts[:, 0] - pts[:, 1]
    top_left = pts[np.argmin(sums)]
    bottom_right = pts[np.argmax(sums)]
    top_right = pts[np.argmax(diffs)]
    bottom_left = pts[np.argmin(diffs)]
    return [
        (float(top_left[0]), float(top_left[1])),
        (float(top_right[0]), float(top_right[1])),
        (float(bottom_right[0]), float(bottom_right[1])),
        (float(bottom_left[0]), float(bottom_left[1])),
    ]


def detect_gray_rectangle(frame: np.ndarray, args: argparse.Namespace) -> list[tuple[float, float]]:
    b, g, r = cv2.split(frame)
    max_channel = np.maximum(np.maximum(b, g), r)
    min_channel = np.minimum(np.minimum(b, g), r)
    gray_like = (max_channel - min_channel) <= args.gray_max_channel_diff
    intensity = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    intensity_mask = (intensity >= args.gray_min) & (intensity <= args.gray_max)
    mask = np.where(gray_like & intensity_mask, 255, 0).astype(np.uint8)

    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise SystemExit("Gray rectangle detection failed: no suitable contour found.")

    frame_area = frame.shape[0] * frame.shape[1]
    frame_h, frame_w = frame.shape[:2]
    min_area = frame_area * args.min_rectangle_area_ratio
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        touches_border = (
            x <= args.gray_border_margin_px
            or y <= args.gray_border_margin_px
            or (x + w) >= (frame_w - args.gray_border_margin_px)
            or (y + h) >= (frame_h - args.gray_border_margin_px)
        )
        if touches_border:
            continue
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
        if len(approx) == 4:
            candidates.append((area, approx.reshape(4, 2)))

    if not candidates:
        filtered = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            touches_border = (
                x <= args.gray_border_margin_px
                or y <= args.gray_border_margin_px
                or (x + w) >= (frame_w - args.gray_border_margin_px)
                or (y + h) >= (frame_h - args.gray_border_margin_px)
            )
            if not touches_border:
                filtered.append(contour)
        biggest = max(filtered if filtered else contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(biggest)
        box = cv2.boxPoints(rect)
        return order_corners(box)

    candidates.sort(key=lambda item: item[0], reverse=True)
    return order_corners(candidates[0][1])


def save_gray_area_debug_images(
    output_dir: Path,
    cropped_first_frame: np.ndarray,
    arena_corners: list[tuple[float, float]],
    matrix: np.ndarray,
    warp_width_px: int,
    warp_height_px: int,
) -> tuple[Path, Path]:
    overlay = cropped_first_frame.copy()
    polygon = np.array(arena_corners, dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(overlay, [polygon], isClosed=True, color=(0, 255, 0), thickness=4)
    for index, (x, y) in enumerate(arena_corners):
        cv2.circle(overlay, (int(round(x)), int(round(y))), 8, (0, 0, 255), -1)
        cv2.putText(
            overlay,
            str(index),
            (int(round(x)) + 8, int(round(y)) - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    gray_area_only = cv2.warpPerspective(
        cropped_first_frame,
        matrix,
        (warp_width_px, warp_height_px),
    )

    overlay_path = output_dir / "gray_rectangle_detected.png"
    gray_area_only_path = output_dir / "gray_area_only.png"
    cv2.imwrite(str(overlay_path), overlay)
    cv2.imwrite(str(gray_area_only_path), gray_area_only)
    return overlay_path, gray_area_only_path


def ensure_opencv_readable_video(video_path: Path, output_dir: Path) -> Path:
    cap = cv2.VideoCapture(str(video_path))
    opened = cap.isOpened()
    ok, _ = cap.read() if opened else (False, None)
    cap.release()
    if opened and ok:
        return video_path

    ffmpeg_path = shutil.which("ffmpeg")
    if ffmpeg_path is None:
        raise SystemExit(
            f"Could not read first frame from video: {video_path}. "
            "OpenCV decode failed and ffmpeg is not available for fallback conversion."
        )

    converted_path = output_dir / f"{video_path.stem}_opencv_fallback.mp4"
    cmd = [
        ffmpeg_path,
        "-y",
        "-i",
        str(video_path),
        "-an",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(converted_path),
    ]
    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if result.returncode != 0 or not converted_path.exists():
        raise SystemExit(
            f"Could not read first frame from video: {video_path}\n"
            "OpenCV decode failed and ffmpeg fallback conversion also failed.\n"
            f"ffmpeg stderr:\n{result.stderr}"
        )

    cap = cv2.VideoCapture(str(converted_path))
    opened = cap.isOpened()
    ok, _ = cap.read() if opened else (False, None)
    cap.release()
    if not opened or not ok:
        raise SystemExit(
            f"ffmpeg converted the video, but OpenCV still could not read the first frame: {converted_path}"
        )
    return converted_path


def create_aruco_detector(dictionary_name: str):
    if not hasattr(cv2, "aruco"):
        raise SystemExit("This OpenCV build does not include cv2.aruco.")
    dict_id = getattr(cv2.aruco, dictionary_name, None)
    if dict_id is None:
        raise SystemExit(f"Unknown ArUco dictionary: {dictionary_name}")
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    if hasattr(cv2.aruco, "ArucoDetector"):
        if hasattr(cv2.aruco, "DetectorParameters"):
            params = cv2.aruco.DetectorParameters()
        elif hasattr(cv2.aruco, "DetectorParameters_create"):
            params = cv2.aruco.DetectorParameters_create()
        else:
            raise SystemExit("This OpenCV ArUco build has no detector parameter constructor.")
        return ("new", cv2.aruco.ArucoDetector(dictionary, params))
    if hasattr(cv2.aruco, "detectMarkers"):
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            params = cv2.aruco.DetectorParameters_create()
        elif hasattr(cv2.aruco, "DetectorParameters"):
            params = cv2.aruco.DetectorParameters()
        else:
            raise SystemExit("This OpenCV ArUco build has no detector parameter constructor.")
        return ("legacy", (dictionary, params))
    raise SystemExit("This OpenCV ArUco build does not provide a marker detector API.")


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def build_homography(
    arena_corners: list[tuple[float, float]],
    arena_width_m: float,
    arena_height_m: float,
    pixels_per_meter: int,
) -> tuple[np.ndarray, int, int]:
    warp_width_px = int(round(arena_width_m * pixels_per_meter))
    warp_height_px = int(round(arena_height_m * pixels_per_meter))
    src = np.array(arena_corners, dtype=np.float32)
    dst = np.array(
        [
            [0.0, 0.0],
            [warp_width_px - 1.0, 0.0],
            [warp_width_px - 1.0, warp_height_px - 1.0],
            [0.0, warp_height_px - 1.0],
        ],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(src, dst)
    return matrix, warp_width_px, warp_height_px


def detect_robots(
    warped_frame: np.ndarray,
    detector,
) -> tuple[list[tuple[int, float, float, float, float]], np.ndarray]:
    gray = cv2.cvtColor(warped_frame, cv2.COLOR_BGR2GRAY)
    gray = np.ascontiguousarray(gray)
    detector_mode, detector_obj = detector
    if detector_mode == "new":
        corners, ids, _ = detector_obj.detectMarkers(gray)
    else:
        dictionary, params = detector_obj
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            dictionary,
            parameters=params,
        )
    debug = warped_frame.copy()
    detections = []
    if ids is None or len(ids) == 0:
        return detections, debug

    for marker_corners, marker_id in zip(corners, ids.flatten()):
        if int(marker_id) not in DEFAULT_ARUCO_IDS:
            continue
        pts = marker_corners.reshape(4, 2)
        cx = float(np.mean(pts[:, 0]))
        cy = float(np.mean(pts[:, 1]))
        top_mid = 0.5 * (pts[0] + pts[1])
        bottom_mid = 0.5 * (pts[3] + pts[2])
        heading = top_mid - bottom_mid
        heading_norm = float(np.linalg.norm(heading))
        if heading_norm > 1e-6:
            hx = float(heading[0] / heading_norm)
            hy = float(heading[1] / heading_norm)
        else:
            hx, hy = 0.0, -1.0
        poly = np.round(pts).astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(debug, [poly], isClosed=True, color=(0, 255, 0), thickness=2)
        cv2.putText(
            debug,
            str(int(marker_id)),
            (int(round(cx)) + 8, int(round(cy)) - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        detections.append((int(marker_id), cx, cy, hx, hy))
    return detections, debug


def update_tracks_from_markers(
    tracks: dict[int, Track],
    detections: list[tuple[int, float, float, float, float]],
    frame_index: int,
    pixels_per_meter: int,
    max_missed_frames: int,
) -> None:
    seen_ids = set()
    center_offset_m = (DEFAULT_MARKER_TO_BACK_M - DEFAULT_MARKER_TO_FRONT_M) * 0.5
    center_offset_px = center_offset_m * pixels_per_meter
    for marker_id, dx, dy, hx, hy in detections:
        seen_ids.add(marker_id)
        robot_cx = dx - (hx * center_offset_px)
        robot_cy = dy - (hy * center_offset_px)
        track = tracks.get(marker_id)
        if track is None:
            tracks[marker_id] = Track(
                track_id=marker_id,
                centroid_px=(robot_cx, robot_cy),
                centroid_m=(robot_cx / pixels_per_meter, robot_cy / pixels_per_meter),
                last_frame_index=frame_index,
                heading_xy=(hx, hy),
            )
            continue
        track.centroid_px = (robot_cx, robot_cy)
        track.centroid_m = (robot_cx / pixels_per_meter, robot_cy / pixels_per_meter)
        track.last_frame_index = frame_index
        track.heading_xy = (hx, hy)
        track.missed_frames = 0
        track.age_frames += 1

    for track_id in list(tracks.keys()):
        if track_id in seen_ids:
            continue
        tracks[track_id].missed_frames += 1
        if tracks[track_id].missed_frames > max_missed_frames:
            del tracks[track_id]


def build_coverage_grid(
    arena_width_m: float,
    arena_height_m: float,
    grid_size_m: float,
) -> tuple[np.ndarray, np.ndarray, int, int]:
    grid_width = int(math.ceil(arena_width_m / grid_size_m))
    grid_height = int(math.ceil(arena_height_m / grid_size_m))
    return (
        np.zeros((grid_height, grid_width), dtype=np.uint8),
        np.full((grid_height, grid_width), -1, dtype=np.int16),
        grid_width,
        grid_height,
    )


def mark_coverage(
    coverage_grid: np.ndarray,
    coverage_owner_grid: np.ndarray,
    track_id: int,
    x_m: float,
    y_m: float,
    robot_width_m: float,
    robot_height_m: float,
    grid_size_m: float,
) -> list[tuple[int, int]]:
    grid_height, grid_width = coverage_grid.shape
    half_width = robot_width_m * 0.5
    half_height = robot_height_m * 0.5
    min_x = x_m - half_width
    max_x = x_m + half_width
    min_y = y_m - half_height
    max_y = y_m + half_height
    min_col = max(0, int(math.floor(min_x / grid_size_m)))
    max_col = min(grid_width - 1, int(math.floor(max_x / grid_size_m)))
    min_row = max(0, int(math.floor(min_y / grid_size_m)))
    max_row = min(grid_height - 1, int(math.floor(max_y / grid_size_m)))
    newly_visited = []
    for row in range(min_row, max_row + 1):
        for col in range(min_col, max_col + 1):
            if coverage_grid[row, col] == 0:
                coverage_grid[row, col] = 1
                coverage_owner_grid[row, col] = track_id
                newly_visited.append((row, col))
    return newly_visited


def get_robot_color_bgr(track_id: int) -> tuple[int, int, int]:
    return ROBOT_COLORS_BGR[track_id % len(ROBOT_COLORS_BGR)]


def get_robot_color_mpl(track_id: int) -> str:
    return ROBOT_COLORS_MPL[track_id % len(ROBOT_COLORS_MPL)]


def coverage_cell_record(
    row: int,
    col: int,
    grid_width: int,
    grid_size_m: float,
) -> tuple[int, float, float, float, float]:
    idx = row * grid_width + col
    min_x = col * grid_size_m
    min_y = row * grid_size_m
    center_x = min_x + 0.5 * grid_size_m
    center_y = min_y + 0.5 * grid_size_m
    return idx, min_x, min_y, center_x, center_y


def pair_key(a: int, b: int) -> str:
    low, high = sorted((a, b))
    return f"robot_{low}<->robot_{high}"


def wall_key(track_id: int, wall_name: str) -> str:
    return f"robot_{track_id}<->{wall_name}"


def maybe_register_bump(
    bump_state: BumpState,
    key: str,
    now_s: float,
    in_contact: bool,
    cooldown_s: float,
) -> bool:
    if in_contact:
        if bump_state.active_pairs.get(key) is None:
            last_event = bump_state.last_event_time.get(key, -1e9)
            bump_state.active_pairs[key] = now_s
            if (now_s - last_event) >= cooldown_s:
                bump_state.last_event_time[key] = now_s
                return True
    else:
        bump_state.active_pairs.pop(key, None)
    return False


def nearest_wall(
    x_m: float,
    y_m: float,
    arena_width_m: float,
    arena_height_m: float,
    robot_width_m: float,
    robot_height_m: float,
    wall_detection_padding_m: float,
) -> tuple[str, float]:
    half_width = robot_width_m * 0.5
    half_height = robot_height_m * 0.5
    distances = {
        "left_wall": x_m - half_width + wall_detection_padding_m,
        "right_wall": (arena_width_m - half_width + wall_detection_padding_m) - x_m,
        "top_wall": y_m - half_height + wall_detection_padding_m,
        "bottom_wall": (arena_height_m - half_height + wall_detection_padding_m) - y_m,
    }
    wall_name = min(distances, key=distances.get)
    return wall_name, distances[wall_name]


def rectangles_overlap(
    ax_m: float,
    ay_m: float,
    bx_m: float,
    by_m: float,
    robot_width_m: float,
    robot_height_m: float,
    margin_m: float = 0.0,
) -> bool:
    half_width = robot_width_m * 0.5
    half_height = robot_height_m * 0.5
    return (
        abs(ax_m - bx_m) <= (robot_width_m + margin_m)
        and abs(ay_m - by_m) <= (robot_height_m + margin_m)
    )


def create_debug_writer(
    enabled: bool,
    output_dir: Path,
    fps: float,
    frame_size: tuple[int, int],
) -> cv2.VideoWriter | None:
    if not enabled:
        return None
    debug_path = output_dir / "debug_rectified.mp4"
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    return cv2.VideoWriter(str(debug_path), fourcc, fps, frame_size)


def draw_debug_overlay(
    frame: np.ndarray,
    detections_px: list[tuple[float, float]],
    tracks: dict[int, Track],
    coverage_pct: float,
    bump_total: int,
    bump_robot: int,
    bump_wall: int,
) -> np.ndarray:
    canvas = frame.copy()
    for x, y in detections_px:
        cv2.circle(canvas, (int(round(x)), int(round(y))), 10, (0, 255, 255), 2)
    for track in tracks.values():
        x_px, y_px = track.centroid_px
        cv2.circle(canvas, (int(round(x_px)), int(round(y_px))), 8, (0, 0, 255), -1)
        cv2.putText(
            canvas,
            f"id {track.track_id}",
            (int(round(x_px)) + 10, int(round(y_px)) - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    cv2.putText(
        canvas,
        f"coverage={coverage_pct:.2f}% bumps={bump_total} robot={bump_robot} wall={bump_wall}",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return canvas


def write_coverage_map(
    output_dir: Path,
    coverage_grid: np.ndarray,
    coverage_owner_grid: np.ndarray,
    tracks_history: dict[int, list[tuple[float, float]]],
    arena_width_m: float,
    arena_height_m: float,
    pixels_per_meter: int,
) -> None:
    height_px = int(round(arena_height_m * pixels_per_meter))
    width_px = int(round(arena_width_m * pixels_per_meter))
    image = np.zeros((height_px, width_px, 3), dtype=np.uint8)
    image[:] = (20, 20, 120)

    grid_h, grid_w = coverage_grid.shape
    cell_w = width_px / grid_w
    cell_h = height_px / grid_h
    for row in range(grid_h):
        for col in range(grid_w):
            if coverage_grid[row, col] == 0:
                continue
            x0 = int(round(col * cell_w))
            y0 = int(round(row * cell_h))
            x1 = int(round((col + 1) * cell_w))
            y1 = int(round((row + 1) * cell_h))
            owner_track_id = int(coverage_owner_grid[row, col])
            color = get_robot_color_bgr(owner_track_id) if owner_track_id >= 0 else (40, 170, 40)
            cv2.rectangle(image, (x0, y0), (x1, y1), color, thickness=-1)

    for track_id, points in sorted(tracks_history.items()):
        if len(points) < 2:
            continue
        color = get_robot_color_bgr(track_id)
        pixel_points = [
            (int(round(x_m * pixels_per_meter)), int(round(y_m * pixels_per_meter)))
            for x_m, y_m in points
        ]
        for p0, p1 in zip(pixel_points, pixel_points[1:]):
            cv2.line(image, p0, p1, color, 2, cv2.LINE_AA)

    cv2.imwrite(str(output_dir / "coverage_map_video.png"), image)


def plot_coverage_map_matplotlib(
    output_dir: Path,
    coverage_grid: np.ndarray,
    coverage_owner_grid: np.ndarray,
    tracks_history: dict[int, list[tuple[float, float]]],
    arena_width_m: float,
    arena_height_m: float,
    grid_size_m: float,
) -> Path:
    fig, ax = plt.subplots(figsize=(9, 6))
    ax.set_xlabel("X (m)", fontsize=14)
    ax.set_ylabel("Y (m)", fontsize=14)
    ax.set_aspect("equal")
    ax.set_xlim(0.0, arena_width_m)
    ax.set_ylim(0.0, arena_height_m)

    grid_h, grid_w = coverage_grid.shape
    for row in range(grid_h):
        for col in range(grid_w):
            x = col * grid_size_m
            y = row * grid_size_m
            if coverage_grid[row, col]:
                owner_track_id = int(coverage_owner_grid[row, col])
                color = get_robot_color_mpl(owner_track_id) if owner_track_id >= 0 else "green"
            else:
                color = "red"
            rect = plt.Rectangle(
                (x, y),
                grid_size_m,
                grid_size_m,
                facecolor=color,
                edgecolor="black",
                alpha=0.3,
            )
            ax.add_patch(rect)

    for track_id, points in sorted(tracks_history.items()):
        if len(points) < 2:
            continue
        xs, ys = zip(*points)
        ax.plot(
            xs,
            ys,
            linewidth=1.2,
            color=get_robot_color_mpl(track_id),
            label=f"robot_{track_id}",
        )

    visited_cells = int(coverage_grid.sum())
    free_cells = int(coverage_grid.size)
    ax.text(
        0.02,
        0.98,
        f"Visited {visited_cells}/{free_cells} cells",
        transform=ax.transAxes,
        fontsize=12,
        verticalalignment="top",
        bbox=dict(facecolor="white", alpha=0.7, edgecolor="none"),
    )
    if tracks_history:
        ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), fontsize=10)
    fig.tight_layout()
    out_path = output_dir / "coverage_map_plot.png"
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return out_path


def plot_collisions_with_coverage(
    output_dir: Path,
    coverage_history: list[tuple[float, float]],
    bump_history: list[tuple[float, int, int]],
) -> Path:
    cov_times = [t for t, _ in coverage_history]
    cov_values = [v for _, v in coverage_history]
    if not cov_times:
        cov_times = [0.0]
        cov_values = [0.0]

    if bump_history:
        t = [item[0] for item in bump_history]
        rob = [item[1] for item in bump_history]
        obs = [item[2] for item in bump_history]
    else:
        t = list(cov_times)
        rob = [0] * len(t)
        obs = [0] * len(t)

    if t and t[0] > 0.0:
        t = [0.0] + t
        rob = [0] + rob
        obs = [0] + obs

    if cov_times and t[-1] < cov_times[-1]:
        t.append(cov_times[-1])
        rob.append(rob[-1])
        obs.append(obs[-1])

    fig, ax = plt.subplots(figsize=(9, 4))
    ax_cov = ax.twinx()
    ax.plot(t, rob, label="Robot collisions")
    ax.plot(t, obs, label="Obstacle collisions")
    ax_cov.plot(cov_times, cov_values, color="tab:green", label="Coverage (%)")

    ax.set_xlabel("Time (s)", fontsize=14)
    ax.set_ylabel("Collisions (cumulative)", fontsize=14)
    ax_cov.set_ylabel("Coverage (%)", fontsize=14)
    ax.set_xlim(left=0.0)
    ax.set_ylim(bottom=0.0)
    ax_cov.set_ylim(0, 105)
    ax.grid(True, linestyle="--", alpha=0.4)

    handles, labels = [], []
    for axis in (ax, ax_cov):
        h, l = axis.get_legend_handles_labels()
        handles += h
        labels += l
    if handles:
        ax.legend(handles, labels, loc="upper left", fontsize=10)
    fig.tight_layout()
    out_path = output_dir / "collisions_vs_time_video.png"
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return out_path


def analyze_video(args: argparse.Namespace) -> Path:
    video_path = Path(args.video).expanduser().resolve()
    if not video_path.exists():
        raise SystemExit(f"Video not found: {video_path}")

    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else video_path.with_name(f"{video_path.stem}_video_analysis")
    )
    ensure_dir(output_dir)
    readable_video_path = ensure_opencv_readable_video(video_path, output_dir)

    capture = cv2.VideoCapture(str(readable_video_path))
    if not capture.isOpened():
        raise SystemExit(f"Could not open video: {readable_video_path}")

    fps = capture.get(cv2.CAP_PROP_FPS)
    if not fps or math.isnan(fps):
        fps = 30.0
    total_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0)

    ok, first_frame = capture.read()
    if not ok:
        raise SystemExit(f"Could not read first frame from video: {readable_video_path}")

    cropped_first_frame = apply_crop(first_frame, args)
    if args.detect_gray_rectangle:
        arena_corners = detect_gray_rectangle(cropped_first_frame, args)
    else:
        arena_corners = adjust_corners_for_crop(
            [parse_corner(value) for value in args.arena_corners],
            args,
        )
    matrix, warp_width_px, warp_height_px = build_homography(
        arena_corners,
        args.arena_width_m,
        args.arena_height_m,
        args.pixels_per_meter,
    )
    overlay_path, gray_area_only_path = save_gray_area_debug_images(
        output_dir,
        cropped_first_frame,
        arena_corners,
        matrix,
        warp_width_px,
        warp_height_px,
    )
    capture.set(cv2.CAP_PROP_POS_FRAMES, 0)

    aruco_detector = create_aruco_detector(DEFAULT_ARUCO_DICT)
    coverage_grid, coverage_owner_grid, grid_width, _ = build_coverage_grid(
        args.arena_width_m, args.arena_height_m, args.grid_size_m
    )

    coverage_csv_path = output_dir / "coverage_timeseries.csv"
    paths_csv_path = output_dir / "coverage_paths.csv"
    visited_csv_path = output_dir / "coverage_visited_cells.csv"
    bumps_csv_path = output_dir / f"bumps_global_{time.strftime('%H%M%S')}.csv"
    summary_path = output_dir / "summary.json"

    debug_writer = create_debug_writer(
        args.save_debug_video,
        output_dir,
        fps / max(1, args.frame_step),
        (warp_width_px, warp_height_px),
    )

    tracks: dict[int, Track] = {}
    tracks_history: dict[int, list[tuple[float, float]]] = {}
    coverage_history: list[tuple[float, float]] = []
    bump_history: list[tuple[float, int, int]] = []
    processed_frames = 0
    bump_total = 0
    bump_robot = 0
    bump_wall = 0
    bump_state = BumpState()
    wall_state = BumpState()

    with (
        coverage_csv_path.open("w", newline="", encoding="utf-8") as coverage_file,
        paths_csv_path.open("w", newline="", encoding="utf-8") as paths_file,
        visited_csv_path.open("w", newline="", encoding="utf-8") as visited_file,
        bumps_csv_path.open("w", newline="", encoding="utf-8") as bumps_file,
    ):
        coverage_writer = csv.writer(coverage_file)
        coverage_writer.writerow(["time_s", "coverage_pct", "visited_cells", "free_cells"])

        paths_writer = csv.writer(paths_file)
        paths_writer.writerow(["frame_index", "time_s", "robot", "x", "y"])

        visited_writer = csv.writer(visited_file)
        visited_writer.writerow(
            [
                "frame_index",
                "time_s",
                "robot",
                "cell_index",
                "cell_min_x",
                "cell_min_y",
                "cell_center_x",
                "cell_center_y",
            ]
        )

        bumps_writer = csv.writer(bumps_file)
        bumps_writer.writerow(
            [
                "stamp_sec",
                "stamp_nsec",
                "label",
                "total_index",
                "robot_index",
                "obstacle_index",
                "bump_type",
                "key",
                "entity_a",
                "entity_b",
                "avg_contact_x",
                "avg_contact_y",
                "avg_contact_z",
                "source_topic",
            ]
        )

        frame_index = -1
        while True:
            ok, frame = capture.read()
            if not ok:
                break

            frame_index += 1
            if frame_index % max(1, args.frame_step) != 0:
                continue

            processed_frames += 1
            time_s = frame_index / fps
            frame = apply_crop(frame, args)
            warped_frame = cv2.warpPerspective(frame, matrix, (warp_width_px, warp_height_px))
            detections, debug_detection_frame = detect_robots(
                warped_frame,
                aruco_detector,
            )

            if processed_frames <= max(0, args.warmup_frames):
                continue

            update_tracks_from_markers(
                tracks,
                detections,
                frame_index,
                args.pixels_per_meter,
                args.max_missed_frames,
            )

            current_tracks = [track for track in tracks.values() if track.last_frame_index == frame_index]

            for track in current_tracks:
                x_m, y_m = track.centroid_m
                tracks_history.setdefault(track.track_id, []).append((x_m, y_m))
                paths_writer.writerow(
                    [frame_index, f"{time_s:.3f}", f"robot_{track.track_id}", f"{x_m:.4f}", f"{y_m:.4f}"]
                )
                new_cells = mark_coverage(
                    coverage_grid,
                    coverage_owner_grid,
                    track.track_id,
                    x_m,
                    y_m,
                    args.robot_width_m,
                    args.robot_height_m,
                    args.grid_size_m,
                )
                for row, col in new_cells:
                    cell_idx, min_x, min_y, center_x, center_y = coverage_cell_record(
                        row, col, grid_width, args.grid_size_m
                    )
                    visited_writer.writerow(
                        [
                            frame_index,
                            f"{time_s:.3f}",
                            f"robot_{track.track_id}",
                            cell_idx,
                            f"{min_x:.4f}",
                            f"{min_y:.4f}",
                            f"{center_x:.4f}",
                            f"{center_y:.4f}",
                        ]
                    )

            pair_keys_this_frame = set()

            for i, track_a in enumerate(current_tracks):
                ax_m, ay_m = track_a.centroid_m
                for track_b in current_tracks[i + 1:]:
                    bx_m, by_m = track_b.centroid_m
                    key = pair_key(track_a.track_id, track_b.track_id)
                    pair_keys_this_frame.add(key)
                    in_contact = rectangles_overlap(
                        ax_m,
                        ay_m,
                        bx_m,
                        by_m,
                        args.robot_width_m,
                        args.robot_height_m,
                        margin_m=0.0,
                    )
                    if key in bump_state.active_pairs and rectangles_overlap(
                        ax_m,
                        ay_m,
                        bx_m,
                        by_m,
                        args.robot_width_m,
                        args.robot_height_m,
                        margin_m=args.wall_release_margin_m,
                    ):
                        in_contact = True
                    elif key in bump_state.active_pairs:
                        in_contact = False

                    if maybe_register_bump(bump_state, key, time_s, in_contact, args.bump_cooldown_sec):
                        bump_total += 1
                        bump_robot += 1
                        bump_history.append((time_s, bump_robot, bump_wall))
                        avg_x = (ax_m + bx_m) * 0.5
                        avg_y = (ay_m + by_m) * 0.5
                        sec = int(time_s)
                        nsec = int((time_s - sec) * 1e9)
                        bumps_writer.writerow(
                            [
                                sec,
                                nsec,
                                "global",
                                bump_total,
                                bump_robot,
                                bump_wall,
                                "robot",
                                key,
                                f"robot_{track_a.track_id}",
                                f"robot_{track_b.track_id}",
                                f"{avg_x:.4f}",
                                f"{avg_y:.4f}",
                                "0.0000",
                                "video",
                            ]
                        )

            for key in list(bump_state.active_pairs.keys()):
                if key not in pair_keys_this_frame:
                    bump_state.active_pairs.pop(key, None)

            wall_keys_this_frame = set()
            for track in current_tracks:
                x_m, y_m = track.centroid_m
                wall_name, wall_distance_m = nearest_wall(
                    x_m,
                    y_m,
                    args.arena_width_m,
                    args.arena_height_m,
                    args.robot_width_m,
                    args.robot_height_m,
                    DEFAULT_WALL_DETECTION_PADDING_M,
                )
                key = wall_key(track.track_id, wall_name)
                wall_keys_this_frame.add(key)
                in_contact = wall_distance_m <= args.wall_bump_margin_m
                if key in wall_state.active_pairs and wall_distance_m < args.wall_release_margin_m:
                    in_contact = True
                elif key in wall_state.active_pairs and wall_distance_m >= args.wall_release_margin_m:
                    in_contact = False

                if maybe_register_bump(wall_state, key, time_s, in_contact, args.bump_cooldown_sec):
                    bump_total += 1
                    bump_wall += 1
                    bump_history.append((time_s, bump_robot, bump_wall))
                    sec = int(time_s)
                    nsec = int((time_s - sec) * 1e9)
                    bumps_writer.writerow(
                        [
                            sec,
                            nsec,
                            "global",
                            bump_total,
                            bump_robot,
                            bump_wall,
                            "obstacle",
                            key,
                            f"robot_{track.track_id}",
                            wall_name,
                            f"{x_m:.4f}",
                            f"{y_m:.4f}",
                            "0.0000",
                            "video",
                        ]
                    )

            for key in list(wall_state.active_pairs.keys()):
                if key not in wall_keys_this_frame:
                    wall_state.active_pairs.pop(key, None)

            visited_cells = int(coverage_grid.sum())
            free_cells = int(coverage_grid.size)
            coverage_pct = (visited_cells / max(1, free_cells)) * 100.0
            coverage_history.append((time_s, coverage_pct))
            coverage_writer.writerow(
                [f"{time_s:.3f}", f"{coverage_pct:.3f}", visited_cells, free_cells]
            )

            if debug_writer is not None:
                debug_frame = draw_debug_overlay(
                    debug_detection_frame,
                    [(x, y) for _, x, y, _, _ in detections],
                    tracks,
                    coverage_pct,
                    bump_total,
                    bump_robot,
                    bump_wall,
                )
                debug_writer.write(debug_frame)

    capture.release()
    if debug_writer is not None:
        debug_writer.release()

    write_coverage_map(
        output_dir,
        coverage_grid,
        coverage_owner_grid,
        tracks_history,
        args.arena_width_m,
        args.arena_height_m,
        args.pixels_per_meter,
    )
    coverage_map_plot_path = plot_coverage_map_matplotlib(
        output_dir,
        coverage_grid,
        coverage_owner_grid,
        tracks_history,
        args.arena_width_m,
        args.arena_height_m,
        args.grid_size_m,
    )
    collisions_plot_path = plot_collisions_with_coverage(
        output_dir,
        coverage_history,
        bump_history,
    )

    visited_cells = int(coverage_grid.sum())
    free_cells = int(coverage_grid.size)
    summary = {
        "video": str(video_path),
        "opencv_video_source": str(readable_video_path),
        "output_dir": str(output_dir),
        "crop_left": args.crop_left,
        "crop_right": args.crop_right,
        "crop_top": args.crop_top,
        "crop_bottom": args.crop_bottom,
        "detect_gray_rectangle": bool(args.detect_gray_rectangle),
        "robot_detection_mode": "aruco",
        "aruco_dictionary": DEFAULT_ARUCO_DICT,
        "aruco_ids": DEFAULT_ARUCO_IDS,
        "detected_arena_corners": arena_corners,
        "arena_width_m": args.arena_width_m,
        "arena_height_m": args.arena_height_m,
        "pixels_per_meter": args.pixels_per_meter,
        "grid_size_m": args.grid_size_m,
        "robot_width_m": args.robot_width_m,
        "robot_height_m": args.robot_height_m,
        "marker_to_front_m": DEFAULT_MARKER_TO_FRONT_M,
        "marker_to_back_m": DEFAULT_MARKER_TO_BACK_M,
        "wall_detection_padding_m": DEFAULT_WALL_DETECTION_PADDING_M,
        "fps": fps,
        "total_frames": total_frames,
        "processed_frames": processed_frames,
        "visited_cells": visited_cells,
        "free_cells": free_cells,
        "coverage_pct": (visited_cells / max(1, free_cells)) * 100.0,
        "bump_total": bump_total,
        "bump_robot": bump_robot,
        "bump_wall": bump_wall,
        "files": {
            "coverage_timeseries_csv": str(coverage_csv_path),
            "coverage_paths_csv": str(paths_csv_path),
            "coverage_visited_cells_csv": str(visited_csv_path),
            "bumps_csv": str(bumps_csv_path),
            "coverage_map_png": str(output_dir / "coverage_map_video.png"),
            "coverage_map_plot_png": str(coverage_map_plot_path),
            "collisions_plot_png": str(collisions_plot_path),
            "gray_rectangle_detected_png": str(overlay_path),
            "gray_area_only_png": str(gray_area_only_path),
            "summary_json": str(summary_path),
        },
    }
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return output_dir


def main() -> None:
    args = parse_args()
    validate_args(args)
    output_dir = analyze_video(args)
    print(f"[OK] video analysis saved to {output_dir}")


if __name__ == "__main__":
    main()
