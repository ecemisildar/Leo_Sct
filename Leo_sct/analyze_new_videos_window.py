#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from analyze_video_batch_window import (
    build_aggregate_outputs,
    sanitize_stem,
    trim_video,
    analyze_trimmed_video,
)


REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_VIDEO_DIR = REPO_ROOT / "new_videos"
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "new_videos_analysis_out"
DEFAULT_OUTPUT_TAG = "new_videos_trim10s_len180s"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Analyze a flat folder of real robot videos over a fixed time window "
            "and generate aggregate heatmap and collision/coverage plots."
        )
    )
    parser.add_argument("--video-dir", type=Path, default=DEFAULT_VIDEO_DIR)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--output-tag", default=DEFAULT_OUTPUT_TAG)
    parser.add_argument("--trim-start", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument(
        "--group-label",
        default="new_videos",
        help="Label used in aggregate CSV/plot columns.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Rebuild trimmed clips and rerun analysis even when outputs already exist.",
    )
    parser.add_argument(
        "--use-video-top-corners",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use the video frame's upper corners as the arena top corners.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    video_dir = args.video_dir.resolve()
    output_root = args.output_root.resolve()
    analysis_root = output_root / "window_trim10s_len180s"

    videos = sorted(video_dir.glob("*.mp4"))
    if not videos:
        raise SystemExit(f"No .mp4 videos found in {video_dir}")

    manifest: list[dict[str, str]] = []
    for video_path in videos:
        safe_stem = sanitize_stem(video_path.stem)
        analysis_dir = analysis_root / safe_stem
        trimmed_path = analysis_dir / f"{safe_stem}_{args.output_tag}.mp4"

        trim_video(video_path, trimmed_path, args.trim_start, args.duration, args.force)
        analyzer_args = ["--use-video-top-corners"] if args.use_video_top_corners else []
        analyze_trimmed_video(trimmed_path, analysis_dir, args.force, analyzer_args)

        manifest.append(
            {
                "model": args.group_label,
                "controller": "all",
                "source_video": str(video_path),
                "trimmed_video": str(trimmed_path),
                "analysis_dir": str(analysis_dir),
            }
        )

    aggregate_outputs = build_aggregate_outputs(
        manifest=manifest,
        duration=args.duration,
        output_dir=output_root,
        output_tag=args.output_tag,
    )

    manifest_path = output_root / f"analysis_manifest_{args.output_tag}.json"
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
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
