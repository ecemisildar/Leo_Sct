#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

import matplotlib

if not matplotlib.get_backend().lower().startswith("agg"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image


REPO_ROOT = Path(__file__).resolve().parent
SOURCE_PNG = REPO_ROOT / "video_analysis_out" / "latest_video_screenshots" / "frames_contact_sheet_0_30_60_90_120_150.png"
OUT_PNG = REPO_ROOT / "LATEX FIGURES" / "frames_contact_sheet_0_30_60_90_120_150.png"
FONT_SIZE = 12
TIMES = ["0s", "30s", "60s", "90s", "120s", "150s"]

# Crop only the frame areas from the existing contact sheet, excluding title whitespace.
FRAME_CROPS = [
    (20, 63, 980, 603),
    (1010, 63, 1970, 603),
    (2000, 63, 2960, 603),
    (20, 816, 980, 1356),
    (1010, 816, 1970, 1356),
    (2000, 816, 2960, 1356),
]


def main() -> None:
    image = Image.open(SOURCE_PNG).convert("RGB")
    frames = [image.crop(box) for box in FRAME_CROPS]

    fig, axes = plt.subplots(2, 3, figsize=(14.895, 6.875), dpi=200)
    for ax, frame, title in zip(axes.flat, frames, TIMES):
        ax.imshow(frame)
        ax.set_title(title, fontsize=FONT_SIZE)
        ax.set_axis_off()

    fig.subplots_adjust(left=0.007, right=0.993, top=0.965, bottom=0.015, wspace=0.03, hspace=0.23)
    OUT_PNG.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_PNG, dpi=200)
    plt.close(fig)
    print(f"[OK] wrote {OUT_PNG}")


if __name__ == "__main__":
    main()
