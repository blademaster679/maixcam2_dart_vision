#!/usr/bin/env python3
r"""Render source-frame-aligned RGB replay results as a side-by-side H.264 MP4.

Example (both JSONL files must already be complete):
  python3 tools/video_replay/render_comparison.py \
    --input recordings/maixcam2/2026-09-20/clip_000013_test5min/record_180fps.mp4 \
    --baseline artifacts/clip13/baseline.jsonl \
    --optimized artifacts/clip13/optimized.jsonl \
    --frame-step 6 --output artifacts/clip13/comparison.mp4

Coordinates default to the source video's native dimensions. For resized replay
results, provide their processing dimensions using --coordinates-width/height.
This visualization shows RGB replay detections, not calibrated angles or control
validity. The red quadrilateral spans the two detected bars; it is not a measured
physical board boundary. Orange marks predicted lamps or cached bar geometry;
the header distinguishes a fresh bar detection from a cached result.
Input videos with variable frame rate are displayed on
their nominal FPS timeline, matching dart_video_replay.
"""

import argparse
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

import cv2
import numpy as np


HEADER_HEIGHT = 64
GREEN = (70, 255, 70)
RED = (50, 70, 255)
ORANGE = (0, 175, 255)
WHITE = (235, 235, 235)


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return number


def load_rows(path):
    """Use original source frame indices, rejecting ambiguous duplicate records."""
    rows = {}
    previous = -1
    with path.open(encoding="utf-8") as source:
        for line_number, line in enumerate(source, 1):
            if not line.strip():
                continue
            row = json.loads(line)
            frame = row.get("frame")
            if type(frame) is not int or frame < 0 or frame <= previous:
                raise ValueError(f"{path}:{line_number}: frame must be a strictly increasing integer")
            pipeline = row.get("replay_pipeline", "rgb")
            if pipeline != "rgb":
                raise ValueError(f"{path}:{line_number}: expected RGB replay, got {pipeline!r}")
            rows[frame] = row
            previous = frame
    if not rows:
        raise ValueError(f"no replay rows in {path}")
    return rows


def draw_cross(image, center, color, radius=7):
    x, y = center
    cv2.line(image, (x - radius, y), (x + radius, y), color, 1, cv2.LINE_AA)
    cv2.line(image, (x, y - radius), (x, y + radius), color, 1, cv2.LINE_AA)


def draw_panel(source, row, label, frame_index, fps, panel_size, coordinate_size):
    width, height = panel_size
    scale_x, scale_y = width / coordinate_size[0], height / coordinate_size[1]
    resized = cv2.resize(source, (width, height), interpolation=cv2.INTER_AREA)

    def point(x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError(f"non-finite detection coordinates at source frame {frame_index}")
        return round(x * scale_x), round(y * scale_y)

    green = row.get("green", row)
    predicted = bool(green.get("predicted", False))
    if green.get("valid", False):
        center = point(green["center_x"], green["center_y"])
        color = ORANGE if predicted else GREEN
        radius = max(7, 4 + round(0.5 * max(green.get("bbox_w", 0) * scale_x,
                                          green.get("bbox_h", 0) * scale_y)))
        cv2.circle(resized, center, radius, color, 2, cv2.LINE_AA)
        draw_cross(resized, center, color, radius + 4)
        green_state = "PREDICTED" if predicted else "DETECTED"
    else:
        green_state = str(green.get("state", "LOST")).upper()

    armor = row.get("armor", {})
    armor_observed = bool(row.get("armor_detection_ran", False))
    armor_color = RED if armor_observed else ORANGE
    if armor.get("valid", False):
        bar_points = []
        for name in ("left_bar", "right_bar"):
            bar = armor[name]
            top = point(bar["top"]["x"], bar["top"]["y"])
            bottom = point(bar["bottom"]["x"], bar["bottom"]["y"])
            cv2.line(resized, top, bottom, armor_color, 3, cv2.LINE_AA)
            bar_points.append((top, bottom))
        polygon = np.array([bar_points[0][0], bar_points[1][0],
                            bar_points[1][1], bar_points[0][1]], dtype=np.int32)
        cv2.polylines(resized, [polygon], True, armor_color, 1, cv2.LINE_AA)
        if armor.get("center", {}).get("valid", False):
            draw_cross(resized, point(armor["center"]["x"], armor["center"]["y"]), armor_color, 5)

    panel = np.zeros((height + HEADER_HEIGHT, width, 3), dtype=np.uint8)
    panel[:HEADER_HEIGHT] = (24, 24, 24)
    panel[HEADER_HEIGHT:] = resized
    time_s = frame_index / fps
    text_scale = min(0.55, max(0.32, width / 1222))
    cv2.putText(panel, f"{label} | RGB REPLAY | {time_s:6.2f}s | f={frame_index}",
                (10, 23), cv2.FONT_HERSHEY_SIMPLEX, text_scale, WHITE, 1, cv2.LINE_AA)
    armor_state = ("DETECTED" if armor_observed else "CACHED") if armor.get("valid", False) else "NONE"
    cv2.putText(panel, f"GREEN {green_state}   RED BARS {armor_state}",
                (10, 49), cv2.FONT_HERSHEY_SIMPLEX, text_scale,
                ORANGE if predicted else WHITE, 1, cv2.LINE_AA)
    return panel


def render(arguments):
    if arguments.output.resolve() in {arguments.input.resolve(), arguments.baseline.resolve(),
                                      arguments.optimized.resolve()}:
        raise ValueError("output must differ from all input files")
    if arguments.output.exists() and not arguments.overwrite:
        raise FileExistsError(f"output exists; use --overwrite: {arguments.output}")
    if (arguments.coordinates_width is None) != (arguments.coordinates_height is None):
        raise ValueError("provide both --coordinates-width and --coordinates-height")
    ffmpeg = shutil.which(arguments.ffmpeg)
    if not ffmpeg:
        raise FileNotFoundError(f"ffmpeg executable not found: {arguments.ffmpeg}")
    baseline = load_rows(arguments.baseline)
    optimized = load_rows(arguments.optimized)
    capture = cv2.VideoCapture(str(arguments.input))
    if not capture.isOpened():
        raise ValueError(f"cannot open input video: {arguments.input}")
    process = None
    temporary_path = None
    stderr = tempfile.TemporaryFile()
    try:
        source_width = round(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        source_height = round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        source_fps = capture.get(cv2.CAP_PROP_FPS)
        if source_width <= 0 or source_height <= 0 or not math.isfinite(source_fps) or source_fps <= 0:
            raise ValueError("input video has invalid dimensions or nominal FPS")
        scale = min(1.0, 672 / source_width, 380 / source_height)
        # yuv420p requires even dimensions. Preserve aspect ratio to within 1 px.
        panel_width = max(2, round(source_width * scale / 2) * 2)
        panel_height = max(2, round(source_height * scale / 2) * 2)
        coordinate_size = (arguments.coordinates_width or source_width,
                           arguments.coordinates_height or source_height)
        output_fps = source_fps / arguments.frame_step
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(prefix=arguments.output.stem + ".",
                                         suffix=".partial.mp4", dir=arguments.output.parent,
                                         delete=False) as temporary:
            temporary_path = Path(temporary.name)
        process = subprocess.Popen(
            [ffmpeg, "-hide_banner", "-loglevel", "error", "-y",
             "-f", "rawvideo", "-pix_fmt", "bgr24", "-s:v",
             f"{2 * panel_width}x{panel_height + HEADER_HEIGHT}",
             "-r", f"{output_fps:.10f}", "-i", "pipe:0", "-an",
             "-c:v", "libx264", "-preset", "fast", "-crf", "20",
             "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(temporary_path)],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=stderr,
        )
        frame_index = 0
        rendered_frames = 0
        while (arguments.max_frames is None or frame_index < arguments.max_frames) and capture.grab():
            if frame_index % arguments.frame_step == 0:
                for name, rows in (("baseline", baseline), ("optimized", optimized)):
                    if frame_index not in rows:
                        raise ValueError(f"{name} JSONL missing source frame {frame_index}; "
                                         "use complete results at the same frame step")
                decoded, frame = capture.retrieve()
                if not decoded:
                    raise ValueError(f"cannot decode source frame {frame_index}")
                panels = [draw_panel(frame, rows[frame_index], label, frame_index,
                                     source_fps, (panel_width, panel_height), coordinate_size)
                          for rows, label in ((baseline, "BASELINE"), (optimized, "OPTIMIZED"))]
                process.stdin.write(np.hstack(panels).tobytes())
                rendered_frames += 1
                if rendered_frames % 300 == 0:
                    print(f"rendered {rendered_frames} frames, source time {frame_index / source_fps:.1f}s",
                          file=sys.stderr, flush=True)
            frame_index += 1
        if rendered_frames == 0:
            raise ValueError("input contains no decodable frames")
        process.stdin.close()
        result = process.wait(timeout=60)
        if result:
            stderr.seek(0)
            raise RuntimeError(f"ffmpeg exited {result}: {stderr.read().decode(errors='replace')}")
        temporary_path.replace(arguments.output)
        temporary_path = None
        print(json.dumps({"output": str(arguments.output), "frames": rendered_frames,
                          "source_frames_read": frame_index, "frame_step": arguments.frame_step,
                          "source_fps": source_fps, "output_fps": output_fps,
                          "width": 2 * panel_width, "height": panel_height + HEADER_HEIGHT,
                          "replay_pipeline": "rgb"}))
    finally:
        capture.release()
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if process is not None and process.stdin is not None and not process.stdin.closed:
            try:
                process.stdin.close()
            except OSError:
                pass
        stderr.close()
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--optimized", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frame-step", type=positive_int, default=6)
    parser.add_argument("--max-frames", type=positive_int, help="limit source frames, not rendered frames")
    parser.add_argument("--coordinates-width", type=positive_int)
    parser.add_argument("--coordinates-height", type=positive_int)
    parser.add_argument("--ffmpeg", default="ffmpeg")
    parser.add_argument("--overwrite", action="store_true")
    arguments = parser.parse_args()
    try:
        render(arguments)
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        parser.exit(1, f"error: {error}\n")


if __name__ == "__main__":
    main()
