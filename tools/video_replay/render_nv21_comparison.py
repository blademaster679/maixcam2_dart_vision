#!/usr/bin/env python3
"""Render exact-source NV21 board observations, never infer missing detections.

Each main section samples nominal input ticks at 30 fps and backfills only an
exact-sequence completed vision observation onto its own original source image.
This is an offline observation visualization, not a capture of live output.
Sparse events and a known false match are shown again as labeled stills.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

import cv2
from PIL import Image, ImageDraw, ImageFont

WIDTH, HEIGHT, PANEL, SOURCE_Y, SOURCE_H = 1280, 900, 640, 190, 362
RESAMPLE = getattr(Image, "Resampling", Image)
BG, WHITE, MUTED = (18, 25, 37), (235, 240, 248), (162, 175, 196)
CYAN, YELLOW, GREEN, ORANGE, RED = (69, 207, 231), (255, 211, 65), (73, 242, 132), (255, 157, 65), (255, 89, 101)
CASES = [("small", "远处小目标"), ("bars", "较近双条板"), ("large", "大绿灯近景"), ("empty", "无目标场景")]


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1048576), b""):
            digest.update(block)
    return digest.hexdigest()


def read_csv(path):
    with path.open() as handle:
        return list(csv.DictReader(handle))


class Run:
    def __init__(self, path):
        self.path = path
        self.meta = json.loads((path / "replay.json").read_text())
        self.analysis = json.loads((path / "analysis.json").read_text())
        if not self.analysis["validation_pass"]:
            raise ValueError(f"unvalidated run: {path}")
        self.feed = {int(row["sequence"]): row for row in read_csv(path / "feed.csv")}
        self.vision = {int(row["sequence"]): row for row in read_csv(path / "vision.csv")}
        self.geometry = {}
        with (path / "targets.jsonl").open() as handle:
            for line in handle:
                target = json.loads(line)
                sequence = target["source_sequence"]
                row = self.vision.get(sequence)
                if not row or row["armor_ran"] != "1" or row["armor_valid"] != "1":
                    continue
                source_time = int(row["received_us"])
                if not (target["source_metadata_valid"] and target["armor"]["valid"] and
                        target["source_received_us"] == source_time == target["armor_source_received_us"]):
                    continue
                if sequence in self.geometry and self.geometry[sequence] != target["armor"]:
                    raise ValueError(f"conflicting geometry: {path}/{sequence}")
                self.geometry[sequence] = target["armor"]
        required = {seq for seq, row in self.vision.items() if row["armor_ran"] == row["armor_valid"] == "1"}
        if required != self.geometry.keys():
            raise ValueError(f"missing fresh geometry: {path}: {required - self.geometry.keys()}")
        for seq, row in self.vision.items():
            source = self.feed[seq]
            if row["received_us"] != source["received_us"] or row["pts_raw"] != source["pts_raw"]:
                raise ValueError("vision/feed metadata mismatch")

    def source_frame(self, sequence):
        expected = self.meta["source_first_frame"] + (sequence - 1) % self.meta["cache_frames"]
        if sequence in self.feed and int(self.feed[sequence]["source_frame"]) != expected:
            raise ValueError("non-contiguous source cache")
        return expected


def load_source_frames(path, runs):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise ValueError("cannot open source video")
    if (round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))) != (1344, 760):
        raise ValueError("unexpected source dimensions")
    if abs(cap.get(cv2.CAP_PROP_FPS) - 180) > .01:
        raise ValueError("expected nominal 180 fps input")
    cache = {}
    try:
        for key, _ in CASES:
            run = runs[key, "baseline"]
            first = run.meta["source_first_frame"]
            count = run.meta["cache_frames"]
            other = runs[key, "optimized"]
            if (first, count) != (other.meta["source_first_frame"], other.meta["cache_frames"]):
                raise ValueError("comparison windows differ")
            if not cap.set(cv2.CAP_PROP_POS_FRAMES, first):
                raise ValueError("cannot seek source window")
            for frame in range(first, first + count):
                ok, image = cap.read()
                if not ok or abs(cap.get(cv2.CAP_PROP_POS_FRAMES) - frame - 1) > .1:
                    raise ValueError(f"incorrect source position: {frame}")
                cache[frame] = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    finally:
        cap.release()
    return cache


class Painter:
    def __init__(self, font):
        self.fonts = {size: ImageFont.truetype(str(font), size) for size in (16, 18, 20, 22, 28, 38)}
        self.latin = {size: ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", size)
                      for size in self.fonts}

    def text(self, draw, xy, value, size=20, color=WHITE):
        # Droid's CJK fallback has no ASCII glyphs; choose a real Latin font for
        # digits and units so benchmark values cannot silently become boxes.
        x, y = xy
        groups = []
        for char in value:
            latin = ord(char) < 0x3000
            if groups and groups[-1][0] == latin:
                groups[-1][1] += char
            else:
                groups.append([latin, char])
        for latin, part in groups:
            font = self.latin[size] if latin else self.fonts[size]
            draw.text((x, y + size), part, font=font, fill=color, anchor="ls")
            x += draw.textlength(part, font=font)

    def annotated(self, rgb, row, geometry):
        image = Image.fromarray(rgb)
        draw = ImageDraw.Draw(image)
        if row is None:
            return image, None, "本源帧未执行检测", MUTED
        x, y, width, height = [int(row[name]) for name in ("roi_x", "roi_y", "roi_width", "roi_height")]
        draw.rectangle((x, y, x + width - 1, y + height - 1), outline=CYAN, width=3)
        size = float(row["green_size"])
        if row["direct_green"] == "1":
            status, color = "绿灯：已确认直接观测", GREEN
        elif row["green_valid"] == row["green_predicted"] == "1":
            status, color = "绿灯：预测", ORANGE
        elif row["green_state"] == "1":
            status, color = "绿灯：候选，尚未确认", YELLOW
        else:
            status, color = "绿灯：未确认目标", MUTED
        if size > 0 and (row["green_valid"] == "1" or row["green_state"] == "1"):
            gx, gy = float(row["green_x"]), float(row["green_y"])
            radius = max(5, size / 2 + 2)
            draw.ellipse((gx - radius, gy - radius, gx + radius, gy + radius), outline=color, width=3)
            draw.line((gx - 7, gy, gx + 7, gy), fill=color, width=2)
            draw.line((gx, gy - 7, gx, gy + 7), fill=color, width=2)
        elif row["green_state"] == "1":
            status = "候选状态；本次未命中"
        if geometry:
            points = []
            for name in ("left_bar", "right_bar"):
                bar = geometry[name]
                top = (bar["top"]["x"], bar["top"]["y"])
                bottom = (bar["bottom"]["x"], bar["bottom"]["y"])
                draw.line((top, bottom), fill=RED, width=4)
                points.append((top, bottom))
            draw.line([points[0][0], points[1][0], points[1][1], points[0][1], points[0][0]], fill=RED, width=2)
        return image, (x, y, width, height), status, color

    def frame(self, title, context, runs, source, key, sequence, note=""):
        canvas = Image.new("RGB", (WIDTH, HEIGHT), BG)
        draw = ImageDraw.Draw(canvas)
        self.text(draw, (18, 10), title, 28)
        self.text(draw, (18, 49), context, 18, MUTED)
        draw.line((640, 80, 640, 804), fill=(58, 69, 87), width=2)
        for index, profile in enumerate(("baseline", "optimized")):
            left = index * PANEL
            run = runs[key, profile]
            row = run.vision.get(sequence)
            frame = run.source_frame(sequence)
            geometry = run.geometry.get(sequence)
            image, roi, status, color = self.annotated(source[frame], row, geometry)
            rate = run.analysis["rates"]
            self.text(draw, (left + 14, 82), "原参数" if profile == "baseline" else "第13段实验参数", 22)
            self.text(draw, (left + 427, 87), "速率统计：3–19.5 s", 16, MUTED)
            self.text(draw, (left + 14, 116), f"检测调用 {rate['green_calls_hz']:.2f} Hz  |  直接绿灯 {rate['direct_green_outputs_hz']:.2f} Hz", 18)
            self.text(draw, (left + 14, 146), f"双条调用 {rate['armor_calls_hz']:.2f} Hz  |  有效双条输出 {rate['armor_direct_valid_outputs_hz']:.2f} Hz", 18, MUTED)
            canvas.paste(image.resize((PANEL, SOURCE_H), RESAMPLE.LANCZOS), (left, SOURCE_Y))
            if roi:
                x, y, w, h = roi
                zoom = image.crop((x, y, x + w, y + h)).resize((224, 224), RESAMPLE.NEAREST)
                canvas.paste(zoom, (left + 12, 571))
            else:
                draw.rectangle((left + 12, 571, left + 236, 795), fill=(30, 39, 53))
                self.text(draw, (left + 35, 660), "无本帧检测结果", 18, MUTED)
            tx = left + 250
            self.text(draw, (tx, 571), status, 20, color)
            self.text(draw, (tx, 608), f"源帧 {frame}  |  原片 {frame / 180:.3f} s", 18)
            self.text(draw, (tx, 638), f"提交序号 {sequence}  |  循环 {(sequence - 1) // run.meta['cache_frames']}", 18, MUTED)
            if row:
                self.text(draw, (tx, 668), f"ROI {row['roi_width']}×{row['roi_height']}  |  候选 {row['proposal_count']}", 18, CYAN)
                detect_ms = float(row["detect_us"]) / 1000
                vision_ms = (int(row["finished_us"]) - int(row["started_us"])) / 1000
                self.text(draw, (tx, 698), f"本次检测 {detect_ms:.2f} ms / 视觉 {vision_ms:.2f} ms", 18)
                armor = "双条：本次有效输出（待核验）" if geometry else "双条：本次无有效输出"
                if row["armor_valid"] == "1" and not geometry:
                    armor = "双条：缓存结果，未绘制旧几何"
                self.text(draw, (tx, 730), armor, 18, RED if geometry else MUTED)
            else:
                self.text(draw, (tx, 685), "调度跳过或等待槽替换", 18, MUTED)
                self.text(draw, (tx, 720), "不能据此判定目标漏检", 18, MUTED)
        draw.rectangle((0, 807, WIDTH, HEIGHT), fill=(10, 17, 28))
        for x, label, color in ((18, "青：当前ROI", CYAN), (230, "黄：未确认候选", YELLOW), (478, "绿：直接观测", GREEN),
                                (706, "橙：预测", ORANGE), (918, "红：算法双条结果", RED)):
            self.text(draw, (x, 814), label, 18, color)
        self.text(draw, (18, 849), note or "源帧对齐的日志回填；30 fps画面抽样不代表检测帧率。红线不保证正确，也不是板体分割边界。", 18, MUTED)
        return canvas

    def slate(self, heading, lines):
        canvas = Image.new("RGB", (WIDTH, HEIGHT), BG)
        draw = ImageDraw.Draw(canvas)
        self.text(draw, (70, 130), heading, 38)
        for index, line in enumerate(lines):
            self.text(draw, (70, 230 + index * 70), line, 22, WHITE if index == 0 else MUTED)
        return canvas


def render(args):
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite {args.output}")
    runs = {(case, profile): Run(args.results / f"{case}_{profile}")
            for case, _ in CASES for profile in ("baseline", "optimized")}
    sources = load_source_frames(args.input, runs)
    painter = Painter(args.font)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    audit = {"source_video": str(args.input), "source_sha256": sha(args.input), "render_fps": 30,
             "width": WIDTH, "height": HEIGHT, "method": "offline_exact_sequence_observations_backfilled_on_their_own_source_frames",
             "benchmark_not_reexecuted": True, "video_sampling_not_accuracy": True,
             "source_display": "original_MP4_decoded_RGB_not_raw_NV21", "sections": [], "input_hashes": {}}
    for run in runs.values():
        for name in ("feed.csv", "vision.csv", "targets.jsonl", "analysis.json"):
            audit["input_hashes"][str(run.path / name)] = sha(run.path / name)
    frames = 0
    temporary = tempfile.NamedTemporaryFile(suffix=".mp4", prefix="nv21-render-", dir=args.output.parent, delete=False)
    temporary.close()
    temporary_path = Path(temporary.name)
    stderr = tempfile.TemporaryFile()
    process = subprocess.Popen(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo",
        "-pix_fmt", "rgb24", "-s:v", f"{WIDTH}x{HEIGHT}", "-r", "30", "-i", "pipe:0", "-an",
        "-c:v", "libx264", "-preset", "fast", "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        str(temporary_path)], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=stderr)

    def emit(frame, repeat=1):
        nonlocal frames
        data = frame.tobytes()
        for _ in range(repeat):
            process.stdin.write(data)
            frames += 1

    def section(label, details):
        audit["sections"].append({"start_seconds": frames / 30, "label": label, **details})

    try:
        section("说明", {})
        emit(painter.slate("第13段录像 · NV21板端识别对照", [
            "左：原参数    右：第13段实验参数", "4个半秒窗口，每组循环20秒；识别在MaixCAM2上执行。",
            "本视频按板端日志回填到对应源帧，不重新运行检测。", "检测调用约86.55～90 Hz，不等于成功识别频率。",
            "末尾包含候选轮询、双条误配和大ROI耗时特写。"]), 90)
        for key, name in CASES:
            section(name, {"case": key, "source_first_frame": runs[key, "baseline"].meta["source_first_frame"],
                           "duration_seconds": 20, "input_tick_step": 6, "playback": "1x_nominal_feed_timeline"})
            for index in range(600):
                sequence = 1 + index * 6
                image = painter.frame(f"NV21板端对照 · {name}", f"片段循环实验  {index / 30:05.2f} / 20.00 s  |  对齐原始输入序号；不同步模拟控制端显示",
                                      runs, sources, key, sequence)
                emit(image)
                if index == 180:
                    image.save(args.output.with_name(f"preview_{key}.jpg"), quality=93)
            print(f"rendered {key}: {frames / 30:.1f}s", flush=True)
        section("候选轮询逐帧特写", {"case": "bars", "playback": "five_observations_each_0.8s"})
        rows = [seq for seq, row in runs["bars", "optimized"].vision.items()
                if int(row["received_us"]) >= runs["bars", "optimized"].meta["feed_started_us"] + 3000000]
        for seq in rows[:5]:
            emit(painter.frame("候选轮询 · 连续5次检测逐帧查看", "每个观察帧停留0.8秒；注意ROI离开绿灯后仍计入确认窗口",
                               runs, sources, "bars", seq,
                               "板端该窗口每5次只有1次候选命中，无法满足最近5次至少3次命中的确认条件。"), 24)
        for seq, label in ((1, "5310：双条位置目检对齐"), (41, "5350：水平棕色结构误配"), (61, "5370：双条位置目检对齐")):
            section(label, {"case": "bars", "sequence": seq, "playback": "3s_still_AI_visual_review_not_manual_GT"})
            picture = painter.frame(f"双条结果特写 · {label}", "AI目检示例，非用户人工真值；精确使用该源帧的本次检测几何",
                                    runs, sources, "bars", seq,
                                    "源帧5350的红线落在水平棕色结构，算法仍返回有效；有效输出不等于识别正确。" if seq == 41
                                    else "红线连接的是算法灯条端点；黄色绿灯仍是未确认候选。")
            emit(picture, 90)
            if seq == 41:
                picture.save(args.output.with_name("preview_false_pair.jpg"), quality=95)
        run = runs["large", "optimized"]
        window = run.analysis["window"]
        seq = next(seq for seq, row in run.vision.items() if int(row["roi_width"]) == 384 and
                   window["start_us"] <= int(row["started_us"]) < int(row["finished_us"]) < window["end_us_exclusive"])
        section("大ROI特写", {"case": "large", "sequence": seq, "playback": "4s_still"})
        image = painter.frame("大ROI跟踪 · 单次处理开销", "整体平均值由96×96搜索主导；384×384 ROI需要单独看耗时",
                              runs, sources, "large", seq, "稳态384×384样本共12次：检测均值27.11ms，视觉调用均值38.30ms；不能持续90Hz。")
        emit(image, 120)
        image.save(args.output.with_name("preview_large_roi.jpg"), quality=95)
        section("结果说明", {})
        emit(painter.slate("当前结论", ["同一NV21流程已完成板端验证，日志与源帧可追溯。",
            "小目标和双条窗口尚未确认绿灯；大目标只有短暂确认。", "已观察到双条误配，不能把有效输出当作正确识别。",
            "优先修复连续确认策略，并复测大ROI稳定跟踪负载。", "完整数字见检测效果报告和检测帧率报告。"]), 120)
        process.stdin.close()
        process.wait(timeout=120)
        if process.returncode:
            stderr.seek(0)
            raise RuntimeError(stderr.read().decode(errors="replace"))
        temporary_path.replace(args.output)
        audit.update(rendered_frames=frames, duration_seconds=frames / 30, output_sha256=sha(args.output))
        args.output.with_suffix(".json").write_text(json.dumps(audit, indent=2, ensure_ascii=False) + "\n")
        print(args.output, frames, f"{frames / 30:.1f}s", flush=True)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        stderr.close()
        temporary_path.unlink(missing_ok=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--font", type=Path, default=Path("/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf"))
    render(parser.parse_args())
