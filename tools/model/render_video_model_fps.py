#!/usr/bin/env python3
"""Render actual board result frames and wall-clock FPS; never invent detections."""
import argparse
import bisect
import csv
import json
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFont

BILINEAR = getattr(Image, 'Resampling', Image).BILINEAR
FONT = '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'
FONTS = {s: ImageFont.truetype(FONT, s) for s in (19, 22, 25, 30, 40)}
LATIN = {s: ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', s) for s in FONTS}
CASES = [('26', '远处小目标'), ('29.5', '较近目标 / 双条板'),
         ('62', '近景大绿灯'), ('74', '无目标场景')]
W, H, FPS = 1600, 900, 30
BG = (15, 22, 34)
WHITE = (230, 237, 246)
MUTED = (158, 176, 194)
GREEN = (65, 238, 137)
CYAN = (62, 211, 255)
PINK = (255, 108, 206)
YELLOW = (255, 205, 86)


def text(draw, xy, content, size=25, color=WHITE):
    # DroidSansFallback covers CJK but omits ASCII: explicitly select fonts.
    x, y = xy
    runs = []
    for ch in content:
        cjk = '\u2e80' <= ch <= '\u9fff' or '\uff00' <= ch <= '\uffef'
        if runs and runs[-1][0] == cjk:
            runs[-1][1] += ch
        else:
            runs.append([cjk, ch])
    for cjk, run in runs:
        font = FONTS[size] if cjk else LATIN[size]
        draw.text((x, y), run, font=font, fill=color)
        x += draw.textlength(run, font=font)


def cross(draw, xy, color, radius=12):
    x, y = xy
    draw.line((x-radius, y, x+radius, y), fill=color, width=3)
    draw.line((x, y-radius, x, y+radius), fill=color, width=3)
    draw.ellipse((x-5, y-5, x+5, y+5), outline=color, width=2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', type=Path, required=True)
    parser.add_argument('--results', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit('refusing to overwrite video')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ff = subprocess.Popen(['ffmpeg', '-v', 'error', '-nostdin', '-f', 'rawvideo',
        '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', str(FPS), '-i', '-', '-an',
        '-c:v', 'libx264', '-preset', 'fast', '-crf', '21', '-pix_fmt', 'yuv420p',
        '-movflags', '+faststart', str(args.output)], stdin=subprocess.PIPE)
    audit = []
    frame_count = 0
    preview_saved = False
    try:
        for section, (start, label) in enumerate(CASES):
            stem = args.results / f's{start}_i33'
            rows = [{k: float(v) for k, v in r.items()}
                    for r in csv.DictReader(Path(str(stem)+'.csv').open())]
            summary = [json.loads(line) for line in Path(str(stem)+'.log').read_text().splitlines()
                       if line.startswith('{"feed_hz"')][0]
            ends = [r['end_ms']/1000 for r in rows]
            model_rows = [r for r in rows if r['model_calls']]
            model_ends = [r['end_ms']/1000 for r in model_rows]
            classical_ends = [r['end_ms']/1000 for r in rows if r['classical']]
            command = ['ffmpeg', '-v', 'error', '-nostdin', '-ss', start, '-i', str(args.video),
                '-frames:v', '180', '-vf',
                'scale=480:360:flags=bilinear:in_range=full:out_range=full:in_color_matrix=bt601',
                '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-']
            raw = subprocess.check_output(command)
            cache = np.frombuffer(raw, dtype=np.uint8).reshape(180, 360, 480, 3)
            duration = summary['seconds']
            for f in range(round(duration*FPS)):
                t = f/FPS
                idx = bisect.bisect_right(ends, t)-1
                model_idx = bisect.bisect_right(model_ends, t)-1
                row = rows[idx] if idx >= 0 else None
                recent_model = model_rows[model_idx] if model_idx >= 0 else None
                im = Image.new('RGB', (W, H), BG)
                d = ImageDraw.Draw(im)
                text(d, (28, 15), 'MaixCAM2 | V4 模型＋传统视觉 · 板端实测回放', 30)
                text(d, (28, 58), f'{section+1}/4  {label}  ·  源录像 {start}s 起，1 秒窗口循环测试', 22, MUTED)
                if row:
                    frame = Image.fromarray(cache[int(row['cache_index'])]).resize((960, 720), BILINEAR)
                    im.paste(frame, (24, 100))
                    d = ImageDraw.Draw(im)
                    if row['green_valid']:
                        color = YELLOW if row['green_predicted'] else GREEN
                        cross(d, (24+2*row['green_x'], 100+2*row['green_y']), color)
                        x, y = 24+2*row['green_x'], 100+2*row['green_y']
                        radius = max(9, row['green_size'])
                        d.ellipse((x-radius, y-radius, x+radius, y+radius), outline=color, width=2)
                    if row['model_calls']:
                        x, y = 24+2*row['roi_x'], 100+2*row['roi_y']
                        d.rectangle((x, y, x+2*row['roi_w'], y+2*row['roi_h']), outline=CYAN, width=2)
                        if row['model_valid']:
                            cross(d, (24+2*row['model_x'], 100+2*row['model_y']), PINK, 16)
                    text(d, (30, 824), f"结果序号 {int(row['sequence'])}   结果帧延迟 {max(0,t-row['sequence']/180)*1000:.0f} ms", 22)
                else:
                    text(d, (240, 430), '等待第一帧识别完成', 30, MUTED)
                x = 1010
                text(d, (x, 102), '模拟输入 180 FPS  |  V4 / NPU1', 25, CYAN)
                text(d, (x, 140), 'RGB 480×360 → 模型 ROI 320×320', 22, MUTED)
                text(d, (x, 183), '本场景稳态实测（3–19.5 秒）', 22)
                text(d, (x, 222), f"处理 {summary['processed_fps']:.1f} FPS", 40)
                text(d, (x, 280), f"模型 {summary['model_fps']:.1f} 次/秒", 40, PINK)
                text(d, (x, 335), f"传统检测 {summary['classical_fps']:.1f} 次/秒", 25, GREEN)
                count = lambda values: bisect.bisect_right(values,t)-bisect.bisect_right(values,max(0,t-1))
                if t>=1:
                    text(d, (x, 379), f'近1秒：处理 {count(ends)} / 模型 {count(model_ends)} 次', 22)
                else:
                    text(d, (x, 379), '近1秒：统计预热中', 22, MUTED)
                if recent_model:
                    r = recent_model
                    outcome = '中心点通过' if r['model_valid'] else '未通过'
                    text(d, (x, 423), f"最近模型：{outcome}  /  {r['model_ms']:.1f} ms", 25, PINK if r['model_valid'] else YELLOW)
                    text(d, (x, 462), f"该结果距今 {(t-r['end_ms']/1000)*1000:.0f} ms  |  分数 {r['model_score']:.2f}", 19, MUTED)
                    a, b, w, h = (int(r[k]) for k in ('roi_x','roi_y','roi_w','roi_h'))
                    crop = Image.fromarray(cache[int(r['cache_index'])]).crop((a,b,a+w,b+h))
                    side = 264
                    crop = crop.resize((side,side), BILINEAR)
                    im.paste(crop, (x, 504)); d = ImageDraw.Draw(im)
                    d.rectangle((x,504,x+side,504+side),outline=CYAN,width=2)
                    if r['model_valid']:
                        cross(d,(x+(r['model_x']-a)*side/w,504+(r['model_y']-b)*side/h),PINK)
                    text(d, (x+282, 525), '模型输入区域', 22, CYAN)
                    text(d, (x+282, 562), '保留其原帧', 22, MUTED)
                    text(d, (x+282, 599), '不移贴到新帧', 22, MUTED)
                else:
                    text(d, (x, 440), '尚无模型调用 / 等待候选区域', 25, MUTED)
                text(d, (x, 786), '绿：传统观测  黄：预测  粉：模型中心', 19)
                text(d, (28, 859), f'测试时间 {t:05.2f}s / {duration:.0f}s  |  按完成时间显示，等待时保持上一结果帧', 19, MUTED)
                text(d, (1010, 859), '导出30 FPS · 非实时相机采集', 19, MUTED)
                ff.stdin.write(im.tobytes())
                if section==1 and f>=FPS*5 and row and row['model_valid'] and not preview_saved:
                    im.save(args.output.with_suffix('.jpg'), quality=93)
                    preview_saved = True
                frame_count += 1
            audit.append({'label':label, 'source_start':float(start), 'output_start_seconds':section*duration,
                          'summary':summary, 'rows':len(rows)})
            print('rendered', label, flush=True)
        ff.stdin.close()
        if ff.wait()!=0:
            raise RuntimeError('ffmpeg encoding failed')
    except BaseException:
        ff.kill(); ff.wait(); raise
    args.output.with_suffix('.json').write_text(json.dumps({'output_frames':frame_count, 'output_fps':FPS,
        'display':'latest completed result on its own source frame; no future detections',
        'model_preview':'latest actual model ROI on its own source frame', 'sections':audit},ensure_ascii=False,indent=2)+'\n')
    print(args.output)

if __name__ == '__main__':
    main()
