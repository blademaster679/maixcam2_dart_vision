#!/usr/bin/env python3
"""MaixCAM2 center-only image replay; writes predictions and measured latency."""
import argparse
import json
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--confidence', type=float, default=0.3)
    args = parser.parse_args()
    from maix import image, nn
    detector = nn.YOLO11(model=args.model, dual_buff=False)
    assert detector.input_width() == 320 and detector.input_height() == 320
    assert list(detector.labels) == ['dart_target']
    rows = json.loads(args.manifest.read_text())
    root = args.manifest.parent
    predictions = []
    for index, row in enumerate(rows):
        print('replay', index, row['image'], flush=True)
        frame = image.load(str(root / row['image']), format=image.Format.FMT_RGB888)
        gain = min(320 / frame.width(), 320 / frame.height())
        width, height = round(frame.width() * gain), round(frame.height() * gain)
        left, top = round((320 - width) / 2 - .1), round((320 - height) / 2 - .1)
        resized = frame.resize(width, height, image.Fit.FIT_FILL, image.ResizeMethod.BILINEAR)
        prepared = image.Image(320, 320, image.Format.FMT_RGB888,
                               bg=image.Color.from_rgb(114, 114, 114))
        prepared.draw_image(left, top, resized)
        if index == 0:
            for _ in range(3):
                detector.detect(prepared, conf_th=args.confidence, iou_th=.45,
                                fit=image.Fit.FIT_FILL, keypoint_th=.35)
        started = time.monotonic_ns()
        objects = detector.detect(prepared, conf_th=args.confidence, iou_th=.45,
                                  fit=image.Fit.FIT_FILL, keypoint_th=.35)
        latency_ms = (time.monotonic_ns() - started) / 1e6
        centers = []
        for obj in objects:
            points = list(obj.points)
            if len(points) != 10:
                raise RuntimeError('Expected five keypoints in converted output')
            if points[0] >= 0 and points[1] >= 0:
                centers.append(dict(center=[(points[0] - left) / gain,
                                             (points[1] - top) / gain], score=float(obj.score)))
        predictions.append(dict(id=row.get('id', index), image=row['image'],
                                predictions=centers, inference_ms=latency_ms))
    latencies = sorted(r['inference_ms'] for r in predictions)
    report = dict(model=args.model, confidence=args.confidence,
                  supervised_keypoints=[0], images=len(rows), predictions=predictions,
                  inference_p50_ms=latencies[len(latencies) // 2] if latencies else None,
                  inference_p95_ms=latencies[min(len(latencies)-1, int(len(latencies)*.95))]
                  if latencies else None,
                  note='Offline RGB replay; timing covers NPU forward and postprocessing, '
                       'excludes image load and explicit resize/padding, and is not camera FPS')
    args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != 'predictions'}))


if __name__ == '__main__':
    main()
