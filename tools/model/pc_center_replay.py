#!/usr/bin/env python3
"""Floating-point reference on the exact images staged for MaixCAM2 replay."""
import argparse
import json
import os
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', required=True)
    parser.add_argument('--replay', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--device', default='cpu')
    args = parser.parse_args()
    os.environ.setdefault('YOLO_CONFIG_DIR', str(args.replay.resolve() / '.ultralytics'))
    os.environ.setdefault('MPLCONFIGDIR', str(args.replay.resolve() / '.matplotlib'))
    import torch
    from ultralytics import YOLO
    torch.set_num_threads(4)
    rows = json.loads((args.replay / 'manifest.json').read_text())
    model = YOLO(args.weights)
    results = model.predict([str(args.replay / row['image']) for row in rows],
                            imgsz=320, conf=.1, iou=.45, device=args.device,
                            batch=16, verbose=False, stream=True)
    predictions = []
    for index, result in enumerate(results):
        row = rows[index]
        centers = [dict(center=result.keypoints.xy[j, 0].cpu().tolist(),
                        score=float(result.boxes.conf[j]))
                   for j in range(len(result.boxes)) if float(result.keypoints.conf[j, 0]) >= .35]
        predictions.append(dict(id=row['id'], image=row['image'], predictions=centers))
    assert len(predictions) == len(rows)
    args.output.write_text(json.dumps(dict(model=args.weights, confidence=.1,
                                          predictions=predictions), indent=2))
    print(f'Wrote {len(predictions)} floating-point reference predictions')


if __name__ == '__main__':
    main()
