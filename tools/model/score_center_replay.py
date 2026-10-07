#!/usr/bin/env python3
"""Apply the handoff's fixed center tolerance and association to board replay."""
import argparse
import json
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--handoff', type=Path, required=True)
    parser.add_argument('--replay', type=Path, required=True)
    parser.add_argument('--predictions', type=Path, required=True)
    parser.add_argument('--threshold', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(args.handoff.resolve() / 'training/green_fusion_v3'))
    from experiment import metrics
    from fusion import combine_evidence
    import numpy as np
    manifest = json.loads((args.replay / 'manifest.json').read_text())
    real = json.loads((args.replay / 'real_frames.json').read_text())
    report = json.loads(args.predictions.read_text())
    predictions = report['predictions']
    assert len(predictions) == len(manifest)
    assert report['confidence'] <= args.threshold
    syn_rows, syn_predictions = [], []
    real_predictions = [[] for _ in real]
    for row, result in zip(manifest, predictions):
        assert row['id'] == result['id'] and row['image'] == result['image']
        if row['domain'] == 'synthetic_roi':
            gt = row['ground_truth']
            syn_rows.append(dict(**gt, present=not gt['negative'], box=gt['bbox']))
            syn_predictions.append([p for p in result['predictions'] if p['score'] >= args.threshold])
        else:
            proposal = row['proposal']
            x, y = row['offset']
            # Same bounds and association as the original offline fusion.
            import cv2
            frame = cv2.imread(str(args.replay / row['image']))
            height, width = frame.shape[:2]
            for pred in result['predictions']:
                local = np.asarray(pred['center'], dtype=float)
                center = local + [x, y]
                if not np.isfinite(center).all() or not (0 <= local[0] < width and 0 <= local[1] < height):
                    continue
                if np.linalg.norm(center - proposal['center']) > max(6, 1.5 * proposal['diameter']):
                    continue
                real_predictions[row['frame_index']].append(dict(
                    center=center.tolist(), score=pred['score'], source=proposal['source'],
                    classical_score=proposal['classical_score']))
    unique = []
    for group in real_predictions:
        kept = []
        for pred in sorted(group, key=lambda p: p['score'], reverse=True):
            if not any(np.linalg.norm(np.subtract(pred['center'], q['center'])) < 4 for q in kept):
                kept.append(pred)
        unique.append(kept)
    result = dict(threshold=args.threshold, synthetic_roi_test=metrics(syn_rows, syn_predictions),
                  real_pipeline_test=metrics(real, [[p for p in ps if p['score'] >= args.threshold] for ps in unique]),
                  complementary_test=metrics(real, [combine_evidence(r['proposals'], ps, args.threshold)
                                                     for r, ps in zip(real, unique)]),
                  inference_p50_ms=report.get('inference_p50_ms'),
                  inference_p95_ms=report.get('inference_p95_ms'),
                  limitations=['Previously inspected single video, not independent blind testing.',
                               'Real proposals are replayed from PC; this is not live camera end-to-end accuracy.',
                               'Integer board centers and board resize can differ from floating-point PC decoding.'])
    args.output.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
