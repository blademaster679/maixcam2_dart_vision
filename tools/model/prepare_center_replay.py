#!/usr/bin/env python3
"""Stage unchanged synthetic test ROIs and real holdout proposal crops for replay."""
import argparse
import json
import shutil
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--handoff', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = args.handoff.resolve() / 'training'
    sys.path.insert(0, str(root / 'green_fusion_v3'))
    from fusion import bounds
    import cv2
    out = args.output.resolve()
    (out / 'images').mkdir(parents=True, exist_ok=True)
    v4 = root / 'green_blender_v4'
    rows = []
    synthetic = json.loads((v4 / 'dataset/manifest.json').read_text())
    for row in synthetic:
        if row['source'] != 'blender_v4' or row['split'] != 'test':
            continue
        name = 'images/synthetic_' + Path(row['image']).name
        shutil.copy2(v4 / row['image'], out / name)
        rows.append(dict(id=len(rows), image=name, domain='synthetic_roi',
                         ground_truth=row, offset=[0, 0]))
    real = [r for r in json.loads((v4 / 'candidates.json').read_text()) if r['split'] == 'test']
    for frame_index, row in enumerate(real):
        frame = cv2.imread(str(root / 'green_v2' / row['image']))
        if frame is None:
            raise ValueError(row['image'])
        for proposal_index, proposal in enumerate(row['proposals']):
            x, y, side = bounds(proposal['center'], proposal['diameter'], frame.shape)
            name = f'images/real_{frame_index:03d}_{proposal_index:03d}.png'
            assert cv2.imwrite(str(out / name), frame[y:y+side, x:x+side])
            rows.append(dict(id=len(rows), image=name, domain='real_roi',
                             frame_index=frame_index, proposal=proposal, offset=[x, y]))
    (out / 'manifest.json').write_text(json.dumps(rows, indent=2))
    (out / 'real_frames.json').write_text(json.dumps(real, indent=2))
    shutil.copy2(Path(__file__).with_name('board_center_replay.py'), out)
    print(json.dumps(dict(images=len(rows), real_frames=len(real), output=str(out))))


if __name__ == '__main__':
    main()
