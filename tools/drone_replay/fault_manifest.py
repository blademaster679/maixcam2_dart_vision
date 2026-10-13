"""Explicit RGB blackout fixtures; never edit the source images or scene."""
import argparse
import json
from pathlib import Path
import shutil

import cv2
import numpy as np
from replay import sha, save


def make(source, output, mode, limit=40, first=10, count=6):
    source, output = Path(source).resolve(), Path(output).resolve()
    original = json.loads(source.read_text())
    if mode in ('blackout', 'blackout_reacquire') and not (0 < first < first + count < limit):
        raise ValueError('blackout needs visible frames before and after it')
    output.mkdir(parents=True, exist_ok=False); (output / 'images').mkdir()
    rows = []
    for index, item in enumerate(original['frames'][:limit]):
        # Explicit synthetic reappearance of the earlier recorded view, not a new flight.
        image_index = (index - first - count) % first if mode == 'blackout_reacquire' and index >= first + count else index
        image_source = original['frames'][image_index]
        path = source.parent / image_source['image']
        if sha(path) != image_source['sha256']: raise ValueError('source image changed')
        blank = mode == 'no_target' or first <= index < first + count
        target = output / f'images/{index:06d}.png'
        if blank:
            if not cv2.imwrite(str(target), np.zeros((original['height'], original['width'], 3), np.uint8)):
                raise RuntimeError('fixture save failed')
        else: shutil.copy2(path, target)
        rows.append(dict(item, image=str(target.relative_to(output)), sha256=sha(target),
                         fixture_image_source_sequence=image_source['sequence']))
    result = dict(original, frames=rows, fixture=dict(mode=mode, first=first, count=count,
        alteration='RGB blackout; prior views repeated after blackout' if mode == 'blackout_reacquire' else 'RGB replaced with black',
        physical_scene_modified=False, synthetic_fixture=True, source_manifest_sha256=sha(source)))
    save(output / 'manifest.json', result)
    print(json.dumps(dict(frames=len(rows), mode=mode, output=str(output))))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    p.add_argument('--mode', choices=['blackout', 'blackout_reacquire', 'no_target'], required=True)
    p.add_argument('--limit', type=int, default=40); p.add_argument('--first', type=int, default=10); p.add_argument('--count', type=int, default=6)
    a = p.parse_args(); make(a.manifest, a.output, a.mode, a.limit, a.first, a.count)
