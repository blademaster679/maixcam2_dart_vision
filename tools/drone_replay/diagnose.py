"""Compare actual saved model crops and scores without selecting thresholds on them."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess

import cv2
import numpy as np
from PIL import Image, ImageDraw
from replay import ROOT, decode, save, sha

UPSTREAM = 'fd7ff396994f0da69c67573ef4b52c531abc1e6b'
REFERENCES = (
    'tools/model/board_center_replay.py',
    'tools/model/board_center_validator.cpp',
    'tools/model/board_async_center_full.cpp',
    'main/src/npu_pose_validator.cpp',
    'config/green_detector.conf',
)


def diagnose(run, output):
    run, output = Path(run).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    info = json.loads((run / 'model_io.json').read_text())
    index = json.loads((run / 'inference_manifest.json').read_text())
    jobs = json.loads((run / 'jobs.json').read_text())
    if info != index['model_io'] or not jobs:
        raise ValueError('nonempty saved inference with matching model I/O required')
    references = {}
    for name in REFERENCES:
        content = subprocess.check_output(['git', '-C', str(ROOT), 'show', f'{UPSTREAM}:{name}'])
        target = output / 'upstream_reference' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(content)
        references[name] = hashlib.sha256(content).hexdigest()
    rows = []
    for job in jobs:
        input_path = run / 'model_inputs' / job['name'] / 'images.bin'
        entry = index['entries'][sha(input_path)]
        rgb = np.fromfile(input_path, np.uint8).reshape(info['inputs'][0]['shape'][1:])
        tensors = {}
        for tensor in info['outputs']:
            path = run / entry[tensor['name']]['path']
            if sha(path) != entry[tensor['name']]['sha256']: raise ValueError('saved tensor changed')
            array = np.fromfile(path, np.float32)
            if array.size != np.prod(tensor['shape']) or not np.isfinite(array).all():
                raise ValueError('invalid saved tensor')
            tensors[tensor['name']] = array
        predictions = {}
        for name, threshold in [('generic_template', .5), ('upstream_center_probe', .3)]:
            result = decode(tensors, info, job['transform'], threshold, .35)
            result['associated'] = bool(result['valid'] and np.linalg.norm(
                np.asarray(result['center_px']) - job['candidate'][:2]) <= max(12., job['candidate'][2]))
            predictions[name] = result
        filename = job['name'] + '.png'
        if not cv2.imwrite(str(output / filename), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
            raise RuntimeError('crop save failed')
        rows.append(dict(frame_index=job['frame_index'], candidate_index=job['candidate_index'],
                         input_sha256=sha(input_path), image=filename, transform=job['transform'],
                         candidate=job['candidate'], predictions=predictions))
    save(output / 'per_frame.json', rows)
    selected = [i for i in (0, 8, 12, 26, 30, 39) if i < len(rows)]
    canvas = Image.new('RGB', (1020, 760), '#202020'); draw = ImageDraw.Draw(canvas)
    for position, i in enumerate(selected):
        row = rows[i]; x = position % 3 * 340 + 10; y = position // 3 * 380 + 45
        canvas.paste(Image.open(output / row['image']), (x, y))
        p = row['predictions']['upstream_center_probe']
        draw.text((x, y - 32), f"Frame {row['frame_index']}, class max {p['maximum_class_score']:.3f}\nROI {row['transform']['crop_width']} px", fill='white')
    canvas.save(output / 'comparison.png')
    summary = dict(passed=True, scope='diagnostic saved-input comparison; no accuracy acceptance', jobs=len(rows),
        source_inference_manifest_sha256=sha(run / 'inference_manifest.json'), source=str(run),
        thresholds={'generic_template': .5, 'upstream_center_probe': .3},
        comparison={name:dict(associated=sum(r['predictions'][name]['associated'] for r in rows),
                             rejection_reasons=dict(Counter(r['predictions'][name]['reason'] for r in rows)))
                    for name in ('generic_template', 'upstream_center_probe')},
        output_hashes={p.name:sha(p) for p in output.iterdir() if p.is_file()},
        upstream_commit=UPSTREAM, upstream_source_sha256=references,
        source_jobs_sha256=sha(run / 'jobs.json'), source_io_sha256=sha(run / 'model_io.json'),
        preprocessing_audit=dict(
            source_roi='diameter * 8; clamp 64..384; even size; floor/clamp to image',
            sampling='step 2 when ROI width >= 256, otherwise step 1',
            model_input='RGB uint8 NHWC; bilinear letterbox 114; no extra division by 255',
            postprocessing='MaixCDK equations; keypoint 0 only; keypoint threshold 0.35; NMS 0.45',
            agreement='source-rule audit; board and host pixel equivalence unverified'),
        real_device_configuration_verified=False, original_crop_strategy_preserved=True)
    save(output / 'summary.json', summary); print(json.dumps(summary))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); diagnose(a.run, a.output)
