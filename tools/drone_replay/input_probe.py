#!/usr/bin/env python3
"""Fixed paired RGB/NV21 probe; never select thresholds or flight parameters."""
import argparse
import json
import math
from pathlib import Path
import re
import shutil
import subprocess

import cv2
import numpy as np

from replay import Core, HERE, ROOT, decode, index_inference, inspect_model, prepare, save, sha


def tensors(directory, info, expected=None):
    result = {}
    for item in info['outputs']:
        name = item['name']
        path = directory / (re.sub(r'[^a-zA-Z0-9_.-]', '_', name) + '.bin')
        if expected is not None and sha(path) != expected[name]['sha256']:
            raise ValueError('baseline tensor hash changed')
        values = np.fromfile(path, np.float32)
        if values.size != math.prod(item['shape']) or not np.isfinite(values).all():
            raise ValueError('invalid output tensor')
        result[name] = values
    return result


def run(args):
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=False)
    core = None
    try:
        baseline = args.baseline.resolve(); manifest = args.manifest.resolve()
        cached = json.loads((baseline / 'inference_manifest.json').read_text())
        bundle = cached['model_sha256']
        for path, digest in bundle.items():
            if sha(path) != digest: raise ValueError('model bundle changed')
        model = next(Path(p) for p in bundle if p.endswith('_npu.axmodel'))
        info = inspect_model(args.toolchain.resolve(), model)
        version = subprocess.check_output([str(args.toolchain.resolve() / 'bin/pulsar2'), 'version'], text=True).strip()
        if cached['model_io'] != info or cached['toolchain'] != version or cached['model_sha256'] != bundle:
            raise ValueError('model/toolchain contract mismatch')
        source = json.loads(manifest.read_text())
        if source['pixel_format'] != 'RGB888': raise ValueError('RGB input required')
        cfg = json.loads(args.configuration.read_text())
        core = Core(args.library.resolve(), source['nominal_fps'], cfg['maximum_component_aspect'])
        old_jobs = json.loads((baseline / 'jobs.json').read_text())
        selected = []
        for frame in args.frames:
            jobs = [j for j in old_jobs if j['frame_index'] == frame]
            if len(jobs) != 1: raise ValueError('probe requires exactly one recorded candidate per frame')
            selected.append(jobs[0])
        # Freeze the bounded cases and unchanged decision rules before inference.
        save(output / 'registration.json', dict(frames=args.frames, jobs=selected, configuration=cfg,
            manifest=str(manifest), manifest_sha256=sha(manifest), baseline=str(baseline),
            baseline_index_sha256=sha(baseline / 'inference_manifest.json'), library_sha256=sha(args.library),
            experiment='fixed candidate/ROI; original RGB vs synthetic NV21 then original C++ RGB conversion',
            sensor_pixel_equivalence_verified=False, threshold_search=False, safe_for_control=False))
        save(output / 'model_io.json', info); save(output / 'model_bundle.json', bundle)
        shutil.copy2(args.configuration, output / 'configuration.json')
        snapshot = output / 'source_snapshot'; snapshot.mkdir()
        hashes = {}
        for path in [HERE / 'input_probe.py', HERE / 'replay.py', HERE / 'drone_core.cpp',
                     ROOT / 'main/src/nv21_pipeline.cpp']:
            hashes[str(path.relative_to(ROOT))] = sha(path)
            shutil.copy2(path, snapshot / path.name)
        save(output / 'source_hashes.json', hashes)
        jobs = []; pairs = []; montage = []
        for index, old in enumerate(selected):
            row = source['frames'][old['frame_index']]; path = manifest.parent / row['image']
            if sha(path) != row['sha256']: raise ValueError('image hash changed')
            bgr = cv2.imread(str(path))
            if bgr is None or bgr.shape != (source['height'], source['width'], 3):
                raise ValueError('invalid source image')
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            prepared, transform = prepare(rgb, old['candidate'], 320)
            old_input = baseline / 'model_inputs' / old['name'] / 'images.bin'
            digest = sha(old_input)
            if prepared.tobytes() != old_input.read_bytes() or transform != old['transform']:
                raise ValueError('baseline preprocessing mismatch')
            expected = cached['entries'][digest]
            base = decode(tensors(baseline / 'model_outputs' / old['name'], info, expected),
                          info, transform, cfg['model_confidence'], cfg['keypoint_confidence'])
            variant, vtransform = prepare(core.board_rgb(rgb), old['candidate'], 320)
            if transform != vtransform: raise ValueError('crop transform changed')
            name = f'{index:06d}'; directory = output / 'model_inputs' / name; directory.mkdir(parents=True)
            variant.tofile(directory / 'images.bin')
            jobs.append(dict(name=name, frame_index=old['frame_index'], candidate_index=0,
                             candidate=old['candidate'], transform=transform))
            difference = np.abs(prepared.astype(np.int16) - variant.astype(np.int16))
            pairs.append(dict(frame_index=old['frame_index'], baseline=base, original_input_sha256=digest,
                              variant_input_sha256=sha(directory / 'images.bin'),
                              mean_absolute_channel_difference=float(difference.mean()),
                              maximum_channel_difference=int(difference.max())))
            panels = []
            for label, image in [('RGB', prepared), ('NV21 roundtrip', variant)]:
                panel = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                cv2.putText(panel, f"Frame {old['frame_index']} {label}", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255), 1)
                panels.append(panel)
            montage.append(np.hstack(panels))
        save(output / 'jobs.json', jobs)
        if not cv2.imwrite(str(output / 'comparison.png'), np.vstack(montage)):
            raise RuntimeError('cannot save comparison')
        listing = output / 'list.txt'; listing.write_text(''.join(j['name'] + '\n' for j in jobs))
        command = [str(args.toolchain.resolve() / 'bin/pulsar2'), 'run', '--model', str(model),
                   '--input_dir', str(output / 'model_inputs'), '--output_dir', str(output / 'model_outputs'),
                   '--list', str(listing)]
        save(output / 'model_command.json', command)
        with (output / 'pulsar2.log').open('w') as log:
            subprocess.run(command, cwd=output, stdout=log, stderr=subprocess.STDOUT, check=True,
                           timeout=cfg['model_timeout_s'])
        index_inference(output, bundle, info, version)
        for pair, job in zip(pairs, jobs):
            pair['variant'] = decode(tensors(output / 'model_outputs' / job['name'], info), info,
                                     job['transform'], cfg['model_confidence'], cfg['keypoint_confidence'])
            for key in ('baseline', 'variant'):
                result = pair[key]
                result['associated'] = bool(result['valid'] and math.dist(result['center_px'], job['candidate'][:2])
                                             <= max(12., job['candidate'][2]))
        save(output / 'per_frame.json', pairs)
        summary = dict(passed=True, diagnostic_completed=True, run_type='headless paired offline AXMODEL inference',
            frames=args.frames, fresh_model_inferences=len(jobs), recorded_model_inferences_used=len(jobs),
            baseline_associated=sum(p['baseline']['associated'] for p in pairs),
            variant_associated=sum(p['variant']['associated'] for p in pairs),
            continuous_tracking_evaluated=False, sensor_pixel_equivalence_verified=False,
            safe_for_control=False, configuration_changed=False)
        save(output / 'summary.json', summary)
        print(json.dumps(summary))
    except Exception as error:
        save(output / 'summary.json', dict(passed=False, error=str(error), safe_for_control=False))
        raise
    finally:
        if core is not None: core.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('baseline', 'manifest', 'toolchain', 'library', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--configuration', type=Path, default=HERE / 'drone_replay.json')
    parser.add_argument('--frames', type=int, nargs='+', default=[8, 26, 30, 39])
    args = parser.parse_args()
    if not args.frames or min(args.frames) < 0 or len(set(args.frames)) != len(args.frames):
        parser.error('unique nonnegative frame indices required')
    run(args)
