#!/usr/bin/env python3
"""Offline green-center replay using the supplied compiled AXMODEL and C++ core."""
import argparse
import configparser
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import time

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


class Core:
    def __init__(self, library, fps, maximum_component_aspect=8.):
        self.library = Path(library).resolve()
        self.dll = ctypes.CDLL(str(self.library))
        self.dll.dg_error.restype = ctypes.c_char_p
        self.maximum_component_aspect = maximum_component_aspect
        self.dll.dg_proposals.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p, ctypes.c_int, ctypes.c_float]
        self.dll.dg_create.argtypes = [ctypes.c_uint64]; self.dll.dg_create.restype = ctypes.c_void_p
        self.dll.dg_destroy.argtypes = [ctypes.c_void_p]
        self.dll.dg_update.argtypes = [ctypes.c_void_p, ctypes.c_uint64, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
        self.tracker = self.dll.dg_create(round(1e6 / fps))
        if not self.tracker: raise RuntimeError(self.dll.dg_error().decode())

    def proposals(self, rgb):
        rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
        result = np.zeros((5, 4), np.float32)
        count = self.dll.dg_proposals(rgb.ctypes.data, rgb.shape[1], rgb.shape[0], result.ctypes.data, 5, self.maximum_component_aspect)
        if count < 0: raise RuntimeError(self.dll.dg_error().decode())
        return result[:count].copy()

    def board_rgb(self, rgb):
        """Diagnostic synthetic NV21 roundtrip using upstream conversion."""
        rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError('expected RGB image')
        output = np.empty_like(rgb)
        self.dll.dg_board_rgb.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p]
        code = self.dll.dg_board_rgb(rgb.ctypes.data, rgb.shape[1], rgb.shape[0], output.ctypes.data)
        if code: raise RuntimeError(self.dll.dg_error().decode())
        return output

    def update(self, ts, candidates, confirmations):
        if isinstance(ts, bool) or not isinstance(ts, int) or not 0 <= ts < 2**64:
            raise ValueError('timestamp_us must be a nonnegative integer')
        candidates = np.ascontiguousarray(candidates, np.float32).reshape(-1, 4)
        confirmations = np.ascontiguousarray(confirmations, np.float32).reshape(-1, 2)
        if len(candidates) != len(confirmations): raise ValueError('confirmation count mismatch')
        result = np.zeros(14, np.float32)
        code = self.dll.dg_update(self.tracker, ts, candidates.ctypes.data, confirmations.ctypes.data,
                                 len(candidates), result.ctypes.data)
        if code: raise RuntimeError(self.dll.dg_error().decode())
        valid = bool(result[0]); selected = int(result[12])
        return dict(valid=valid, model_confirmed=bool(result[1]), confirmation_hits=int(result[2]),
                    track_generation=int(result[3]), raw_center_px=result[4:6].tolist() if selected >= 0 else None,
                    diameter_px=float(result[6]), smooth_valid=bool(result[7]),
                    smooth_center_px=result[8:10].tolist() if result[7] else None,
                    state=('LOST', 'ACQUIRING', 'TRACKING')[int(result[10])],
                    invalid_reason=('valid', 'nonincreasing timestamp', 'no pixel candidate',
                                    'model rejected candidate', 'awaiting consecutive confirmations')[int(result[11])],
                    selected_candidate=selected, confidence=float(result[13]), predicted=False,
                    safe_for_control=False)

    def close(self):
        if self.tracker: self.dll.dg_destroy(self.tracker); self.tracker = None


def prepare(rgb, candidate, input_size):
    """Mirror source_roi, step-2 sampling, and explicit upstream letterbox."""
    x, y, diameter, _ = candidate
    size = (int(np.clip(diameter * 8, 64, 384)) + 1) // 2 * 2
    height, width = rgb.shape[:2]; rw, rh = min(size, width), min(size, height)
    left = int(np.clip(math.floor(float(x) - rw // 2), 0, width - rw))
    top = int(np.clip(math.floor(float(y) - rh // 2), 0, height - rh))
    step = 2 if rw >= 256 else 1
    crop = rgb[top:top + rh, left:left + rw][::step, ::step]
    gain = min(input_size / crop.shape[1], input_size / crop.shape[0])
    w, h = round(crop.shape[1] * gain), round(crop.shape[0] * gain)
    pad_x = round((input_size - w) / 2 - .1); pad_y = round((input_size - h) / 2 - .1)
    image = np.full((input_size, input_size, 3), 114, np.uint8)
    image[pad_y:pad_y + h, pad_x:pad_x + w] = cv2.resize(crop, (w, h), interpolation=cv2.INTER_LINEAR)
    return image, dict(left=left, top=top, step=step, gain=gain, pad_x=pad_x, pad_y=pad_y,
                      crop_width=crop.shape[1], crop_height=crop.shape[0])


def sigmoid(values):
    return 1. / (1. + np.exp(-np.clip(values, -80, 80)))


def decode(outputs, info, transform, confidence=.5, keypoint_threshold=.35):
    """MaixCDK YOLO11 NHWC DFL/NMS/pose equations; consume keypoint 0 only."""
    proposals = []; offset = 0; maximum_score = 0.
    pose_info = next(x for x in info['outputs'] if len(x['shape']) == 3)
    keypoints = outputs[pose_info['name']].reshape(pose_info['shape'])[0]
    for stride in (8, 16, 32):
        layer = outputs[f'maix_det_stride_{stride}'].reshape(1, -1, 65)[0]
        grid = info['inputs'][0]['shape'][1] // stride
        scores = sigmoid(layer[:, 64])
        maximum_score = max(maximum_score, float(scores.max()))
        for anchor in np.flatnonzero(scores > confidence):
            logits = layer[anchor, :64].reshape(4, 16)
            probability = np.exp(logits - logits.max(1, keepdims=True))
            probability /= probability.sum(1, keepdims=True)
            distances = probability @ np.arange(16)
            ax, ay = int(anchor % grid), int(anchor // grid)
            box = np.array([ax + .5 - distances[0], ay + .5 - distances[1],
                            ax + .5 + distances[2], ay + .5 + distances[3]]) * stride
            proposals.append((float(scores[anchor]), box, offset + int(anchor), ax, ay, stride))
        offset += len(layer)
    proposals.sort(key=lambda p: p[0], reverse=True)
    kept = []
    for proposal in proposals:
        box = proposal[1]
        def iou(other):
            lo = np.maximum(box[:2], other[:2]); hi = np.minimum(box[2:], other[2:])
            intersection = float(np.maximum(hi - lo, 0).prod())
            total = float(np.maximum(box[2:] - box[:2], 0).prod() + np.maximum(other[2:] - other[:2], 0).prod())
            return intersection / max(total - intersection, 1e-12)
        if not any(iou(p[1]) > .45 for p in kept): kept.append(proposal)
    if not kept: return dict(valid=False, reason='no model detection', confidence=0., maximum_class_score=maximum_score)
    score, _, index, ax, ay, stride = kept[0]
    kp = keypoints[index, :3]; kp_score = float(sigmoid(kp[2]))
    if kp_score <= keypoint_threshold:
        return dict(valid=False, reason='green keypoint confidence', confidence=score, keypoint_confidence=kp_score,
                    maximum_class_score=maximum_score)
    # SDK stores integer keypoint coordinates before source coordinate correction.
    px, py = int((kp[0] * 2 + ax) * stride), int((kp[1] * 2 + ay) * stride)
    local = [(px - transform['pad_x']) / transform['gain'], (py - transform['pad_y']) / transform['gain']]
    valid = 0 <= local[0] < transform['crop_width'] and 0 <= local[1] < transform['crop_height']
    center = [transform['left'] + local[0] * transform['step'], transform['top'] + local[1] * transform['step']]
    return dict(valid=valid, reason='valid' if valid else 'keypoint outside crop', center_px=center,
                confidence=score, keypoint_confidence=kp_score, maximum_class_score=maximum_score)


def inspect_model(toolchain, model):
    env = os.environ.copy(); env['PYTHONHOME'] = str(toolchain / 'python3'); env['PYTHONPATH'] = str(toolchain / 'pulsar2')
    command = [str(toolchain / 'lib/ld-linux-x86-64.so.2'), '--library-path', str(toolchain / 'lib'),
               str(toolchain / 'python3/bin/python3'), str(HERE / 'inspect_model.py'), str(model)]
    info = json.loads(subprocess.check_output(command, env=env, text=True))
    shape = info['inputs'][0]['shape']
    if len(info['inputs']) != 1 or info['inputs'][0]['dtype'] != 2 or shape != [1, 320, 320, 3]:
        raise ValueError('expected compiled uint8 NHWC RGB 320x320 input; do not normalize twice')
    expected = {f'maix_det_stride_{s}': [1, 320 // s, 320 // s, 65] for s in (8, 16, 32)}
    actual = {v['name']: v['shape'] for v in info['outputs']}
    if len(actual) != 4 or any(actual.get(k) != v for k, v in expected.items()) or not any(v == [1, 2100, 15] for v in actual.values()):
        raise ValueError('unsupported YOLO11 pose output contract')
    if any(v['dtype'] != 1 for v in info['outputs']): raise ValueError('expected float32 outputs')
    return info


def index_inference(output, bundle_hashes, info, version):
    """Freeze input/output hashes after a real simulator invocation."""
    output = Path(output)
    entries = {}
    for job in json.loads((output / 'jobs.json').read_text()):
        input_path = output / 'model_inputs' / job['name'] / 'images.bin'
        tensors = {}
        for tensor in info['outputs']:
            filename = re.sub(r'[^a-zA-Z0-9_.-]', '_', tensor['name']) + '.bin'
            path = output / 'model_outputs' / job['name'] / filename
            if not path.is_file() or path.stat().st_size != math.prod(tensor['shape']) * 4:
                raise ValueError('cannot index incomplete inference')
            tensors[tensor['name']] = dict(path=str(path.relative_to(output)), sha256=sha(path))
        entries[sha(input_path)] = tensors
    result = dict(model_sha256=bundle_hashes, toolchain=version, model_io=info, entries=entries)
    save(output / 'inference_manifest.json', result)
    return result


def run(manifest, mud, toolchain, library, configuration, output, limit=None, expect_target='unspecified', reuse_inference=None):
    manifest, mud, toolchain, library, configuration, output = map(lambda p: Path(p).resolve(),
        (manifest, mud, toolchain, library, configuration, output))
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic(); core = None
    try:
        cfg = json.loads(configuration.read_text())
        bundle = configparser.ConfigParser(); bundle.read(mud)
        if bundle.get('extra', 'model_type') != 'yolo11' or bundle.get('extra', 'type') != 'pose' or bundle.get('extra', 'labels') != 'dart_target' or bundle.get('extra', 'input_type') != 'rgb':
            raise ValueError('expected single-class dart_target YOLO11 pose RGB bundle')
        model = mud.parent / bundle.get('basic', 'model_npu')
        bundle_hashes = {str(p): sha(p) for p in (mud, model, mud.parent / bundle.get('basic', 'model_vnpu'))}
        save(output / 'model_bundle.json', bundle_hashes)
        info = inspect_model(toolchain, model); save(output / 'model_io.json', info)
        version = subprocess.check_output([str(toolchain / 'bin/pulsar2'), 'version'], text=True).strip()
        if 'version: 6.0' not in version: raise ValueError('Pulsar2 6.0 required')
        source_files = sorted(set([*ROOT.glob('main/include/dart/*.hpp'), *ROOT.glob('main/src/*.cpp'),
                                  *HERE.glob('*.py'), HERE / 'drone_replay.json', HERE / 'CMakeLists.txt',
                                  HERE / 'drone_core.cpp', ROOT / 'tools/video_replay/maix_image.hpp',
                                  ROOT / 'tools/video_replay/maix_image_opencv.cpp']))
        snapshot = output / 'source_snapshot'; snapshot.mkdir()
        source_hashes = {}
        for source_file in source_files:
            digest = sha(source_file); source_hashes[str(source_file.relative_to(ROOT))] = digest
            (snapshot / (digest + '-' + source_file.name)).write_bytes(source_file.read_bytes())
        save(output / 'source_hashes.json', source_hashes)
        inputs = json.loads(manifest.read_text()); rows = inputs['frames'][:limit]
        if not rows or inputs['pixel_format'] != 'RGB888': raise ValueError('nonempty RGB manifest required')
        if not math.isfinite(inputs['nominal_fps']) or inputs['nominal_fps'] <= 0: raise ValueError('positive finite fps required')
        core = Core(library, inputs['nominal_fps'], cfg['maximum_component_aspect'])
        input_dir = output / 'model_inputs'; input_dir.mkdir()
        jobs = []; candidates = []
        for row in rows:
            path = manifest.parent / row['image']
            if sha(path) != row['sha256']: raise ValueError('input image hash changed')
            bgr = cv2.imread(str(path));
            if bgr is None: raise ValueError('cannot decode image')
            if bgr.shape[:2] != (inputs['height'], inputs['width']): raise ValueError('image dimensions changed')
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            current = core.proposals(rgb); candidates.append(current)
            for index, candidate in enumerate(current):
                prepared, transform = prepare(rgb, candidate, 320)
                name = f'{len(jobs):06d}'; directory = input_dir / name; directory.mkdir()
                prepared.tofile(directory / 'images.bin')
                jobs.append(dict(name=name, frame_index=len(candidates) - 1, candidate_index=index,
                                 transform=transform, candidate=candidate.tolist()))
        save(output / 'jobs.json', jobs)
        model_outputs = output / 'model_outputs'
        if reuse_inference is not None:
            cached_root = Path(reuse_inference).resolve()
            cached = json.loads((cached_root / 'inference_manifest.json').read_text())
            if cached['model_sha256'] != bundle_hashes or cached['toolchain'] != version or cached['model_io'] != info:
                raise ValueError('recorded inference model or toolchain mismatch')
            for job in jobs:
                digest = sha(input_dir / job['name'] / 'images.bin')
                tensors = cached['entries'].get(digest)
                if tensors is None: raise ValueError('prepared RGB input missing from recorded inference')
                target = model_outputs / job['name']; target.mkdir(parents=True)
                for tensor in info['outputs']:
                    entry = tensors[tensor['name']]; source = cached_root / entry['path']
                    if sha(source) != entry['sha256']: raise ValueError('recorded model output changed')
                    shutil.copy2(source, target / source.name)
            save(output / 'inference_reuse.json', dict(source=str(cached_root), manifest_sha256=sha(cached_root / 'inference_manifest.json')))
        elif jobs:
            listing = output / 'list.txt'; listing.write_text(''.join(j['name'] + '\n' for j in jobs))
            command = [str(toolchain / 'bin/pulsar2'), 'run', '--model', str(model), '--input_dir', str(input_dir),
                       '--output_dir', str(model_outputs), '--list', str(listing)]
            save(output / 'model_command.json', command)
            with (output / 'pulsar2.log').open('w') as log:
                subprocess.run(command, cwd=output, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=cfg['model_timeout_s'])
        if jobs: index_inference(output, bundle_hashes, info, version)
        confirmed = [np.zeros((len(c), 2), np.float32) for c in candidates]; model_results = [[] for _ in rows]
        for job in jobs:
            arrays = {}
            for tensor in info['outputs']:
                filename = re.sub(r'[^a-zA-Z0-9_.-]', '_', tensor['name']) + '.bin'
                values = np.fromfile(model_outputs / job['name'] / filename, dtype=np.float32)
                if values.size != math.prod(tensor['shape']) or not np.isfinite(values).all(): raise ValueError('invalid model tensor')
                arrays[tensor['name']] = values
            result = decode(arrays, info, job['transform'], cfg['model_confidence'], cfg['keypoint_confidence'])
            candidate = job['candidate']
            associated = result['valid'] and math.dist(result['center_px'], candidate[:2]) <= max(12., candidate[2])
            result.update(candidate_index=job['candidate_index'], associated=bool(associated))
            confirmed[job['frame_index']][job['candidate_index']] = [associated, result['confidence']]
            model_results[job['frame_index']].append(result)
        writer = cv2.VideoWriter(str(output / 'annotated.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), inputs['nominal_fps'], (inputs['width'], inputs['height']))
        if not writer.isOpened(): raise RuntimeError('video writer unavailable')
        estimates = []
        try:
            with (output / 'results.jsonl').open('w') as stream:
                for index, row in enumerate(rows):
                    estimate = core.update(row['timestamp_us'], candidates[index], confirmed[index])
                    estimate.update(sequence=row['sequence'], timestamp_us=row['timestamp_us'],
                                    candidates=candidates[index].tolist(), model_results=model_results[index],
                                    model_ran=bool(model_results[index]))
                    estimates.append(estimate); stream.write(json.dumps(estimate, allow_nan=False) + '\n')
                    frame = cv2.imread(str(manifest.parent / row['image']))
                    if estimate['valid']:
                        center = tuple(round(v) for v in estimate['smooth_center_px'])
                        cv2.drawMarker(frame, center, (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                    for candidate in candidates[index]: cv2.circle(frame, tuple(round(v) for v in candidate[:2]), 4, (0, 165, 255), 1)
                    cv2.putText(frame, f"{estimate['state']} MODEL {int(estimate['model_confirmed'])} t={row['timestamp_us']/1e6:.3f}", (12, 25), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1)
                    if inputs.get('fixture'):
                        cv2.putText(frame, 'SYNTHETIC FIXTURE: ' + inputs['fixture']['mode'], (12, 48),
                                    cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 200, 255), 1)
                    writer.write(frame)
        finally: writer.release()
        observed = sum(r['valid'] for r in estimates)
        quality = (observed >= 2 if expect_target == 'present' else observed == 0 if expect_target == 'absent' else None)
        pipeline_passed = bool(jobs) or expect_target == 'absent'
        summary = dict(passed=pipeline_passed and quality is not False, pipeline_passed=pipeline_passed,
                       model_inference_validated=bool(jobs), tracking_check_passed=quality,
                       expectation=expect_target,
                       run_type='recorded AXMODEL tensor replay' if reuse_inference else 'offline compiled AXMODEL RGB replay', frames=len(rows),
                       model_inferences=len(jobs), fresh_model_inferences=0 if reuse_inference else len(jobs),
                       recorded_model_inferences_used=len(jobs) if reuse_inference else 0,
                       model_positive=sum(bool(v[0] > 0) for frame in confirmed for v in frame),
                       observed_frames=observed, valid_fraction=observed / len(rows),
                       wall_time_s=time.monotonic() - started, toolchain=version,
                       input_manifest_sha256=sha(manifest), model_sha256=bundle_hashes, library_sha256=sha(library),
                       configuration=cfg, configuration_sha256=sha(configuration),
                       source_commit=subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
                       training_supervision_verified=False, pose_used=False, truth_in_detector=False,
                       hardware_performance_validated=False, flight_control=False)
        summary['input_fixture'] = inputs.get('fixture')
        save(output / 'summary.json', summary); print(json.dumps(summary)); return summary
    except Exception as exc:
        save(output / 'summary.json', dict(passed=False, pipeline_passed=False,
             run_type='offline compiled AXMODEL RGB replay', error=f'{type(exc).__name__}: {exc}',
             wall_time_s=time.monotonic() - started))
        raise
    finally:
        if core is not None: core.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', type=Path, required=True); p.add_argument('--mud', type=Path, required=True)
    p.add_argument('--toolchain', type=Path, required=True); p.add_argument('--library', type=Path, required=True)
    p.add_argument('--config', type=Path, default=HERE / 'drone_replay.json'); p.add_argument('--output', type=Path, required=True)
    p.add_argument('--limit', type=int); p.add_argument('--expect-target', choices=['present', 'absent', 'unspecified'], default='unspecified')
    p.add_argument('--reuse-inference', type=Path, help='reuse hashed tensors for identical model and prepared RGB bytes')
    a = p.parse_args()
    if a.limit is not None and a.limit <= 0: p.error('--limit must be positive')
    result = run(a.manifest, a.mud, a.toolchain, a.library, a.config, a.output, a.limit, a.expect_target, a.reuse_inference)
    return 0 if result['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
