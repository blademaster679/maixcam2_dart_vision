"""Behavior checks for shared C++ tracking and compiled-output interpretation."""
import os
from pathlib import Path

import numpy as np
import pytest
from replay import Core, decode, prepare


@pytest.fixture
def core():
    library = os.environ.get('DRONE_CORE_LIBRARY')
    if not library: pytest.fail('build core and set DRONE_CORE_LIBRARY')
    result = Core(Path(library), 20.)
    yield result
    result.close()


def candidate(x=100., y=100.):
    return np.array([[x, y, 8., 24.]], np.float32)


def positive(): return np.array([[1., .8]], np.float32)
def negative(): return np.array([[0., .2]], np.float32)


def test_confirmation_at_20_hz_and_duplicate_zero_clock(core):
    assert not core.update(0, candidate(), positive())['valid']
    assert not core.update(0, candidate(), positive())['valid']
    assert not core.update(50000, candidate(), positive())['valid']
    assert core.update(100000, candidate(102), positive())['valid']
    assert not core.update(90000, candidate(), positive())['valid']


def test_same_track_confirmation_and_negative_expiry(core):
    core.update(1000000, candidate(), positive())
    assert core.update(1050000, candidate(), positive())['valid']
    result = core.update(1100000, candidate(), negative())
    assert result['valid'] and not result['model_confirmed']
    assert not core.update(1150000, candidate(), negative())['valid']


def test_rejected_measurement_retains_raw_pixels_for_diagnosis(core):
    result = core.update(1000000, candidate(), negative())
    assert not result['valid'] and result['raw_center_px'] == [100., 100.]
    with pytest.raises(ValueError): core.update(2**64, candidate(), positive())


def test_missing_pixels_never_become_prediction(core):
    core.update(1000000, candidate(), positive()); core.update(1050000, candidate(), positive())
    r = core.update(1100000, np.empty((0, 4)), np.empty((0, 2)))
    assert not r['valid'] and not r['predicted'] and r['raw_center_px'] is None
    assert not core.update(1150000, candidate(), positive())['valid']
    assert core.update(1200000, candidate(), positive())['valid']


def test_large_gap_or_target_jump_requires_new_confirmation(core):
    core.update(1000000, candidate(), positive()); core.update(1050000, candidate(), positive())
    assert not core.update(1100000, candidate(500), positive())['valid']
    assert core.update(1150000, candidate(500), positive())['valid']
    assert not core.update(1400000, candidate(500), positive())['valid']


def test_blue_patch_does_not_propose_and_green_uses_full_frame(core):
    image = np.zeros((240, 320, 3), np.uint8)
    image[20:80, 20:90] = [10, 80, 245]
    assert len(core.proposals(image)) == 0
    image[180:188, 270:278] = [0, 255, 0]
    candidates = core.proposals(image)
    assert len(candidates) == 1 and np.linalg.norm(candidates[0, :2] - [273.5, 183.5]) < 2


def test_preprocessing_preserves_rgb_bytes_without_normalization():
    image = np.full((240, 320, 3), [210, 60, 15], np.uint8)
    prepared, transform = prepare(image, candidate(100, 100)[0], 320)
    assert prepared.dtype == np.uint8 and prepared.shape == (320, 320, 3)
    assert np.array_equal(prepared[160, 160], [210, 60, 15])
    assert transform['gain'] == 5


def test_board_conversion_preserves_rgb_order_and_rejects_odd_shape(core):
    # Uniform saturated channels distinguish RGB from BGR; chroma sampling
    # also changes a one-pixel edge, so this is deliberately not a no-op.
    image = np.zeros((12, 16, 3), np.uint8)
    image[:, :8] = [255, 0, 0]; image[:, 8:] = [0, 255, 0]
    converted = core.board_rgb(image)
    assert np.linalg.norm(converted[4, 4].astype(float) - [255, 0, 0]) < 3
    assert np.linalg.norm(converted[4, 12].astype(float) - [0, 255, 0]) < 3
    image[4, 5] = [0, 0, 255]
    converted = core.board_rgb(image)
    assert not np.array_equal(converted[4, 5], image[4, 5])
    with pytest.raises(RuntimeError): core.board_rgb(image[:11])


def test_maix_pose_equations_and_ignored_armor_keypoints():
    info = dict(inputs=[dict(shape=[1, 320, 320, 3])], outputs=[])
    outputs = {}
    for stride in (8, 16, 32):
        name = f'maix_det_stride_{stride}'; shape = [1, 320 // stride, 320 // stride, 65]
        info['outputs'].append(dict(name=name, shape=shape))
        outputs[name] = np.zeros(shape, np.float32); outputs[name][..., 64] = -20
    outputs['maix_det_stride_8'][0, 20, 17, 64] = 8
    name = 'pose'; info['outputs'].append(dict(name=name, shape=[1, 2100, 15]))
    outputs[name] = np.full((1, 2100, 15), 999., np.float32)
    outputs[name][0, 20 * 40 + 17, :3] = [.125, .375, 8]
    transform = dict(gain=2., pad_x=4, pad_y=8, left=10, top=20, step=1, crop_width=200, crop_height=200)
    result = decode(outputs, info, transform)
    assert result['valid'] and result['center_px'] == [77., 99.]
    outputs[name][0, 20 * 40 + 17, 2] = -20
    assert not decode(outputs, info, transform)['valid']


def test_quantized_boundary_score_matches_center_probe_not_generic_template():
    info = dict(inputs=[dict(shape=[1, 320, 320, 3])], outputs=[])
    outputs = {}
    for stride in (8, 16, 32):
        name = f'maix_det_stride_{stride}'; shape = [1, 320 // stride, 320 // stride, 65]
        info['outputs'].append(dict(name=name, shape=shape))
        outputs[name] = np.zeros(shape, np.float32); outputs[name][..., 64] = -20
    outputs['maix_det_stride_8'][0, 20, 17, 64] = 0
    info['outputs'].append(dict(name='pose', shape=[1, 2100, 15]))
    outputs['pose'] = np.zeros((1, 2100, 15), np.float32)
    outputs['pose'][0, 20 * 40 + 17, :3] = [.125, .375, 8]
    transform = dict(gain=2., pad_x=4, pad_y=8, left=10, top=20, step=1, crop_width=200, crop_height=200)
    r = decode(outputs, info, transform, .5)
    assert not r['valid'] and r['maximum_class_score'] == .5
    assert decode(outputs, info, transform, .3)['valid']
