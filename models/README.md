# YOLO11n-Pose model contract

The runtime model is intentionally not committed. It must be trained from the
competition camera data and converted for AX620E before enabling `npu.enabled`.

The single class is `dart_target`. Keypoint order is fixed:

1. green-lamp center;
2. left bar top;
3. left bar bottom;
4. right bar top;
5. right bar bottom.

Use visibility `0` for unavailable distant bar endpoints, `1` for
occluded/partly reliable points, and `2` for visible points. Horizontal flips
must use `flip_idx: [0, 3, 4, 1, 2]`.

For the October 2026 green-center handoff, only keypoint 0 is supervised.
Set `npu.center_only=true` and `npu.input_size=320` for these models. This
discards points 1–4 and prevents a center-only prediction from creating or
validating armor geometry. The original five-point models retain the default
`npu.center_only=false`. The high-FPS NV21 pipeline still rejects NPU models;
these settings apply to the normal RGB application and image replay only.
The MUD must enable `input_cache_flush = true` when feeding CPU-preprocessed
RGB crops. Board replay found inconsistent predictions with the handoff's
original `false` setting. Center-only inference uses explicit bilinear resize
and padding value 114, then maps keypoint 0 back into source coordinates.

`tools/model/train_center_handoff.py` creates a separate continuation experiment
without replacing the original weights. `prepare_center_replay.py`,
`pc_center_replay.py`, `board_center_replay.py`, and `score_center_replay.py`
provide matching PC/board image replay and scoring. Replay uses stored proposals
and must not be reported as live camera end-to-end accuracy or capture FPS.

Workflow:

```bash
python3 tools/dataset/validate_pose_dataset.py --root DATASET

python3 tools/model/train_yolo11_pose.py train \
  --data DATASET/dataset.yaml --imgsz 256

python3 tools/model/train_yolo11_pose.py evaluate \
  --weights runs/dart_pose/yolo11n_pose_256/weights/best.pt \
  --data DATASET/dataset.yaml --sizes 192,256,320 \
  --output reports/model_size_comparison.json

python3 tools/model/train_yolo11_pose.py export \
  --weights runs/dart_pose/yolo11n_pose_256/weights/best.pt --imgsz 256
```

Inspect the fixed-shape ONNX in Netron, then use the actual four pose output
node names with `prepare_pulsar2.py`. Do not copy output names from another
model; Ultralytics versions can change them.

```bash
python3 tools/model/prepare_pulsar2.py \
  --onnx /path/to/best.onnx \
  --calibration-images /path/to/calibration_100 \
  --output /path/to/pulsar2_work \
  --output-tensor NODE_0 --output-tensor NODE_1 \
  --output-tensor NODE_2 --output-tensor NODE_3
```

Run the generated commands inside the official Pulsar2 container. Then stage
the complete bundle:

```bash
python3 tools/model/stage_runtime_model.py \
  --mud /path/to/pulsar2_work/out/dart_target_pose.mud
```

This writes the MUD and both full-NPU/virtual-NPU AXMODEL files to
`models/runtime/`. `app.yaml` packages that directory as application
`models/`, so the configured path is `models/dart_target_pose.mud`.

Official references:

- <https://wiki.sipeed.com/maixpy/doc/zh/ai_model_converter/maixcam2.html>
- <https://wiki.sipeed.com/maixpy/doc/zh/vision/customize_model_yolov8.html>
