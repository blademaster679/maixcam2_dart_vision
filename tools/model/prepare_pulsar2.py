#!/usr/bin/env python3
"""Prepare AX620E Pulsar2 configs, calibration tar, and pose MUD metadata."""

from __future__ import annotations

import argparse
import json
import shutil
import tarfile
from pathlib import Path


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def conversion_config(mode: str, input_tensor: str,
                      output_tensors: list[str], calibration_tar: str,
                      calibration_size: int) -> dict:
    return {
        "model_type": "ONNX",
        "npu_mode": mode,
        "quant": {
            "input_configs": [{
                "tensor_name": input_tensor,
                "calibration_dataset": calibration_tar,
                "calibration_size": calibration_size,
                "calibration_mean": [0, 0, 0],
                "calibration_std": [255, 255, 255],
            }],
            "calibration_method": "MinMax",
            "precision_analysis": True,
        },
        "input_processors": [{
            "tensor_name": input_tensor,
            "tensor_format": "RGB",
            "tensor_layout": "NCHW",
            "src_format": "RGB",
            "src_dtype": "U8",
            "src_layout": "NHWC",
            "csc_mode": "NoCSC",
        }],
        "output_processors": [
            {"tensor_name": tensor, "dst_perm": [0, 2, 3, 1]}
            for tensor in output_tensors
        ],
        "compiler": {
            "check": 3,
            "check_mode": "CheckOutput",
            "check_cosine_simularity": 0.9,
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--onnx", type=Path, required=True)
    parser.add_argument("--calibration-images", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--input-tensor", default="images")
    parser.add_argument("--output-tensor", action="append", required=True,
                        help="post-processing input node seen in Netron; repeat for all nodes")
    parser.add_argument("--model-name", default="dart_target_pose")
    args = parser.parse_args()
    if not args.onnx.is_file():
        raise SystemExit(f"ONNX file not found: {args.onnx}")
    images = sorted(path for path in args.calibration_images.rglob("*")
                    if path.suffix.lower() in IMAGE_SUFFIXES)
    if not 20 <= len(images) <= 100:
        raise SystemExit("Pulsar2 calibration set must contain 20 to 100 images")
    # MaixCDK's YOLO11 pose postprocessor currently expects four output tensors.
    if len(args.output_tensor) != 4:
        raise SystemExit("YOLO11-Pose requires four verified output tensors; inspect ONNX in Netron")
    args.output.mkdir(parents=True, exist_ok=True)
    local_onnx = args.output / "export.onnx"
    shutil.copy2(args.onnx, local_onnx)
    dataset_dir = args.output / "datasets"
    config_dir = args.output / "config"
    out_dir = args.output / "out"
    dataset_dir.mkdir(exist_ok=True)
    config_dir.mkdir(exist_ok=True)
    out_dir.mkdir(exist_ok=True)
    calibration_tar = dataset_dir / "train.tar"
    with tarfile.open(calibration_tar, "w") as archive:
        for index, image in enumerate(images):
            archive.add(image, arcname=f"{index:03d}_{image.name}")
    relative_tar = "datasets/train.tar"
    for filename, mode in (("pose.npu.json", "NPU2"),
                           ("pose.vnpu.json", "NPU1")):
        config = conversion_config(mode, args.input_tensor, args.output_tensor,
                                   relative_tar, len(images))
        (config_dir / filename).write_text(json.dumps(config, indent=2) + "\n",
                                           encoding="utf-8")
    mud = f"""[basic]
type = axmodel
model_npu = {args.model_name}_npu.axmodel
model_vnpu = {args.model_name}_vnpu.axmodel

[extra]
model_type = yolo11
type = pose
input_type = rgb
labels = dart_target
input_cache = true
output_cache = true
input_cache_flush = true
output_cache_inval = true
mean = 0,0,0
scale = 0.00392156862745098,0.00392156862745098,0.00392156862745098
"""
    (out_dir / f"{args.model_name}.mud").write_text(mud, encoding="utf-8")
    commands = f"""# Run inside the official Pulsar2 container from this directory.
pulsar2 build --target_hardware AX620E --input export.onnx --output_dir tmp_npu --config config/pose.npu.json
cp tmp_npu/compiled.axmodel out/{args.model_name}_npu.axmodel
pulsar2 build --target_hardware AX620E --input export.onnx --output_dir tmp_vnpu --config config/pose.vnpu.json
cp tmp_vnpu/compiled.axmodel out/{args.model_name}_vnpu.axmodel
"""
    (args.output / "PULSAR2_COMMANDS.txt").write_text(commands, encoding="utf-8")
    print(f"prepared Pulsar2 workspace: {args.output}")
    print("Important: verify all four --output-tensor names and their layouts in Netron.")


if __name__ == "__main__":
    main()
