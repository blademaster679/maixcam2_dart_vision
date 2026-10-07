#!/usr/bin/env python3
"""Fine-tune a migrated handoff in a separate experiment; preserve old weights."""
import argparse
import hashlib
import json
import os
import shutil
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--handoff', type=Path, required=True)
    parser.add_argument('--name', default='green_linux_v5')
    parser.add_argument('--epochs', type=int, default=20)
    parser.add_argument('--batch', type=int, default=16)
    parser.add_argument('--device', default='0')
    args = parser.parse_args()
    training = args.handoff.resolve() / 'training'
    source = training / 'green_blender_v4'
    experiment = training / args.name
    experiment.mkdir()  # Never overwrite or accidentally resume an old experiment.
    os.environ['YOLO_CONFIG_DIR'] = str(training / '.ultralytics')
    os.environ['MPLCONFIGDIR'] = str(training / '.matplotlib')
    os.environ['WANDB_MODE'] = 'disabled'
    from ultralytics import YOLO, settings
    import torch
    torch.set_num_threads(6)
    settings.update({k: False for k in ('sync', 'wandb', 'mlflow', 'clearml',
                                      'comet', 'dvc', 'neptune') if k in settings})
    # Copy metadata; image/label files are shared read-only, caches stay local.
    dataset = experiment / 'dataset'
    dataset.mkdir()
    for name in ('images', 'labels'):
        shutil.copytree(source / 'dataset' / name, dataset / name,
                        copy_function=os.link)
    for name in ('manifest.json', 'data.yaml'):
        shutil.copy2(source / 'dataset' / name, dataset / name)
    yaml = dataset / 'data.yaml'
    lines = yaml.read_text().splitlines()
    yaml.write_text('\n'.join('path: ' + json.dumps(str(dataset))
                             if line.startswith('path:') else line for line in lines) + '\n')
    for name in ('dataset_audit.json', 'candidates.json'):
        shutil.copy2(source / name, experiment / name)
    initial = Path(json.loads((source / 'training_result.json').read_text())['best'])
    initial_hash = hashlib.sha256(initial.read_bytes()).hexdigest()
    manifest = json.loads((dataset / 'manifest.json').read_text())
    assert all((experiment / row['image']).is_file() and
               (experiment / row['label']).is_file() for row in manifest)
    report = dict(initial_weights=str(initial), initial_sha256=initial_hash,
                  epochs_requested=args.epochs, input_size=320, batch=args.batch,
                  training_images=sum(r['split'] == 'train' for r in manifest),
                  selection='best validation fitness; no test data used in training',
                  purpose='Low-learning-rate continuation of V4; improvement is unproven',
                  supervised_keypoints=[0], device=args.device, status='training')
    (experiment / 'training_result.json').write_text(json.dumps(report, indent=2))
    model = YOLO(str(initial))
    assert model.names == {0: 'dart_target'} and model.task == 'pose'
    assert list(model.model.model[-1].kpt_shape) == [5, 3]
    model.train(data=str(yaml), imgsz=320, epochs=args.epochs, patience=8,
                batch=args.batch, device=args.device, workers=0,
                project=str(experiment / 'runs'), name='center_finetune',
                exist_ok=False, optimizer='AdamW', lr0=0.00005, freeze=5,
                amp=args.device != 'cpu', seed=20261007, deterministic=True,
                hsv_h=.005, hsv_s=.15, hsv_v=.25, degrees=25,
                translate=.12, scale=.3, fliplr=.5, flipud=.5,
                mosaic=0, mixup=0, plots=False)
    assert hashlib.sha256(initial.read_bytes()).hexdigest() == initial_hash
    report.update(best=str(model.trainer.best), epochs_completed=model.trainer.epoch + 1,
                  status='completed', torch=torch.__version__,
                  gpu=torch.cuda.get_device_name(0) if args.device != 'cpu' else None)
    (experiment / 'training_result.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
