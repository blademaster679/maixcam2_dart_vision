#!/usr/bin/env python3
"""Independently recompute steady-window rates from board replay CSV and logs."""
import argparse
import csv
import json
import math
from pathlib import Path


def analyze(path):
    summaries = [json.loads(line) for line in path.with_suffix('.log').read_text().splitlines()
                 if line.startswith('{"feed_hz"')]
    if len(summaries) != 1:
        raise ValueError(f'{path}: expected exactly one successful summary')
    summary = summaries[0]
    rows = list(csv.DictReader(path.open()))
    seq = [int(r['sequence']) for r in rows]
    if any(b <= a for a, b in zip(seq, seq[1:])):
        raise ValueError('duplicate/out-of-order source frame')
    if any(int(r['cache_index']) != int(r['sequence']) % summary['cache_frames'] for r in rows):
        raise ValueError('cache/source mapping mismatch')
    low, high = 3000, (summary['seconds'] - 0.5) * 1000
    window = (high - low) / 1000
    selected = [r for r in rows if low <= float(r['end_ms']) < high]
    mapping = {'processed_fps': len(selected),
               'classical_fps': sum(int(r['classical']) for r in selected),
               'model_fps': sum(int(r['model_calls']) for r in selected),
               'green_observed_fps': sum(int(r['green_observed']) for r in selected)}
    for field, count in mapping.items():
        if not math.isclose(summary[field], count / window, abs_tol=1e-6):
            raise ValueError(f'{field} accounting mismatch')
    summary['csv'] = path.name
    summary['not_processed_fraction'] = 1 - summary['processed_fps'] / summary['feed_hz']
    if 'detect_ms' in rows[0]:
        classical = [float(r['detect_ms']) for r in selected if int(r['classical'])]
        summary['classical_slot_mean_ms'] = sum(classical) / len(classical) if classical else 0
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    results = [analyze(path) for path in sorted(args.directory.glob('s*_i*.csv'))]
    if not results:
        raise SystemExit('no completed runs')
    output = args.directory / 'RESULTS.json'
    output.write_text(json.dumps(results, indent=2) + '\n')
    for r in results:
        print(f"start={r['video_start_seconds']:4.1f}s interval={r['interval_ms']:3d} "
              f"processed={r['processed_fps']:6.2f} classical={r['classical_fps']:6.2f} "
              f"model={r['model_fps']:6.2f} model_mean_ms={r['model']['mean_ms']:5.2f}")

if __name__ == '__main__':
    main()
