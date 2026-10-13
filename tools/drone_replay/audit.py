"""Independent finite replay checks; never turn coverage into detection accuracy."""
import argparse
import json
from pathlib import Path
from replay import sha, save


def read(directory):
    directory = Path(directory).resolve()
    summary = json.loads((directory / 'summary.json').read_text())
    rows = [json.loads(line) for line in (directory / 'results.jsonl').read_text().splitlines()]
    if not summary.get('passed') or len(rows) != summary['frames']:
        raise ValueError('qualified complete replay required: ' + str(directory))
    if any(row['predicted'] or row['safe_for_control'] for row in rows):
        raise ValueError('offline replay must not provide predicted/control-qualified output')
    return directory, summary, rows


def audit(nominal, blackout, reacquire, no_target, output):
    output = Path(output); output.mkdir(parents=True, exist_ok=False)
    runs = [read(p) for p in (nominal, blackout, reacquire, no_target)]
    n, b, r, z = [value[2] for value in runs]
    checks = dict(nominal_has_confirmed_pixels=any(x['valid'] for x in n),
                  blackout_has_no_observations=not any(x['valid'] for x in b[10:16]),
                  synthetic_blackout_has_no_observations=not any(x['valid'] for x in r[10:16]),
                  synthetic_reappearance_confirmed=any(x['valid'] and x['model_confirmed'] for x in r[16:]),
                  no_target_stays_invalid=not any(x['valid'] for x in z))
    result = dict(passed=all(checks.values()), checks=checks,
                  scope='offline replay software function checks; no accuracy or flight qualification',
                  nominal_valid_frames=sum(bool(x['valid']) for x in n), nominal_frames=len(n),
                  nominal_model_positive=runs[0][1]['model_positive'],
                  natural_reacquisition=any(x['valid'] for x in b[16:]),
                  continuous_tracking=all(x['valid'] for x in n[1:]),
                  synthetic_reappearance='previous recorded frames repeated with explicit fixture timestamps',
                  inputs={str(p): dict(summary_sha256=sha(p / 'summary.json'), results_sha256=sha(p / 'results.jsonl'))
                          for p, _, _ in runs})
    save(output / 'summary.json', result); print(json.dumps(result))
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('nominal', 'blackout', 'reacquire', 'no-target', 'output'):
        p.add_argument('--' + name, type=Path, required=True)
    a = p.parse_args(); result = audit(a.nominal, a.blackout, a.reacquire, a.no_target, a.output)
    raise SystemExit(0 if result['passed'] else 1)
