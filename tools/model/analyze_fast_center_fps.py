#!/usr/bin/env python3
"""Audit actual observation rate, freshness, gaps and synthetic loop boundaries."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('directory',type=Path)
    ap.add_argument('--prefix',default='fast_v4_s')
    args=ap.parse_args();results=[]
    paths=sorted(args.directory.glob(args.prefix+'*.csv'))
    if not paths:
        ap.error('no matching CSV files')
    for path in paths:
        summary=json.loads(path.with_suffix('.json').read_text())
        rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(path.open())]
        for a,b in zip(rows,rows[1:]):
            assert b['sequence']>a['sequence'] and b['end_ms']>a['end_ms']
        selected=[r for r in rows if 3000<=r['end_ms']<19500]
        observed=[r for r in selected if r['observed']]
        assert math.isclose(len(observed)/16.5,summary['observed_hz'],abs_tol=1e-6)
        assert math.isclose(len(selected)/16.5,summary['processed_hz'],abs_tol=1e-6)
        for r in observed:
            assert r['confirmation_hits']>=2
            assert 0<=r['x']<1344 and 0<=r['y']<760
            assert math.isclose(r['age_ms'],r['end_ms']-r['sequence']*1000/180,abs_tol=1e-5)
        if rows and 'result_accepted' in rows[0]:
            for r in rows:
                ts=1000000+int(r['sequence'])*1000000//180
                if r['result_accepted']:
                    assert r['model_ran'] and 0<=ts-r['result_ts']<=100000
                if r['observed']:
                    assert 0<r['last_positive']<=ts and ts-r['last_positive']<=100000
                    assert r['x']==r['measured_x'] and r['y']==r['measured_y']
                    if r['model_valid']:
                        assert r['result_accepted'] and r['generation']==r['result_generation']
                if r['result_accepted'] and not r['raw_model_valid']:
                    assert not r['observed'] and r['confirmation_hits']==0
        if observed:
            assert math.isclose(float(np.percentile([r['age_ms'] for r in observed],95)),summary['observed_age']['p95_ms'],abs_tol=1e-5)
        gaps=[(b['end_ms']-a['end_ms'],int(a['sequence']//90)!=int(b['sequence']//90)) for a,b in zip(observed,observed[1:])]
        within=[g for g,cross in gaps if not cross];cross=[g for g,cross in gaps if cross]
        rolling=[sum(t-1000<r['end_ms']<=t for r in observed) for t in range(4000,19001,100)]
        summary.update(csv=path.name,observations_older_than_30ms=sum(r['age_ms']>=30 for r in observed),
            max_gap_same_loop_ms=max(within,default=0),max_gap_cross_loop_ms=max(cross,default=0),
            min_rolling_1s_observations=min(rolling,default=0),
            observations_within_budget=sum(r['age_ms']<30 for r in observed),
            note='Boundary classification is descriptive, not proof that looping caused every dropout; no ground truth accuracy validation.')
        results.append(summary)
        print(path.name,'observed Hz',summary['observed_hz'],'age p95',summary['observed_age']['p95_ms'],
              'max',summary['observed_age']['max_ms'],'max same-loop gap',summary['max_gap_same_loop_ms'],'rolling min',summary['min_rolling_1s_observations'])
    (args.directory/(args.prefix+'AUDIT.json')).write_text(json.dumps(results,indent=2)+'\n')

if __name__=='__main__':main()
