#!/usr/bin/env python3
import csv,json,argparse
from pathlib import Path
import numpy as np

def rows(path):return [{k:float(v) for k,v in r.items()} for r in csv.DictReader(path.open())]
def main():
 ap=argparse.ArgumentParser();ap.add_argument('directory',type=Path);p=ap.parse_args().directory
 r=rows(p/'observations.csv');h=rows(p/'observations_host.csv');assert len(r)==len(h)==13766
 for i,x in enumerate(r):
  assert abs(x['smooth_x']-h[i]['smooth_x'])<1e-6 and abs(x['smooth_y']-h[i]['smooth_y'])<1e-6
  if x['output_predicted']:assert x['smooth_valid'] and 0<x['observation_age_ms']<=60
  if x['smooth_valid'] and not x['output_predicted']:assert x['observed']
  if x['result_accepted'] and not x['raw_model_valid']:assert not x['smooth_valid']
 def steps(valid,x,y):
  return [float(np.hypot(b[x]-a[x],b[y]-a[y])) for a,b in zip(r,r[1:]) if a[valid] and b[valid]]
 raw=steps('observed','x','y');filtered=steps('smooth_valid','smooth_x','smooth_y')
 def gaps(field):
  gaps=[];start=None
  for i,x in enumerate(r):
   if not x[field] and start is None:start=i
   if x[field] and start is not None:
    if start>0:gaps.append(i-start)
    start=None
  return gaps
 old=gaps('observed');new=gaps('smooth_valid')
 a={'frames':len(r),'raw_observations':sum(x['observed'] for x in r),'output_frames':sum(x['smooth_valid'] for x in r),
 'measured_updates':sum(x['smooth_valid'] and not x['output_predicted'] for x in r),'predicted_frames':sum(x['output_predicted'] for x in r),
 'filled_missing_frames':sum(x['output_predicted'] and not x['observed'] for x in r),'gated_measurements':sum(x['jump_rejected'] for x in r),
 'raw_observations_with_no_output':sum(x['observed'] and not x['smooth_valid'] for x in r),
 'max_prediction_age_ms':max(x['observation_age_ms'] for x in r),'raw_contiguous_step_max_px':max(raw),'output_contiguous_step_max_px':max(filtered),
 'raw_step_p95_px':float(np.percentile(raw,95)),'output_step_p95_px':float(np.percentile(filtered,95)),
 'raw_gap_runs_1_to_5_frames':sum(1<=n<=5 for n in old),'output_gap_runs_1_to_5_frames':sum(1<=n<=5 for n in new),
 'raw_gap_runs_total':len(old),'output_gap_runs_total':len(new),
 'board_filter_and_parse_p95_us':float(np.percentile([x['filter_us'] for x in r],95)),
 'note':'Gating includes generation reconfirmation, not proven false positives. Gap counts and maximum steps do not measure accuracy. Predictions are not new observations.'}
 (p/'prediction_audit.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(a,indent=2))
if __name__=='__main__':main()
