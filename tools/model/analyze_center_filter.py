#!/usr/bin/env python3
"""Audit smoothing without treating trajectory roughness as ground-truth error."""
import csv,json,argparse
from pathlib import Path
import numpy as np

def read(p):return [{k:float(v) for k,v in r.items()} for r in csv.DictReader(p.open())]
def main():
 ap=argparse.ArgumentParser();ap.add_argument('directory',type=Path);a=ap.parse_args();p=a.directory
 rows=read(p/'observations.csv');host=read(p/'observations_host.csv')
 assert len(rows)==len(host)==13766
 raw=[];smooth=[];offset=[];resets=0
 for i,r in enumerate(rows):
  assert r['observed']==r['smooth_valid']
  assert abs(r['smooth_x']-host[i]['smooth_x'])<1e-6 and abs(r['smooth_y']-host[i]['smooth_y'])<1e-6
  if r['observed']:
   resets+=r['smooth_reset'];offset.append(np.hypot(r['x']-r['smooth_x'],r['y']-r['smooth_y']))
   if r['smooth_reset']:assert abs(r['x']-r['smooth_x'])<1e-6 and abs(r['y']-r['smooth_y'])<1e-6
  if i<2:continue
  w=rows[i-2:i+1]
  if not all(x['observed'] and not x['smooth_reset'] for x in w) or len({x['generation'] for x in w})!=1:continue
  raw.append(np.hypot(w[2]['x']-2*w[1]['x']+w[0]['x'],w[2]['y']-2*w[1]['y']+w[0]['y']))
  smooth.append(np.hypot(w[2]['smooth_x']-2*w[1]['smooth_x']+w[0]['smooth_x'],w[2]['smooth_y']-2*w[1]['smooth_y']+w[0]['smooth_y']))
 rms=lambda v:float(np.sqrt(np.mean(np.square(v))))
 result={'frames':len(rows),'valid_frames':sum(r['observed'] for r in rows),'resets':resets,'second_difference_samples':len(raw),
  'raw_second_difference_rms_px':rms(raw),'smooth_second_difference_rms_px':rms(smooth),'roughness_reduction_fraction':1-rms(smooth)/rms(raw),
  'offset_from_raw_p95_px':float(np.percentile(offset,95)),'offset_from_raw_max_px':float(max(offset)),
  'board_filter_and_parse_p95_us':float(np.percentile([r['filter_us'] for r in rows],95)),
  'board_filter_and_parse_max_us':max(r['filter_us'] for r in rows),
  'note':'Second differences measure trajectory roughness, including real acceleration; not center error or measured latency. No prediction-only outputs; host/board match <1e-6 pixel.'}
 (p/'filter_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
