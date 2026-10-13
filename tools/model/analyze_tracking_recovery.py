#!/usr/bin/env python3
import csv,json,argparse
from pathlib import Path

def load(p):return [{k:float(v) for k,v in r.items()} for r in csv.DictReader(p.open())]
def stats(r,key):
 gaps=[];start=None
 for i in range(len(r)+1):
  yes=i==len(r) or r[i][key]
  if not yes and start is None:start=i
  elif yes and start is not None:gaps.append(i-start);start=None
 return {'valid_frames':sum(bool(x[key]) for x in r),'missing_frames':sum(gaps),'missing_ms':sum(gaps)*1000/180,'gap_count':len(gaps),'max_gap_ms':max(gaps,default=0)*1000/180}
def main():
 ap=argparse.ArgumentParser();ap.add_argument('directory',type=Path);a=ap.parse_args();p=a.directory
 old=load(Path('artifacts/predictive_center_v2_20261007/observations.csv'));results=[]
 for start,duration,lo,hi in ((12,4,13,15),(65,6,66,70),(74,2,74,76)):
  r=load(p/f'range{start}.csv');meta=json.loads((p/f'range{start}.json').read_text());assert len(r)==duration*180 and meta['complete_range']
  assert meta['start_seconds']==start
  for i,x in enumerate(r):
   assert x['sequence']==i
   if x['hard_reject']:assert x['result_accepted'] and not x['raw_model_valid'] and not x['observed']
   if x['output_predicted']:assert x['output_valid'] and 0<x['observation_age_ms']<=60
   if x['observed']:assert x['confirmation_hits']>=2
  window=r[(lo-start)*180:(hi-start)*180];before=old[lo*180:hi*180]
  results.append({'range':[lo,hi],'old_output':stats(before,'smooth_valid'),'new_output':stats(window,'output_valid'),
   'old_raw':stats(before,'observed'),'new_raw':stats(window,'observed'),
   'predicted_frames':sum(bool(x['output_predicted']) for x in window),
   'tolerated_negative_results':sum(x['result_accepted'] and not x['raw_model_valid'] and not x['hard_reject'] for x in window),
   'hard_reject_results':sum(x['hard_reject'] for x in window)})
 (p/'COMPARISON.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(results,indent=2))
if __name__=='__main__':main()
