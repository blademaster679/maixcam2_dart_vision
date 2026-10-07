#!/usr/bin/env python3
"""Audit full-source recovery and compare old/new output gaps by source frame."""
import argparse,csv,json
from pathlib import Path

def load(path):return [{k:float(v) for k,v in x.items()} for x in csv.DictReader(path.open())]
def stats(r,key,offset=0):
 gaps=[];start=None
 for i in range(len(r)+1):
  valid=i==len(r) or r[i][key]
  if not valid and start is None:start=i
  elif valid and start is not None:
   gaps.append({'start_s':(start+offset)/180,'end_s':(i+offset)/180,'frames':i-start,'duration_ms':(i-start)*1000/180});start=None
 return {'valid_frames':sum(bool(x[key]) for x in r),'missing_frames':sum(x['frames'] for x in gaps),
  'missing_seconds':sum(x['frames'] for x in gaps)/180,'gap_count':len(gaps),'max_gap_ms':max((x['duration_ms'] for x in gaps),default=0),'gaps':gaps}
def main():
 ap=argparse.ArgumentParser();ap.add_argument('directory',type=Path);p=ap.parse_args().directory
 old=load(Path('artifacts/predictive_center_v2_20261007/observations.csv'));r=load(p/'observations.csv')
 meta=json.loads((p/'summary.json').read_text());assert (p/'exit_code').read_text().strip()=='0'
 assert meta['complete_source'] and len(r)==len(old)==meta['processed_frames']==13766
 interruptions=[]
 for i,x in enumerate(r):
  assert x['sequence']==i
  if i:assert x['wall_end_ms']>r[i-1]['wall_end_ms']
  if x['observed']:
   assert x['confirmation_hits']>=2 and x['x']==x['measured_x'] and x['y']==x['measured_y']
   assert 0<=1000000+round(i*1000000/180)-x['last_positive']<=100001
  if x['hard_reject']:assert x['result_accepted'] and not x['raw_model_valid'] and not x['observed']
  if x['output_valid']:
   assert 0<=x['output_x']<1344 and 0<=x['output_y']<760
   if x['output_predicted']:assert 0<x['observation_age_ms']<=60
   else:assert x['observed']
  else:assert not x['output_predicted']
  if i and r[i-1]['observed'] and not x['observed']:
   cause='consecutive_model_negative' if x['hard_reject'] else 'positive_confirmation_expired' if 1000000+round(i*1000000/180)-r[i-1]['last_positive']>100000 else 'local_measurement_loss'
   interruptions.append({'time_s':i/180,'cause':cause})
 windows=[]
 for a,b in ((13,15),(15,16),(35,36),(48,49),(66,70),(74,76)):
  windows.append({'range':[a,b],'old':stats(old[a*180:b*180],'smooth_valid',a*180),'new':stats(r[a*180:b*180],'output_valid',a*180),'new_raw':stats(r[a*180:b*180],'observed',a*180),'predicted_frames':sum(x['output_predicted'] for x in r[a*180:b*180])})
 oldevents=json.loads((p/'OLD_INTERRUPTS.json').read_text());checks=[]
 for event in oldevents:
  if event['cause']!='same_track_model_negative':continue
  start=event['start_frame'];end=start+event['frames'];c=r[start:end]
  checks.append(dict(event,new_missing_frames=sum(not x['output_valid'] for x in c),new_missing_raw_frames=sum(not x['observed'] for x in c),new_predicted_frames=sum(x['output_predicted'] for x in c)))
 audit={'frames':len(r),'old_raw':stats(old,'observed'),'old_output':stats(old,'smooth_valid'),
  'new_raw':stats(r,'observed'),'new_output':stats(r,'output_valid'),'predicted_frames':sum(x['output_predicted'] for x in r),
  'tolerated_negative_results':sum(x['result_accepted'] and not x['raw_model_valid'] and not x['hard_reject'] for x in r),
  'old_negative_event_checks':checks,'new_interruptions':interruptions,'windows':windows,'accuracy_validated':False,
  'note':'No-output gaps include absent or out-of-view targets. Sequential offline decoding is not real-time camera FPS or control latency.'}
 (p/'FULL_COMPARISON.json').write_text(json.dumps(audit,indent=2)+'\n')
 print(json.dumps({k:{a:b for a,b in v.items() if a!='gaps'} for k,v in audit.items() if k in ('old_output','new_output','new_raw')},indent=2))
 print('old single-negative triggers',len(checks),'now complete measured coverage',sum(x['new_missing_raw_frames']==0 for x in checks),'prediction frames',audit['predicted_frames'])
 for w in windows:print(w['range'],'missing',w['old']['missing_frames'],'->',w['new']['missing_frames'],'pred',w['predicted_frames'])
if __name__=='__main__':main()
