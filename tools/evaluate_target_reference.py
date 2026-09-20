#!/usr/bin/env python3
"""Compare pixel detections with sparse, independently reviewed reference frames.

Uncertain labels are excluded; these counts are not full-video accuracy or
independent-scene generalization. Coordinates must use the source resolution.
"""
import argparse,json,math,statistics
from pathlib import Path

def evaluate(rows,reference):
 missing=[x["frame"] for x in reference if x["frame"] not in rows]
 if missing:raise ValueError(f"Replay missing {len(missing)} reference frames: {missing[:10]}")
 result={}
 for split in sorted({x['split'] for x in reference}):
  items=[x for x in reference if x['split']==split]
  stats={'sampled_frames':len(items),'missing_replay_rows':0}
  for kind in ('green','armor'):
   positives=hits=negatives=false_positives=wrong=uncertain=0;errors=[]
   for ref in items:
    row=rows.get(ref['frame'])
    if row is None:continue
    status=ref[kind+'_status'];d=row[kind]
    observed=d['valid'] and (not d.get('predicted',False) if kind=='green' else row.get('armor_detection_ran',False))
    if status=='uncertain':uncertain+=1;continue
    if status=='absent':
     negatives+=1;false_positives+=int(observed);continue
    positives+=1
    if not observed:continue
    center=[d['center_x'],d['center_y']] if kind=='green' else [d['center']['x'],d['center']['y']]
    error=math.dist(center,ref[kind+'_center'])
    # Coarse reference placement: lamp radius or >=4 px. Armor requires
    # both center and bar-center proximity, so a random nearby pair is not a hit.
    if kind=='green':matched=error<=max(4,ref['green_radius'])
    else:
     x0,y0,x1,y1=ref['armor_bbox'];pad=3
     bar_centers=[d['left_bar']['center'],d['right_bar']['center']]
     separation=math.hypot(bar_centers[0]['x']-bar_centers[1]['x'],bar_centers[0]['y']-bar_centers[1]['y'])
     matched=(error<=max(4,(x1-x0)*.2) and separation>=.5*max(x1-x0,y1-y0) and all(x0-pad<=c['x']<=x1+pad and y0-pad<=c['y']<=y1+pad for c in bar_centers))
    if matched:hits+=1;errors.append(error)
    else:wrong+=1
   stats[kind]={'visible_reference_frames':positives,'matched_direct_detections':hits,'missed_or_wrong':positives-hits,'wrong_location_detections':wrong,'absent_reference_frames':negatives,'false_detections_on_absent':false_positives,'uncertain_excluded':uncertain,'matched_center_error_median_px':statistics.median(errors) if errors else None,'matched_center_error_max_px':max(errors) if errors else None}
  stats['missing_replay_rows']=sum(x['frame'] not in rows for x in items)
  result[split]=stats
 return result

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--reference',type=Path,required=True);ap.add_argument('--replay',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
 rows={r['frame']:r for r in map(json.loads,a.replay.read_text().splitlines())};ref=list(map(json.loads,a.reference.read_text().splitlines()))
 out={'replay':str(a.replay),'reference':str(a.reference),'scope':'Sparse AI visually reviewed reference; not human ground truth; uncertain labels excluded; same-video development/heldout, not independent scenes.','results':evaluate(rows,ref)}
 a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out['results'],indent=2))
if __name__=='__main__':main()
