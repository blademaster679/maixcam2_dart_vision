#!/usr/bin/env python3
"""Render exact-source board offline observations on a complete, single-view video."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import subprocess

import cv2
import numpy as np


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''): h.update(block)
    return h.hexdigest()


def label(frame, value, x, y, color, scale=.5):
    (w, h), baseline = cv2.getTextSize(value, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x = max(2, min(round(x), frame.shape[1]-w-5))
    y = max(h+3, min(round(y), frame.shape[0]-baseline-3))
    cv2.rectangle(frame, (x-2,y-h-2), (x+w+2,y+baseline+1), (18,18,18), -1)
    cv2.putText(frame,value,(x,y),cv2.FONT_HERSHEY_SIMPLEX,scale,color,1,cv2.LINE_AA)


def dashed_box(frame, p1, p2, color):
    x1,y1=p1; x2,y2=p2
    for x in range(x1,x2,8):
        for y in (y1,y2): cv2.line(frame,(x,y),(min(x+4,x2),y),color,1,cv2.LINE_AA)
    for y in range(y1,y2,8):
        for x in (x1,x2): cv2.line(frame,(x,y),(x,min(y+4,y2)),color,1,cv2.LINE_AA)


def draw(frame, row, source_frame, source_fps):
    colors={'measured':(80,245,100),'candidate':(30,225,255),'predicted':(20,160,255),'armor':(60,70,245)}
    statuses=[]; drawn=[]
    if row:
        g=row['green']; state=str(g['state']).lower()
        mode='predicted' if g['valid'] and g['predicted'] else 'measured' if g['valid'] else 'candidate' if state=='candidate' and g['apparent_size']>0 else None
        if mode:
            x,y=g['center_x'],g['center_y']; radius=max(7,g['apparent_size']/2+4)
            if not all(math.isfinite(v) for v in (x,y,radius)): raise ValueError('nonfinite green position')
            x1=max(0,round(x-radius)); y1=max(0,round(y-radius)); x2=min(frame.shape[1]-1,round(x+radius)); y2=min(frame.shape[0]-1,round(y+radius))
            if x1<=x2 and y1<=y2:
                if mode=='measured': cv2.rectangle(frame,(x1,y1),(x2,y2),colors[mode],2,cv2.LINE_AA)
                else: dashed_box(frame,(x1,y1),(x2,y2),colors[mode])
                cv2.drawMarker(frame,(round(x),round(y)),colors[mode],cv2.MARKER_CROSS,8,1,cv2.LINE_AA)
                tag={'measured':'GREEN','candidate':'CANDIDATE','predicted':'GREEN PREDICTED'}[mode]
                label(frame,tag,x2+5,y1+12,colors[mode])
                drawn.append({'kind':mode,'center':[x,y],'box':[x1,y1,x2,y2]})
            statuses.append('Green: '+mode.upper())
        else: statuses.append('Green: NOT CONFIRMED')
        a=row['armor']
        fresh=row['armor_detection_ran'] and a['valid'] and row['armor_source_received_us']==row['source_received_us']
        if fresh:
            points=[]
            for name in ('left_bar','right_bar'):
                bar=a[name]
                for key in ('top','bottom'):
                    p=bar[key]
                    if not p['valid'] or not all(math.isfinite(p[k]) for k in ('x','y')): raise ValueError('invalid armor endpoint')
                    points.append((round(p['x']),round(p['y'])))
                cv2.line(frame,points[-2],points[-1],colors['armor'],3,cv2.LINE_AA)
            x1=max(0,min(p[0] for p in points)-5); y1=max(0,min(p[1] for p in points)-5)
            x2=min(frame.shape[1]-1,max(p[0] for p in points)+5); y2=min(frame.shape[0]-1,max(p[1] for p in points)+5)
            cv2.rectangle(frame,(x1,y1),(x2,y2),colors['armor'],1,cv2.LINE_AA)
            label(frame,'RED PAIR',x2+5,y1+12,colors['armor'])
            drawn.append({'kind':'fresh_armor','points':points,'box':[x1,y1,x2,y2]})
            statuses.append('Bars: DETECTED')
        else: statuses.append('Bars: '+('CACHED (not drawn)' if a['valid'] else 'NOT CONFIRMED'))
    else: statuses=['NO OBSERVATION - no boxes transferred from adjacent frames']
    # Small translucent legend; keep the original framing and pixel coordinates.
    frame[:65]=(frame[:65].astype(np.float32)*.32).astype(np.uint8)
    seconds=source_frame/source_fps
    text=f'CLIP 13 | {int(seconds//60):02d}:{seconds%60:05.2f} | '+ '   '.join(statuses)
    cv2.putText(frame,text,(14,24),cv2.FONT_HERSHEY_SIMPLEX,.55,(240,240,240),1,cv2.LINE_AA)
    cv2.putText(frame,'Green: measured   Yellow: candidate   Orange: predicted   Red: paired bars',(14,51),cv2.FONT_HERSHEY_SIMPLEX,.5,(220,220,220),1,cv2.LINE_AA)
    return drawn


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,required=True)
    ap.add_argument('--observations',type=Path,required=True)
    ap.add_argument('--summary',type=Path,required=True)
    ap.add_argument('--run',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--fps',type=int,default=30)
    args=ap.parse_args()
    audit_path=args.output.with_suffix('.json')
    if args.output.exists() or audit_path.exists(): raise ValueError('refusing to overwrite output/audit')
    run=json.loads(args.run.read_text());summary=json.loads(args.summary.read_text())
    if run.get('exit_code')!=0 or summary.get('failed',True) or summary.get('interrupted',False) or not summary.get('complete_source',False): raise ValueError('offline run did not complete successfully')
    if sha(args.source)!=run['input_sha256']: raise ValueError('source differs from board input')
    for path in (args.observations,args.summary):
        if sha(path)!=run.get('result_sha256',{}).get(path.name):
            raise ValueError('result file differs from completed run: '+path.name)
    records={}; previous=-1
    for line_number,line in enumerate(args.observations.read_text().splitlines(),1):
        row=json.loads(line);n=row['source_frame']
        if n<=previous: raise ValueError('non-increasing/duplicate source observation')
        if row['source_sequence']!=n+1 or row['timestamp_us']!=row['source_received_us'] or not row['source_metadata_valid']: raise ValueError('source metadata mismatch')
        if row['safe_for_control'] or row['angles_valid'] or row['model_ran'] or row['pose']['valid']: raise ValueError('unexpected calibrated/model output')
        if not row['classical_detection_ran']: raise ValueError('not a detector observation')
        records[n]=(line_number,row);previous=n
    if len(records)!=summary['processed_observations']: raise ValueError('observation count differs from completed run')
    cap=cv2.VideoCapture(str(args.source))
    if not cap.isOpened(): raise ValueError('cannot decode source')
    width=round(cap.get(cv2.CAP_PROP_FRAME_WIDTH));height=round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT));source_fps=cap.get(cv2.CAP_PROP_FPS)
    if args.fps<=0: raise ValueError('output FPS must be positive')
    if round(cap.get(cv2.CAP_PROP_FRAME_COUNT)) != summary['decoded_frames']: raise ValueError('source frame count differs from completed board run')
    step=round(source_fps/args.fps)
    if args.fps<=0 or step<1 or abs(source_fps/step-args.fps)>.001: raise ValueError('output FPS must divide native source FPS')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    cmd=['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-n','-f','rawvideo','-pix_fmt','bgr24','-s',f'{width}x{height}','-r',str(args.fps),'-i','pipe:0','-an','-c:v','libx264','-preset','fast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(args.output)]
    audit={'method':'complete_source_single_view_exact_frame_overlay','realtime_fps_measurement':False,'board_inference':run.get('board_machine')=='aarch64','source_sha256':sha(args.source),'observations_sha256':sha(args.observations),'summary_sha256':sha(args.summary),'run_sha256':sha(args.run),'source_fps':source_fps,'output_fps':args.fps,'source_sampling_step':step,'width':width,'height':height,'no_interpolation':True,'cached_armor_not_drawn':True,'frames':[]}
    counts=Counter();process=subprocess.Popen(cmd,stdin=subprocess.PIPE,stderr=subprocess.PIPE);index=0;decoded=0
    try:
        while True:
            ok=cap.grab()
            if not ok: break
            decoded+=1
            if index%step==0:
                ok,frame=cap.retrieve()
                if not ok: raise ValueError('source frame retrieval failed')
                entry=records.get(index);row=entry[1] if entry else None
                marks=draw(frame,row,index,source_fps)
                for mark in marks: counts[mark['kind']]+=1
                if not row: counts['no_observation']+=1
                audit['frames'].append({'output_frame':len(audit['frames']),'source_frame':index,'source_time_s':index/source_fps,'observation_line':entry[0] if entry else None,'marks':marks})
                process.stdin.write(frame.tobytes())
            index+=1
        cap.release();process.stdin.close();error=process.stderr.read().decode();code=process.wait()
        if code: raise RuntimeError('encoder failure: '+error)
        if records and max(records)>=decoded: raise ValueError('observations extend past decoded source')
        expected=summary.get('decoded_frames')
        if expected is not None and expected!=decoded: raise ValueError('offline run did not cover complete source')
        audit.update(source_frames=decoded,observations=len(records),rendered_frames=len(audit['frames']),duration_s=len(audit['frames'])/args.fps,mark_counts=dict(counts),output_sha256=sha(args.output),output_bytes=args.output.stat().st_size)
        audit_path.write_text(json.dumps(audit,indent=2)+'\n')
        print(json.dumps({k:v for k,v in audit.items() if k!='frames'},indent=2))
    except BaseException:
        cap.release()
        if process.poll() is None: process.terminate();process.wait(timeout=10)
        raise


if __name__=='__main__': main()
