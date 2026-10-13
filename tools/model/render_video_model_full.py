#!/usr/bin/env python3
"""Overlay complete board recognition on every original frame, size and FPS unchanged."""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import subprocess
import cv2
import numpy as np
from PIL import Image, ImageDraw
from render_video_model_fps import text


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()


def tag(frame, value, x, y, color):
    size=cv2.getTextSize(value,cv2.FONT_HERSHEY_SIMPLEX,.48,1)[0]
    x=max(2,min(int(x),frame.shape[1]-size[0]-3));y=max(110,min(int(y),frame.shape[0]-45))
    cv2.rectangle(frame,(x-2,y-size[1]-3),(x+size[0]+2,y+4),(15,20,26),-1)
    cv2.putText(frame,value,(x,y),cv2.FONT_HERSHEY_SIMPLEX,.48,color,1,cv2.LINE_AA)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,required=True)
    ap.add_argument('--results',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    temporary=args.output.with_name(args.output.stem+'.partial.mp4')
    if args.output.exists() or temporary.exists():raise ValueError('refusing to overwrite video')
    summary=json.loads((args.results/'observations.jsonl.summary.json').read_text())
    if not summary['complete_source'] or (args.results/'exit_code').read_text().strip()!='0':
        raise ValueError('board replay incomplete')
    meta=json.loads((args.results/'provenance.json').read_text())
    if sha(args.source)!=meta['source_sha256']:raise ValueError('source video differs from board')
    if sha(args.results/'observations.jsonl')!=meta['observations_sha256']:raise ValueError('changed observations')
    width,height=summary['source_width'],summary['source_height'];fps=summary['source_fps']
    total=summary['processed_frames'];sx,sy=width/480,height/360
    cap=cv2.VideoCapture(str(args.source))
    if not cap.isOpened():raise ValueError('video open failed')
    if int(cap.get(cv2.CAP_PROP_FRAME_COUNT))!=total:raise ValueError('frame count mismatch')
    if abs(cap.get(cv2.CAP_PROP_FPS)-fps)>.001:raise ValueError('fps mismatch')
    # Original frame rate is retained: model updates on odd source frames are
    # not accidentally lost by converting 180fps to 30fps before annotation.
    writer=subprocess.Popen(['ffmpeg','-v','error','-nostdin','-f','rawvideo','-pix_fmt','bgr24',
        '-s',f'{width}x{height}','-r',str(fps),'-i','-','-i',str(args.source),
        '-map','0:v:0','-map','1:a?','-c:v','libx264','-preset','fast','-crf','20',
        '-pix_fmt','yuv420p','-c:a','copy','-movflags','+faststart',str(temporary)],stdin=subprocess.PIPE)
    head=Image.new('RGB',(width,102));d=ImageDraw.Draw(head)
    text(d,(14,4),'完整原片 | 传统视觉 + V4 模型 | 板端离线逐帧识别',25)
    text(d,(14,39),'原片/导出 180 FPS   识别计算吞吐：',22)
    text(d,(680,39),'模型调用吞吐：',22)
    text(d,(14,73),'吞吐按最近180帧实际识别耗时统计，不含解码；不是实时摄像头FPS',19,(190,204,219))
    header=np.array(head)[:,:,::-1].copy()
    foot=Image.new('RGB',(width,34));d=ImageDraw.Draw(foot)
    text(d,(12,5),'绿：观测  橙：预测  黄：候选  青：模型区域  粉：模型中心  红：双条',19)
    footer=np.array(foot)[:,:,::-1].copy()
    window=deque();sum_work=0.;sum_models=0
    latest_model=None;drawn_models=0;drawn_armor=0;green_frames=0
    source_json=args.results/'observations.jsonl'
    processed=0;model_calls=0
    try:
        with source_json.open() as stream:
            for i,line in enumerate(stream):
                row=json.loads(line)
                if row['source_frame']!=i:raise ValueError('source row mismatch')
                if abs(row['source_time_us']/1e6-i/fps)>.001:raise ValueError('source time mismatch')
                ok,frame=cap.read()
                if not ok:raise ValueError('source ended before observations')
                if frame.shape[:2]!=(height,width):raise ValueError('source dimensions changed')
                def point(x,y):return (round(x*sx),round(y*sy))
                def marker(x,y,color,radius=14):
                    cv2.drawMarker(frame,point(x,y),color,cv2.MARKER_CROSS,radius,2,cv2.LINE_AA)
                if row['classical_detection_ran']:
                    for candidate in row['candidates']:
                        x,y=candidate['center_x'],candidate['center_y']
                        cv2.circle(frame,point(x,y),6,(50,210,245),1,cv2.LINE_AA)
                g=row['green']
                if g['valid']:
                    green_frames+=1;color=(40,155,255) if g['predicted'] else (90,245,80)
                    marker(g['center_x'],g['center_y'],color)
                    radius=max(4,g['apparent_size']/2)
                    a=point(g['center_x']-radius,g['center_y']-radius);b=point(g['center_x']+radius,g['center_y']+radius)
                    cv2.rectangle(frame,a,b,color,1,cv2.LINE_AA)
                    tag(frame,'PREDICTED' if g['predicted'] else 'GREEN',b[0]+5,a[1],color)
                a=row['armor']
                # Cached bars are not transferred to a new source frame.
                if row['armor_detection_ran'] and a['valid']:
                    drawn_armor+=1
                    for name in ['left_bar','right_bar']:
                        bar=a[name];p,q=bar['top'],bar['bottom']
                        if p['valid'] and q['valid']:
                            cv2.line(frame,point(p['x'],p['y']),point(q['x'],q['y']),(70,65,255),3,cv2.LINE_AA)
                model=row['npu_current']
                if model['ran']:
                    model_calls+=1
                    latest_model=(i,model,row['model_inference_ms'])
                    x,y,w,h=model['roi'];p,q=point(x,y),point(x+w,y+h)
                    cv2.rectangle(frame,p,q,(255,210,60),1,cv2.LINE_AA)
                    if model['valid']:
                        drawn_models+=1;marker(model['x'],model['y'],(208,100,255),22)
                    tag(frame,f"V4 {'YES' if model['valid'] else 'NO'} {model['score']:.2f}",p[0],p[1]-6,(208,100,255))
                window.append((row['work_ms'],int(model['ran'])))
                sum_work+=row['work_ms'];sum_models+=int(model['ran'])
                if len(window)>180:
                    old=window.popleft();sum_work-=old[0];sum_models-=old[1]
                throughput=len(window)*1000/max(sum_work,1e-6)
                model_rate=sum_models*1000/max(sum_work,1e-6)
                frame[:102]=cv2.addWeighted(frame[:102],.3,header,1,0)
                frame[-34:]=cv2.addWeighted(frame[-34:],.3,footer,1,0)
                cv2.putText(frame,f'{throughput:.1f} FPS',(405,61),cv2.FONT_HERSHEY_SIMPLEX,.72,(245,245,245),2,cv2.LINE_AA)
                cv2.putText(frame,f'{model_rate:.1f} /s',(860,61),cv2.FONT_HERSHEY_SIMPLEX,.72,(208,100,255),2,cv2.LINE_AA)
                cv2.putText(frame,'Track: '+str(g['state']),(1050,61),cv2.FONT_HERSHEY_SIMPLEX,.5,(230,230,230),1,cv2.LINE_AA)
                cv2.putText(frame,f'{i/fps:05.2f}s / {summary["duration_seconds"]:.2f}s | #{i+1}/{total}',(960,25),cv2.FONT_HERSHEY_SIMPLEX,.47,(240,240,240),1,cv2.LINE_AA)
                status='NPU: waiting for candidate'
                if latest_model:
                    fi,m,ms=latest_model
                    status=f"Last NPU: {'YES' if m['valid'] else 'NO'} {m['score']:.2f} | {ms:.1f}ms | age {(i-fi)*1000/fps:.0f}ms"
                cv2.putText(frame,status,(820,height-11),cv2.FONT_HERSHEY_SIMPLEX,.42,(240,240,240),1,cv2.LINE_AA)
                writer.stdin.write(frame.tobytes());processed+=1
                if i in (180*26+1,180*30+1,180*62+1):
                    cv2.imwrite(str(args.output.with_name(f'preview_{i}.jpg')),frame)
                if (i+1)%1800==0:print(f'rendered {i+1}/{total}',flush=True)
        ok,_=cap.read()
        if ok or processed!=total:raise ValueError('incomplete frame coverage')
        if model_calls!=summary['model_calls'] or drawn_models!=summary['model_positives']:
            raise ValueError('model result coverage mismatch')
        writer.stdin.close()
        if writer.wait()!=0:raise RuntimeError('ffmpeg failed')
    except BaseException:
        writer.kill();writer.wait();raise
    finally:cap.release()
    temporary.rename(args.output)
    args.output.with_suffix('.json').write_text(json.dumps({'source_sha256':meta['source_sha256'],
        'observations_sha256':meta['observations_sha256'],'output_sha256':sha(args.output),
        'source_frames':total,'output_frames':processed,'width':width,'height':height,'fps':fps,
        'model_calls':model_calls,'model_positive_frames_drawn':drawn_models,'fresh_armor_frames_drawn':drawn_armor,
        'green_frames_drawn':green_frames,'coordinate_scale':[sx,sy],
        'no_crop':True,'no_frame_repetition':True,'no_source_frame_omitted':True,
        'throughput':'rolling 180 source frames / actual measured processing time; excludes decoding',
        'source_time_mapping':'one-to-one source frame; cached model status text only, no stale geometry'},indent=2)+'\n')
    print(args.output)

if __name__=='__main__':main()
