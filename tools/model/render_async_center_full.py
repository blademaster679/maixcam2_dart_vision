#!/usr/bin/env python3
"""Render all original frames with board asynchronous offline observations."""
import argparse,csv,json,hashlib,subprocess
from collections import deque
from pathlib import Path
import cv2
import numpy as np
from PIL import Image,ImageDraw
from render_video_model_fps import text

def sha(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for block in iter(lambda:f.read(1048576),b''):h.update(block)
    return h.hexdigest()

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,required=True);ap.add_argument('--results',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--smooth',action='store_true',help='Draw board-filtered center and raw reference')
    ap.add_argument('--predict',action='store_true',help='Render bounded prediction separately from measured output')
    args=ap.parse_args()
    if args.predict:args.smooth=True
    p=args.results;s=json.loads((p/'summary.json').read_text())
    assert (p/'exit_code').read_text().strip()=='0' and s['complete_source']
    assert sha(args.source)==(p/'board_source_sha256.txt').read_text().split()[0], 'board/source hash mismatch'
    rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader((p/'observations.csv').open())]
    integrated_prediction=bool(args.predict and rows and 'output_valid' in rows[0])
    if integrated_prediction:
        for r in rows:
            r['smooth_valid']=r['output_valid'];r['smooth_reset']=r['output_reacquired']
            r['smooth_x']=r['output_x'];r['smooth_y']=r['output_y']
    n=s['processed_frames'];assert len(rows)==n
    obs=[];negative=0
    for i,r in enumerate(rows):
        assert r['sequence']==i
        if i:assert r['wall_end_ms']>rows[i-1]['wall_end_ms']
        if args.smooth:
            if args.predict:
                assert not r['output_predicted'] or (r['smooth_valid'] and 0<r['observation_age_ms']<=60)
                assert not (r['smooth_valid'] and not r['output_predicted']) or r['observed']
                if r['result_accepted'] and not r['raw_model_valid'] and 'hard_reject' not in r:assert not r['smooth_valid']
            else:assert r['smooth_valid']==r['observed']
            assert np.isfinite(r['smooth_x']) and np.isfinite(r['smooth_y'])
        if r['observed']:
            assert r['confirmation_hits']>=2 and r['x']==r['measured_x'] and r['y']==r['measured_y']
            assert 0<=r['x']<1344 and 0<=r['y']<760
            assert -1<=1000000+round(i*1000000/180)-r['last_positive']<=100001
            obs.append(i)
        if r['result_accepted'] and not r['raw_model_valid']:
            if r.get('hard_reject',1):assert not r['observed']
            negative+=1
    assert len(obs)==s['observed_frames']
    gaps=[(b-a)/180 for a,b in zip(obs,obs[1:])]
    runs=[];start=None
    for i,r in enumerate(rows+[{'observed':1}]):
        if not r['observed'] and start is None:start=i
        elif r['observed'] and start is not None:runs.append({'start_s':start/180,'end_s':i/180,'duration_s':(i-start)/180});start=None
    audit={'frames':n,'observed_frames':len(obs),'observation_fraction':len(obs)/n,'accepted_negatives':negative,
           'max_source_observation_gap_s':max(gaps,default=0),'longest_no_output_runs':sorted(runs,key=lambda r:r['duration_s'],reverse=True)[:20],
           'accuracy_validated':False,'note':'No-output intervals can include absence of a target; these are not ground-truth missed detections.',
           'source_sha256':sha(args.source),'csv_sha256':sha(p/'observations.csv'),
           'offline_processed_hz':n*1000/s['wall_ms'],'offline_observation_hz':len(obs)*1000/s['wall_ms']}
    (p/'audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    cap=cv2.VideoCapture(str(args.source));assert cap.isOpened()
    assert int(cap.get(cv2.CAP_PROP_FRAME_COUNT))==n and abs(cap.get(cv2.CAP_PROP_FPS)-180)<.001
    tmp=args.output.with_name(args.output.stem+'.partial.mp4')
    assert not args.output.exists() and not tmp.exists()
    header=Image.new('RGB',(1344,104));d=ImageDraw.Draw(header)
    text(d,(12,4),('完整原片 | 流程修复：否定复核 + 原区域恢复 + 短时预测' if integrated_prediction else '完整原片 | 水平运动优先 + 短时预测 + 跳点拦截' if args.predict else '完整原片 | 卡尔曼平滑中心与原始中心对比' if args.smooth else '完整原片 | 绿灯中心定位 + 独立模型线程 | 板端逐帧离线回放'),25)
    text(d,(12,40),('原片/导出 180 FPS；下方为本轮板端含解码离线吞吐，不是实时相机帧率' if integrated_prediction else '原片/导出 180 FPS；帧率沿用原板端离线回放，滤波额外耗时单独验证' if args.smooth else '原片/导出 180 FPS；下方为含解码的实际离线吞吐，不是实时相机帧率'),22)
    head=np.array(header)[:,:,::-1].copy()
    foot=Image.new('RGB',(1344,34));d=ImageDraw.Draw(foot)
    text(d,(12,5),('绿：测量更新   黄：预测（最多60ms）   蓝：原始测量；预测不算新观测，超时或确认否定后停止' if args.predict else '绿：卡尔曼平滑中心   蓝：原始中心   橙：未确认候选；失锁不输出，重新捕获重置；未做精度真值验收' if args.smooth else '绿：当前图像有效中心   橙：尚未确认候选   无有效中心时不沿用旧坐标；未进行人工真值验收'),19)
    foot=np.array(foot)[:,:,::-1].copy()
    writer=subprocess.Popen(['ffmpeg','-v','error','-nostdin','-f','rawvideo','-pix_fmt','bgr24','-s','1344x760','-r','180','-i','-',
        '-i',str(args.source),'-map','0:v:0','-map','1:a?','-c:v','libx264','-threads','4','-preset','fast','-crf','20',
        '-pix_fmt','yuv420p','-c:a','copy','-movflags','+faststart',str(tmp)],stdin=subprocess.PIPE)
    window=deque();valid_count=0;last_model='NPU: waiting';hud_y=0
    try:
        for i,r in enumerate(rows):
            ok,frame=cap.read();assert ok and frame.shape[:2]==(760,1344)
            valid=bool(r['smooth_valid'] if args.predict else r['observed'])
            predicted=args.predict and bool(r['output_predicted'])
            color=(30,220,255) if predicted else (70,245,90) if valid else (20,170,255)
            if valid or (not args.predict and r['measured_size']>0 and r['confirmation_hits']>0):
                x,y=round(r['measured_x']),round(r['measured_y']);radius=max(6,round(r['measured_size']/2))
                if args.smooth and valid:
                    raw=(x,y);x,y=round(r['smooth_x']),round(r['smooth_y'])
                    if r['observed']:
                        cv2.drawMarker(frame,raw,(255,160,30),cv2.MARKER_TILTED_CROSS,16,1,cv2.LINE_AA)
                        cv2.line(frame,raw,(x,y),(255,160,30),1,cv2.LINE_AA)
                if args.predict:radius=12  # fixed reticle, not a diameter estimate
                cv2.circle(frame,(x,y),radius,color,2,cv2.LINE_AA)
                cv2.drawMarker(frame,(x,y),color,cv2.MARKER_CROSS,20,2,cv2.LINE_AA)
                cv2.putText(frame,('PREDICTED' if predicted else 'FILTERED' if args.predict else 'KALMAN' if args.smooth else 'GREEN') if valid else 'UNCONFIRMED',(min(x+12,1180),max(120,y-12)),cv2.FONT_HERSHEY_SIMPLEX,.55,color,2,cv2.LINE_AA)
            if r['model_ran']:
                state=('YES' if r['raw_model_valid'] else 'NO') if r['result_accepted'] else 'STALE / REJECTED'
                last_model=f"NPU: {state} | {r['model_ms']:.1f}ms"
            if args.predict:last_model=f"Observation age: {r['observation_age_ms']:.1f} ms"
            window.append((i,r));valid_count+=int(r['observed'])
            if len(window)>180:valid_count-=int(window.popleft()[1]['observed'])
            first=window[0][0];wall_before=rows[first-1]['wall_end_ms'] if first else 0
            seconds=max(.000001,(r['wall_end_ms']-wall_before)/1000)
            process_hz=len(window)/seconds;obs_hz=valid_count/seconds
            if integrated_prediction and valid:
                if r['smooth_y']<140:hud_y=622
                elif r['smooth_y']>180:hud_y=0
            frame[hud_y:hud_y+104]=cv2.addWeighted(frame[hud_y:hud_y+104],.22,head,1,0)
            frame[-34:]=cv2.addWeighted(frame[-34:],.22,foot,1,0)
            label=f"Offline processed: {process_hz:.1f} FPS | valid observations: {obs_hz:.1f} Hz | {'PREDICTED' if predicted else 'VALID' if valid else 'NO VALID CENTER'}"
            cv2.putText(frame,label,(12,hud_y+90),cv2.FONT_HERSHEY_SIMPLEX,.50,color,1,cv2.LINE_AA)
            cv2.putText(frame,f'{i/180:05.2f}s / {n/180:.2f}s | #{i+1}/{n}',(1000,hud_y+25),cv2.FONT_HERSHEY_SIMPLEX,.43,(245,245,245),1,cv2.LINE_AA)
            cv2.putText(frame,last_model,(890,hud_y+90),cv2.FONT_HERSHEY_SIMPLEX,.43,(240,210,130),1,cv2.LINE_AA)
            writer.stdin.write(frame.tobytes())
            if i in (2767,2827,2857,4681,5401,6409,8701,11161,11887):cv2.imwrite(str(p/f'preview_{i}.jpg'),frame)
            if (i+1)%1800==0:print(f'rendered {i+1}/{n}',flush=True)
        assert not cap.read()[0]
        writer.stdin.close();assert writer.wait()==0
    except BaseException:
        writer.kill();writer.wait();raise
    finally:cap.release()
    probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-count_frames','-select_streams','v:0','-show_entries','stream=width,height,r_frame_rate,nb_read_frames,duration','-of','json',str(tmp)]))['streams'][0]
    assert int(probe['nb_read_frames'])==n and probe['width']==1344 and probe['height']==760 and probe['r_frame_rate']=='180/1'
    assert abs(float(probe['duration'])-n/180)<.001
    tmp.rename(args.output)
    audit['output_filter']='integrated board recovery and bounded prediction' if integrated_prediction else 'bounded predictive center on saved board measurements' if args.predict else 'causal CenterKalman on saved board measurements' if args.smooth else 'none'
    audit['output_sha256']=sha(args.output);audit['output_probe']=probe
    (p/'audit.json').write_text(json.dumps(audit,indent=2)+'\n');print(args.output)
if __name__=='__main__':main()
