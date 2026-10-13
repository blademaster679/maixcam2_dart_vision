#!/usr/bin/env python3
import csv,json,subprocess,argparse
from pathlib import Path
import cv2,numpy as np
from PIL import Image,ImageDraw
from render_video_model_fps import text
from analyze_tracking_recovery import load
ap=argparse.ArgumentParser();ap.add_argument('directory',type=Path);p=ap.parse_args().directory
old=load(Path('artifacts/predictive_center_v2_20261007/observations.csv'))
cap=cv2.VideoCapture('artifacts/video_model_overlay_20261007/board_source.mp4')
out=p/'recovery_comparison_13_15_66_70.mp4';tmp=p/'recovery_comparison.partial.mp4'
assert not out.exists() and not tmp.exists()
head=Image.new('RGB',(1344,100));d=ImageDraw.Draw(head)
text(d,(12,4),'修复前后问题片段对比 | 原画面 / 180 FPS | 板端离线逐帧验证',25)
text(d,(12,39),'紫：修复前   绿：修复后测量更新   黄：修复后预测；不同颜色可能重合',22)
head=np.array(head)[:,:,::-1].copy()
writer=subprocess.Popen(['ffmpeg','-v','error','-nostdin','-f','rawvideo','-pix_fmt','bgr24','-s','1344x760','-r','180','-i','-','-c:v','libx264','-threads','4','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(tmp)],stdin=subprocess.PIPE)
count=0
try:
 for start,lo,hi in ((12,13,15),(65,66,70)):
  new=load(p/f'range{start}.csv');cap.set(cv2.CAP_PROP_POS_FRAMES,lo*180)
  for i in range(lo*180,hi*180):
   ok,f=cap.read();assert ok
   a=old[i];b=new[i-start*180]
   if a['smooth_valid']:cv2.circle(f,(round(a['smooth_x']),round(a['smooth_y'])),17,(220,70,240),2,cv2.LINE_AA)
   color=(20,230,255) if b['output_predicted'] else (50,240,80)
   if b['output_valid']:cv2.drawMarker(f,(round(b['output_x']),round(b['output_y'])),color,cv2.MARKER_CROSS,23,2,cv2.LINE_AA)
   f[:100]=cv2.addWeighted(f[:100],.22,head,1,0)
   state='PREDICTED' if b['output_predicted'] else 'MEASURED' if b['output_valid'] else 'NO OUTPUT'
   cv2.putText(f,f"Source {i/180:.3f}s | before: {'VALID' if a['smooth_valid'] else 'MISSING'} | after: {state}",(12,88),cv2.FONT_HERSHEY_SIMPLEX,.65,color,1,cv2.LINE_AA)
   writer.stdin.write(f.tobytes());count+=1
   if i in (2380,11887,12115):cv2.imwrite(str(p/f'preview_{i}.jpg'),f)
 writer.stdin.close();assert writer.wait()==0
except BaseException:
 writer.kill();writer.wait();raise
finally:cap.release()
probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-count_frames','-select_streams','v:0','-show_entries','stream=nb_read_frames,width,height,r_frame_rate,duration','-of','json',str(tmp)]))['streams'][0]
assert count==1080 and int(probe['nb_read_frames'])==1080 and probe['r_frame_rate']=='180/1'
tmp.rename(out);(p/'VIDEO_CHECK.json').write_text(json.dumps(probe,indent=2)+'\n');print(out)
