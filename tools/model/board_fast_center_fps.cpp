// Experimental source-native green-center loop with periodic model confirmation.
// No actuator output, armor or pose; candidates and observations are separately counted.
#define main unchanged_nv21_benchmark_main
#include "../video_replay/board_nv21_replay.cpp"
#undef main
#include "dart/fast_center_measurement.hpp"

int main(int argc,char**argv) {
 try {
  if(argc!=7)throw std::runtime_error("usage: board_fast_center_fps VIDEO CONFIG START SECONDS OUTPUT.csv SUMMARY.json");
  if(std::filesystem::exists(argv[5])||std::filesystem::exists(argv[6]))throw std::runtime_error("refusing overwrite");
  Options o;o.input=argv[1];o.config=argv[2];o.start_seconds=std::stod(argv[3]);o.seconds=std::stod(argv[4]);o.cache_frames=90;
  if(!std::isfinite(o.seconds)||o.seconds<5||o.seconds>60||!std::isfinite(o.start_seconds)||o.start_seconds<0)throw std::runtime_error("invalid bounds");
  cv::setNumThreads(1);av_log_set_level(AV_LOG_ERROR);
  if(proc_kib("/proc/meminfo","MemAvailable:")*1024<o.cache_frames*kFrameBytes+64*1024*1024)throw std::runtime_error("insufficient RAM");
  Decoder decoder;decoder.open(o);std::vector<CachedFrame> cache;cache.reserve(o.cache_frames);
  while(cache.size()<o.cache_frames) {
   DecodeTiming t;if(!decoder.next(t))throw std::runtime_error("cache EOF");
   const auto source_us=av_rescale_q(decoder.frame->best_effort_timestamp-decoder.stream_start,decoder.timebase,AVRational{1,1000000});
   if(source_us<std::llround(o.start_seconds*1000000))continue;
   CachedFrame f;f.data.resize(kFrameBytes);f.pts=decoder.frame->best_effort_timestamp;f.timestamp_us=source_us;decoder.convert(f);cache.push_back(std::move(f));
  }
  auto cfg=dart::load_application_config(o.config);
  if(!cfg.npu.enabled||!cfg.npu.center_only)throw std::runtime_error("requires deployed V4 center model");
  auto model=dart::create_yolo_pose_validator(cfg.npu);
  auto view=[](const CachedFrame&f){return dart::Nv21View{f.data.data(),f.data.data()+kWidth*kHeight,kWidth,kHeight,kWidth,kWidth};};
  auto measure=[](const dart::Nv21View&v,float x,float y,float gate) {
   const auto roi=dart::source_roi(x,y,128,kWidth,kHeight);auto im=dart::nv21_rgb_region(v,roi);
   auto m=dart::measure_fast_green(static_cast<const uint8_t*>(im->data()),roi.width,roi.height,x-roi.x,y-roi.y,gate);
   if(m.valid){m.x+=roi.x;m.y+=roi.y;}return m;
  };
  struct Row {uint64_t seq;double end,work,age,model_ms;bool observed,model_ran,model_valid,search;float x,y,size,score;int hits;float measured_x,measured_y,measured_size,model_x,model_y;bool raw_model_valid;};
  std::vector<Row> rows;rows.reserve(static_cast<size_t>(o.seconds*180)+1);
  bool tracked=false;int hits=0;float x=0,y=0,size=8,vx=0,vy=0;uint64_t last_measurement=0,last_positive=0,last_model=0;size_t cursor=0;
  const auto begin=Clock::now();int64_t last=-1;
  while(elapsed_ms(begin)<o.seconds*1000) {
   int64_t seq=static_cast<int64_t>(elapsed_ms(begin)*.18);
   if(seq<=last){std::this_thread::sleep_until(begin+std::chrono::nanoseconds((last+1)*1000000000LL/180));continue;}
   const auto started=Clock::now();if(elapsed_ms(begin)>=o.seconds*1000)break;
   const uint64_t ts=1000000+seq*1000000ULL/180;const auto v=view(cache[seq%cache.size()]);
   bool model_ran=false,model_valid=false,search=false;double model_ms=0;float score=0,model_x=0,model_y=0;bool raw_model_valid=false;
   dart::FastCenterMeasurement m;
   bool was_tracked=tracked;
   const float dt=last_measurement?std::min(.1F,(ts-last_measurement)/1000000.0F):0;
   if(tracked && ts-last_positive<=100000) {
    // Prediction only places the ROI. Every accepted output still comes from
    // current pixels; never report this predicted anchor as a measurement.
    const float px=x+(size>=16?std::clamp(vx*dt,-48.0F,48.0F):0),py=y+(size>=16?std::clamp(vy*dt,-48.0F,48.0F):0);
    m=measure(v,px,py,std::clamp(size,16.0F,48.0F));
    if(m.valid&&size>=12&&(m.diameter<size*.5F||m.diameter>size*2.0F))m.valid=false;
   }
   if(!m.valid){tracked=false;hits=0;was_tracked=false;}
   const bool due=!last_model || ts-last_model>=33000;
   if(due) {
    if(!tracked) {
     search=true;auto candidates=dart::nv21_green_proposals(v);
     if(!candidates.empty()) {auto p=candidates[cursor++%candidates.size()];m=measure(v,p.x,p.y,48);}
    }
    if(m.valid) {
     const auto roi=dart::source_roi(m.x,m.y,(std::clamp(static_cast<int>(m.diameter*8),64,384)+1)/2*2,kWidth,kHeight);
     const auto npu_start=Clock::now();const int step=roi.width>=256?2:1;auto rgb=dart::nv21_rgb_region(v,roi,step);
     auto p=model->validate(*rgb,{0,0,rgb->width(),rgb->height()});model_ms=elapsed_ms(npu_start);model_ran=true;last_model=ts;score=p.confidence;
     raw_model_valid=p.valid;model_x=p.keypoints[0].x*step+roi.x;model_y=p.keypoints[0].y*step+roi.y;
     if(p.valid) {
      const float px=p.keypoints[0].x*step+roi.x,py=p.keypoints[0].y*step+roi.y;
      model_valid=std::hypot(px-m.x,py-m.y)<=std::max(12.0F,m.diameter);
      if(model_valid) {auto aligned=measure(v,px,py,std::max(12.0F,m.diameter));if(aligned.valid)m=aligned;else model_valid=false;}
     }
     if(model_valid){++hits;last_positive=ts;tracked=true;}else{hits=0;tracked=false;}
    }else last_model=ts;
   }
   if(m.valid&&tracked){
    if(was_tracked&&dt>0){vx=.5F*vx+.5F*std::clamp((m.x-x)/dt,-2000.0F,2000.0F);vy=.5F*vy+.5F*std::clamp((m.y-y)/dt,-2000.0F,2000.0F);}
    else {vx=0;vy=0;}
    x=m.x;y=m.y;size=m.diameter;last_measurement=ts;
   }
   const bool observed=tracked&&m.valid&&hits>=2&&ts-last_positive<=100000;
   const double end=elapsed_ms(begin),work=elapsed_ms(started);
   rows.push_back({static_cast<uint64_t>(seq),end,work,end-seq*1000.0/180,model_ms,observed,model_ran,model_valid,search,x,y,size,score,hits,m.x,m.y,m.diameter,model_x,model_y,raw_model_valid});last=seq;
  }
  std::ofstream csv(argv[5]);csv<<"sequence,end_ms,work_ms,age_ms,model_ms,observed,model_ran,model_valid,search,x,y,size,score,confirmation_hits,measured_x,measured_y,measured_size,model_x,model_y,raw_model_valid\n";
  Samples work,age,valid_age,model_times,gaps;uint64_t count=0,obs=0,calls=0,positive=0;double previous=0;
  for(const auto&r:rows) {
   csv<<std::fixed<<std::setprecision(6)<<r.seq<<','<<r.end<<','<<r.work<<','<<r.age<<','<<r.model_ms<<','<<r.observed<<','<<r.model_ran<<','<<r.model_valid<<','<<r.search<<','<<r.x<<','<<r.y<<','<<r.size<<','<<r.score<<','<<r.hits<<','<<r.measured_x<<','<<r.measured_y<<','<<r.measured_size<<','<<r.model_x<<','<<r.model_y<<','<<r.raw_model_valid<<'\n';
   if(r.end>=3000&&r.end<(o.seconds-.5)*1000){++count;obs+=r.observed;calls+=r.model_ran;positive+=r.model_valid;work.add(r.work);age.add(r.age);if(r.model_ran)model_times.add(r.model_ms);
    if(r.observed){valid_age.add(r.age);if(previous)gaps.add(r.end-previous);previous=r.end;}}
  }
  csv.close();if(!csv)throw std::runtime_error("CSV write failed");double duration=o.seconds-3.5;
  std::ofstream out(argv[6]);out<<std::fixed<<std::setprecision(6)<<"{\"prototype\":\"native_model_confirmed_local_center_v4\",\"start_seconds\":"<<o.start_seconds<<",\"feed_hz\":180,\"cache_frames\":90,\"window_seconds\":"<<duration
   <<",\"processed_hz\":"<<count/duration<<",\"observed_hz\":"<<obs/duration<<",\"model_hz\":"<<calls/duration<<",\"positive_model_hz\":"<<positive/duration
   <<",\"work\":"<<work.json()<<",\"age\":"<<age.json()<<",\"observed_age\":"<<valid_age.json()<<",\"observed_gap\":"<<gaps.json()<<",\"model\":"<<model_times.json()
   <<",\"prediction_counted\":false,\"armor_included\":false,\"motion_included\":false,\"camera_included\":false,\"accuracy_validated\":false}";
  out.close();if(!out)throw std::runtime_error("summary write failed");std::cout<<"complete rows="<<rows.size()<<" observed_hz="<<obs/duration<<std::endl;return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}
}
