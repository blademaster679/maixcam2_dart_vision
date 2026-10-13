// Experimental source-native green-center loop with periodic model confirmation.
// No actuator output, armor or pose; candidates and observations are separately counted.
#define main unchanged_nv21_benchmark_main
#include "../video_replay/board_nv21_replay.cpp"
#undef main
#include "dart/fast_center_measurement.hpp"
#include "dart/latest_worker.hpp"
#include "dart/center_kalman.hpp"
#include "dart/predictive_center.hpp"
#include "dart/model_confirmation_gate.hpp"

int main(int argc,char**argv) {
 try {
  if(argc!=5&&argc!=7)throw std::runtime_error("usage: board_async_center_full VIDEO CONFIG OUTPUT.csv SUMMARY.json");
  if(std::filesystem::exists(argv[3])||std::filesystem::exists(argv[4]))throw std::runtime_error("refusing overwrite");
  Options o;o.input=argv[1];o.config=argv[2];
  const bool ranged=argc==7;double range_seconds=0;
  if(ranged){o.start_seconds=std::stod(argv[5]);range_seconds=std::stod(argv[6]);if(!std::isfinite(o.start_seconds)||!std::isfinite(range_seconds)||o.start_seconds<0||range_seconds<=0||range_seconds>80)throw std::runtime_error("invalid range");}
  cv::setNumThreads(1);av_log_set_level(AV_LOG_ERROR);
  Decoder decoder;decoder.open(o);
  const auto expected=ranged?static_cast<int64_t>(std::llround(range_seconds*180)):decoder.format->streams[decoder.stream]->nb_frames;
  auto cfg=dart::load_application_config(o.config);
  if(!cfg.npu.enabled||!cfg.npu.center_only)throw std::runtime_error("requires deployed V4 center model");
  auto model=dart::create_yolo_pose_validator(cfg.npu);
  auto view=[](const CachedFrame&f){return dart::Nv21View{f.data.data(),f.data.data()+kWidth*kHeight,kWidth,kHeight,kWidth,kWidth};};
  auto measure=[](const dart::Nv21View&v,float x,float y,float gate) {
   const auto roi=dart::source_roi(x,y,128,kWidth,kHeight);auto im=dart::nv21_rgb_region(v,roi);
   auto m=dart::measure_fast_green(static_cast<const uint8_t*>(im->data()),roi.width,roi.height,x-roi.x,y-roi.y,gate);
   if(m.valid){m.x+=roi.x;m.y+=roi.y;}return m;
  };
  struct Job {std::shared_ptr<const CachedFrame> frame; uint64_t generation,ts; dart::FastCenterMeasurement measurement;};
  struct Result {uint64_t generation,ts;bool valid;float score,x,y;double ms;};
  dart::LatestWorker<Job,Result> worker([&](Job job) {
   const auto started=Clock::now();const auto &m=job.measurement;
   const auto roi=dart::source_roi(m.x,m.y,(std::clamp(static_cast<int>(m.diameter*8),64,384)+1)/2*2,kWidth,kHeight);
   const int step=roi.width>=256?2:1;
   auto rgb=dart::nv21_rgb_region(view(*job.frame),roi,step);
   auto p=model->validate(*rgb,{0,0,rgb->width(),rgb->height()});
   const float px=p.keypoints[0].x*step+roi.x,py=p.keypoints[0].y*step+roi.y;
   const bool valid=p.valid&&std::hypot(px-m.x,py-m.y)<=std::max(12.0F,m.diameter);
   return Result{job.generation,job.ts,valid,p.confidence,px,py,elapsed_ms(started)};
  });
  dart::CenterKalman output_filter;
  dart::PredictiveCenter predictive_filter;
  dart::ModelConfirmationGate confirmation_gate;
  uint64_t generation=0,rejected_results=0,last_applied=0,submitted=0;
  struct Row {uint64_t seq;double end,work,age,model_ms;bool observed,model_ran,model_valid,search;float x,y,size,score;int hits;float measured_x,measured_y,measured_size,model_x,model_y;bool raw_model_valid;uint64_t generation,result_generation,result_ts,last_positive;bool accepted;bool hard_reject;dart::SmoothedCenter smooth;dart::PredictiveCenterResult predictive;double wall_end,decode_ms;};
  std::vector<Row> rows;rows.reserve(expected>0?expected:14000);
  bool tracked=false;int hits=0;float x=0,y=0,size=8,vx=0,vy=0;uint64_t last_measurement=0,last_positive=0,last_model=0;size_t cursor=0;float recovery_x=0,recovery_y=0;uint64_t last_confirmed_measurement=0;
  const auto begin=Clock::now();int64_t last=-1;
  double total_decode=0;int64_t first_pts=AV_NOPTS_VALUE;
  while(true) {
   auto decode_start=Clock::now();DecodeTiming timing;
   if(ranged&&last+1>=expected)break;
   if(!decoder.next(timing))break;
   if(ranged&&av_rescale_q(decoder.frame->best_effort_timestamp-decoder.stream_start,decoder.timebase,AVRational{1,1000000})<std::llround(o.start_seconds*1000000))continue;
   auto frame=std::make_shared<CachedFrame>();frame->data.resize(kFrameBytes);decoder.convert(*frame);
   const auto pts=decoder.frame->best_effort_timestamp;
   if(first_pts==AV_NOPTS_VALUE)first_pts=pts;
   const uint64_t source_us=av_rescale_q(pts-first_pts,decoder.timebase,AVRational{1,1000000});
   const int64_t seq=last+1;
   if(std::abs(double(source_us)-seq*1000000.0/180)>2)throw std::runtime_error("discontinuous PTS");
   const double decode_ms=elapsed_ms(decode_start);total_decode+=decode_ms;
   const auto started=Clock::now();
   const uint64_t ts=1000000+source_us;const auto v=view(*frame);
   bool model_ran=false,model_valid=false,search=false;double model_ms=0;float score=0,model_x=0,model_y=0;bool raw_model_valid=false,accepted=false,hard_reject=false;uint64_t result_generation=0,result_ts=0;
   // Results confirm their original image and track, never supply a current position.
   if(auto result=worker.take()) {
    result_generation=result->generation;result_ts=result->ts;
    model_ran=true;model_ms=result->ms;score=result->score;
    model_x=result->x;model_y=result->y;raw_model_valid=result->valid;
    if(result->generation==generation&&tracked&&result->ts>last_applied&&ts>=result->ts&&ts-result->ts<=100000) {
     accepted=true;last_applied=result->ts;model_valid=result->valid;
     hard_reject=confirmation_gate.reject(generation,model_valid,hits>=2);
     if(model_valid){++hits;last_positive=result->ts;}
     else if(hard_reject){tracked=false;hits=0;++generation;}
    } else ++rejected_results;
   }
   dart::FastCenterMeasurement m;
   bool was_tracked=tracked;
   const float dt=last_measurement?std::min(.1F,(ts-last_measurement)/1000000.0F):0;
   if(tracked && (!last_positive || ts-last_positive<=100000)) {
    // Prediction only places the ROI. Every accepted output still comes from
    // current pixels; never report this predicted anchor as a measurement.
    const float px=x+(size>=16?std::clamp(vx*dt,-48.0F,48.0F):0),py=y+(size>=16?std::clamp(vy*dt,-48.0F,48.0F):0);
    m=measure(v,px,py,std::clamp(size,16.0F,48.0F));
    if(m.valid&&size>=12&&(m.diameter<size*.5F||m.diameter>size*2.0F))m.valid=false;
   }
   if(!m.valid){if(tracked)++generation;tracked=false;hits=0;was_tracked=false;last_positive=0;}
   const bool due=!last_model || ts-last_model>=33000;
   if(due) {
    if(!tracked) {
     search=true;
     // Revisit the previous lamp before unrelated global proposals.
     if(last_confirmed_measurement&&ts-last_confirmed_measurement<=100000)m=measure(v,recovery_x,recovery_y,32);
     if(!m.valid){auto candidates=dart::nv21_green_proposals(v);
      if(!candidates.empty()){auto p=candidates[cursor++%candidates.size()];m=measure(v,p.x,p.y,48);}}
    }
    if(m.valid) {
     if(!tracked){++generation;tracked=true;hits=0;last_positive=0;}
     worker.submit(Job{frame,generation,ts,m});++submitted;
    }
    last_model=ts;
   }

   if(m.valid&&tracked){
    if(was_tracked&&dt>0){vx=.5F*vx+.5F*std::clamp((m.x-x)/dt,-2000.0F,2000.0F);vy=.5F*vy+.5F*std::clamp((m.y-y)/dt,-2000.0F,2000.0F);}
    else {vx=0;vy=0;}
    x=m.x;y=m.y;size=m.diameter;last_measurement=ts;
   }
   const bool observed=tracked&&m.valid&&hits>=2&&ts-last_positive<=100000;
   if(observed){recovery_x=x;recovery_y=y;last_confirmed_measurement=ts;}
   const auto smooth=output_filter.update(observed,x,y,size,ts,generation);
   const auto predictive=predictive_filter.update(observed,x,y,size,ts,generation,hard_reject,result_generation);
   const double work=elapsed_ms(started),end=seq*1000.0/180+work;
   rows.push_back({static_cast<uint64_t>(seq),end,work,end-seq*1000.0/180,model_ms,observed,model_ran,model_valid,search,x,y,size,score,hits,m.x,m.y,m.diameter,model_x,model_y,raw_model_valid,generation,result_generation,result_ts,last_positive,accepted,hard_reject,smooth,predictive,elapsed_ms(begin),decode_ms});last=seq;
   if((seq+1)%180==0)std::cout<<"processed="<<seq+1<<" source_s="<<source_us/1000000.0<<" wall_s="<<elapsed_ms(begin)/1000<<std::endl;
  }
  std::ofstream csv(argv[3]);csv<<"sequence,end_ms,work_ms,age_ms,model_ms,observed,model_ran,model_valid,search,x,y,size,score,confirmation_hits,measured_x,measured_y,measured_size,model_x,model_y,raw_model_valid,generation,result_generation,result_ts,last_positive,result_accepted,hard_reject,smooth_valid,smooth_reset,smooth_x,smooth_y,output_valid,output_predicted,jump_rejected,output_reacquired,output_x,output_y,observation_age_ms,wall_end_ms,decode_ms\n";
  Samples work,age,valid_age,model_times,gaps;uint64_t count=0,obs=0,calls=0,positive=0;double previous=0;
  for(const auto&r:rows) {
   csv<<std::fixed<<std::setprecision(6)<<r.seq<<','<<r.end<<','<<r.work<<','<<r.age<<','<<r.model_ms<<','<<r.observed<<','<<r.model_ran<<','<<r.model_valid<<','<<r.search<<','<<r.x<<','<<r.y<<','<<r.size<<','<<r.score<<','<<r.hits<<','<<r.measured_x<<','<<r.measured_y<<','<<r.measured_size<<','<<r.model_x<<','<<r.model_y<<','<<r.raw_model_valid<<','<<r.generation<<','<<r.result_generation<<','<<r.result_ts<<','<<r.last_positive<<','<<r.accepted<<','<<r.hard_reject<<','<<r.smooth.valid<<','<<r.smooth.reset<<','<<r.smooth.x<<','<<r.smooth.y<<','<<r.predictive.valid<<','<<r.predictive.predicted<<','<<r.predictive.rejected<<','<<r.predictive.reacquired<<','<<r.predictive.x<<','<<r.predictive.y<<','<<r.predictive.age_ms<<','<<r.wall_end<<','<<r.decode_ms<<'\n';
   if(true){++count;obs+=r.observed;calls+=r.model_ran;positive+=r.model_valid;work.add(r.work);age.add(r.age);if(r.model_ran)model_times.add(r.model_ms);
    if(r.observed){valid_age.add(r.age);if(previous)gaps.add(r.seq*1000.0/180-previous);previous=r.seq*1000.0/180;}}
  }
  csv.close();if(!csv||rows.empty()||(expected>0&&rows.size()!=static_cast<size_t>(expected)))throw std::runtime_error("incomplete output");
  std::ofstream out(argv[4]);out<<std::fixed<<std::setprecision(6)<<"{\"complete_source\":"<<(ranged?"false":"true")<<",\"complete_range\":true,\"start_seconds\":"<<o.start_seconds<<",\"offline_replay\":true,\"processed_frames\":"<<count
   <<",\"observed_frames\":"<<obs<<",\"source_fps\":180,\"source_width\":1344,\"source_height\":760,\"duration_seconds\":"<<count/180.0
   <<",\"wall_ms\":"<<elapsed_ms(begin)<<",\"decode_ms\":"<<total_decode<<",\"model_results\":"<<calls<<",\"model_positive\":"<<positive
   <<",\"submitted\":"<<submitted<<",\"rejected_results\":"<<rejected_results<<",\"work\":"<<work.json()<<",\"observed_work\":"<<valid_age.json()
   <<",\"source_observed_gaps\":"<<gaps.json()<<",\"model\":"<<model_times.json()<<",\"accuracy_validated\":false,\"camera_included\":false}";
  out.close();if(!out)throw std::runtime_error("summary write failed");return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}
}
