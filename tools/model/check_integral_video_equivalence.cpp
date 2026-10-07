// Host differential test of exact pixel sums vs integral sums on a full video.
#include "dart/green_detector.hpp"
#include "dart/target_json.hpp"
#include "dart/visual_motion.hpp"
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
int main(int argc,char**argv) {
 try {
  if(argc!=4) throw std::runtime_error("usage: check_integral_video_equivalence VIDEO CONFIG RESULT.json");
  auto c=dart::load_application_config(argv[2]);c.detector.integral_peak_statistics=false;
  auto fast=c.detector;fast.integral_peak_statistics=true;
  dart::GreenLightDetector a(c.detector,c.armor,c.target_geometry,c.npu),b(fast,c.armor,c.target_geometry,c.npu);
  dart::VisualMotionEstimator motion(c.visual_motion);auto previous=dart::TrackState::Lost;
  cv::setNumThreads(1);cv::VideoCapture cap(argv[1]);if(!cap.isOpened())throw std::runtime_error("video open failed");
  const auto expected=static_cast<uint64_t>(cap.get(cv::CAP_PROP_FRAME_COUNT));const auto fps=cap.get(cv::CAP_PROP_FPS);
  cv::Mat frame,rgb,small;uint64_t n=0;double slow_ms=0,fast_ms=0;
  auto now=[](){return std::chrono::steady_clock::now();};auto ms=[&](auto t){return std::chrono::duration<double,std::milli>(now()-t).count();};
  while(cap.read(frame)) {
   cv::resize(frame,small,{c.camera.width,c.camera.height},0,0,cv::INTER_LINEAR);cv::cvtColor(small,rgb,cv::COLOR_BGR2RGB);
   maix::image::Image im(rgb);const auto ts=1000000+static_cast<uint64_t>(n*1000000/fps);
   dart::MotionPrior prior;bool allowed=!c.visual_motion.tracking_only||previous==dart::TrackState::Tracking;
   if(!allowed)motion.reset();
   if(c.visual_motion.enabled&&allowed&&n%c.visual_motion.interval_frames==0)prior=motion.update(im,ts);
   auto begin=now();auto ra=a.process(im,ts,prior.valid?&prior:nullptr);slow_ms+=ms(begin);
   begin=now();auto rb=b.process(im,ts,prior.valid?&prior:nullptr);fast_ms+=ms(begin);
   ra.classical_detection_ms=rb.classical_detection_ms=0;
   auto sa=dart::target_estimate_json(ra,&a.last_candidates()),sb=dart::target_estimate_json(rb,&b.last_candidates());
   if(sa!=sb) {
    std::ofstream diff(std::string(argv[3])+".difference.jsonl");diff<<sa<<'\n'<<sb<<'\n';
    throw std::runtime_error("differential mismatch at frame "+std::to_string(n));
   }
   previous=ra.green.state;++n;
   if(n%1800==0)std::cout<<"checked="<<n<<std::endl;
  }
  if(!n||n!=expected)throw std::runtime_error("incomplete video");
  std::ofstream out(argv[3]);out<<"{\"frames\":"<<n<<",\"all_results_equal\":true,\"slow_ms\":"<<slow_ms<<",\"fast_ms\":"<<fast_ms<<",\"scope\":\"host RGB detector, common motion prior, NPU validator omitted in both; full result and candidate JSON equality excluding execution time\"}";
  std::cout<<"PASS frames="<<n<<" slow_ms="<<slow_ms<<" fast_ms="<<fast_ms<<std::endl;return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}
}
