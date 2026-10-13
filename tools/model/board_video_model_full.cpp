// Continuous source-timed offline replay. Every decoded frame is processed;
// this is not the bounded, looping real-time FPS benchmark.
#define main cached_fps_benchmark_main
#include "board_video_model_fps.cpp"
#undef main

int main(int argc,char **argv) {
    try {
        if(argc!=4) throw std::runtime_error("usage: board_video_model_full VIDEO CONFIG OUTPUT.jsonl");
        const std::string output=argv[3];
        if(std::filesystem::exists(output)||std::filesystem::exists(output+".summary.json"))
            throw std::runtime_error("refusing to overwrite results");
        auto cfg=dart::load_application_config(argv[2]);
        if(cfg.camera.width!=480||cfg.camera.height!=360||!cfg.npu.enabled||!cfg.npu.center_only)
            throw std::runtime_error("requires deployed RGB480x360 center-only NPU configuration");
        cv::setNumThreads(1); av_log_set_level(AV_LOG_ERROR);
        Decoder decoder; Options options;options.input=argv[1];decoder.open(options);
        if(std::abs(decoder.fps-180)>0.1) throw std::runtime_error("expected 180fps source");
        auto *stream=decoder.format->streams[decoder.stream];
        const auto expected=stream->nb_frames;
        auto meter=std::make_shared<Meter>();meter->inner=dart::create_yolo_pose_validator(cfg.npu);
        dart::GreenLightDetector detector(cfg.detector,cfg.armor,cfg.target_geometry,cfg.npu,meter);
        dart::VisualMotionEstimator motion(cfg.visual_motion);
        auto previous=dart::TrackState::Lost;
        std::ofstream out(output); if(!out) throw std::runtime_error("cannot open JSONL");
        maix::image::Image image(480,360,maix::image::FMT_RGB888);
        cv::Mat rgb(360,480,CV_8UC3,image.data());
        uint64_t n=0; int64_t first_pts=AV_NOPTS_VALUE,last_pts=AV_NOPTS_VALUE;
        const auto begin=Clock::now(); double work_total=0;
        while(true) {
            DecodeTiming timing;if(!decoder.next(timing)) break;
            const auto pts=decoder.frame->best_effort_timestamp;
            if(pts==AV_NOPTS_VALUE) throw std::runtime_error("missing PTS");
            if(first_pts==AV_NOPTS_VALUE) first_pts=pts;
            if(last_pts!=AV_NOPTS_VALUE && std::abs((pts-last_pts)*av_q2d(stream->time_base)-1.0/180)>0.001)
                throw std::runtime_error("discontinuous source PTS");
            last_pts=pts;
            decoder.rgb(rgb);
            const uint64_t source_us=std::llround((pts-first_pts)*av_q2d(stream->time_base)*1000000);
            const uint64_t timestamp=1000000+source_us;
            auto started=Clock::now(); dart::MotionPrior prior;
            bool allowed=!cfg.visual_motion.tracking_only||previous==dart::TrackState::Tracking;
            if(!allowed) motion.reset();
            if(cfg.visual_motion.enabled&&allowed&&n%std::max(1,cfg.visual_motion.interval_frames)==0)
                prior=motion.update(image,timestamp);
            auto before=meter->calls;
            auto result=detector.process(image,timestamp,prior.valid?&prior:nullptr);
            double work=elapsed_ms(started);work_total+=work;
            previous=result.green.state;
            if(result.model_ran!=(meter->calls>before)) throw std::runtime_error("model call mismatch");
            result.safe_for_control=false; result.angles_valid=false; result.pose.valid=false;
            std::ostringstream extra;
            extra<<std::fixed<<std::setprecision(6)<<",\"source_frame\":"<<n<<",\"source_time_us\":"<<source_us
                <<",\"pts\":"<<pts<<",\"work_ms\":"<<work<<",\"decode_ms\":"<<timing.total
                <<",\"wall_end_ms\":"<<elapsed_ms(begin)<<",\"offline_replay\":true";
            extra<<",\"npu_current\":{\"ran\":"<<(result.model_ran?"true":"false");
            if(result.model_ran) {
                const auto &p=meter->last_pose;const auto &r=meter->last_roi;
                extra<<",\"valid\":"<<(p.valid?"true":"false")<<",\"score\":"<<p.confidence
                    <<",\"x\":"<<p.keypoints[0].x<<",\"y\":"<<p.keypoints[0].y
                    <<",\"roi\":["<<r.x<<','<<r.y<<','<<r.width<<','<<r.height<<']';
            }
            extra<<'}';
            dart::write_target_estimate_json(out,result,&detector.last_candidates(),extra.str());out<<'\n';
            if(!out) throw std::runtime_error("JSONL write failed");
            ++n;
            if(n%180==0) {out.flush();std::cout<<"processed="<<n<<" source_s="<<source_us/1000000.0<<" wall_s="<<elapsed_ms(begin)/1000<<std::endl;}
        }
        out.close();if(!out||!n||(expected>0&&n!=static_cast<uint64_t>(expected)))
            throw std::runtime_error("incomplete source or output");
        std::ofstream summary(output+".summary.json");
        summary<<std::fixed<<std::setprecision(6)<<"{\"complete_source\":true,\"processed_frames\":"<<n
            <<",\"expected_frames\":"<<expected<<",\"source_fps\":"<<decoder.fps
            <<",\"source_width\":"<<decoder.codec->width<<",\"source_height\":"<<decoder.codec->height
            <<",\"first_pts\":"<<first_pts<<",\"last_pts\":"<<last_pts
            <<",\"duration_seconds\":"<<(last_pts-first_pts)*av_q2d(stream->time_base)+1.0/decoder.fps
            <<",\"work_total_ms\":"<<work_total<<",\"wall_ms\":"<<elapsed_ms(begin)
            <<",\"model_calls\":"<<meter->calls<<",\"model_positives\":"<<meter->positives<<'}';
        summary.close();if(!summary) throw std::runtime_error("summary write failed");
        return 0;
    }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}
}
