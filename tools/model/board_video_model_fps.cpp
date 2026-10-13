// Cached 180 Hz RGB video replay using the real application and Maix NPU.
// Decode/resize precede timing; this does not exercise VIN, ISP or the NV21 pipeline.
#include "dart/green_detector.hpp"
#include "dart/target_json.hpp"
#include "dart/visual_motion.hpp"
#include "maix_image.hpp"
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libswscale/swscale.h>
}
#include <opencv2/core.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
double elapsed_ms(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
struct Options { std::string input; int decode_threads=1; };
void av_check(int result, const char *where) {
    if(result >= 0) return;
    char message[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(result, message, sizeof(message));
    throw std::runtime_error(std::string(where) + ": " + message);
}
struct DecodeTiming { double demux = 0, decode = 0, total = 0; };
class Decoder {
public:
    AVFormatContext *format = nullptr;
    AVCodecContext *codec = nullptr;
    AVPacket *packet = nullptr;
    AVFrame *frame = nullptr;
    SwsContext *conversion = nullptr;
    int stream = -1;
    bool flushed = false;
    double fps = 0;
    std::string codec_name;
    ~Decoder() {
        sws_freeContext(conversion);
        av_frame_free(&frame); av_packet_free(&packet);
        avcodec_free_context(&codec); avformat_close_input(&format);
    }
    void open(const Options &options) {
        av_check(avformat_open_input(&format, options.input.c_str(), nullptr, nullptr), "open video");
        av_check(avformat_find_stream_info(format, nullptr), "read stream information");
        stream = av_find_best_stream(format, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
        av_check(stream, "find video stream");
        const auto *parameters = format->streams[stream]->codecpar;
        const AVCodec *implementation = avcodec_find_decoder(parameters->codec_id);
        if(!implementation) throw std::runtime_error("software decoder unavailable");
        codec_name = implementation->name;
        codec = avcodec_alloc_context3(implementation);
        if(!codec) throw std::bad_alloc();
        av_check(avcodec_parameters_to_context(codec, parameters), "copy codec parameters");
        codec->thread_count = options.decode_threads;
        codec->err_recognition |= AV_EF_EXPLODE;
        av_check(avcodec_open2(codec, implementation, nullptr), "open software decoder");
        packet = av_packet_alloc(); frame = av_frame_alloc();
        if(!packet || !frame) throw std::bad_alloc();
        fps = av_q2d(av_guess_frame_rate(format, format->streams[stream], nullptr));
        if(!std::isfinite(fps) || fps <= 0) throw std::runtime_error("input has no valid nominal frame rate");
    }
    bool next(DecodeTiming &timing) {
        const auto start = Clock::now();
        while(true) {
            auto stage = Clock::now();
            const int received = avcodec_receive_frame(codec, frame);
            timing.decode += elapsed_ms(stage);
            if(received == 0) {
                if((frame->flags & AV_FRAME_FLAG_CORRUPT) || frame->decode_error_flags)
                    throw std::runtime_error("decoder reported corrupt/concealed source frame");
                timing.total = elapsed_ms(start); return true;
            }
            if(received == AVERROR_EOF) { timing.total = elapsed_ms(start); return false; }
            av_check(received == AVERROR(EAGAIN) ? 0 : received, "receive decoded frame");
            if(flushed) throw std::runtime_error("decoder requested input after EOF flush");
            while(true) {
                stage = Clock::now();
                const int read = av_read_frame(format, packet);
                timing.demux += elapsed_ms(stage);
                if(read == AVERROR_EOF) {
                    stage = Clock::now();
                    av_check(avcodec_send_packet(codec, nullptr), "flush software decoder");
                    timing.decode += elapsed_ms(stage); flushed = true; break;
                }
                av_check(read, "read encoded packet");
                if(packet->stream_index != stream) { av_packet_unref(packet); continue; }
                stage = Clock::now();
                const int sent = avcodec_send_packet(codec, packet);
                timing.decode += elapsed_ms(stage);
                av_packet_unref(packet); av_check(sent, "send encoded packet"); break;
            }
        }
    }
    void rgb(cv::Mat &output) {
        conversion = sws_getCachedContext(conversion, frame->width, frame->height,
            static_cast<AVPixelFormat>(frame->format), output.cols, output.rows,
            AV_PIX_FMT_RGB24, SWS_BILINEAR, nullptr, nullptr, nullptr);
        if(!conversion) throw std::runtime_error("cannot create RGB converter");
        int matrix=SWS_CS_ITU601;
        switch(frame->colorspace) {
        case AVCOL_SPC_BT709: matrix=SWS_CS_ITU709; break;
        case AVCOL_SPC_BT470BG: case AVCOL_SPC_SMPTE170M: break;
        default: throw std::runtime_error("unsupported/unlabelled video color matrix");
        }
        if(frame->color_range!=AVCOL_RANGE_JPEG && frame->color_range!=AVCOL_RANGE_MPEG)
            throw std::runtime_error("unlabelled video color range");
        av_check(sws_setColorspaceDetails(conversion, sws_getCoefficients(matrix),
            frame->color_range==AVCOL_RANGE_JPEG, sws_getCoefficients(matrix), 1,
            0, 1<<16, 1<<16), "configure RGB colorspace");
        uint8_t *dest[] = {output.data, nullptr, nullptr, nullptr};
        int stride[] = {static_cast<int>(output.step[0]), 0, 0, 0};
        const int converted = sws_scale(conversion, frame->data, frame->linesize, 0,
                                         frame->height, dest, stride);
        if(converted != output.rows) throw std::runtime_error("incomplete RGB conversion");
    }
};
struct Samples {
    std::vector<double> values;
    double sum = 0;
    void add(double ms) { values.push_back(ms); sum += ms; }
    std::string json() const {
        auto sorted = values; std::sort(sorted.begin(), sorted.end());
        const auto percentile = [&](double q) {
            if(sorted.empty()) return 0.0;
            const double position = (sorted.size() - 1) * q;
            const auto index = static_cast<size_t>(position);
            return sorted[index] + (sorted[std::min(index + 1, sorted.size() - 1)] - sorted[index]) * (position - index);
        };
        std::ostringstream out; out << std::fixed << std::setprecision(6)
            << "{\"count\":" << values.size() << ",\"total_ms\":" << sum
            << ",\"mean_ms\":" << (values.empty() ? 0 : sum / values.size())
            << ",\"p50_ms\":" << percentile(0.5) << ",\"p95_ms\":" << percentile(0.95)
            << ",\"max_ms\":" << (sorted.empty() ? 0 : sorted.back()) << '}';
        return out.str();
    }
};
} // namespace
#include <thread>
#include <memory>

// This wrapper counts real validate() calls, including crop/resize, forward and
// postprocessing. Predicted outputs and cached validation never increment it.
struct Meter : dart::TargetPoseValidator {
    std::shared_ptr<dart::TargetPoseValidator> inner;
    uint64_t calls=0, positives=0;
    double last_ms=0;
    dart::PoseValidation last_pose;
    dart::CandidateRoi last_roi;
    dart::PoseValidation validate(maix::image::Image &im,const dart::CandidateRoi &roi) override {
        auto start=Clock::now(); auto r=inner->validate(im,roi);
        last_ms=elapsed_ms(start); last_pose=r; last_roi=roi; ++calls; positives+=r.valid; return r;
    }
};
int main(int argc,char **argv) {
    try {
        if(argc!=7 && argc!=8) throw std::runtime_error("usage: board_video_model_fps VIDEO CONFIG START_SECONDS SECONDS INTERVAL_MS OUTPUT.csv [current|integral] (interval -1 disables NPU)");
        const double start_seconds=std::stod(argv[3]), seconds=std::stod(argv[4]);
        const int interval=std::stoi(argv[5]);
        if(!std::isfinite(start_seconds)||start_seconds<0||!std::isfinite(seconds)||seconds<5||seconds>60||interval < -1||interval>1000)
            throw std::runtime_error("invalid bounded test parameters");
        if(std::filesystem::exists(argv[6])) throw std::runtime_error("refusing to overwrite output");
        const std::string profile=argc==8?argv[7]:"current";
        if(profile!="current" && profile!="integral") throw std::runtime_error("invalid profile");
        auto cfg=dart::load_application_config(argv[2]);
        if(profile=="integral") cfg.detector.integral_peak_statistics=true;
        if(cfg.camera.width!=480||cfg.camera.height!=360||!cfg.npu.center_only)
            throw std::runtime_error("this benchmark requires deployed 480x360 center-only config");
        cfg.npu.enabled=interval>=0;
        if(cfg.npu.enabled) cfg.npu.interval_ms=interval;
        cv::setNumThreads(1); av_log_set_level(AV_LOG_ERROR);
        Decoder decoder; Options o; o.input=argv[1]; decoder.open(o);
        if(std::abs(decoder.fps-180)>0.1) throw std::runtime_error("video must be nominal 180 fps");
        auto *stream=decoder.format->streams[decoder.stream];
        const auto origin=stream->start_time==AV_NOPTS_VALUE ? 0 : stream->start_time;
        const int64_t seek=origin+std::llround(start_seconds/av_q2d(stream->time_base));
        av_check(av_seek_frame(decoder.format,decoder.stream,seek,AVSEEK_FLAG_BACKWARD),"seek");
        avcodec_flush_buffers(decoder.codec);
        std::vector<std::unique_ptr<maix::image::Image>> cache;
        std::vector<int64_t> pts;
        while(cache.size()<180) {
            DecodeTiming t; if(!decoder.next(t)) break;
            auto p=decoder.frame->best_effort_timestamp;
            if(p==AV_NOPTS_VALUE) throw std::runtime_error("missing source PTS");
            if(p<seek) continue;
            auto image=std::make_unique<maix::image::Image>(480,360,maix::image::FMT_RGB888);
            cv::Mat rgb(360,480,CV_8UC3,image->data()); decoder.rgb(rgb);
            cache.push_back(std::move(image)); pts.push_back(p);
        }
        if(cache.size()!=180) throw std::runtime_error("need 180 cached frames");
        for(size_t i=1;i<pts.size();++i) {
            double dt=(pts[i]-pts[i-1])*av_q2d(stream->time_base);
            if(dt<=0||std::abs(dt-1.0/180)>0.001) throw std::runtime_error("source PTS cadence is not continuous 180 fps");
        }
        auto meter=std::make_shared<Meter>();
        if(cfg.npu.enabled) meter->inner=dart::create_yolo_pose_validator(cfg.npu);
        dart::GreenLightDetector detector(cfg.detector,cfg.armor,cfg.target_geometry,cfg.npu,
                                          cfg.npu.enabled?meter:nullptr);
        dart::VisualMotionEstimator motion(cfg.visual_motion);
        auto previous=dart::TrackState::Lost;
        struct Row { uint64_t seq,cache_index,calls,positives; double start,end,work,model,motion,detect; bool classical,armor,observed; dart::GreenLightDetection green; dart::CandidateRoi roi; dart::PoseValidation pose; };
        std::vector<Row> rows; rows.reserve(static_cast<size_t>(seconds*180)+1);
        const auto begin=Clock::now();
        int64_t last=-1;
        // A virtual latest-frame source at 180 Hz: never replay overdue frames
        // in a catch-up burst. The modulo boundary is a synthetic video loop.
        while(elapsed_ms(begin)<seconds*1000) {
            int64_t seq=static_cast<int64_t>(elapsed_ms(begin)*0.18);
            if(seq<=last) {
                std::this_thread::sleep_until(begin+std::chrono::nanoseconds((last+1)*1000000000LL/180));
                continue;
            }
            const double started=elapsed_ms(begin);
            if(started>=seconds*1000) break;
            auto &im=*cache[seq%cache.size()];
            uint64_t timestamp=1000000ULL+seq*1000000ULL/180;
            dart::MotionPrior prior;
            bool allowed=!cfg.visual_motion.tracking_only||previous==dart::TrackState::Tracking;
            if(!allowed) motion.reset();
            if(cfg.visual_motion.enabled&&allowed&&rows.size()%std::max(1,cfg.visual_motion.interval_frames)==0)
                prior=motion.update(im,timestamp);
            const double motion_ms=elapsed_ms(begin)-started;
            auto detect_start=Clock::now();
            auto before=meter->calls, positive_before=meter->positives;
            auto target=detector.process(im,timestamp,prior.valid?&prior:nullptr);
            const double detect_ms=elapsed_ms(detect_start);
            previous=target.green.state;
            const double ended=elapsed_ms(begin);
            const auto calls=meter->calls-before;
            if(static_cast<bool>(calls)!=target.model_ran) throw std::runtime_error("model-call accounting mismatch");
            rows.push_back({static_cast<uint64_t>(seq),static_cast<uint64_t>(seq%cache.size()),calls,
                meter->positives-positive_before,started,ended,ended-started,calls?meter->last_ms:0,motion_ms,detect_ms,
                target.classical_detection_ran,target.armor_detection_ran,target.green.valid&&!target.green.predicted,
                target.green,calls?meter->last_roi:dart::CandidateRoi{},calls?meter->last_pose:dart::PoseValidation{}});
            last=seq;
        }
        std::ofstream out(argv[6]);
        out<<"sequence,cache_index,start_ms,end_ms,work_ms,model_ms,model_calls,model_positive,classical,armor,green_observed,motion_ms,detect_ms,green_valid,green_predicted,green_x,green_y,green_size,roi_x,roi_y,roi_w,roi_h,model_valid,model_score,model_x,model_y\n";
        Samples work,model,motion_stats,detect_stats; uint64_t n=0,calls=0,positive=0,classical=0,observed=0;
        for(const auto&r:rows) {
            out<<std::fixed<<std::setprecision(6)<<r.seq<<','<<r.cache_index<<','<<r.start<<','<<r.end<<','<<r.work<<','<<r.model<<','<<r.calls<<','<<r.positives<<','<<r.classical<<','<<r.armor<<','<<r.observed<<','<<r.motion<<','<<r.detect
               <<','<<r.green.valid<<','<<r.green.predicted<<','<<r.green.center_x<<','<<r.green.center_y<<','<<r.green.apparent_size
               <<','<<r.roi.x<<','<<r.roi.y<<','<<r.roi.width<<','<<r.roi.height
               <<','<<r.pose.valid<<','<<r.pose.confidence<<','<<r.pose.keypoints[0].x<<','<<r.pose.keypoints[0].y<<'\n';
            // Same completion-time window for all rates. Exclude startup 3s
            // and final 0.5s; raw rows allow independent recalculation.
            if(r.end>=3000 && r.end<(seconds-0.5)*1000) {
                ++n; calls+=r.calls; positive+=r.positives; classical+=r.classical; observed+=r.observed;
                work.add(r.work); motion_stats.add(r.motion); detect_stats.add(r.detect); if(r.calls) model.add(r.model);
            }
        }
        out.close(); if(!out) throw std::runtime_error("CSV write failed");
        const double window=seconds-3.5;
        std::cout<<std::fixed<<std::setprecision(6)
            <<"{\"feed_hz\":180,\"profile\":\""<<profile<<"\",\"rgb_width\":480,\"rgb_height\":360,\"model_input\":320,\"interval_ms\":"<<interval
            <<",\"video_start_seconds\":"<<start_seconds<<",\"cache_frames\":"<<cache.size()<<",\"seconds\":"<<seconds
            <<",\"window_seconds\":"<<window<<",\"processed\":"<<n<<",\"model_calls\":"<<calls<<",\"model_positive\":"<<positive
            <<",\"processed_fps\":"<<n/window<<",\"classical_fps\":"<<classical/window<<",\"model_fps\":"<<calls/window
            <<",\"green_observed_fps\":"<<observed/window<<",\"work\":"<<work.json()<<",\"model\":"<<model.json()
            <<",\"motion\":"<<motion_stats.json()<<",\"detect\":"<<detect_stats.json()
            <<",\"hardware_capture_included\":false,\"decode_resize_included\":false,\"synthetic_loop\":true}\n";
        return 0;
    } catch(const std::exception&e) { std::cerr<<e.what()<<'\n'; return 1; }
}
