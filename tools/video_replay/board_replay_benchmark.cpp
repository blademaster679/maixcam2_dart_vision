// Offline RGB detector benchmark, not the live VIN/NV21 scheduling pipeline.
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
struct Options {
    std::string input, config, jsonl, summary, first_rgb;
    uint64_t frame_step = 1, max_frames = 0, warmup_frames = 30;
    int decode_threads = 1;
    bool native_resolution = false;
};
uint64_t number(const std::string &value, const std::string &name) {
    if(value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
        throw std::runtime_error(name + " requires an unsigned integer");
    return std::stoull(value);
}
Options parse(int argc, char **argv) {
    Options o;
    for(int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if(key == "--native-resolution") { o.native_resolution = true; continue; }
        if(key == "--no-overlay-video") continue;
        if(key == "--help" || key == "-h") {
            std::cout << "board_replay_benchmark --input VIDEO.mp4 --config FILE.conf "
                "[--jsonl OUTPUT.jsonl] [--summary OUTPUT.json] [--frame-step N] "
                "[--max-frames N] [--warmup-frames N] [--decode-threads N] [--first-rgb FILE.rgb] "
                "[--native-resolution] [--no-overlay-video]\n"
                "No JSONL unless requested. Always diagnostic RGB software-decoder replay.\n"
                "max-frames counts source frames; warmup-frames counts processed frames.\n";
            std::exit(0);
        }
        if(++i >= argc) throw std::runtime_error(key + " requires a value");
        const std::string value = argv[i];
        if(key == "--input") o.input = value;
        else if(key == "--config") o.config = value;
        else if(key == "--jsonl") o.jsonl = value;
        else if(key == "--summary") o.summary = value;
        else if(key == "--first-rgb") o.first_rgb = value;
        else if(key == "--frame-step") o.frame_step = number(value, key);
        else if(key == "--max-frames") o.max_frames = number(value, key);
        else if(key == "--warmup-frames") o.warmup_frames = number(value, key);
        else if(key == "--decode-threads") {
            const auto n = number(value, key);
            if(n == 0 || n > 16) throw std::runtime_error("decode threads must be in 1..16");
            o.decode_threads = static_cast<int>(n);
        } else throw std::runtime_error("unknown argument: " + key);
    }
    if(o.input.empty() || o.config.empty()) throw std::runtime_error("--input and --config required");
    if(!o.frame_step) throw std::runtime_error("--frame-step must be positive");
    const auto same_path = [](const std::string &a, const std::string &b) {
        return !a.empty() && !b.empty() &&
            std::filesystem::weakly_canonical(a) == std::filesystem::weakly_canonical(b);
    };
    if(same_path(o.input,o.jsonl) || same_path(o.input,o.summary) ||
       same_path(o.config,o.jsonl) || same_path(o.config,o.summary) || same_path(o.jsonl,o.summary) ||
       same_path(o.first_rgb,o.input) || same_path(o.first_rgb,o.config) ||
       same_path(o.first_rgb,o.jsonl) || same_path(o.first_rgb,o.summary))
        throw std::runtime_error("output paths must differ from inputs and each other");
    return o;
}
void parent_directory(const std::string &path) {
    if(path.empty()) return;
    auto parent = std::filesystem::path(path).parent_path();
    if(!parent.empty()) std::filesystem::create_directories(parent);
}
std::string json_string(const std::string &value) {
    std::ostringstream out; out << '"';
    for(const unsigned char c : value) {
        if(c == '"' || c == '\\') out << '\\' << c;
        else if(c < 0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
        else out << c;
    }
    out << '"'; return out.str();
}
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
void invalidate(dart::TargetEstimate &target) {
    target.safe_for_control = false; target.angles_valid = false; target.pose.valid = false;
    target.yaw_rad = target.pitch_rad = target.green.yaw_rad = target.green.pitch_rad = 0;
    target.line_of_sight_camera = {0,0,0};
    target.line_of_sight_rate_rad_s = {0,0}; target.angular_covariance = {0,0};
}
void scale_native_config(dart::ApplicationConfig &c, int width, int height) {
    const float x = static_cast<float>(width) / c.camera.width;
    const float y = static_cast<float>(height) / c.camera.height;
    const float spatial = std::sqrt(x * y), variance = x * y;
    auto &d = c.detector;
    d.camera_model.fx *= x; d.camera_model.fy *= y;
    d.camera_model.principal_x *= x; d.camera_model.principal_y *= y;
    d.center_prior_radius_px *= spatial; d.initial_size_reference_px *= spatial;
    d.min_core_size_px *= spatial; d.min_halo_size_px *= spatial;
    d.confirmation_gate_px *= spatial; d.gate_min_px *= spatial; d.gate_max_px *= spatial;
    d.ring_margin_px = std::lround(d.ring_margin_px * spatial);
    d.merge_margin_px = std::lround(d.merge_margin_px * spatial);
    d.process_noise_position *= variance; d.process_noise_velocity *= variance;
    d.measurement_noise_position *= variance;
}
int run(const Options &o) {
    const auto application_start = Clock::now();
    cv::setNumThreads(1);
    av_log_set_level(AV_LOG_ERROR);
    auto config = dart::load_application_config(o.config);
    if(config.npu.enabled)
        throw std::runtime_error("RGB replay benchmark does not implement an NPU validator");
    Decoder decoder; decoder.open(o);
    const int source_width = decoder.codec->width, source_height = decoder.codec->height;
    const int64_t expected_source_frames = decoder.format->streams[decoder.stream]->nb_frames;
    const int width = o.native_resolution ? source_width : config.camera.width;
    const int height = o.native_resolution ? source_height : config.camera.height;
    if(width <= 0 || height <= 0) throw std::runtime_error("invalid processing dimensions");
    if(o.native_resolution) scale_native_config(config, width, height);
    cv::Mat rgb(height, width, CV_8UC3);
    dart::GreenLightDetector detector(config.detector, config.armor, config.target_geometry, config.npu, nullptr);
    dart::VisualMotionEstimator motion(config.visual_motion);
    std::ofstream jsonl;
    if(!o.jsonl.empty()) {
        parent_directory(o.jsonl); jsonl.open(o.jsonl);
        if(!jsonl) throw std::runtime_error("cannot open JSONL output");
    }
    Samples demux, decode, input, convert, detect, motion_stats, processing, json_io;
    uint64_t source_frames = 0, processed = 0, measured = 0, measured_source = 0;
    uint64_t green_observed = 0, green_predicted = 0, armor_observed = 0;
    uint64_t classical_calls = 0, armor_calls = 0, direct_armor = 0;
    auto previous_state = dart::TrackState::Lost;
    auto measured_start = Clock::now();
    const auto loop_start = measured_start;
    bool measured_started = o.warmup_frames == 0, reached_eof = false;
    double first_rgb_dump_ms = 0;
    int source_pixel_format = -1, source_color_range = -1, source_colorspace = -1;
    while(o.max_frames == 0 || source_frames < o.max_frames) {
        const bool measure = processed >= o.warmup_frames;
        if(measure && !measured_started) { measured_start = Clock::now(); measured_started = true; }
        DecodeTiming timing;
        if(!decoder.next(timing)) { reached_eof = true; break; }
        if(decoder.frame->width != source_width || decoder.frame->height != source_height)
            throw std::runtime_error("source resolution changed during replay");
        const uint64_t frame_index = source_frames++;
        if(measure) {
            ++measured_source; demux.add(timing.demux); decode.add(timing.decode); input.add(timing.total);
        }
        if(frame_index % o.frame_step != 0) continue;
        const auto convert_start = Clock::now(); decoder.rgb(rgb);
        maix::image::Image image(rgb);
        const double convert_ms = elapsed_ms(convert_start);
        if(frame_index == 0) {
            source_pixel_format = decoder.frame->format;
            source_color_range = decoder.frame->color_range;
            source_colorspace = decoder.frame->colorspace;
            if(!o.first_rgb.empty()) {
                const auto dump_start = Clock::now();
                parent_directory(o.first_rgb);
                std::ofstream dump(o.first_rgb, std::ios::binary);
                dump.write(reinterpret_cast<const char *>(rgb.data), rgb.total() * rgb.elemSize());
                dump.close(); if(!dump) throw std::runtime_error("first RGB output failed");
                first_rgb_dump_ms = elapsed_ms(dump_start);
            }
        }
        const uint64_t timestamp = std::llround(frame_index * 1000000.0 / decoder.fps);
        const auto motion_start = Clock::now();
        dart::MotionPrior prior;
        const bool motion_allowed = !config.visual_motion.tracking_only || previous_state == dart::TrackState::Tracking;
        if(config.visual_motion.tracking_only && !motion_allowed) motion.reset();
        if(config.visual_motion.enabled && motion_allowed &&
           processed % static_cast<uint64_t>(config.visual_motion.interval_frames) == 0)
            prior = motion.update(image, timestamp);
        const double motion_ms = elapsed_ms(motion_start);
        const auto detect_start = Clock::now();
        auto target = detector.process(image, timestamp, prior.valid ? &prior : nullptr);
        const double detector_ms = elapsed_ms(detect_start);
        invalidate(target); previous_state = target.green.state;
        if(measure) {
            ++measured; convert.add(convert_ms); detect.add(detector_ms); motion_stats.add(motion_ms);
            processing.add(detector_ms + motion_ms);
            green_observed += target.green.valid && !target.green.predicted;
            green_predicted += target.green.valid && target.green.predicted;
            armor_observed += target.armor.valid;
            classical_calls += target.classical_detection_ran;
            armor_calls += target.armor_detection_ran;
            direct_armor += target.armor.valid && target.armor_detection_ran;
        }
        if(jsonl.is_open()) {
            const auto io_start = Clock::now();
            std::ostringstream extra;
            extra << std::fixed << std::setprecision(6) << ",\"frame\":" << frame_index
                << ",\"frame_step\":" << o.frame_step << ",\"warmup\":" << (measure ? "false" : "true")
                << ",\"replay_pipeline\":\"rgb\",\"replay_diagnostic_only\":true"
                << ",\"replay_timestamps\":\"nominal_source_fps\",\"decoder_backend\":\"ffmpeg_software\""
                << ",\"processing_ms\":" << detector_ms + motion_ms << ",\"detector_ms\":" << detector_ms
                << ",\"motion_ms\":" << motion_ms << ",\"convert_ms\":" << convert_ms
                << ",\"decode_demux_ms\":" << timing.total;
            dart::write_target_estimate_json(jsonl, target, &detector.last_candidates(), extra.str());
            jsonl << '\n';
            if(!jsonl) throw std::runtime_error("JSONL output write failed");
            if(!measure && processed + 1 == o.warmup_frames) jsonl.flush();
            if(measure) json_io.add(elapsed_ms(io_start));
        }
        ++processed;
        if(processed % 300 == 0) std::cerr << "processed=" << processed << " source_frames=" << source_frames << '\n';
    }
    double final_json_flush_ms = 0;
    if(jsonl.is_open()) {
        const auto start = Clock::now(); jsonl.flush(); jsonl.close(); final_json_flush_ms = elapsed_ms(start);
        if(!jsonl) throw std::runtime_error("JSONL output flush failed");
    }
    const double measured_wall_ms = measured_started ? elapsed_ms(measured_start) : 0;
    const double loop_wall_ms = elapsed_ms(loop_start), application_wall_ms = elapsed_ms(application_start);
    if(!source_frames) throw std::runtime_error("no source frames decoded");
    if(reached_eof && expected_source_frames > 0 &&
       source_frames != static_cast<uint64_t>(expected_source_frames))
        throw std::runtime_error("decoded source frame count differs from container frame count");
    if(!measured) throw std::runtime_error("no measured frames remain after warmup; reduce --warmup-frames");
    std::ostringstream summary; summary << std::fixed << std::setprecision(6)
        << "{\"input\":" << json_string(o.input) << ",\"config\":" << json_string(o.config)
        << ",\"jsonl\":" << json_string(o.jsonl) << ",\"benchmark\":\"rgb_software_decode_replay\""
        << ",\"replay_pipeline\":\"rgb\",\"replay_diagnostic_only\":true,\"safe_for_control\":false,\"angles_valid\":false"
        << ",\"hardware_capture_included\":false,\"hardware_decode_included\":false,\"overlay_video\":false,\"npu_included\":false"
        << ",\"source_width\":" << source_width << ",\"source_height\":" << source_height
        << ",\"processing_width\":" << width << ",\"processing_height\":" << height
        << ",\"source_fps\":" << decoder.fps << ",\"frame_step\":" << o.frame_step
        << ",\"container_frames\":" << expected_source_frames
        << ",\"source_pixel_format\":" << source_pixel_format
        << ",\"source_color_range\":" << source_color_range << ",\"source_colorspace\":" << source_colorspace
        << ",\"decoder\":" << json_string(decoder.codec_name) << ",\"decode_threads\":" << o.decode_threads
        << ",\"opencv_threads\":1,\"ffmpeg_version\":" << json_string(av_version_info())
        << ",\"opencv_version\":" << json_string(CV_VERSION)
        << ",\"source_frames_read\":" << source_frames << ",\"processed_frames\":" << processed
        << ",\"warmup_processed_frames\":" << std::min(processed,o.warmup_frames)
        << ",\"measured_source_frames\":" << measured_source << ",\"measured_frames\":" << measured
        << ",\"reached_eof\":" << (reached_eof ? "true" : "false")
        << ",\"warmup_updates_tracker\":true,\"jsonl_includes_warmup\":true"
        << ",\"jsonl_serialization_and_writes_in_wall\":" << (!o.jsonl.empty() ? "true" : "false")
        << ",\"output_durability\":\"userspace_flush_not_fsync\""
        << ",\"application_wall_ms\":" << application_wall_ms << ",\"loop_wall_ms\":" << loop_wall_ms
        << ",\"measured_wall_ms\":" << measured_wall_ms
        << ",\"wall_processed_fps\":" << measured * 1000.0 / measured_wall_ms
        << ",\"wall_source_fps\":" << measured_source * 1000.0 / measured_wall_ms
        << ",\"detector_only_fps\":" << measured * 1000.0 / detect.sum
        << ",\"detector_and_motion_fps\":" << measured * 1000.0 / processing.sum
        << ",\"green_observed_frames\":" << green_observed << ",\"green_predicted_frames\":" << green_predicted
        << ",\"armor_frames\":" << armor_observed
        << ",\"classical_detection_calls\":" << classical_calls << ",\"armor_detection_calls\":" << armor_calls
        << ",\"direct_armor_frames\":" << direct_armor
        << ",\"read_demux\":" << demux.json() << ",\"decode\":" << decode.json()
        << ",\"decode_demux\":" << input.json() << ",\"rgb_convert_resize\":" << convert.json()
        << ",\"detector\":" << detect.json() << ",\"visual_motion\":" << motion_stats.json()
        << ",\"detector_and_motion\":" << processing.json() << ",\"json_io\":" << json_io.json()
        << ",\"first_rgb\":" << json_string(o.first_rgb)
        << ",\"first_rgb_dump_ms\":" << first_rgb_dump_ms
        << ",\"final_json_flush_ms\":" << final_json_flush_ms << '}';
    if(!o.summary.empty()) {
        parent_directory(o.summary); std::ofstream file(o.summary); file << summary.str() << '\n';
        file.close(); if(!file) throw std::runtime_error("summary output failed");
    }
    std::cout << summary.str() << '\n'; return 0;
}
} // namespace
int main(int argc,char **argv) {
    try { return run(parse(argc,argv)); }
    catch(const std::exception &error) { std::cerr << "fatal: " << error.what() << '\n'; return 1; }
}
