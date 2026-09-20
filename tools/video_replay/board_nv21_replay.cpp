// Camera-free, bounded cached-video source for the unchanged HighFpsPipeline.
// Decoding is completed before the timed 180 Hz feed; this is not end-to-end
// video decoding throughput and does not reproduce an original VIN pixel stream.
#include "dart/nv21_pipeline.hpp"
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}
#include <opencv2/core.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
constexpr int kWidth = 1344, kHeight = 760, kFeedHz = 180;
constexpr uint64_t kFrameBytes = uint64_t(kWidth) * kHeight * 3 / 2;
constexpr uint64_t kCacheBudget = 160ULL * 1024 * 1024;
volatile std::sig_atomic_t interrupted = 0;
void handle_signal(int) { interrupted = 1; }
uint64_t now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        Clock::now().time_since_epoch()).count();
}
double elapsed_ms(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
std::string quote(const std::string &value) {
    std::ostringstream out; out << '"';
    for(unsigned char c : value) {
        if(c == '"' || c == '\\') out << '\\' << c;
        else if(c < 32) out << "\\u" << std::hex << std::setw(4)
                            << std::setfill('0') << int(c) << std::dec;
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
uint64_t proc_kib(const char *path, const char *field) {
    std::ifstream file(path); std::string line;
    while(std::getline(file, line)) {
        std::istringstream values(line); std::string key; uint64_t kib = 0;
        if(values >> key >> kib && key == field) return kib;
    }
    return 0;
}
struct Options {
    std::string input, config, summary = "replay.json";
    double start_seconds = 0, seconds = 20;
    uint64_t cache_frames = 90;
};
Options parse(int argc, char **argv) {
    Options o;
    for(int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if(key == "--help" || key == "-h") {
            std::cout << "board_nv21_replay --input VIDEO.mp4 --config CONFIG.conf "
                "[--start-seconds 0] [--cache-frames 90] [--seconds 20] "
                "[--summary replay.json]\n"
                "Decode one bounded window, then loop immutable limited-range BT.601 NV21 "
                "frames into HighFpsPipeline at 180 Hz. No camera or NPU.\n"
                "Run in an empty output directory; creates feed.csv, business.json, "
                "vision.csv, motion.csv and targets.jsonl. Maximum 120 seconds, 160 MiB cache.\n";
            std::exit(0);
        }
        if(++i >= argc) throw std::runtime_error(key + " requires a value");
        const std::string value = argv[i]; std::size_t consumed = 0;
        if(key == "--input") o.input = value;
        else if(key == "--config") o.config = value;
        else if(key == "--summary") o.summary = value;
        else if(key == "--start-seconds") {
            o.start_seconds = std::stod(value, &consumed);
            if(consumed != value.size()) throw std::runtime_error("invalid --start-seconds");
        } else if(key == "--seconds") {
            o.seconds = std::stod(value, &consumed);
            if(consumed != value.size()) throw std::runtime_error("invalid --seconds");
        } else if(key == "--cache-frames") {
            if(value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
                throw std::runtime_error("--cache-frames requires an unsigned integer");
            o.cache_frames = std::stoull(value);
        } else throw std::runtime_error("unknown argument: " + key);
    }
    if(o.input.empty() || o.config.empty() || o.summary.empty())
        throw std::runtime_error("--input and --config are required; summary path must be nonempty");
    if(!std::isfinite(o.start_seconds) || o.start_seconds < 0 || o.start_seconds > 86400)
        throw std::runtime_error("--start-seconds must be within 0..86400");
    if(!std::isfinite(o.seconds) || o.seconds <= 0 || o.seconds > 120)
        throw std::runtime_error("--seconds must be within (0,120]");
    if(!o.cache_frames || o.cache_frames > kCacheBudget / kFrameBytes)
        throw std::runtime_error("--cache-frames exceeds the 160 MiB cache budget or is zero");
    const auto summary = std::filesystem::weakly_canonical(o.summary);
    if(summary == std::filesystem::weakly_canonical(o.input) ||
       summary == std::filesystem::weakly_canonical(o.config))
        throw std::runtime_error("summary must differ from input and configuration");
    for(const char *path : {"feed.csv","business.json","vision.csv","motion.csv","targets.jsonl"}) {
        if(std::filesystem::exists(path))
            throw std::runtime_error(std::string("refusing to overwrite existing pipeline output: ") + path);
        if(summary == std::filesystem::weakly_canonical(path))
            throw std::runtime_error("summary path conflicts with a pipeline output");
    }
    if(std::filesystem::exists(o.summary))
        throw std::runtime_error("refusing to overwrite existing replay summary");
    return o;
}
struct Samples {
    std::vector<double> values;
    double total = 0;
    void add(double value) { values.push_back(value); total += value; }
    std::string json() const {
        auto sorted = values; std::sort(sorted.begin(), sorted.end());
        auto quantile = [&](double p) {
            if(sorted.empty()) return 0.0;
            const double index = (sorted.size() - 1) * p;
            const auto low = static_cast<std::size_t>(index);
            return sorted[low] + (sorted[std::min(low + 1, sorted.size() - 1)] - sorted[low]) * (index - low);
        };
        std::ostringstream out; out << std::fixed << std::setprecision(6)
            << "{\"count\":" << values.size() << ",\"total_ms\":" << total
            << ",\"mean_ms\":" << (values.empty() ? 0 : total / values.size())
            << ",\"p50_ms\":" << quantile(0.5) << ",\"p95_ms\":" << quantile(0.95)
            << ",\"max_ms\":" << (sorted.empty() ? 0 : sorted.back()) << '}';
        return out.str();
    }
};
struct DecodeTiming { double read_ms = 0, decode_ms = 0; };
struct CachedFrame {
    std::vector<uint8_t> data;
    int64_t source_frame = 0, pts = 0, timestamp_us = 0;
};
class CachedLease final : public dart::Nv21Frame {
public:
    explicit CachedLease(std::shared_ptr<const CachedFrame> frame) : frame_(std::move(frame)) {}
    dart::Nv21View map() override {
        return {frame_->data.data(), frame_->data.data() + kWidth * kHeight,
                kWidth, kHeight, kWidth, kWidth};
    }
private:
    std::shared_ptr<const CachedFrame> frame_;
};
class Decoder {
public:
    AVFormatContext *format = nullptr;
    AVCodecContext *codec = nullptr;
    AVPacket *packet = nullptr;
    AVFrame *frame = nullptr;
    SwsContext *converter = nullptr;
    int stream = -1;
    bool flushed = false, range_assumed = false, space_assumed = false;
    double fps = 0, seek_ms = 0;
    AVRational timebase{};
    int64_t stream_start = 0;
    bool stream_start_assumed_zero = false;
    std::string codec_name;
    ~Decoder() {
        sws_freeContext(converter); av_frame_free(&frame); av_packet_free(&packet);
        avcodec_free_context(&codec); avformat_close_input(&format);
    }
    void open(const Options &o) {
        av_check(avformat_open_input(&format, o.input.c_str(), nullptr, nullptr), "open video");
        av_check(avformat_find_stream_info(format, nullptr), "read stream information");
        stream = av_find_best_stream(format, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
        av_check(stream, "find video stream");
        const auto *s = format->streams[stream];
        if(s->codecpar->width != kWidth || s->codecpar->height != kHeight)
            throw std::runtime_error("NV21 replay requires native 1344x760 input");
        fps = av_q2d(av_guess_frame_rate(format, format->streams[stream], nullptr));
        if(!std::isfinite(fps) || std::fabs(fps - kFeedHz) > 0.01)
            throw std::runtime_error("NV21 replay requires nominal 180 fps input");
        timebase = s->time_base;
        if(timebase.num <= 0 || timebase.den <= 0) throw std::runtime_error("invalid stream timebase");
        stream_start_assumed_zero = s->start_time == AV_NOPTS_VALUE;
        stream_start = stream_start_assumed_zero ? 0 : s->start_time;
        const AVCodec *implementation = avcodec_find_decoder(s->codecpar->codec_id);
        if(!implementation) throw std::runtime_error("software decoder unavailable");
        codec_name = implementation->name;
        codec = avcodec_alloc_context3(implementation);
        if(!codec) throw std::bad_alloc();
        av_check(avcodec_parameters_to_context(codec, s->codecpar), "copy codec parameters");
        codec->thread_count = 1; codec->err_recognition |= AV_EF_EXPLODE;
        av_check(avcodec_open2(codec, implementation, nullptr), "open software decoder");
        packet = av_packet_alloc(); frame = av_frame_alloc();
        if(!packet || !frame) throw std::bad_alloc();
        if(o.start_seconds > 0) {
            if(stream_start_assumed_zero)
                throw std::runtime_error("nonzero seek requires a known stream start timestamp");
            const int64_t relative = std::llround(o.start_seconds / av_q2d(timebase));
            const auto start = Clock::now();
            av_check(av_seek_frame(format, stream, stream_start + relative, AVSEEK_FLAG_BACKWARD), "seek window");
            seek_ms = elapsed_ms(start); avcodec_flush_buffers(codec);
        }
    }
    bool next(DecodeTiming &timing) {
        while(!interrupted) {
            auto start = Clock::now();
            const int received = avcodec_receive_frame(codec, frame);
            timing.decode_ms += elapsed_ms(start);
            if(received == 0) {
                if((frame->flags & AV_FRAME_FLAG_CORRUPT) || frame->decode_error_flags)
                    throw std::runtime_error("corrupt/concealed decoded frame");
                if(frame->width != kWidth || frame->height != kHeight)
                    throw std::runtime_error("decoded dimensions changed");
                if(frame->best_effort_timestamp == AV_NOPTS_VALUE)
                    throw std::runtime_error("source frame has no reliable presentation timestamp");
                return true;
            }
            if(received == AVERROR_EOF) return false;
            av_check(received == AVERROR(EAGAIN) ? 0 : received, "receive decoded frame");
            if(flushed) throw std::runtime_error("decoder requested input after EOF");
            while(!interrupted) {
                start = Clock::now(); const int read = av_read_frame(format, packet);
                timing.read_ms += elapsed_ms(start);
                if(read == AVERROR_EOF) {
                    start = Clock::now(); av_check(avcodec_send_packet(codec, nullptr), "flush decoder");
                    timing.decode_ms += elapsed_ms(start); flushed = true; break;
                }
                av_check(read, "read packet");
                if(packet->stream_index != stream) { av_packet_unref(packet); continue; }
                start = Clock::now(); const int sent = avcodec_send_packet(codec, packet);
                timing.decode_ms += elapsed_ms(start); av_packet_unref(packet);
                av_check(sent, "send packet"); break;
            }
        }
        throw std::runtime_error("interrupted during window decoding");
    }
    int source_matrix() {
        switch(frame->colorspace) {
        case AVCOL_SPC_BT709: return SWS_CS_ITU709;
        case AVCOL_SPC_FCC: return SWS_CS_FCC;
        case AVCOL_SPC_BT470BG:
        case AVCOL_SPC_SMPTE170M: return SWS_CS_ITU601;
        case AVCOL_SPC_SMPTE240M: return SWS_CS_SMPTE240M;
        case AVCOL_SPC_BT2020_NCL: return SWS_CS_BT2020;
        case AVCOL_SPC_UNSPECIFIED:
            space_assumed = true; return SWS_CS_ITU601;
        default: throw std::runtime_error("unsupported source colorspace for NV21 conversion");
        }
    }
    int source_full_range() {
        if(frame->color_range == AVCOL_RANGE_JPEG) return 1;
        if(frame->color_range == AVCOL_RANGE_MPEG) return 0;
        range_assumed = true;
        switch(static_cast<AVPixelFormat>(frame->format)) {
        case AV_PIX_FMT_YUVJ420P: case AV_PIX_FMT_YUVJ422P:
        case AV_PIX_FMT_YUVJ444P: case AV_PIX_FMT_YUVJ440P: return 1;
        default: return 0;
        }
    }
    void convert(CachedFrame &target) {
        converter = sws_getCachedContext(converter, kWidth, kHeight,
            static_cast<AVPixelFormat>(frame->format), kWidth, kHeight, AV_PIX_FMT_NV21,
            SWS_BILINEAR, nullptr, nullptr, nullptr);
        if(!converter) throw std::runtime_error("cannot allocate NV21 converter");
        av_check(sws_setColorspaceDetails(converter,
            sws_getCoefficients(source_matrix()), source_full_range(),
            sws_getCoefficients(SWS_CS_ITU601), 0, 0, 1 << 16, 1 << 16),
            "configure limited-range BT.601 NV21 conversion");
        uint8_t *dest[] = {target.data.data(), target.data.data() + kWidth * kHeight, nullptr, nullptr};
        int strides[] = {kWidth, kWidth, 0, 0};
        if(sws_scale(converter, frame->data, frame->linesize, 0, kHeight, dest, strides) != kHeight)
            throw std::runtime_error("incomplete NV21 conversion");
    }
};
struct FeedRow {
    uint64_t sequence = 0, pts = 0, loop = 0, cache_index = 0;
    uint64_t scheduled = 0, received = 0, finished = 0, skipped_before = 0;
    int64_t source_frame = 0, source_pts = 0, source_us = 0;
};
void write_feed(const std::vector<FeedRow> &rows) {
    std::ofstream out("feed.csv");
    out << "sequence,pts_raw,source_frame,source_pts_raw,source_timestamp_us,source_loop,"
           "cache_index,scheduled_us,received_us,submit_finished_us,skipped_ticks_before\n";
    for(const auto &r : rows)
        out << r.sequence << ',' << r.pts << ',' << r.source_frame << ',' << r.source_pts << ','
            << r.source_us << ',' << r.loop << ',' << r.cache_index << ',' << r.scheduled << ','
            << r.received << ',' << r.finished << ',' << r.skipped_before << '\n';
    out.close(); if(!out) throw std::runtime_error("feed.csv write failed");
}
int run(const Options &o) {
    cv::setNumThreads(1); av_log_set_level(AV_LOG_ERROR);
    const auto application_start = Clock::now();
    const uint64_t available_before_kib = proc_kib("/proc/meminfo", "MemAvailable:");
    const uint64_t cache_bytes = o.cache_frames * kFrameBytes;
    if(available_before_kib && cache_bytes + 48ULL * 1024 * 1024 > available_before_kib * 1024)
        throw std::runtime_error("insufficient available RAM for requested cache plus 48 MiB headroom");
    const auto config = dart::load_application_config(o.config);
    if(config.camera.width != kWidth || config.camera.height != kHeight || config.camera.fps != kFeedHz ||
       config.npu.enabled || config.target_geometry.pose_enabled || config.detector.classical_interval_frames != 1)
        throw std::runtime_error("requires 1344x760/180 config, per-observation detector, NPU and pose disabled");
    const auto preload_start = Clock::now();
    Decoder decoder; decoder.open(o);
    Samples read_times, decode_times, convert_times, allocation_times;
    std::vector<std::shared_ptr<const CachedFrame>> cache; cache.reserve(o.cache_frames);
    uint64_t decoded_frames = 0, preroll_frames = 0;
    int pixel_format = -1, color_range = -1, colorspace = -1;
    const int64_t requested_start_us = std::llround(o.start_seconds * 1000000.0);
    while(cache.size() < o.cache_frames) {
        DecodeTiming timing;
        if(!decoder.next(timing)) throw std::runtime_error("EOF before complete cache window; reduce cache or start earlier");
        read_times.add(timing.read_ms); decode_times.add(timing.decode_ms); ++decoded_frames;
        const int64_t source_us = av_rescale_q(decoder.frame->best_effort_timestamp - decoder.stream_start,
                                               decoder.timebase, AVRational{1,1000000});
        if(source_us < requested_start_us) { ++preroll_frames; continue; }
        if(!cache.empty() && decoder.frame->best_effort_timestamp <= cache.back()->pts)
            throw std::runtime_error("non-increasing source PTS in cache window");
        if(cache.empty()) {
            pixel_format = decoder.frame->format; color_range = decoder.frame->color_range;
            colorspace = decoder.frame->colorspace;
        } else if(pixel_format != decoder.frame->format || color_range != decoder.frame->color_range ||
                  colorspace != decoder.frame->colorspace)
            throw std::runtime_error("source color properties changed within cache window");
        auto start = Clock::now(); auto item = std::make_shared<CachedFrame>();
        item->data.resize(kFrameBytes); allocation_times.add(elapsed_ms(start));
        item->pts = decoder.frame->best_effort_timestamp; item->timestamp_us = source_us;
        item->source_frame = std::llround(av_q2d(decoder.timebase) *
            (item->pts - decoder.stream_start) * decoder.fps);
        start = Clock::now(); decoder.convert(*item); convert_times.add(elapsed_ms(start));
        cache.push_back(std::move(item));
    }
    const double preload_wall_ms = elapsed_ms(preload_start);
    const uint64_t rss_cached_kib = proc_kib("/proc/self/status", "VmRSS:");
    const uint64_t duration_us = std::llround(o.seconds * 1000000.0);
    const uint64_t planned_ticks = (duration_us * kFeedHz + 999999) / 1000000;
    std::vector<FeedRow> feed; feed.reserve(planned_ticks);
    dart::HighFpsPipeline pipeline(config, false, false, dart::PipelineInputSource::CachedVideo);
    const uint64_t origin = now_us(), stop_deadline = origin + duration_us;
    uint64_t tick = 0, skipped = 0, skipped_pending = 0;
    bool failed = false; std::string failure;
    try {
        while(tick < planned_ticks && !interrupted) {
            if(pipeline.failed()) throw std::runtime_error("HighFpsPipeline reported failure");
            const auto now = now_us();
            if(now >= stop_deadline) break;
            const uint64_t due_tick = (now - origin) * kFeedHz / 1000000;
            if(due_tick > tick) { skipped_pending += due_tick - tick; tick = due_tick; }
            if(tick >= planned_ticks) break;
            const uint64_t scheduled = origin + (tick * 1000000 + kFeedHz - 1) / kFeedHz;
            std::this_thread::sleep_until(Clock::time_point(std::chrono::microseconds(scheduled)));
            auto lease = std::make_shared<CachedLease>(cache[tick % cache.size()]);
            const uint64_t received = now_us();
            if(received >= stop_deadline || interrupted) break;
            const uint64_t actual_tick = (received - origin) * kFeedHz / 1000000;
            if(actual_tick > tick) { skipped_pending += actual_tick - tick; tick = actual_tick; continue; }
            const auto &source = cache[tick % cache.size()];
            lease->metadata = {tick + 1, tick * 1000000 / kFeedHz, received};
            pipeline.submit(std::move(lease));
            feed.push_back({tick + 1, tick * 1000000 / kFeedHz, tick / cache.size(), tick % cache.size(),
                            scheduled, received, now_us(), skipped_pending,
                            source->source_frame, source->pts, source->timestamp_us});
            skipped += skipped_pending; skipped_pending = 0; ++tick;
        }
    } catch(const std::exception &error) {
        failed = true; failure = error.what();
    }
    // Let the pipeline consume the final scheduled slot before normal shutdown.
    // No additional frames are sent during this remaining fraction of a tick.
    if(!failed && !interrupted)
        std::this_thread::sleep_until(Clock::time_point(std::chrono::microseconds(stop_deadline)));
    const uint64_t feed_stopped = now_us();
    pipeline.finish();
    const uint64_t finished = now_us();
    if(pipeline.failed()) { failed = true; if(failure.empty()) failure = "HighFpsPipeline failed"; }
    write_feed(feed);
    const uint64_t tail_ticks = planned_ticks - (feed.empty() ? 0 : feed.back().sequence);
    const char *pixel_name = av_get_pix_fmt_name(static_cast<AVPixelFormat>(pixel_format));
    const char *range_name = av_color_range_name(static_cast<AVColorRange>(color_range));
    const char *space_name = av_color_space_name(static_cast<AVColorSpace>(colorspace));
    std::ostringstream summary; summary << std::fixed << std::setprecision(6)
        << "{\"input_mode\":\"cached_video_nv21\",\"input\":" << quote(o.input)
        << ",\"config\":" << quote(o.config) << ",\"failed\":" << (failed ? "true" : "false")
        << ",\"failure\":" << quote(failure) << ",\"interrupted\":" << (interrupted ? "true" : "false")
        << ",\"hardware_capture_included\":false,\"hardware_decode_included\":false,\"npu_included\":false"
        << ",\"decode_in_timed_feed\":false,\"looping\":true,\"source_fps\":" << decoder.fps
        << ",\"width\":" << kWidth << ",\"height\":" << kHeight << ",\"feed_hz\":" << kFeedHz
        << ",\"requested_start_seconds\":" << o.start_seconds << ",\"requested_seconds\":" << o.seconds
        << ",\"cache_frames\":" << cache.size() << ",\"cache_bytes\":" << cache_bytes
        << ",\"cache_budget_bytes\":" << kCacheBudget << ",\"available_before_kib\":" << available_before_kib
        << ",\"rss_after_preload_kib\":" << rss_cached_kib
        << ",\"rss_after_finish_kib\":" << proc_kib("/proc/self/status", "VmRSS:")
        << ",\"peak_rss_kib\":" << proc_kib("/proc/self/status", "VmHWM:")
        << ",\"source_first_frame\":" << cache.front()->source_frame << ",\"source_last_frame\":" << cache.back()->source_frame
        << ",\"source_first_timestamp_us\":" << cache.front()->timestamp_us
        << ",\"source_last_timestamp_us\":" << cache.back()->timestamp_us
        << ",\"source_frame_index_method\":\"derived_from_pts\",\"source_pts_timebase_num\":" << decoder.timebase.num
        << ",\"source_pts_timebase_den\":" << decoder.timebase.den << ",\"stream_start_pts\":" << decoder.stream_start
        << ",\"stream_start_assumed_zero\":" << (decoder.stream_start_assumed_zero ? "true" : "false")
        << ",\"metadata_pts_kind\":\"synthetic_tick_microseconds\",\"metadata_received_clock\":\"steady_monotonic\""
        << ",\"feed_started_us\":" << origin << ",\"feed_stopped_us\":" << feed_stopped << ",\"finished_us\":" << finished
        << ",\"planned_ticks\":" << planned_ticks << ",\"submitted_frames\":" << feed.size()
        << ",\"skipped_ticks\":" << skipped << ",\"unsubmitted_ticks_after_last\":" << tail_ticks
        << ",\"source_loops\":" << (feed.empty() ? 0 : feed.back().loop + 1)
        << ",\"first_scheduled_us\":" << (feed.empty() ? 0 : feed.front().scheduled)
        << ",\"last_scheduled_us\":" << (feed.empty() ? 0 : feed.back().scheduled)
        << ",\"planned_tick_formula\":\"origin+ceil(tick*1000000/180)\""
        << ",\"feed_wall_ms\":" << (feed_stopped - origin) / 1000.0
        << ",\"finish_wall_ms\":" << (finished - feed_stopped) / 1000.0
        << ",\"preload_wall_ms\":" << preload_wall_ms << ",\"seek_ms\":" << decoder.seek_ms
        << ",\"decoded_frames_including_preroll\":" << decoded_frames << ",\"preroll_frames\":" << preroll_frames
        << ",\"read_demux\":" << read_times.json() << ",\"software_decode\":" << decode_times.json()
        << ",\"nv21_conversion\":" << convert_times.json() << ",\"cache_allocation\":" << allocation_times.json()
        << ",\"decoder\":" << quote(decoder.codec_name) << ",\"decode_threads\":1,\"opencv_threads\":1"
        << ",\"ffmpeg_version\":" << quote(av_version_info())
        << ",\"source_pixel_format\":" << quote(pixel_name ? pixel_name : "unknown")
        << ",\"source_color_range\":" << quote(range_name ? range_name : "unknown")
        << ",\"source_colorspace\":" << quote(space_name ? space_name : "unknown")
        << ",\"source_range_assumed\":" << (decoder.range_assumed ? "true" : "false")
        << ",\"source_space_assumed\":" << (decoder.space_assumed ? "true" : "false")
        << ",\"output_pixel_format\":\"nv21\",\"output_color_range\":\"limited\",\"output_colorspace\":\"bt601\""
        << ",\"color_conversion\":\"libswscale_source_metadata_to_limited_bt601_for_pipeline_Y_minus_16\""
        << ",\"cached_frames_are_original_vin\":false,\"safe_for_control\":false,\"angles_valid\":false"
        << ",\"feed_csv_written_after_finish\":true,\"application_wall_ms\":" << elapsed_ms(application_start) << '}';
    const auto parent = std::filesystem::path(o.summary).parent_path();
    if(!parent.empty()) std::filesystem::create_directories(parent);
    std::ofstream output(o.summary); output << summary.str() << '\n'; output.close();
    if(!output) throw std::runtime_error("replay summary write failed");
    std::cout << summary.str() << '\n';
    return failed ? 1 : interrupted ? 130 : 0;
}
} // namespace
int main(int argc, char **argv) {
    std::signal(SIGINT, handle_signal); std::signal(SIGTERM, handle_signal);
    try { return run(parse(argc, argv)); }
    catch(const std::exception &error) { std::cerr << "fatal: " << error.what() << '\n'; return 1; }
}
