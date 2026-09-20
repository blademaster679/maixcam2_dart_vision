// Continuous source-time diagnostic replay. This is deliberately isolated
// from the live pipeline: decoder time never changes tracking time, and no
// realtime throughput claim can be made from this executable.
// Reuse the audited decoder/color conversion rather than fork FFmpeg handling.
#define main cached_replay_main
#include "board_nv21_replay.cpp"
#undef main
#include "dart/roi_scheduler.hpp"
#include "dart/target_json.hpp"
#include "dart/visual_motion.hpp"
#include <limits>

namespace offline {
struct Arguments {
    std::string input, config;
    uint64_t max_frames = 0;
};

Arguments arguments(int argc, char **argv) {
    Arguments result;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "--help" || key == "-h") {
            std::cout << "board_nv21_offline --input VIDEO.mp4 --config CONFIG.conf "
                "[--max-frames 0]\n"
                "Continuous native 1344x760@180 source-time NV21 ROI replay. "
                "No camera, NPU, looping, or realtime performance measurement.\n"
                "Creates observations.jsonl and offline_summary.json in an empty output directory.\n";
            std::exit(0);
        }
        if (++i >= argc) throw std::runtime_error(key + " requires a value");
        const std::string value = argv[i];
        if (key == "--input") result.input = value;
        else if (key == "--config") result.config = value;
        else if (key == "--max-frames") {
            if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
                throw std::runtime_error("--max-frames requires an unsigned integer");
            result.max_frames = std::stoull(value);
        } else throw std::runtime_error("unknown argument: " + key);
    }
    if (result.input.empty() || result.config.empty())
        throw std::runtime_error("--input and --config are required");
    for (const char *path : {"observations.jsonl", "offline_summary.json"}) {
        if (std::filesystem::exists(path))
            throw std::runtime_error(std::string("refusing to overwrite existing output: ") + path);
        const auto output = std::filesystem::weakly_canonical(path);
        if (output == std::filesystem::weakly_canonical(result.input) ||
            output == std::filesystem::weakly_canonical(result.config))
            throw std::runtime_error("output must differ from input and configuration");
    }
    return result;
}

int run(const Arguments &args) {
    cv::setNumThreads(1);
    av_log_set_level(AV_LOG_ERROR);
    const auto started = Clock::now();
    Decoder decoder;
    Samples read_times, decode_times, convert_times, vision_times, motion_times;
    uint64_t decoded = 0, processed = 0, skipped = 0;
    uint64_t searches = 0, armor_calls = 0, motion_calls = 0, prior_uses = 0;
    uint64_t direct_green = 0, predicted_green = 0, direct_armor = 0;
    int64_t previous_pts = AV_NOPTS_VALUE, last_source_frame = -1, last_source_us = -1;
    int64_t expected_frames = 0;
    int pixel_format = -1, color_range = -1, colorspace = -1;
    bool failed = false, reached_eof = false;
    std::string failure;
    std::ofstream observations;
    int green_hz = 0, search_hz = 0, armor_hz = 0, motion_hz = 0;
    try {
        const auto config = dart::load_application_config(args.config);
        if (config.camera.width != kWidth || config.camera.height != kHeight ||
            config.camera.fps != kFeedHz || config.npu.enabled ||
            config.target_geometry.pose_enabled || config.detector.classical_interval_frames != 1)
            throw std::runtime_error("requires 1344x760/180 config, per-observation detector, NPU and pose disabled");
        green_hz = config.highfps.green_hz; search_hz = config.highfps.search_hz;
        armor_hz = config.highfps.armor_hz; motion_hz = config.highfps.motion_hz;
        Options decode_options; decode_options.input = args.input;
        decoder.open(decode_options);
        expected_frames = decoder.format->streams[decoder.stream]->nb_frames;
        observations.open("observations.jsonl");
        if (!observations) throw std::runtime_error("cannot create observations.jsonl");
        dart::GreenLightDetector detector(config.detector, config.armor, config.target_geometry, config.npu);
        dart::RoiScheduler roi_scheduler(config.detector, config.highfps.green_hz);
        dart::TimestampSchedule measurement(config.highfps.green_hz), search(config.highfps.search_hz),
            armor_schedule(config.highfps.armor_hz), motion_schedule(config.highfps.motion_hz);
        dart::VisualMotionEstimator motion_estimator(config.visual_motion);
        dart::MotionPrior pending_prior;
        bool pending_prior_available = false;
        uint64_t previous_motion_us = 0, armor_source_us = 0;
        std::vector<dart::Point2f> proposals;
        CachedFrame storage; storage.data.resize(kFrameBytes);
        while (!interrupted && (!args.max_frames || decoded < args.max_frames)) {
            DecodeTiming timing;
            if (!decoder.next(timing)) { reached_eof = true; break; }
            read_times.add(timing.read_ms); decode_times.add(timing.decode_ms);
            const int64_t pts = decoder.frame->best_effort_timestamp;
            if (pts < 0 || pts < decoder.stream_start ||
                (previous_pts != AV_NOPTS_VALUE && pts <= previous_pts))
                throw std::runtime_error("negative or non-increasing source PTS");
            const int64_t relative_pts = pts - decoder.stream_start;
            const int64_t source_frame = av_rescale_q_rnd(relative_pts, decoder.timebase,
                AVRational{1, kFeedHz}, AV_ROUND_NEAR_INF);
            const int64_t expected_pts = av_rescale_q(static_cast<int64_t>(decoded),
                AVRational{1, kFeedHz}, decoder.timebase);
            if (source_frame < 0 || static_cast<uint64_t>(source_frame) != decoded ||
                std::llabs(relative_pts - expected_pts) > 1)
                throw std::runtime_error("source PTS is not a contiguous 180 fps frame timeline");
            const int64_t source_us = av_rescale_q(relative_pts, decoder.timebase, AVRational{1, 1000000});
            // Rational deadlines use ceil; use the same rounding for source
            // timestamps so exact 180/90/60 Hz boundaries remain aligned.
            const int64_t source_ceil_us = av_rescale_q_rnd(relative_pts, decoder.timebase,
                AVRational{1, 1000000}, AV_ROUND_UP);
            if (source_ceil_us < 0 || static_cast<uint64_t>(source_ceil_us) >
                std::numeric_limits<uint64_t>::max() - 1000000ULL)
                throw std::runtime_error("source timestamp overflow");
            const uint64_t timestamp = 1000000ULL + static_cast<uint64_t>(source_ceil_us);
            if (pixel_format < 0) {
                pixel_format = decoder.frame->format; color_range = decoder.frame->color_range;
                colorspace = decoder.frame->colorspace;
            } else if (pixel_format != decoder.frame->format || color_range != decoder.frame->color_range ||
                       colorspace != decoder.frame->colorspace)
                throw std::runtime_error("source color properties changed");
            previous_pts = pts; last_source_frame = source_frame; last_source_us = source_us;
            ++decoded;
            if (!measurement.due(timestamp)) { ++skipped; continue; }
            auto stage = Clock::now(); decoder.convert(storage); convert_times.add(elapsed_ms(stage));
            const dart::Nv21View view{storage.data.data(), storage.data.data() + kWidth * kHeight,
                                     kWidth, kHeight, kWidth, kWidth};
            const auto vision_started = Clock::now();
            auto prediction = detector.tracker_snapshot(); prediction.predict(timestamp);
            const bool scheduled_search = search.due(timestamp);
            const bool full = scheduled_search || roi_scheduler.needs_global_search(timestamp, prediction);
            if (full) { proposals = dart::nv21_green_proposals(view); ++searches; }
            const auto selection = roi_scheduler.select(proposals, prediction, timestamp, kWidth, kHeight);
            if (selection.reset_tracker) detector.reset();
            const bool tracked = selection.mode == dart::RoiMode::Tracking ||
                                 selection.mode == dart::RoiMode::Coasting;
            const auto roi = selection.roi;
            auto image = dart::nv21_rgb_region(view, roi);
            dart::MotionPrior prior;
            if (pending_prior_available) {
                if (pending_prior.timestamp_us <= timestamp && timestamp - pending_prior.timestamp_us <= 50000)
                    prior = pending_prior;
                pending_prior_available = false;
            }
            const bool motion_ran = config.visual_motion.enabled && tracked && motion_schedule.due(timestamp);
            if (motion_ran) {
                stage = Clock::now();
                auto gray = std::make_unique<maix::image::Image>(kWidth / 8, kHeight / 8,
                    maix::image::Format::FMT_RGB888);
                auto *out = static_cast<uint8_t *>(gray->data());
                for (int y = 0; y < kHeight; y += 8) for (int x = 0; x < kWidth; x += 8) {
                    const auto value = static_cast<uint8_t>(std::clamp(
                        (298 * (view.y[y * view.y_stride + x] - 16) + 128) >> 8, 0, 255));
                    *out++ = value; *out++ = value; *out++ = value;
                }
                if (previous_motion_us && timestamp - previous_motion_us > 100000) motion_estimator.reset();
                previous_motion_us = timestamp;
                pending_prior = motion_estimator.update(*gray, timestamp);
                pending_prior.image_dx_px *= 8; pending_prior.image_dy_px *= 8;
                pending_prior_available = true;
                ++motion_calls; motion_times.add(elapsed_ms(stage));
            }
            const bool armor_due = armor_schedule.due(timestamp);
            auto target = detector.process_region(*image, roi, kWidth, kHeight, timestamp,
                prior.valid ? &prior : nullptr, false, armor_due);
            roi_scheduler.observe(target.green, timestamp, target.classical_detection_ran);
            if (target.armor_detection_ran) armor_source_us = target.armor.valid ? timestamp : 0;
            target.armor_source_received_us = target.armor.valid ? armor_source_us : 0;
            target.source_metadata_valid = true;
            target.source_sequence = static_cast<uint64_t>(source_frame) + 1;
            target.source_pts_raw = static_cast<uint64_t>(pts);
            target.source_received_us = timestamp;
            dart::invalidate_uncalibrated(target);
            const double vision_ms = elapsed_ms(vision_started);
            vision_times.add(vision_ms);
            ++processed;
            if (prior.valid) ++prior_uses;
            if (target.armor_detection_ran) ++armor_calls;
            if (target.green.valid && !target.green.predicted) ++direct_green;
            if (target.green.valid && target.green.predicted) ++predicted_green;
            if (target.armor.valid && target.armor_detection_ran) ++direct_armor;
            std::ostringstream extra;
            extra << ",\"source_frame\":" << source_frame
                << ",\"source_timestamp_us\":" << source_us
                << ",\"replay_pipeline\":\"offline_nv21_roi\",\"offline_not_realtime\":true"
                << ",\"timestamp_source\":\"video_pts_ceil_us_plus_1000000\""
                << ",\"measurement_timestamp_source\":\"video_pts_virtual_clock\""
                << ",\"source_pts_timebase_num\":" << decoder.timebase.num
                << ",\"source_pts_timebase_den\":" << decoder.timebase.den
                << ",\"search_ran\":" << (full ? "true" : "false")
                << ",\"motion_ran\":" << (motion_ran ? "true" : "false")
                << ",\"motion_prior_used\":" << (prior.valid ? "true" : "false")
                << ",\"motion_prior_timestamp_us\":" << (prior.valid ? prior.timestamp_us : 0)
                << ",\"armor_schedule_due\":" << (armor_due ? "true" : "false")
                << ",\"roi_x\":" << roi.x << ",\"roi_y\":" << roi.y
                << ",\"roi_width\":" << roi.width << ",\"roi_height\":" << roi.height
                << ",\"roi_mode\":" << static_cast<int>(selection.mode)
                << ",\"roi_tracker_reset\":" << (selection.reset_tracker ? "true" : "false")
                << ",\"proposal_count\":" << proposals.size()
                << ",\"offline_vision_wall_ms\":" << vision_ms;
            dart::write_target_estimate_json(observations, target, nullptr, extra.str());
            observations << '\n';
            if (!observations) throw std::runtime_error("observations.jsonl write failed");
            if (processed % 900 == 0) {
                observations.flush();
                if (!observations) throw std::runtime_error("observations.jsonl flush failed");
                std::cerr << "decoded=" << decoded << " observations=" << processed
                          << " source_seconds=" << source_us / 1000000.0 << '\n';
            }
        }
        if (reached_eof && expected_frames > 0 && decoded != static_cast<uint64_t>(expected_frames))
            throw std::runtime_error("decoded frame count differs from container frame count");
        if (!decoded && !interrupted) throw std::runtime_error("no source frames decoded");
    } catch (const std::exception &error) {
        failed = !interrupted;
        failure = error.what();
    }
    if (observations.is_open()) {
        observations.close();
        if (!observations) { failed = true; failure += "; observations.jsonl close failed"; }
    }
    const bool complete = reached_eof && !failed && !interrupted;
    const char *pixel_name = av_get_pix_fmt_name(static_cast<AVPixelFormat>(pixel_format));
    const char *range_name = av_color_range_name(static_cast<AVColorRange>(color_range));
    const char *space_name = av_color_space_name(static_cast<AVColorSpace>(colorspace));
    std::ostringstream summary;
    summary << std::fixed << std::setprecision(6)
        << "{\"replay_pipeline\":\"offline_nv21_roi\",\"offline_not_realtime\":true"
        << ",\"input\":" << quote(args.input) << ",\"config\":" << quote(args.config)
        << ",\"failed\":" << (failed ? "true" : "false")
        << ",\"failure\":" << quote(failure) << ",\"interrupted\":" << (interrupted ? "true" : "false")
        << ",\"reached_eof\":" << (reached_eof ? "true" : "false")
        << ",\"complete_source\":" << (complete ? "true" : "false")
        << ",\"truncated_by_max_frames\":" << (!reached_eof && !failed && !interrupted && args.max_frames && decoded >= args.max_frames ? "true" : "false")
        << ",\"max_frames\":" << args.max_frames << ",\"container_frames\":" << expected_frames
        << ",\"decoded_frames\":" << decoded << ",\"processed_observations\":" << processed
        << ",\"scheduled_skipped_frames\":" << skipped
        << ",\"search_calls\":" << searches << ",\"armor_calls\":" << armor_calls
        << ",\"motion_calls\":" << motion_calls << ",\"motion_prior_uses\":" << prior_uses
        << ",\"direct_green_observations\":" << direct_green
        << ",\"predicted_green_observations\":" << predicted_green
        << ",\"direct_armor_observations\":" << direct_armor
        << ",\"requested_green_hz\":" << green_hz << ",\"requested_armor_hz\":" << armor_hz
        << ",\"requested_search_hz\":" << search_hz << ",\"requested_motion_hz\":" << motion_hz
        << ",\"source_fps\":" << decoder.fps << ",\"width\":" << kWidth << ",\"height\":" << kHeight
        << ",\"source_first_frame\":" << (decoded ? 0 : -1) << ",\"source_last_frame\":" << last_source_frame
        << ",\"source_last_timestamp_us\":" << last_source_us
        << ",\"source_pts_timebase_num\":" << decoder.timebase.num
        << ",\"source_pts_timebase_den\":" << decoder.timebase.den
        << ",\"stream_start_pts\":" << decoder.stream_start
        << ",\"stream_start_assumed_zero\":" << (decoder.stream_start_assumed_zero ? "true" : "false")
        << ",\"source_frame_index_method\":\"derived_from_pts_and_checked_against_contiguous_decode_index\""
        << ",\"metadata_received_clock\":\"source_pts_ceil_microseconds_plus_1000000\""
        << ",\"looping\":false,\"source_timeline_preserved\":true,\"hardware_capture_included\":false"
        << ",\"hardware_decode_included\":false,\"npu_included\":false,\"realtime_threads_simulated\":false"
        << ",\"motion_delivery\":\"synchronous_computation_consumed_once_at_next_observation_if_fresh\""
        << ",\"decoder\":" << quote(decoder.codec_name) << ",\"decode_threads\":1,\"opencv_threads\":1"
        << ",\"ffmpeg_version\":" << quote(av_version_info())
        << ",\"source_pixel_format\":" << quote(pixel_name ? pixel_name : "unknown")
        << ",\"source_color_range\":" << quote(range_name ? range_name : "unknown")
        << ",\"source_colorspace\":" << quote(space_name ? space_name : "unknown")
        << ",\"source_range_assumed\":" << (decoder.range_assumed ? "true" : "false")
        << ",\"source_space_assumed\":" << (decoder.space_assumed ? "true" : "false")
        << ",\"output_pixel_format\":\"nv21\",\"output_color_range\":\"limited\",\"output_colorspace\":\"bt601\""
        << ",\"safe_for_control\":false,\"angles_valid\":false"
        << ",\"read_demux\":" << read_times.json() << ",\"software_decode\":" << decode_times.json()
        << ",\"nv21_conversion\":" << convert_times.json() << ",\"offline_vision_wall\":" << vision_times.json()
        << ",\"offline_motion_wall\":" << motion_times.json()
        << ",\"application_wall_ms\":" << elapsed_ms(started)
        << ",\"peak_rss_kib\":" << proc_kib("/proc/self/status", "VmHWM:") << '}';
    std::ofstream output("offline_summary.json");
    output << summary.str() << '\n'; output.close();
    if (!output) throw std::runtime_error("offline_summary.json write failed");
    std::cout << summary.str() << '\n';
    return failed ? 1 : interrupted ? 130 : 0;
}
} // namespace offline

int main(int argc, char **argv) {
    std::signal(SIGINT, handle_signal); std::signal(SIGTERM, handle_signal);
    try { return offline::run(offline::arguments(argc, argv)); }
    catch (const std::exception &error) { std::cerr << "fatal: " << error.what() << '\n'; return 1; }
}
