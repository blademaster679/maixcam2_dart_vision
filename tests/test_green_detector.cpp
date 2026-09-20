#include "dart/green_detector.hpp"
#include "dart/target_json.hpp"
#include "dart/visual_motion.hpp"
#include "dart/nv21_pipeline.hpp"

#include <cmath>
#include <atomic>
#include <chrono>
#include <thread>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const std::string &message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

dart::detail::CandidateObservation observation(
    float x,
    float y,
    float size,
    dart::detail::CandidateSource source = dart::detail::CandidateSource::Halo)
{
    dart::detail::CandidateObservation candidate;
    candidate.center_x = x;
    candidate.center_y = y;
    candidate.bbox_x = static_cast<int>(x - size * 0.5F);
    candidate.bbox_y = static_cast<int>(y - size * 0.5F);
    candidate.bbox_w = static_cast<int>(size);
    candidate.bbox_h = static_cast<int>(size);
    candidate.apparent_size = size;
    candidate.score = 0.9F;
    candidate.association_score = 0.9F;
    candidate.source = source;
    return candidate;
}

maix::image::Image green_blob_image(int image_width,
                                    int image_height,
                                    int x,
                                    int y,
                                    int width,
                                    int height,
                                    float center_x,
                                    float center_y)
{
    maix::image::Image image(image_width, image_height);
    image.fill_rect(x, y, width, height, 10, 250, 15);
    const maix::image::Blob halo(x, y, width, height, center_x, center_y,
                                 width * height, 0.9F);
    const int core_width = std::max(1, width / 2);
    const int core_height = std::max(1, height / 2);
    const int core_x = static_cast<int>(center_x - core_width * 0.5F);
    const int core_y = static_cast<int>(center_y - core_height * 0.5F);
    const maix::image::Blob core(core_x, core_y, core_width, core_height,
                                 center_x, center_y, core_width * core_height, 0.9F);
    image.set_blobs({halo}, {core});
    return image;
}

void test_confirmation_and_loss()
{
    dart::DetectorConfig config;
    dart::detail::TemporalTracker tracker(config);
    const auto candidate = observation(640.0F, 360.0F, 4.0F);

    auto result = tracker.update(&candidate, 1000000, config.camera_model);
    check(result.state == dart::TrackState::Candidate, "first hit is CANDIDATE");
    check(!result.valid, "unconfirmed first hit is not a control-valid result");
    result = tracker.update(&candidate, 1016667, config.camera_model);
    check(result.state == dart::TrackState::Candidate, "second hit is CANDIDATE");
    result = tracker.update(&candidate, 1033334, config.camera_model);
    check(result.state == dart::TrackState::Tracking, "third of five hits enters TRACKING");
    check(result.valid, "confirmed track is control-valid");

    for (int miss = 1; miss <= 4; ++miss) {
        result = tracker.update(nullptr,
                                1033334 + static_cast<uint64_t>(miss) * 16667,
                                config.camera_model);
        check(result.state == dart::TrackState::Tracking,
              "TRACKING is retained before the fifth consecutive miss");
        check(result.valid && result.predicted,
              "a confirmed track bridges a short observation dropout");
    }
    result = tracker.update(nullptr, 1116669, config.camera_model);
    check(result.state == dart::TrackState::Lost, "fifth miss returns to LOST");
    check(!tracker.has_prediction(), "fifth miss clears the prediction");
}

void test_interrupted_acquisition_requires_new_consecutive_hits()
{
    dart::DetectorConfig config;
    dart::detail::TemporalTracker tracker(config);
    const auto candidate = observation(100.0F, 80.0F, 3.0F);
    tracker.update(&candidate, 1000000, config.camera_model);
    tracker.update(nullptr, 1010000, config.camera_model);
    tracker.update(&candidate, 1020000, config.camera_model);
    tracker.update(nullptr, 1030000, config.camera_model);
    auto result = tracker.update(&candidate, 1040000, config.camera_model);
    check(result.state == dart::TrackState::Candidate && !result.valid,
          "rejected observations prevent pooling isolated acquisition hits");
    result = tracker.update(&candidate, 1050000, config.camera_model);
    check(!result.valid, "the second new consecutive observation remains tentative");
    result = tracker.update(&candidate, 1060000, config.camera_model);
    check(result.valid && !result.predicted,
          "three new consecutive real observations complete acquisition");
}

void test_prediction_timeout_and_recovery()
{
    dart::DetectorConfig config;
    config.max_missed_frames = 10;
    config.prediction_max_age_ms = 40;
    dart::detail::TemporalTracker tracker(config);
    const auto candidate = observation(320.0F, 180.0F, 20.0F);
    tracker.update(&candidate, 1000000, config.camera_model);
    tracker.update(&candidate, 1010000, config.camera_model);
    tracker.update(&candidate, 1020000, config.camera_model);

    auto result = tracker.update(nullptr, 1040000, config.camera_model);
    check(result.valid && result.predicted && result.bbox_w > 0,
          "prediction is control-valid inside its time budget");
    result = tracker.update(nullptr, 1070000, config.camera_model);
    check(!result.valid && !result.predicted &&
              result.state == dart::TrackState::Tracking,
          "an expired prediction is withheld before the track is reset");
    result = tracker.update(&candidate, 1080000, config.camera_model);
    check(result.valid && !result.predicted,
          "a fresh measurement resumes observed output after timeout");
}

void test_growing_size_tracker()
{
    dart::DetectorConfig config;
    dart::detail::TemporalTracker tracker(config);
    float previous_size = 0.0F;
    uint64_t timestamp = 1000000;
    int frame = 0;
    for (const float size : {2.0F, 4.0F, 8.0F, 16.0F, 32.0F, 64.0F}) {
        const auto candidate = observation(320.0F, 180.0F, size);
        const auto result = tracker.update(&candidate, timestamp, config.camera_model);
        check(result.valid == (frame >= 2),
              "growing target becomes valid only after confirmation");
        check(result.apparent_size > previous_size,
              "log-size state increases with an approaching target");
        previous_size = result.apparent_size;
        timestamp += 16667;
        ++frame;
    }
}

void test_confirmation_requires_same_candidate()
{
    dart::DetectorConfig config;
    config.confirmation_gate_px = 30.0F;
    dart::detail::TemporalTracker tracker(config);
    const auto first = observation(100.0F, 100.0F, 12.0F);
    const auto second = observation(220.0F, 100.0F, 12.0F);

    tracker.update(&first, 1000000, config.camera_model);
    tracker.update(&second, 1016667, config.camera_model);
    tracker.update(&first, 1033334, config.camera_model);
    const auto result = tracker.update(&second, 1050001, config.camera_model);
    check(result.state == dart::TrackState::Candidate && !result.valid,
          "different distractors cannot combine their hits to confirm a track");
}

void test_hard_association_gate()
{
    dart::DetectorConfig config;
    config.gate_min_px = 50.0F;
    config.gate_max_px = 80.0F;
    config.gate_size_factor = 2.0F;
    config.max_log_size_jump = 1.0F;
    config.max_cross_source_log_size_jump = 2.0F;
    config.min_association_score = 0.05F;
    dart::detail::TemporalTracker tracker(config);
    const auto target = observation(320.0F, 180.0F, 20.0F);
    tracker.update(&target, 1000000, config.camera_model);
    tracker.update(&target, 1016667, config.camera_model);
    tracker.update(&target, 1033334, config.camera_model);

    const auto far_distractor = observation(500.0F, 180.0F, 20.0F);
    check(!tracker.passes_association_gate(far_distractor),
          "candidate outside the hard position gate is rejected");
    const auto tiny_same_source = observation(322.0F, 181.0F, 3.0F);
    check(!tracker.passes_association_gate(tiny_same_source),
          "same-source size collapse is rejected");
    const auto compact_core = observation(
        322.0F, 181.0F, 5.0F,
        dart::detail::CandidateSource::SaturatedCore);
    check(tracker.passes_association_gate(compact_core),
          "nearby halo-to-core transition uses the cross-source size gate");

    const auto rejected = tracker.update(&far_distractor, 1050001,
                                         config.camera_model);
    check(rejected.valid && rejected.predicted &&
              rejected.state == dart::TrackState::Tracking &&
              std::fabs(rejected.center_x - 320.0F) < 3.0F,
          "rejected measurement coasts the prior track without following it");
    const auto recovered = tracker.update(&target, 1066668, config.camera_model);
    check(recovered.valid && std::fabs(recovered.center_x - 320.0F) < 3.0F,
          "rejected distractor does not pull the Kalman state");
}

void test_angles_and_distortion()
{
    dart::CameraModel model;
    model.fx = 400.0F;
    model.fy = 400.0F;
    model.principal_x = 320.0F;
    model.principal_y = 240.0F;

    auto angles = dart::detail::pixel_to_angles(360.0F, 200.0F, model);
    check(angles[0] > 0.0F, "pixel right of principal point has positive yaw");
    check(angles[1] > 0.0F, "pixel above principal point has positive pitch");

    const float undistorted_yaw = angles[0];
    model.k1 = 0.4F;
    angles = dart::detail::pixel_to_angles(360.0F, 200.0F, model);
    check(std::fabs(angles[0]) < std::fabs(undistorted_yaw),
          "positive radial distortion is inverted for the output point");
}

void test_single_pixel_and_reflection()
{
    dart::DetectorConfig config;
    dart::GreenLightDetector detector(config);

    auto one_pixel = green_blob_image(32, 24, 10, 8, 1, 1, 10.0F, 8.0F);
    auto result = detector.process(one_pixel, 1000000);
    result = detector.process(one_pixel, 1016667);
    result = detector.process(one_pixel, 1033334);
    check(result.valid, "single-pixel green target is accepted after confirmation");
    check(result.bbox_w == 1 && result.bbox_h == 1,
          "single-pixel bounding box is preserved");

    detector.reset();
    maix::image::Image reflection(32, 24);
    reflection.set_pixel(10, 8, 180, 190, 180);
    reflection.set_blobs({maix::image::Blob(10, 8, 1, 1, 10.0F, 8.0F)}, {});
    result = detector.process(reflection, 1016667);
    check(!result.valid, "weak green reflection fails RGB dominance threshold");
}

void test_large_growth_and_partial_frame()
{
    dart::DetectorConfig config;
    // This test verifies the blob-center fusion used by the legacy path.
    config.enable_normalized_multiscale = false;
    dart::GreenLightDetector detector(config);
    uint64_t timestamp = 1000000;
    float previous_size = 0.0F;
    struct Size { int width; int height; };
    const std::vector<Size> sizes{{2, 2}, {4, 3}, {8, 5}, {16, 10},
                                  {32, 20}, {50, 40}, {80, 50}};
    int frame = 0;
    for (const auto size : sizes) {
        const int x = 50 - size.width / 2;
        const int y = 50 - size.height / 2;
        auto image = green_blob_image(100, 100, x, y, size.width, size.height,
                                      50.0F, 50.0F);
        const auto result = detector.process(image, timestamp);
        check(result.valid == (frame >= 2),
              "growing image target becomes valid after confirmation");
        check(result.apparent_size >= previous_size,
              "filtered apparent size does not reverse during monotonic growth");
        check(std::fabs(result.center_x - 50.0F) < 1.0F &&
                  std::fabs(result.center_y - 50.0F) < 1.0F,
              "large-blob center remains stable");
        previous_size = result.apparent_size;
        timestamp += 16667;
        ++frame;
    }

    detector.reset();
    auto partial = green_blob_image(40, 30, 0, 8, 8, 8, 2.0F, 12.0F);
    detector.process(partial, timestamp);
    detector.process(partial, timestamp + 16667);
    const auto result = detector.process(partial, timestamp + 33334);
    check(result.valid, "partially clipped target at image edge is accepted");
}

void test_short_occlusion_and_multiple_candidates()
{
    dart::DetectorConfig config;
    dart::GreenLightDetector detector(config);
    uint64_t timestamp = 1000000;
    for (int frame = 0; frame < 3; ++frame) {
        auto image = green_blob_image(120, 90, 46, 36, 8, 8, 50.0F, 40.0F);
        detector.process(image, timestamp);
        timestamp += 16667;
    }

    for (int miss = 0; miss < 2; ++miss) {
        maix::image::Image blank(120, 90);
        blank.set_blobs({}, {});
        const auto result = detector.process(blank, timestamp);
        check(result.state == dart::TrackState::Tracking,
              "short occlusion does not drop TRACKING state");
        check(result.valid && result.predicted,
              "short occlusion returns a marked prediction");
        timestamp += 16667;
    }

    maix::image::Image multiple(120, 90);
    multiple.fill_rect(46, 36, 8, 8, 10, 250, 15);
    multiple.fill_rect(88, 68, 12, 12, 0, 255, 0);
    const maix::image::Blob target(46, 36, 8, 8, 50.0F, 40.0F, 64, 0.9F);
    const maix::image::Blob distractor(88, 68, 12, 12, 94.0F, 74.0F, 144, 1.0F);
    const maix::image::Blob target_core(48, 38, 4, 4, 50.0F, 40.0F, 16, 0.9F);
    const maix::image::Blob distractor_core(91, 71, 6, 6, 94.0F, 74.0F, 36, 1.0F);
    multiple.set_blobs({distractor, target}, {distractor_core, target_core});
    const auto result = detector.process(multiple, timestamp);
    check(result.valid && std::fabs(result.center_x - 50.0F) < 3.0F,
          "temporal association selects the prior target among green objects");
    check(result.state == dart::TrackState::Tracking,
          "target is reacquired after a short occlusion");
}

maix::image::Image low_contrast_green_blob(int center_x, int center_y)
{
    maix::image::Image image(120, 90);
    image.fill_rect(0, 0, 120, 90, 165, 165, 165);
    image.fill_rect(center_x - 4, center_y - 4, 8, 8, 60, 200, 120);
    image.set_blobs(
        {maix::image::Blob(center_x - 4, center_y - 4, 8, 8,
                           static_cast<float>(center_x),
                           static_cast<float>(center_y), 64, 0.9F)},
        {});
    return image;
}

void test_tracking_only_contrast_relaxation()
{
    dart::DetectorConfig config;
    // This case isolates the legacy LAB track-maintenance relaxation. The
    // normalized multi-scale path has its own acquisition tests below.
    config.enable_normalized_multiscale = false;
    config.min_local_contrast = -0.04F;
    config.min_tracking_local_contrast = -0.08F;
    config.contrast_relax_min_association = 0.75F;
    config.gate_min_px = 40.0F;
    config.gate_max_px = 40.0F;

    auto low_contrast = low_contrast_green_blob(50, 40);
    dart::GreenLightDetector acquiring_detector(config);
    const auto acquiring = acquiring_detector.process(low_contrast, 1000000);
    check(acquiring.state == dart::TrackState::Lost && !acquiring.valid,
          "low-contrast target cannot start a new track");

    dart::GreenLightDetector tracking_detector(config);
    uint64_t timestamp = 1000000;
    for (int frame = 0; frame < 3; ++frame) {
        auto high_contrast =
            green_blob_image(120, 90, 46, 36, 8, 8, 50.0F, 40.0F);
        tracking_detector.process(high_contrast, timestamp);
        timestamp += 16667;
    }
    const auto maintained = tracking_detector.process(low_contrast, timestamp);
    check(maintained.valid && maintained.state == dart::TrackState::Tracking,
          "strongly associated low-contrast target maintains tracking");

    timestamp += 16667;
    auto weakly_associated = low_contrast_green_blob(85, 40);
    const auto rejected =
        tracking_detector.process(weakly_associated, timestamp);
    check(rejected.valid && rejected.predicted &&
              rejected.state == dart::TrackState::Tracking &&
              std::fabs(rejected.center_x - 50.0F) < 5.0F,
          "low-contrast distractor is rejected while the prior track coasts");
}

void test_saturated_core_with_green_ring()
{
    dart::DetectorConfig config;
    // Keep this assertion scoped to saturated-core candidate filtering.
    config.enable_normalized_multiscale = false;
    config.enable_saturated_core_candidates = true;
    config.min_core_size_px = 3.0F;
    config.min_core_brightness = 0.88F;
    config.min_core_ring_green_dominance = 0.04F;
    config.center_prior_radius_px = 30.0F;
    config.initial_size_reference_px = 12.0F;
    config.weight_center_prior = 1.0F;
    config.weight_initial_size = 0.3F;
    config.camera_model.principal_x = 50.0F;
    config.camera_model.principal_y = 40.0F;

    maix::image::Image image(100, 80);
    image.fill_rect(38, 28, 24, 24, 0, 255, 0);
    image.fill_rect(45, 35, 10, 10, 255, 255, 255);
    const maix::image::Blob lamp_core(45, 35, 10, 10,
                                      49.5F, 39.5F, 100, 0.95F);
    const maix::image::Blob isolated_noise(51, 66, 1, 1,
                                           51.0F, 66.0F, 1, 1.0F);
    image.set_blobs({}, {isolated_noise, lamp_core});

    dart::GreenLightDetector detector(config);
    detector.process(image, 1000000);
    detector.process(image, 1016667);
    const auto result = detector.process(image, 1033334);
    check(result.valid, "saturated core surrounded by green is accepted");
    check(std::fabs(result.center_x - 49.5F) < 1.0F &&
              std::fabs(result.center_y - 39.5F) < 1.0F,
          "saturated core supplies the lamp center");
    check(result.bbox_w == 10 && result.bbox_h == 10,
          "saturated core supplies the apparent lamp bounds");
    check(detector.last_candidates().size() == 1U &&
              detector.last_candidates().front().saturated_core,
          "sub-minimum isolated highlights are rejected");
}

void draw_thick_line(maix::image::Image &image,
                     float x0,
                     float y0,
                     float x1,
                     float y1,
                     int thickness,
                     uint8_t red,
                     uint8_t green,
                     uint8_t blue)
{
    const int steps = std::max(
        1, static_cast<int>(std::ceil(std::hypot(x1 - x0, y1 - y0) * 2.0F)));
    const int radius = std::max(0, thickness / 2);
    for (int step = 0; step <= steps; ++step) {
        const float alpha = static_cast<float>(step) / steps;
        const int x = static_cast<int>(std::lround(
            (1.0F - alpha) * x0 + alpha * x1));
        const int y = static_cast<int>(std::lround(
            (1.0F - alpha) * y0 + alpha * y1));
        for (int py = -radius; py <= radius; ++py) {
            for (int px = -radius; px <= radius; ++px) {
                image.set_pixel(x + px, y + py, red, green, blue);
            }
        }
    }
}

maix::image::Image armor_target_image(bool rotated)
{
    maix::image::Image image(160, 140);
    image.fill_rect(0, 0, 160, 140, 12, 12, 12);
    float green_x = 80.0F;
    float green_y = 110.0F;
    if (rotated) {
        constexpr float inverse_sqrt_two = 0.70710678F;
        const float down_x = inverse_sqrt_two;
        const float down_y = inverse_sqrt_two;
        const float right_x = inverse_sqrt_two;
        const float right_y = -inverse_sqrt_two;
        const float armor_x = 70.0F;
        const float armor_y = 55.0F;
        green_x = armor_x + 42.0F * down_x;
        green_y = armor_y + 42.0F * down_y;
        for (const float side : {-1.0F, 1.0F}) {
            const float center_x = armor_x + side * 22.0F * right_x;
            const float center_y = armor_y + side * 22.0F * right_y;
            draw_thick_line(image,
                            center_x - 14.0F * down_x,
                            center_y - 14.0F * down_y,
                            center_x + 14.0F * down_x,
                            center_y + 14.0F * down_y,
                            3, 250, 8, 8);
        }
    } else {
        draw_thick_line(image, 50.0F, 30.0F, 50.0F, 70.0F,
                        3, 250, 8, 8);
        draw_thick_line(image, 110.0F, 30.0F, 110.0F, 70.0F,
                        3, 250, 8, 8);
    }
    const int lamp_x = static_cast<int>(std::lround(green_x));
    const int lamp_y = static_cast<int>(std::lround(green_y));
    image.fill_rect(lamp_x - 4, lamp_y - 4, 8, 8, 5, 250, 8);
    image.set_blobs(
        {maix::image::Blob(lamp_x - 4, lamp_y - 4, 8, 8,
                           green_x, green_y, 64, 0.95F)},
        {});
    return image;
}

void test_normalized_multiscale_and_capture_cone()
{
    dart::DetectorConfig config;
    config.enable_legacy_lab_candidates = false;
    config.enable_sparse_component_search = true;
    config.multiscale_downsample = 2;
    config.multiscale_min_scan_step_px = 4;
    config.multiscale_tracking_min_scan_step_px = 2;
    config.enable_capture_cone = false;
    config.min_candidate_score = 0.15F;
    config.min_tracking_score = 0.10F;
    maix::image::Image image(96, 72);
    image.fill_rect(0, 0, 96, 72, 20, 20, 20);
    image.fill_rect(46, 33, 5, 5, 5, 220, 8);
    image.set_blobs({}, {});
    dart::GreenLightDetector detector(config);
    detector.process(image, 1000000);
    detector.process(image, 1016667);
    const auto detection = detector.process(image, 1033334);
    check(detection.valid && std::fabs(detection.center_x - 48.0F) < 2.0F &&
              std::fabs(detection.center_y - 35.0F) < 2.0F,
          "multi-scale normalized response acquires a five-pixel lamp");
    check(!detector.last_candidates().empty() &&
              detector.last_candidates().front().normalized_response > 0.0F,
          "multi-scale candidate exposes its normalized response");

    maix::image::Image dark_noise(96, 72);
    dark_noise.fill_rect(0, 0, 96, 72, 180, 180, 180);
    dark_noise.set_pixel(48, 35, 6, 11, 4);
    dark_noise.set_blobs({}, {});
    dart::GreenLightDetector dark_detector(config);
    auto dark_result = dark_detector.process(dark_noise, 1000000);
    dark_result = dark_detector.process(dark_noise, 1016667);
    dark_result = dark_detector.process(dark_noise, 1033334);
    check(!dark_result.valid,
          "dark chromatic codec noise cannot become a normalized green track");

    config.enable_capture_cone = true;
    config.capture_cone_deg = 1.0F;
    config.camera_model.fx = 400.0F;
    config.camera_model.fy = 400.0F;
    config.camera_model.principal_x = 48.0F;
    config.camera_model.principal_y = 36.0F;
    auto outside = green_blob_image(96, 72, 78, 33, 5, 5, 80.0F, 35.0F);
    dart::GreenLightDetector cone_detector(config);
    auto cone_result = cone_detector.process(outside, 1000000);
    cone_result = cone_detector.process(outside, 1016667);
    cone_result = cone_detector.process(outside, 1033334);
    check(!cone_result.valid && !cone_detector.last_candidates().empty() &&
              !cone_detector.last_candidates().front().inside_capture_cone,
          "out-of-cone candidates are logged but cannot enter control tracking");
}

void test_immediate_full_cone_reacquire()
{
    dart::DetectorConfig config;
    config.enable_normalized_multiscale = false;
    config.enable_capture_cone = false;
    config.gate_min_px = 15.0F;
    config.gate_max_px = 20.0F;
    dart::GreenLightDetector detector(config);
    auto first = green_blob_image(120, 90, 26, 26, 8, 8, 30.0F, 30.0F);
    detector.process(first, 1000000);
    detector.process(first, 1016667);
    auto result = detector.process(first, 1033334);
    check(result.valid, "initial target is confirmed before reacquisition test");

    maix::image::Image blank(120, 90);
    blank.set_blobs({}, {});
    result = detector.process(blank, 1050001);
    check(result.valid && result.predicted,
          "one missing frame enters bounded coasting");

    auto second = green_blob_image(120, 90, 86, 56, 8, 8, 90.0F, 60.0F);
    result = detector.process(second, 1066668);
    check(!result.valid && result.state == dart::TrackState::Candidate,
          "far candidate starts a fresh confirmation immediately after loss");
    detector.process(second, 1083335);
    result = detector.process(second, 1100002);
    check(result.valid && std::fabs(result.center_x - 90.0F) < 3.0F,
          "full-cone reacquisition confirms the relocated target");
}

void test_armor_pair_geometry_and_clutter_budget()
{
    dart::DetectorConfig detector_config;
    detector_config.enable_normalized_multiscale = false;
    detector_config.enable_capture_cone = false;
    // Isolate bar geometry from green acquisition latency. The sequential
    // green-three/armor-three confirmation is covered by armor_stability_tests.
    detector_config.confirm_hits = 1;
    dart::ArmorConfig armor_config;
    armor_config.min_green_size_px = 1.0F;
    const auto detect = [&](maix::image::Image &image) {
        dart::GreenLightDetector detector(detector_config, armor_config);
        detector.process_target(image, 1000000);
        detector.process_target(image, 1016667);
        return detector.process_target(image, 1033334);
    };

    auto stacked = green_blob_image(160, 140, 76, 106, 8, 8, 80, 110);
    stacked.fill_rect(79, 30, 3, 15, 250, 8, 8);
    stacked.fill_rect(79, 65, 3, 15, 250, 8, 8);
    check(!detect(stacked).armor.valid,
          "two collinear fragments of one red bar cannot form an armor pair");

    auto white_pair = green_blob_image(160, 140, 76, 106, 8, 8, 80, 110);
    white_pair.fill_rect(49, 30, 3, 40, 250, 250, 250);
    white_pair.fill_rect(109, 30, 3, 40, 250, 250, 250);
    check(!detect(white_pair).armor.valid,
          "white structures above a green lamp do not count as red bars");
    white_pair.fill_rect(49, 30, 3, 40, 8, 8, 250);
    white_pair.fill_rect(109, 30, 3, 40, 8, 8, 250);
    check(!detect(white_pair).armor.valid,
          "blue structures cannot satisfy a configured red pair");

    auto clutter = green_blob_image(640, 200, 316, 156, 8, 8, 320, 160);
    clutter.fill_rect(309, 132, 3, 14, 200, 100, 100);
    clutter.fill_rect(329, 132, 3, 14, 200, 100, 100);
    for (int row = 0; row < 6; ++row) {
        for (int column = 0; column < 5; ++column) {
            clutter.fill_rect(5 + column * 12, 5 + row * 20,
                              1, 8, 250, 8, 8);
        }
    }
    const auto crowded = detect(clutter);
    check(crowded.armor.valid && std::fabs(crowded.armor.center.x - 320) < 1,
          "geometrically impossible clutter cannot consume the entire bar budget");
    clutter.fill_rect(305, 128, 30, 20, 0, 0, 0);
    check(!detect(clutter).armor.valid,
          "distant red clutter alone cannot form a target around the green lamp");

    auto too_wide = armor_target_image(false);
    check(detect(too_wide).armor.valid,
          "uncalibrated scale gate is disabled by default");
    armor_config.max_separation_to_green_size = 6.0F;
    check(!detect(too_wide).armor.valid,
          "optional green-size gate rejects an implausibly wide background pair");
    auto rotated_pair = armor_target_image(true);
    check(detect(rotated_pair).armor.valid,
          "green-size gate retains a smaller pair at arbitrary roll");
}

void test_rotated_armor_and_planar_pose()
{
    dart::DetectorConfig detector_config;
    detector_config.enable_normalized_multiscale = false;
    detector_config.enable_capture_cone = false;
    // Test rotation, ROI coordinates and PnP with an immediately known anchor;
    // armor_stability_tests independently exercise the full confirmation delay.
    detector_config.confirm_hits = 1;
    dart::ArmorConfig armor_config;
    armor_config.min_green_size_px = 1.0F;
    armor_config.required_pose_hits = 3;

    auto rotated = armor_target_image(true);
    dart::GreenLightDetector rotated_detector(detector_config, armor_config);
    rotated_detector.process_target(rotated, 1000000);
    rotated_detector.process_target(rotated, 1016667);
    const auto rotated_target =
        rotated_detector.process_target(rotated, 1033334);
    check(rotated_target.armor.valid &&
              rotated_target.armor.color == dart::ArmorColor::Red,
          "red armor pair is detected at arbitrary roll");
    check(std::fabs(rotated_target.armor.left_bar.bottom.x -
                    rotated_target.armor.left_bar.top.x) > 5.0F &&
              std::fabs(rotated_target.armor.left_bar.bottom.y -
                        rotated_target.armor.left_bar.top.y) > 5.0F,
          "armor pairing does not assume vertical image-space bars");
    check(rotated_target.guidance_mode == dart::GuidanceMode::Fused &&
              rotated_target.safe_for_control,
          "three armor observations begin the bounded fused aim transition");

    auto source_config=detector_config;
    source_config.camera_model.principal_x+=400;source_config.camera_model.principal_y+=200;
    dart::GreenLightDetector source_detector(source_config,armor_config);
    dart::TargetEstimate source_target;
    for(int n=0;n<3;++n) source_target=source_detector.process_region(
        rotated,{400,200,rotated.width(),rotated.height()},1344,760,1000000+n*16667);
    check(source_target.armor.valid &&
          std::fabs(source_target.armor.center.x-rotated_target.armor.center.x-400)<0.1F &&
          std::fabs(source_target.armor.left_bar.top.y-rotated_target.armor.left_bar.top.y-200)<0.1F,
          "rotated armor center and endpoints map from ROI to source coordinates");

    dart::TargetGeometryConfig geometry;
    geometry.pose_enabled = true;
    geometry.bar_separation_m = 0.12F;
    geometry.bar_length_m = 0.08F;
    geometry.green_offset_m = 0.12F;
    geometry.max_reprojection_error_px = 4.0F;
    geometry.min_pose_separation_px = 16.0F;
    detector_config.camera_model.fx = 400.0F;
    detector_config.camera_model.fy = 400.0F;
    detector_config.camera_model.principal_x = 80.0F;
    detector_config.camera_model.principal_y = 60.0F;
    auto frontal = armor_target_image(false);
    dart::GreenLightDetector pose_detector(detector_config, armor_config,
                                           geometry);
    pose_detector.process_target(frontal, 1000000);
    pose_detector.process_target(frontal, 1016667);
    const auto pose_target = pose_detector.process_target(frontal, 1033334);
    check(pose_target.pose.valid &&
              pose_target.pose.reprojection_error_px <= 4.0F,
          "complete separated endpoints produce a valid planar pose");
    check(std::fabs(pose_target.pose.distance_m - 0.8F) < 0.15F,
          "fronto-parallel pose recovers the synthetic target distance");

    geometry.min_pose_separation_px = 100.0F;
    dart::GreenLightDetector degenerate_detector(detector_config, armor_config,
                                                 geometry);
    degenerate_detector.process_target(frontal, 1000000);
    degenerate_detector.process_target(frontal, 1016667);
    const auto degenerate =
        degenerate_detector.process_target(frontal, 1033334);
    check(!degenerate.pose.valid,
          "insufficient endpoint separation is rejected as a PnP degeneracy");
}

class FakePoseValidator final : public dart::TargetPoseValidator {
public:
    dart::PoseValidation response;
    int calls = 0;
    dart::CandidateRoi last_roi;

    dart::PoseValidation validate(maix::image::Image &,
                                  const dart::CandidateRoi &roi) override
    {
        ++calls;
        last_roi = roi;
        return response;
    }
};

void test_npu_gate_and_control_prediction_budget()
{
    dart::DetectorConfig detector_config;
    detector_config.enable_normalized_multiscale = false;
    detector_config.enable_capture_cone = false;
    dart::NpuConfig npu_config;
    npu_config.enabled = true;
    npu_config.interval_ms = 33;
    auto validator = std::make_shared<FakePoseValidator>();
    validator->response.valid = true;
    validator->response.confidence = 0.90F;
    validator->response.keypoints[0] = {60.0F, 45.0F, true};
    dart::GreenLightDetector detector(detector_config, {}, {}, npu_config,
                                      validator);
    auto image = green_blob_image(120, 90, 56, 41, 8, 8, 60.0F, 45.0F);
    auto target = detector.process_target(image, 1000000);
    check(target.classical_detection_ran,
          "NPU scheduler starts with a classical-search frame");
    target = detector.process_target(image, 1016667);
    check(!target.classical_detection_ran && target.model_ran,
          "NPU inference uses the frame between classical searches");
    detector.process_target(image, 1033334);
    detector.process_target(image, 1050001);
    target = detector.process_target(image, 1066668);
    check(target.safe_for_control && validator->calls >= 2 &&
              validator->last_roi.width == npu_config.min_roi_size_px,
          "alternating classical/NPU slots gate a fixed-size candidate ROI");

    validator->response.keypoints[0] = {110.0F, 10.0F, true};
    target = detector.process_target(image, 1083335);
    check(target.valid && !target.safe_for_control,
          "a recent but spatially inconsistent model result cannot validate control");

    dart::GreenLightDetector unavailable_model(
        detector_config, {}, {}, npu_config, nullptr);
    unavailable_model.process_target(image, 1000000);
    unavailable_model.process_target(image, 1016667);
    unavailable_model.process_target(image, 1033334);
    unavailable_model.process_target(image, 1050001);
    target = unavailable_model.process_target(image, 1066668);
    check(target.valid && !target.safe_for_control,
          "enabled but unavailable NPU model fails control closed");

    dart::DetectorConfig scheduled_config = detector_config;
    scheduled_config.classical_interval_frames = 2;
    dart::GreenLightDetector scheduled_detector(scheduled_config);
    scheduled_detector.process_target(image, 1000000);
    scheduled_detector.process_target(image, 1016667);
    scheduled_detector.process_target(image, 1033334);
    scheduled_detector.process_target(image, 1050001);
    target = scheduled_detector.process_target(image, 1066668);
    check(target.valid && !target.predicted &&
              target.classical_detection_ran,
          "30 Hz classical observations confirm a track while output stays 60 Hz");
    target = scheduled_detector.process_target(image, 1083335);
    check(target.valid && target.predicted && target.safe_for_control &&
              !target.classical_detection_ran &&
              target.green.missed_frames == 0,
          "intentional scheduler coast is control-safe and does not consume a miss");

    dart::NpuConfig no_npu;
    dart::GreenLightDetector prediction_detector(detector_config, {}, {}, no_npu);
    prediction_detector.process_target(image, 1000000);
    prediction_detector.process_target(image, 1016667);
    target = prediction_detector.process_target(image, 1033334);
    check(target.safe_for_control, "observed confirmed target is control-safe");
    maix::image::Image blank(120, 90);
    blank.set_blobs({}, {});
    target = prediction_detector.process_target(blank, 1050001);
    check(target.predicted && target.safe_for_control,
          "first prediction remains within the two-frame safety budget");
    target = prediction_detector.process_target(blank, 1066668);
    check(target.predicted && target.safe_for_control,
          "second prediction remains within the 35 ms safety budget");
    target = prediction_detector.process_target(blank, 1083335);
    check(target.predicted && target.valid && !target.safe_for_control,
          "longer prediction is reported but explicitly control-unsafe");
}

void test_visual_motion_json_and_future_imu_interpolation()
{
    dart::VisualMotionConfig motion_config;
    motion_config.grid_width = 80;
    motion_config.grid_height = 60;
    motion_config.max_shift_px = 5;
    motion_config.min_response = 0.15F;
    maix::image::Image first(160, 120);
    maix::image::Image second(160, 120);
    for (int y = 0; y < 120; ++y) {
        for (int x = 0; x < 160; ++x) {
            const uint8_t value = static_cast<uint8_t>(
                (37 * x + 17 * y + 3 * x * y) & 0xff);
            first.set_pixel(x, y, value, value, value);
            second.set_pixel(x + 8, y + 4, value, value, value);
        }
    }
    dart::VisualMotionEstimator estimator(motion_config);
    const auto initial = estimator.update(first, 1000000);
    const auto shifted = estimator.update(second, 1016667);
    check(!initial.valid && shifted.valid &&
              std::fabs(shifted.image_dx_px - 8.0F) < 2.5F &&
              std::fabs(shifted.image_dy_px - 4.0F) < 2.5F,
          "sparse optical flow plus RANSAC estimates global image motion");

    dart::MotionPrior before;
    before.valid = true;
    before.timestamp_us = 1000000;
    before.orientation_wxyz = {1.0F, 0.0F, 0.0F, 0.0F};
    before.angular_velocity_rad_s = {0.0F, 0.0F, 0.0F};
    before.confidence = 0.8F;
    dart::MotionPrior after = before;
    after.timestamp_us = 1020000;
    after.orientation_wxyz = {0.0F, 0.0F, 0.0F, 1.0F};
    after.angular_velocity_rad_s = {0.0F, 0.0F, 2.0F};
    const auto interpolated =
        dart::interpolate_motion_prior(before, after, 1010000);
    check(interpolated.valid &&
              std::fabs(interpolated.orientation_wxyz[0] - 0.7071F) < 0.01F &&
              std::fabs(interpolated.angular_velocity_rad_s[2] - 1.0F) < 0.01F,
          "future IMU prior supports timestamp interpolation");

    dart::TargetEstimate target;
    target.timestamp_us = 123456;
    target.green.timestamp_us = 123456;
    target.green.center_x = 12.5F;
    target.green.center_y = 9.5F;
    const std::string json = dart::target_estimate_json(target);
    check(json.find("\"schema_version\":2") != std::string::npos &&
              json.find("\"green\":{") != std::string::npos &&
              json.find("\"center_x\":12.500000") != std::string::npos &&
              json.find("\"safe_for_control\":false") != std::string::npos,
          "schema v2 JSON retains nested and legacy flat green fields");
}

void test_configuration()
{
    const auto project_config =
        std::filesystem::path(TEST_PROJECT_ROOT) / "config" / "green_detector.conf";
    const auto config = dart::load_application_config(project_config.string());
    check(config.camera.width == 480 && config.camera.height == 360 &&
              config.camera.fps == 60 && config.camera.exposure_us == 500,
          "sample configuration parses");
    check(std::fabs(config.detector.min_association_score - 0.05F) < 1.0e-6F,
          "association setting parses");
    check(config.detector.enable_saturated_core_candidates &&
              !config.detector.merge_blobs,
          "saturated core and non-merging settings parse");
    check(std::fabs(config.detector.min_core_size_px - 3.0F) < 1.0e-6F &&
              config.detector.weight_center_prior == 0.0F &&
              std::fabs(config.detector.gate_max_px - 45.0F) < 1.0e-6F,
          "core size and center-neutral tracking settings parse");
    check(std::fabs(config.detector.min_tracking_local_contrast + 0.08F) <
                  1.0e-6F &&
              std::fabs(config.detector.contrast_relax_min_association - 0.75F) <
                  1.0e-6F &&
              config.detector.prediction_max_age_ms == 110,
          "tracking-only recovery settings parse");
    check(config.detector.enable_normalized_multiscale &&
              !config.detector.enable_legacy_lab_candidates &&
              config.detector.enable_sparse_component_search &&
              config.detector.multiscale_downsample == 1 &&
              config.detector.multiscale_diameters_px[2] == 6 &&
              std::fabs(config.detector.capture_cone_deg - 6.5F) < 1.0e-6F &&
              config.detector.control_prediction_max_frames == 2 &&
              config.detector.classical_interval_frames == 2 &&
              config.detector.multiscale_min_scan_step_px == 2 &&
              config.detector.multiscale_tracking_min_scan_step_px == 1 &&
              config.detector.multiscale_capture_cone_only &&
              std::fabs(config.detector.multiscale_tracking_roi_radius_px -
                        72.0F) < 1.0e-6F &&
              config.detector.multiscale_full_refresh_interval == 30,
          "v0.2 multi-scale, scheduler and fail-safe settings parse");
    check(config.armor.expected_color == dart::ArmorColor::Red &&
              !config.target_geometry.pose_enabled &&
              !config.npu.enabled && config.npu.input_size == 256 &&
              config.visual_motion.enabled &&
              config.visual_motion.tracking_only &&
              config.visual_motion.interval_frames == 2 &&
              config.visual_motion.grid_width == 48 &&
              config.visual_motion.grid_height == 36 &&
              config.debug.json_log_every_n_frames == 6 &&
              !config.debug.log_candidates,
          "armor, pose, NPU, motion and low-overhead log settings parse");

    const auto invalid_path =
        std::filesystem::temp_directory_path() / "dart_green_detector_invalid.conf";
    {
        std::ofstream invalid(invalid_path);
        invalid << "unknown.key=1\n";
    }
    bool threw = false;
    try {
        (void)dart::load_application_config(invalid_path.string());
    } catch (const std::runtime_error &) {
        threw = true;
    }
    std::filesystem::remove(invalid_path);
    check(threw, "unknown configuration keys are rejected");
    {
        std::ofstream configured(invalid_path);
        std::ifstream source(project_config);
        configured << source.rdbuf();
        configured << "armor.max_pair_longitudinal_to_length=0.45\n";
        configured << "armor.max_separation_to_green_size=6\n";
    }
    const auto alignment_config = dart::load_application_config(invalid_path.string());
    check(std::fabs(alignment_config.armor.max_pair_longitudinal_to_length - 0.45F) < 1.0e-6F &&
              alignment_config.armor.max_separation_to_green_size == 6.0F,
          "armor center-alignment and optional green-size tolerances are configurable");
    std::filesystem::remove(invalid_path);
    {
        std::ofstream invalid(invalid_path);
        std::ifstream source(project_config);
        invalid << source.rdbuf();
        invalid << "armor.max_pair_longitudinal_to_length=-0.1\n";
    }
    bool rejected_alignment = false;
    try { (void)dart::load_application_config(invalid_path.string()); }
    catch (const std::runtime_error &) { rejected_alignment = true; }
    std::filesystem::remove(invalid_path);
    check(rejected_alignment, "negative armor alignment tolerance is rejected");
    for(const auto *bad:{"highfps.green_hz=0", "highfps.green_hz=181", "highfps.armor_hz=91", "highfps.search_hz=-1"}) {
        {std::ofstream invalid(invalid_path);invalid<<bad<<'\n';}
        bool rejected=false;
        try {(void)dart::load_application_config(invalid_path.string());}
        catch(const std::runtime_error&){rejected=true;}
        std::filesystem::remove(invalid_path);
        check(rejected,"invalid cadence is rejected before camera access");
    }
}

void test_nv21_and_source_roi()
{
    for (const auto &dimensions : {std::pair<int,int>{480,360}, {640,480}, {1344,760}}) {
        const int w=dimensions.first,h=dimensions.second,stride=w+16;
        std::vector<uint8_t> y(stride*h,16), vu(stride*h/2,128);
        const int px=w-34,py=h-46;
        for(int yy=py;yy<py+8;++yy) for(int xx=px;xx<px+8;++xx) y[yy*stride+xx]=145;
        for(int yy=py/2;yy<(py+8)/2;++yy) for(int xx=px;xx<px+8;xx+=2) {
            vu[yy*stride+xx]=34;vu[yy*stride+xx+1]=54;
        }
        dart::Nv21View view{y.data(),vu.data(),w,h,stride,stride};
        auto proposals=dart::nv21_green_proposals(view);
        check(!proposals.empty(),"NV21 full field finds off-center green");
        if(proposals.empty()) continue;
        check(std::hypot(proposals[0].x-px,proposals[0].y-py)<12,"proposal in original pixels");
        auto r=dart::source_roi(proposals[0].x,proposals[0].y,96,w,h);
        auto image=dart::nv21_rgb_region(view,r);
        const auto*rgb=static_cast<const uint8_t*>(image->data());
        const int offset=((py-r.y)*r.width+px-r.x)*3;
        check(rgb[offset+1]>240 && rgb[offset]<10 && rgb[offset+2]<10,"NV21 VU and padded stride conversion");
        dart::DetectorConfig c; c.enable_capture_cone=false;
        // The square YUV swatch isolates stride/conversion/source-coordinate
        // behavior; resolved lamp shape is covered by lamp_appearance_tests.
        c.enable_lamp_appearance=false;
        c.camera_model={static_cast<float>(w),static_cast<float>(w),w/2.0F,h/2.0F};
        c.confirm_hits=1; c.confirm_window=1;
        dart::GreenLightDetector detector(c);
        auto target=detector.process_region(*image,r,w,h,10000);
        check(target.green.valid,"existing detector observes original-resolution ROI");
        check(std::hypot(target.green.center_x-(px+3.5F),target.green.center_y-(py+3.5F))<3,"ROI localization translated to source");
        auto snapshot=detector.tracker_snapshot();
        auto local=snapshot.roi_view(r.x,r.y);
        check(std::fabs(local.predicted_x()+r.x-snapshot.predicted_x())<0.01,"tracker normalized state survives ROI origin");
        auto predicted=snapshot.update(nullptr,20000,c.camera_model,false);
        check(predicted.measurement_age_us==10000 && predicted.missed_frames==0,"scheduled control prediction preserves observation miss budget");
        dart::invalidate_uncalibrated(target);
        check(!target.safe_for_control && !target.angles_valid && !target.pose.valid,"uncalibrated ROI is fail closed");
    }
    const auto c=dart::load_application_config(std::string(TEST_PROJECT_ROOT)+"/config/green_detector_full180.conf");
    check(c.camera.width==1344 && c.camera.height==760 && c.camera.fps==180 && !c.npu.enabled && !c.target_geometry.pose_enabled,"independent full180 config");
    check(c.highfps.green_hz==90 && c.highfps.armor_hz==60,"old full180 config retains verified cadence");
    const auto fast=dart::load_application_config(std::string(TEST_PROJECT_ROOT)+"/config/green_detector_full180_fast.conf");
    check(fast.highfps.green_hz==120 && fast.highfps.armor_hz==90 && fast.camera.fps==180 &&
          !fast.npu.enabled && !fast.target_geometry.pose_enabled,"faster observation config preserves camera and model gates");
    const auto maximum=dart::load_application_config(std::string(TEST_PROJECT_ROOT)+"/config/green_detector_full180_max.conf");
    check(maximum.highfps.green_hz==180 && maximum.highfps.armor_hz==120 && maximum.camera.fps==180,
          "maximum green cadence config remains a full180 camera profile");
}

void test_full180_fused_prediction()
{
    auto c=dart::load_application_config(std::string(TEST_PROJECT_ROOT)+"/config/green_detector_full180.conf");
    dart::detail::TemporalTracker tracker(c.detector);
    dart::detail::CandidateObservation lamp;lamp.center_x=672;lamp.center_y=380;lamp.apparent_size=12;lamp.score=.95;
    dart::TargetEstimate source;
    for(int n=0;n<10;++n) {
        source.source_received_us=1000000+n*11112;
        source.green=tracker.update(&lamp,source.source_received_us,c.detector.camera_model);
    }
    check(source.green.valid,"confirmed lamp for fused prediction test");
    source.armor.valid=true;source.armor_source_received_us=source.source_received_us;
    source.state=dart::GuidanceTrackState::Tracking;
    source.aim_point={source.green.center_x+20,source.green.center_y+30,true};
    for(auto mode:{dart::GuidanceMode::Fused,dart::GuidanceMode::ArmorImpact}) {
        source.guidance_mode=mode;
        auto predicted=dart::predict_full180_output(source,tracker,c,source.source_received_us+5556);
        check(predicted.guidance_mode==mode && predicted.armor.valid,"180Hz prediction preserves observed armor fusion mode");
        check(std::abs(predicted.aim_point.x-predicted.green.center_x-20)<.001 &&
              std::abs(predicted.aim_point.y-predicted.green.center_y-30)<.001,"fused aim offset follows lamp prediction");
        check(!predicted.safe_for_control && !predicted.angles_valid && !predicted.armor_detection_ran,
              "predicted geometry remains fail closed and is not a new armor observation");
    }
    auto expired=dart::predict_full180_output(source,tracker,c,
        source.source_received_us+static_cast<uint64_t>(c.armor.cache_max_age_ms)*1000+1);
    check(!expired.armor.valid && expired.guidance_mode==dart::GuidanceMode::LampApproach,
          "geometry expires from actual armor observation time");
    auto lost=dart::predict_full180_output(source,tracker,c,source.source_received_us+600000);
    check(!lost.valid && lost.state==dart::GuidanceTrackState::Search,"stopped measurements age out into full search");
}

void test_pipeline_shutdown_and_map_failure()
{
    auto c=dart::load_application_config(std::string(TEST_PROJECT_ROOT)+"/config/green_detector_full180.conf");
    struct Frame : dart::Nv21Frame {
        std::atomic<int> &released;
        bool fail;
        std::vector<uint8_t> pixels;
        Frame(std::atomic<int>&r,bool f):released(r),fail(f),pixels(1344*760*3/2,128){}
        ~Frame() override { ++released; }
        dart::Nv21View map() override {
            if(fail) throw std::runtime_error("injected DMA map failure");
            return {pixels.data(),pixels.data()+1344*760,1344,760,1344,1344};
        }
    };
    std::atomic<int> released{0};
    {
        dart::HighFpsPipeline pipeline(c);
        auto f=std::make_shared<Frame>(released,true);
        f->metadata={1,1,static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count())};
        pipeline.submit(std::move(f));
        for(int i=0;i<100 && !pipeline.failed();++i) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        check(pipeline.failed(),"map exception propagated without terminating process");
        pipeline.finish(); pipeline.finish();
    }
    check(released==1,"failed mapping frame released exactly once after joined cleanup");
    {
        dart::HighFpsPipeline pipeline(c,false,true,dart::PipelineInputSource::CachedVideo);
        for(int n=0;n<5;++n) {
            auto f=std::make_shared<Frame>(released,false);
            const auto stamp=static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
            f->metadata={static_cast<uint64_t>(n+1),stamp,stamp};
            pipeline.submit(std::move(f));
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        pipeline.finish();check(!pipeline.failed(),"async motion and control pipeline completes real synthetic image work");
        std::ifstream outputs("targets.jsonl");std::string line;
        bool saw_replay_output=false;
        while(std::getline(outputs,line)) {
            check(line.find("\"measurement_timestamp_source\":\"cached_video_submit_host_monotonic\"")!=std::string::npos,
                  "cached video outputs must not claim VIN receive timestamps");
            check(line.find("\"replay_diagnostic_only\":true")!=std::string::npos,
                  "cached video outputs are marked diagnostic");
            saw_replay_output=true;
            if(line.find("\"source_metadata_valid\":true")==std::string::npos) continue;
            const auto stamp=std::stoull(line.substr(line.find("\"timestamp_us\":")+15));
            const auto source=std::stoull(line.substr(line.find("\"source_received_us\":")+21));
            check(stamp>=source,"output timestamp follows acquired source snapshot");
        }
        check(saw_replay_output,"cached replay emitted source-labeled output");
    }
    check(released==6,"all submitted frames released after vision and motion join");
    {
        dart::HighFpsPipeline empty(c); empty.finish();
        check(!empty.failed(),"empty pipeline wakes and joins cleanly");
        std::ifstream summary("business.json"); std::string line;
        std::getline(summary,line);
        check(line.find("\"input_source\":\"vin\"")!=std::string::npos,
              "existing live pipeline retains VIN source by default");
    }
}

void test_integral_peak_equivalence()
{
    dart::DetectorConfig config;config.enable_capture_cone=false;
    config.enable_legacy_lab_candidates=false;config.enable_sparse_component_search=true;
    auto fast_config=config;fast_config.integral_peak_statistics=true;
    dart::GreenLightDetector slow(config),fast(fast_config);
    uint32_t noise=42;
    for(int frame=0;frame<12;++frame) {
        maix::image::Image image(128,128);
        for(int y=0;y<128;++y) for(int x=0;x<128;++x) {
            noise=1664525*noise+1013904223;
            image.set_pixel(x,y,noise>>24,(noise>>16)&255,(noise>>8)&255);
        }
        slow.process(image,1000000+frame*11112);fast.process(image,1000000+frame*11112);
        const auto &a=slow.last_candidates(), &b=fast.last_candidates();
        check(a.size()==b.size(),"integral exact statistics preserve candidate count");
        for(size_t n=0;n<std::min(a.size(),b.size());++n)
            check(a[n].center_x==b[n].center_x && a[n].center_y==b[n].center_y && a[n].score==b[n].score,
                  "integral exact statistics preserve coordinates and score without reducing hypothesis budget");
    }
}

void test_normalized_peak_green_evidence()
{
    const auto candidates_for = [](dart::DetectorConfig config,
                                    uint8_t red, uint8_t green, uint8_t blue,
                                    bool add_white_pixel = false) {
        maix::image::Image image(128, 128);
        image.fill_rect(0, 0, 128, 128, 12, 12, 12);
        image.fill_rect(58, 58, 11, 11, red, green, blue);
        if (add_white_pixel) {
            image.set_pixel(63, 63, 255, 255, 255);
        }
        dart::GreenLightDetector detector(config);
        detector.process(image, 1000000);
        return detector.last_candidates().size();
    };
    for (int mode = 0; mode < 4; ++mode) {
        dart::DetectorConfig config;
        config.enable_capture_cone = false;
        config.enable_legacy_lab_candidates = false;
        config.enable_saturated_core_candidates = false;
        config.enable_sparse_component_search = mode >= 2;
        config.integral_peak_statistics = mode == 3;
        config.multiscale_downsample = mode == 1 ? 2 : 1;
        // Isolate the older optional absolute-brightness gate. The default
        // appearance gate intentionally rejects the yellow control below.
        config.enable_lamp_appearance = false;
        check(config.normalized_min_peak_green == 0,
              "bright-green evidence remains opt-in for existing profiles");
        check(candidates_for(config, 10, 202, 15) > 0 &&
                  candidates_for(config, 250, 255, 15) > 0,
              "disabled evidence gate preserves normalized dim-green and yellow hypotheses");
        config.normalized_min_peak_green = 220;
        check(candidates_for(config, 10, 255, 15) > 0,
              "bright green lamp survives evidence gate in dense, sparse and integral paths");
        check(candidates_for(config, 10, 202, 15) == 0,
              "dim green paper lacks required bright green evidence");
        check(candidates_for(config, 250, 255, 15) == 0,
              "bright yellow lacks green-over-red evidence despite normalized response");
        check(candidates_for(config, 10, 202, 15, true) == 0,
              "separate white highlight cannot satisfy brightness and green dominance");
        check(candidates_for(config, 10, 220, 15) > 0 &&
                  candidates_for(config, 10, 219, 15) == 0,
              "bright green evidence threshold is inclusive and enforced in RGB units");
    }

    dart::DetectorConfig legacy;
    legacy.enable_normalized_multiscale = false;
    legacy.normalized_min_peak_green = 255;
    dart::GreenLightDetector legacy_detector(legacy);
    auto image = green_blob_image(120, 90, 46, 36, 8, 8, 50.0F, 40.0F);
    legacy_detector.process(image, 1000000);
    legacy_detector.process(image, 1016667);
    check(legacy_detector.process(image, 1033334).valid,
          "normalized evidence threshold does not filter legacy LAB detections");

    const auto path = std::filesystem::temp_directory_path() /
                      "dart_normalized_peak_evidence_test.conf";
    for (const int threshold : {-1, 0, 220, 255, 256}) {
        {
            std::ofstream file(path);
            std::ifstream source(std::filesystem::path(TEST_PROJECT_ROOT) /
                                 "config/green_detector.conf");
            file << source.rdbuf();
            file << "\nmultiscale.min_peak_green=" << threshold << '\n';
        }
        bool rejected = false;
        try {
            const auto parsed = dart::load_application_config(path.string());
            check(parsed.detector.normalized_min_peak_green == threshold,
                  "normalized green evidence threshold parses exactly");
        } catch (const std::runtime_error &) { rejected = true; }
        check(rejected == (threshold < 0 || threshold > 255),
              "normalized green evidence threshold rejects values outside 0..255");
    }
    std::filesystem::remove(path);
}

}  // namespace

int main()
{
    test_confirmation_and_loss();
    test_interrupted_acquisition_requires_new_consecutive_hits();
    test_prediction_timeout_and_recovery();
    test_growing_size_tracker();
    test_confirmation_requires_same_candidate();
    test_hard_association_gate();
    test_angles_and_distortion();
    test_single_pixel_and_reflection();
    test_large_growth_and_partial_frame();
    test_short_occlusion_and_multiple_candidates();
    test_tracking_only_contrast_relaxation();
    test_saturated_core_with_green_ring();
    test_normalized_multiscale_and_capture_cone();
    test_immediate_full_cone_reacquire();
    test_rotated_armor_and_planar_pose();
    test_armor_pair_geometry_and_clutter_budget();
    test_npu_gate_and_control_prediction_budget();
    test_visual_motion_json_and_future_imu_interpolation();
    test_configuration();
    test_nv21_and_source_roi();
    test_full180_fused_prediction();
    test_pipeline_shutdown_and_map_failure();
    test_integral_peak_equivalence();
    test_normalized_peak_green_evidence();

    if (failures != 0) {
        std::cerr << failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "all green detector tests passed\n";
    return 0;
}
