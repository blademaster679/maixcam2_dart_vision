#include "dart/green_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

namespace {
constexpr float pi = 3.14159265358979323846F;
int failures = 0;
void check(bool condition, const std::string &message) {
    if (!condition) { std::cerr << "FAIL: " << message << '\n'; ++failures; }
}

dart::DetectorConfig green_config() {
    dart::DetectorConfig config;
    config.enable_normalized_multiscale = false;
    config.enable_capture_cone = false;
    config.confirm_hits = 1;
    config.min_candidate_score = 0.1F;
    config.min_tracking_score = 0.1F;
    config.prediction_max_age_ms = 110;
    return config;
}
dart::ArmorConfig armor_config() {
    dart::ArmorConfig config;
    config.min_green_size_px = 1;
    // Exercise exposure scaling above the configurable sensor-noise floor.
    config.min_brightness = 0.025F;
    config.aim_blend_ms = 0;
    return config;
}

void paint_bar(maix::image::Image &image, float cx, float cy,
               float angle, float length, float width, float exposure) {
    const float dx = std::sin(angle), dy = std::cos(angle);
    const float rx = dy, ry = -dx;
    const int radius = static_cast<int>(std::ceil((length + width) / 2)) + 1;
    for (int y = static_cast<int>(cy) - radius; y <= cy + radius; ++y)
        for (int x = static_cast<int>(cx) - radius; x <= cx + radius; ++x) {
            const float px = x - cx, py = y - cy;
            if (std::fabs(px * dx + py * dy) <= length / 2 &&
                std::fabs(px * rx + py * ry) <= width / 2)
                image.set_pixel(x, y, static_cast<uint8_t>(240 * exposure),
                    static_cast<uint8_t>(24 * exposure), static_cast<uint8_t>(15 * exposure));
        }
}

// Known green blobs isolate pair detection from the independent lamp detector.
// All armor evidence is still extracted from RGB pixels by production code.
maix::image::Image scene(float degrees = 0, float scale = 1, float exposure = 1,
                         bool bars = true, bool lamp = true,
                         int size = 320, float gx = 160, float gy = 160) {
    maix::image::Image image(size, size);
    const auto background = static_cast<uint8_t>(6 * exposure);
    image.fill_rect(0, 0, size, size, background, background, background);
    const float angle = degrees * pi / 180;
    const float dx = std::sin(angle), dy = std::cos(angle);
    const float rx = dy, ry = -dx;
    const float cx = gx - 58 * scale * dx, cy = gy - 58 * scale * dy;
    if (bars) for (float side : {-1.0F, 1.0F})
        paint_bar(image, cx + side * 26 * scale * rx, cy + side * 26 * scale * ry,
                  angle, 32 * scale, 5 * scale, exposure);
    if (lamp) {
        const int radius = std::max(2, static_cast<int>(4 * scale));
        image.fill_rect(static_cast<int>(gx) - radius, static_cast<int>(gy) - radius,
                        2 * radius, 2 * radius, 5, 245, 8);
        image.set_blobs({maix::image::Blob(static_cast<int>(gx) - radius,
            static_cast<int>(gy) - radius, 2 * radius, 2 * radius, gx, gy)}, {});
    }
    return image;
}

dart::TargetEstimate region(dart::GreenLightDetector &detector, maix::image::Image &image,
                            uint64_t timestamp, bool scan = true) {
    return detector.process_region(image, {0, 0, image.width(), image.height()},
        image.width(), image.height(), timestamp, nullptr, false, scan);
}

void test_rotation_scale_exposure() {
    for (float angle : {0.0F, 30.0F, 75.0F, 90.0F, 135.0F, 180.0F, 225.0F, 270.0F, 330.0F})
        for (float scale : {0.5F, 1.0F, 1.5F})
            for (float exposure : {0.35F, 1.0F}) {
                dart::GreenLightDetector detector(green_config(), armor_config());
                auto image = scene(angle, scale, exposure);
                const auto first = region(detector, image, 1000000);
                const auto second = region(detector, image, 1016667);
                const auto third = region(detector, image, 1033334);
                const std::string label = "rotation=" + std::to_string(angle) +
                    " scale=" + std::to_string(scale) + " exposure=" + std::to_string(exposure);
                check(!first.armor.valid && !second.armor.valid,
                      "a single/two-frame pair is tentative: " + label);
                const float expected_x = 160 - 58 * scale * std::sin(angle * pi / 180);
                const float expected_y = 160 - 58 * scale * std::cos(angle * pi / 180);
                check(third.armor.valid &&
                      std::hypot(third.armor.center.x - expected_x,
                                 third.armor.center.y - expected_y) < 2,
                      "coherent pair confirms at arbitrary roll/scale/exposure: " + label);
                check(third.guidance_mode == dart::GuidanceMode::ArmorImpact,
                      "pose/fusion does not repeat the confirmation delay: " + label);
            }
}

maix::image::Image border_scene(int quarter_turns, bool touching) {
    auto source = scene(0, 1, 1, true, true, 160, touching ? 28 : 34, 108);
    maix::image::Image output(160, 160);
    auto rotate = [quarter_turns](int x, int y) {
        for (int n = 0; n < quarter_turns; ++n) { const int next_x = 159 - y; y = x; x = next_x; }
        return std::pair<int, int>{x, y};
    };
    const auto *pixels = static_cast<const uint8_t *>(source.data());
    for (int y = 0; y < 160; ++y) for (int x = 0; x < 160; ++x) {
        const auto p = rotate(x, y); const int index = (y * 160 + x) * 3;
        output.set_pixel(p.first, p.second, pixels[index], pixels[index + 1], pixels[index + 2]);
    }
    const auto green = rotate(touching ? 28 : 34, 108);
    output.set_blobs({maix::image::Blob(green.first - 4, green.second - 4,
        8, 8, static_cast<float>(green.first), static_cast<float>(green.second))}, {});
    return output;
}

void test_boundary_cannot_manufacture_a_bar() {
    for (int rotation = 0; rotation < 4; ++rotation) {
        dart::GreenLightDetector clipped(green_config(), armor_config());
        dart::GreenLightDetector complete(green_config(), armor_config());
        auto border = border_scene(rotation, true), interior = border_scene(rotation, false);
        for (int frame = 0; frame < 5; ++frame) {
            const auto cut = region(clipped, border, 1000000 + frame * 16667);
            const auto intact = region(complete, interior, 1000000 + frame * 16667);
            check(!cut.armor.valid, "edge-truncated component cannot form a pair at any image edge");
            if (frame >= 2) check(intact.armor.valid,
                "the same nearby pair wholly inside the crop remains detectable");
        }
    }
}

void test_confirmation_and_cache_lifecycle() {
    dart::GreenLightDetector detector(green_config(), armor_config());
    auto image = scene();
    check(!region(detector, image, 1000000).armor.valid, "first observation is tentative");
    for (int i = 1; i < 5; ++i) {
        const auto skipped = region(detector, image, 1000000 + i * 5000, false);
        check(!skipped.armor_detection_ran && !skipped.armor.valid,
              "scheduled skips cannot accumulate confirmation hits");
    }
    check(!region(detector, image, 1025000).armor.valid, "second actual scan remains tentative");
    check(region(detector, image, 1040000).armor.valid, "third actual scan confirms");
    const auto cached = region(detector, image, 1060000, false);
    check(!cached.armor_detection_ran && cached.armor.valid, "confirmed geometry can be cached briefly");
    const auto expired = region(detector, image, 1090001, false);
    check(!expired.armor.valid, "cache expiry is based on last actual scan, not last read");
    check(!region(detector, image, 1200000).armor.valid,
          "a long gap requires a new confirmation sequence");
    detector.reset();
    check(!region(detector, image, 2000000).armor.valid, "explicit reset clears armor history");
    check(!region(detector, image, 2000000).armor.valid,
          "reprocessing an identical timestamp does not add a second hit");
    check(!region(detector, image, 2016667).armor.valid, "only two distinct timestamps were observed");
    check(region(detector, image, 2033334).armor.valid, "three distinct fresh observations confirm");
}

void test_pair_switch_and_real_loss() {
    auto config = green_config(); config.prediction_max_age_ms = 0;
    dart::GreenLightDetector detector(config, armor_config());
    auto original = scene(), switched = scene(90), missing = scene(0, 1, 1, false);
    region(detector, original, 1000000); region(detector, original, 1016667);
    check(region(detector, original, 1033334).armor.valid, "initial pair confirms");
    const auto jump = region(detector, switched, 1050001);
    check(jump.armor_detection_ran && !jump.armor.valid,
          "an abrupt change to a geometrically different pair cannot inherit confirmation or cache");
    check(!region(detector, original, 1066668).armor.valid,
          "returning after an inconsistent scan starts a fresh coherent sequence");
    check(!region(detector, original, 1083335).armor.valid, "two recovery observations remain tentative");
    check(region(detector, original, 1100002).armor.valid, "third recovery observation confirms");
    const auto absent = region(detector, missing, 1116669);
    check(absent.armor_detection_ran && !absent.armor.valid,
          "a real scan with no pair invalidates old geometry immediately");
    region(detector, original, 1133336); region(detector, original, 1150003);
    check(region(detector, original, 1166670).armor.valid, "pair can recover after missing bars");
    auto blank = scene(0, 1, 1, false, false);
    check(!region(detector, blank, 1183337).armor.valid, "loss of the green anchor clears armor");
    check(!region(detector, original, 1200004).armor.valid,
          "reappearing after real green loss cannot reuse old armor hit history");
}

void test_smooth_motion_and_predicted_anchor() {
    auto config = green_config(); config.classical_interval_frames = 2;
    dart::GreenLightDetector interleaved(config, armor_config());
    auto image = scene();
    for (int frame = 0; frame < 6; ++frame) {
        const auto target = interleaved.process_target(image, 1000000 + frame * 16667);
        check(target.armor_detection_ran == (frame % 2 == 1),
              "legacy green/armor interleaving is preserved");
        check(target.armor.valid == (frame == 5),
              "only fresh pair scans count while using a recent predicted green anchor");
    }
    dart::GreenLightDetector moving(green_config(), armor_config());
    for (int frame = 0; frame < 7; ++frame) {
        const float roll = 150 + frame * 5.0F, scale = 0.8F + frame * 0.06F;
        auto current = scene(roll, scale, frame % 2 ? 0.4F : 0.9F, true, true,
                             320, 146 + frame * 2.0F, 148 + frame * 1.0F);
        const auto target = region(moving, current, 1000000 + frame * 16667);
        check(target.armor.valid == (frame >= 2),
              "smooth translation, scale, roll and exposure changes preserve coherent confirmation");
    }
}

void test_unconfirmed_green_does_not_scan_or_confirm_armor() {
    auto config = green_config();
    config.confirm_hits = 3;
    config.confirm_window = 5;
    auto image = scene();
    dart::GreenLightDetector detector(config, armor_config());
    for (int frame = 0; frame < 2; ++frame) {
        const auto result = region(detector, image, 1000000 + frame * 16667);
        check(result.green.state == dart::TrackState::Candidate && !result.green.valid,
              "green observations remain tentative until their confirmation threshold");
        check(!result.armor_detection_ran && !result.armor.valid,
              "unconfirmed green acquisition cannot spend an armor scan or add a pair hit");
    }
    const auto first = region(detector, image, 1033334);
    check(first.green.valid && first.armor_detection_ran && !first.armor.valid,
          "green confirmation starts the first actual armor observation");
    const auto skipped = region(detector, image, 1050001, false);
    check(!skipped.armor_detection_ran && !skipped.armor.valid,
          "a scheduled skip after green confirmation cannot advance armor hits");
    check(!region(detector, image, 1066668).armor.valid,
          "the second fresh pair after green confirmation stays tentative");
    check(region(detector, image, 1083335).armor.valid,
          "three fresh pair scans after green confirmation establish armor");

    dart::GreenLightDetector diagnostic(config, armor_config());
    for (int frame = 0; frame < 2; ++frame) {
        const auto result = diagnostic.process_region(image, {0, 0, image.width(), image.height()},
            image.width(), image.height(), 2000000 + frame * 16667, nullptr, true, true);
        check(!result.green.valid && result.armor_detection_ran && !result.armor.valid,
              "explicit diagnostic scan runs on a candidate without contributing confirmation");
    }
    check(!region(diagnostic, image, 2033334).armor.valid,
          "forced candidate scans cannot pre-fill the first confirmed-anchor pair hit");
    check(!region(diagnostic, image, 2050001).armor.valid,
          "forced candidate scans cannot shorten three-scan pair confirmation");
    check(region(diagnostic, image, 2066668).armor.valid,
          "diagnostic mode still requires three actual scans with a confirmed anchor");
}

void test_bar_appearance_rejects_persistent_colored_edges() {
    // Correct geometry and repeated scans cannot establish that a colored
    // surface emits light. Exercise hues and contrast independently of pose.
    struct Surface { uint8_t r, g, b, background; };
    const Surface surfaces[] = {
        {220, 215, 35, 6},   // yellow: red exceeds the average, not both channels
        {220, 35, 215, 6},   // magenta
        {110, 106, 60, 6},   // weak red cast on a warm edge
        {120, 70, 35, 140},  // red/brown edge darker than its surroundings
        {125, 95, 65, 100},  // colored surface without a local luminance peak
    };
    for (float degrees : {0.0F, 47.0F, 90.0F, 153.0F}) {
        for (const auto &surface : surfaces) {
            auto image = scene(degrees);
            auto *pixels = static_cast<uint8_t *>(image.data());
            for (int i = 0; i < image.width() * image.height(); ++i) {
                auto *p = pixels + i * 3;
                if (p[0] == 240) { p[0] = surface.r; p[1] = surface.g; p[2] = surface.b; }
                else if (p[0] == 6 && p[1] == 6 && p[2] == 6)
                    p[0] = p[1] = p[2] = surface.background;
            }
            dart::GreenLightDetector detector(green_config(), armor_config());
            for (int frame = 0; frame < 8; ++frame)
                check(!region(detector, image, 1000000 + frame * 16667).armor.valid,
                    "persistent colored edges cannot become measured armor, rotation=" +
                    std::to_string(degrees));
        }
    }
}

void test_bar_appearance_preserves_white_core_and_blue() {
    for (float degrees : {0.0F, 47.0F, 90.0F, 153.0F})
        for (bool blue : {false, true}) {
            auto image = scene(degrees);
            const float angle = degrees * pi / 180;
            const float dx = std::sin(angle), dy = std::cos(angle);
            const float cx = 160 - 58 * dx, cy = 160 - 58 * dy;
            auto *pixels = static_cast<uint8_t *>(image.data());
            for (int y = 0; y < image.height(); ++y)
                for (int x = 0; x < image.width(); ++x) {
                    auto *p = pixels + (y * image.width() + x) * 3;
                    if (p[0] != 240) continue;
                    // A closed colored rim surrounds an overexposed neutral core.
                    bool core = false;
                    for (float side : {-1.0F, 1.0F}) {
                        const float px = x - (cx + side * 26 * dy);
                        const float py = y - (cy - side * 26 * dx);
                        core = core || (std::fabs(px * dx + py * dy) < 13 &&
                                        std::fabs(px * dy - py * dx) < 1.2F);
                    }
                    if (core) { p[0] = 250; p[1] = 245; p[2] = 240; }
                    else if (blue) std::swap(p[0], p[2]);
                }
            auto config = armor_config();
            if (blue) config.expected_color = dart::ArmorColor::Blue;
            dart::GreenLightDetector detector(green_config(), config);
            for (int frame = 0; frame < 4; ++frame) {
                const auto result = region(detector, image, 1000000 + frame * 16667);
                if (frame >= 2) check(result.armor.valid,
                    "colored bars with neutral bright cores remain observable at arbitrary roll");
            }
        }
}

void test_bar_appearance_with_one_sided_colored_rim() {
    // Clipping and chroma subsampling may leave colored support only on one
    // side of a neutral bright core. The colored component's PCA center is
    // then not the luminance ridge. Do not assume they coincide.
    for (float degrees : {0.0F, 47.0F, 90.0F, 153.0F})
        for (bool blue : {false, true}) {
            auto image = scene(degrees);
            const float angle = degrees * pi / 180;
            const float dx = std::sin(angle), dy = std::cos(angle);
            const float cx = 160 - 58 * dx, cy = 160 - 58 * dy;
            auto *pixels = static_cast<uint8_t *>(image.data());
            for (int y = 0; y < image.height(); ++y)
                for (int x = 0; x < image.width(); ++x) {
                    auto *p = pixels + (y * image.width() + x) * 3;
                    if (p[0] != 240) continue;
                    bool core = false;
                    for (float side : {-1.0F, 1.0F}) {
                        const float px = x - (cx + side * 26 * dy);
                        const float py = y - (cy - side * 26 * dx);
                        core = core || (std::fabs(px * dx + py * dy) <= 16 &&
                                        px * dy - py * dx < 1.3F &&
                                        px * dy - py * dx >= -2.5F);
                    }
                    if (core) { p[0] = 250; p[1] = 245; p[2] = 240; }
                    else if (blue) std::swap(p[0], p[2]);
                }
            auto config = armor_config();
            if (blue) config.expected_color = dart::ArmorColor::Blue;
            dart::GreenLightDetector detector(green_config(), config);
            for (int frame = 0; frame < 4; ++frame) {
                const auto result = region(detector, image, 1000000 + frame * 16667);
                if (frame >= 2) check(result.armor.valid,
                    "one-sided colored rim can support a complete bright bar, angle=" +
                    std::to_string(degrees));
            }
        }
}

void test_bright_surface_next_to_colored_edge_is_not_a_light_core() {
    for (float degrees : {0.0F, 47.0F, 90.0F, 153.0F}) {
        auto image = scene(degrees);
        const float angle = degrees * pi / 180;
        const float dx = std::sin(angle), dy = std::cos(angle);
        const float cx = 160 - 58 * dx, cy = 160 - 58 * dy;
        for (int y = 0; y < image.height(); ++y)
            for (int x = 0; x < image.width(); ++x)
                for (float side : {-1.0F, 1.0F}) {
                    const float px = x - (cx + side * 26 * dy);
                    const float py = y - (cy - side * 26 * dx);
                    const float across = px * dy - py * dx;
                    if (std::fabs(px * dx + py * dy) < 35 && across > 2.5F && across < 35)
                        image.set_pixel(x, y, 230, 230, 230);
                }
        dart::GreenLightDetector detector(green_config(), armor_config());
        for (int frame = 0; frame < 8; ++frame)
            check(!region(detector, image, 1000000 + frame * 16667).armor.valid,
                "an unbounded adjacent bright surface cannot supply a bar's white core");
    }
}
} // namespace

int main() {
    test_rotation_scale_exposure();
    test_boundary_cannot_manufacture_a_bar();
    test_confirmation_and_cache_lifecycle();
    test_pair_switch_and_real_loss();
    test_smooth_motion_and_predicted_anchor();
    test_unconfirmed_green_does_not_scan_or_confirm_armor();
    test_bar_appearance_rejects_persistent_colored_edges();
    test_bar_appearance_preserves_white_core_and_blue();
    test_bar_appearance_with_one_sided_colored_rim();
    test_bright_surface_next_to_colored_edge_is_not_a_light_core();
    if (failures) { std::cerr << failures << " armor stability checks failed\n"; return 1; }
    std::cout << "armor stability checks passed\n";
    return 0;
}
