#include "dart/green_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace {
constexpr int width = 128;
constexpr int height = 96;
constexpr float pi = 3.14159265358979323846F;
int failures = 0;
int positive_scenes = 0;
int negative_scenes = 0;

void check(bool condition, const std::string &message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

dart::DetectorConfig config(bool integral)
{
    dart::DetectorConfig result;
    result.enable_normalized_multiscale = true;
    result.enable_legacy_lab_candidates = false;
    result.enable_sparse_component_search = true;
    result.enable_capture_cone = false;
    result.integral_peak_statistics = integral;
    // Leave acquisition, tracking and appearance thresholds at their defaults.
    // These tests exercise real RGB extraction, never injected LAB blobs.
    return result;
}

uint8_t exposed(int value, float exposure)
{
    return static_cast<uint8_t>(std::clamp(std::lround(value * exposure), 0L, 255L));
}

maix::image::Image background(float exposure = 1.0F)
{
    maix::image::Image image(width, height);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int brightness = 10 + (x + y) / 30;
            image.set_pixel(x, y, exposed(brightness, exposure),
                            exposed(brightness, exposure),
                            exposed(brightness, exposure));
        }
    }
    return image;
}

void ellipse(maix::image::Image &image, float cx, float cy, float diameter,
             float angle, float exposure, bool white_core)
{
    const float cosine = std::cos(angle), sine = std::sin(angle);
    const float major = diameter * 0.5F;
    const float minor = diameter * 0.42F;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const float dx = x - cx, dy = y - cy;
            const float u = (dx * cosine + dy * sine) / major;
            const float v = (-dx * sine + dy * cosine) / minor;
            const float radius_squared = u * u + v * v;
            if (radius_squared > 1.0F) continue;
            if (white_core && radius_squared < 0.20F) {
                image.set_pixel(x, y, exposed(250, exposure), exposed(252, exposure),
                                exposed(248, exposure));
            } else {
                image.set_pixel(x, y, exposed(22, exposure), exposed(240, exposure),
                                exposed(30, exposure));
            }
        }
    }
}

dart::GreenLightDetection run(dart::GreenLightDetector &detector,
                             maix::image::Image &image, int frame)
{
    return detector.process(image, 1000000 + static_cast<uint64_t>(frame) * 11112);
}

void test_lamps_across_scale_exposure_position_and_orientation()
{
    struct Position { float x; float y; };
    const std::vector<Position> positions{{25, 23}, {64, 48}, {101, 69}};
    for (bool integral : {false, true}) {
        for (bool white_core : {false, true}) {
            for (float diameter : {3.0F, 5.0F, 9.0F, 15.0F}) {
                for (float exposure : {0.45F, 1.0F}) {
                    for (float degrees : {0.0F, 47.0F, 123.0F}) {
                        const auto position = positions[positive_scenes % positions.size()];
                        auto image = background(exposure);
                        ellipse(image, position.x, position.y, diameter,
                                degrees * pi / 180.0F, exposure, white_core);
                        dart::GreenLightDetector detector(config(integral));
                        dart::GreenLightDetection output;
                        for (int frame = 0; frame < 5; ++frame) output = run(detector, image, frame);
                        const auto label = "diameter=" + std::to_string(diameter) +
                            " exposure=" + std::to_string(exposure) +
                            " rotation=" + std::to_string(degrees) +
                            " white_core=" + std::to_string(white_core) +
                            " integral=" + std::to_string(integral);
                        check(output.valid && !output.predicted,
                              "compact emissive lamp confirms: " + label);
                        check(!output.valid || std::hypot(output.center_x - position.x,
                                                         output.center_y - position.y) < 2.5F,
                              "confirmed lamp stays near its RGB evidence: " + label);
                        ++positive_scenes;
                    }
                }
            }
        }
    }
}

void reject_persistent_scene(maix::image::Image &image, const std::string &label)
{
    for (bool integral : {false, true}) {
        dart::GreenLightDetector detector(config(integral));
        bool confirmed = false;
        for (int frame = 0; frame < 12; ++frame) {
            const auto output = run(detector, image, frame);
            confirmed = confirmed || output.valid;
        }
        check(!confirmed, "persistent non-lamp cannot acquire a track: " + label +
                          " integral=" + std::to_string(integral));
        ++negative_scenes;
    }
}

void test_persistent_colored_backgrounds()
{
    for (const auto &color : std::vector<std::vector<int>>{
             {235, 230, 25}, {240, 247, 65}, {25, 230, 230}, {25, 240, 230},
             {240, 245, 240}, {255, 255, 255}}) {
        auto image = background();
        image.fill_rect(56, 40, 13, 13, color[0], color[1], color[2]);
        reject_persistent_scene(image, "bright RGB(" + std::to_string(color[0]) + "," +
                                std::to_string(color[1]) + "," + std::to_string(color[2]) + ")");
    }
    for (const auto &color : std::vector<std::vector<int>>{
             {190, 210, 192}, {205, 225, 203}, {180, 206, 181}, {130, 148, 130}}) {
        for (bool white_core : {false, true}) {
            auto image = background();
            for (int y = 37; y <= 53; ++y) for (int x = 50; x <= 66; ++x) {
                const int radius_squared = (x - 58) * (x - 58) + (y - 45) * (y - 45);
                if (radius_squared > 36) continue;
                if (white_core && radius_squared < 8)
                    image.set_pixel(x, y, 248, 250, 247);
                else image.set_pixel(x, y, color[0], color[1], color[2]);
            }
            reject_persistent_scene(image, "compact neutral highlight with weak green cast RGB(" +
                std::to_string(color[0]) + "," + std::to_string(color[1]) + "," +
                std::to_string(color[2]) + ") white_core=" + std::to_string(white_core));
        }
    }
    auto flat = background();
    flat.fill_rect(0, 0, width, height, 25, 180, 40);
    reject_persistent_scene(flat, "uniform green field");

    auto broad = background();
    broad.fill_rect(24, 18, 78, 60, 25, 180, 40);
    reject_persistent_scene(broad, "large flat green rectangle, including its corners");

    auto highlight = background();
    highlight.fill_rect(24, 18, 78, 60, 25, 180, 40);
    highlight.fill_rect(60, 43, 7, 7, 255, 255, 255);
    reject_persistent_scene(highlight, "white reflection inside a broad green surface");

    auto impulse = background();
    impulse.set_pixel(60, 46, 22, 240, 30);
    reject_persistent_scene(impulse, "isolated chromatic impulse pixel");
    impulse.set_pixel(62, 46, 22, 240, 30);
    reject_persistent_scene(impulse, "disconnected impulses cannot share color evidence");

    for (int edge = 0; edge < 4; ++edge) {
        auto image = background();
        if (edge == 0) image.fill_rect(0, 0, width / 2, height, 25, 180, 40);
        if (edge == 1) image.fill_rect(width / 2, 0, width / 2, height, 25, 180, 40);
        if (edge == 2) image.fill_rect(0, 0, width, height / 2, 25, 180, 40);
        if (edge == 3) image.fill_rect(0, height / 2, width, height / 2, 25, 180, 40);
        reject_persistent_scene(image, "green half-plane edge=" + std::to_string(edge));
    }
    for (float degrees : {0.0F, 33.0F, 90.0F, 147.0F}) {
        auto image = background();
        const float angle = degrees * pi / 180.0F;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const float along = (x - 64) * std::cos(angle) + (y - 48) * std::sin(angle);
                const float across = -(x - 64) * std::sin(angle) + (y - 48) * std::cos(angle);
                if (std::fabs(along) < 35 && std::fabs(across) < 1.5F)
                    image.set_pixel(x, y, 22, 235, 30);
            }
        }
        reject_persistent_scene(image, "thin green line rotation=" + std::to_string(degrees));
    }
}

void test_occlusion_and_appearance_loss()
{
    for (bool integral : {false, true}) {
        auto visible = background();
        ellipse(visible, 67, 45, 9, 0.4F, 1, true);
        auto blank = background();
        auto changed = background();
        changed.fill_rect(62, 40, 11, 11, 240, 247, 65);
        dart::GreenLightDetector detector(config(integral));
        for (int frame = 0; frame < 5; ++frame) run(detector, visible, frame);
        for (int frame = 5; frame < 7; ++frame) {
            const auto output = run(detector, blank, frame);
            check(output.valid && output.predicted && output.measurement_age_us > 0,
                  "brief occlusion retains only an explicitly predicted track");
        }
        const auto recovered = run(detector, visible, 7);
        check(recovered.valid && !recovered.predicted,
              "real lamp reappearance resumes measured tracking");
        dart::GreenLightDetection last;
        for (int frame = 8; frame < 24; ++frame) {
            last = run(detector, changed, frame);
            check(!(last.valid && !last.predicted),
                  "association cannot turn an appearance-rejected patch into a fresh measurement");
        }
        check(!last.valid, "appearance loss eventually expires the old track");
    }
}

void test_lamps_on_chromatic_backgrounds()
{
    for (bool integral : {false, true}) {
        for (int ambient_green : {24, 45}) {
            for (int peak_green : {90, 170}) {
                for (float diameter : {5.0F, 11.0F}) {
                    maix::image::Image image(width, height);
                    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
                        // Gradual shadow and optical blur: background hue is
                        // green too, but a real lamp has local signal excess.
                        const float ambient = 0.6F + 0.4F * x / width;
                        const float dx = x - 65.0F, dy = y - 43.0F;
                        const float gaussian = std::exp(-8.0F * (dx * dx + dy * dy) /
                                                        (diameter * diameter));
                        const int red = static_cast<int>(ambient * 8 + gaussian * 8);
                        const int green = static_cast<int>(ambient * ambient_green +
                                                          gaussian * (peak_green - ambient_green));
                        const int blue = static_cast<int>(ambient * 12 + gaussian * 8);
                        image.set_pixel(x, y, red, green, blue);
                    }
                    dart::GreenLightDetector detector(config(integral));
                    dart::GreenLightDetection output;
                    for (int frame = 0; frame < 5; ++frame) output = run(detector, image, frame);
                    check(output.valid && !output.predicted &&
                          std::hypot(output.center_x - 65, output.center_y - 43) < 2.5F,
                          "locally brighter blurred lamp survives green ambient illumination: bg=" +
                          std::to_string(ambient_green) + " peak=" + std::to_string(peak_green) +
                          " diameter=" + std::to_string(diameter));
                    ++positive_scenes;
                }
            }
        }
    }
}

void test_large_lamps_and_gray_mount()
{
    for (bool integral : {false, true}) {
        for (float diameter : {25.0F, 35.0F, 51.0F}) {
            for (bool white_core : {false, true}) {
                auto image = background();
                ellipse(image, 64, 48, diameter, 0.7F, 0.7F, white_core);
                dart::GreenLightDetector detector(config(integral));
                dart::GreenLightDetection output;
                for (int frame = 0; frame < 5; ++frame) output = run(detector, image, frame);
                check(output.valid && !output.predicted &&
                      std::hypot(output.center_x - 64, output.center_y - 48) < 2.5F,
                      "nearby larger lamp is validated as one whole emitter: diameter=" +
                      std::to_string(diameter) + " white_core=" + std::to_string(white_core));
                ++positive_scenes;
            }
        }
        auto mounted = background();
        mounted.fill_rect(68, 45, 60, 7, 70, 70, 70);
        ellipse(mounted, 64, 48, 9, 0, 0.5F, false);
        dart::GreenLightDetector detector(config(integral));
        dart::GreenLightDetection output;
        for (int frame = 0; frame < 5; ++frame) output = run(detector, mounted, frame);
        check(output.valid && !output.predicted &&
              std::hypot(output.center_x - 64, output.center_y - 48) < 2.5F,
              "dim gray mount touching the lamp is not a saturated bright core");
        ++positive_scenes;
    }
}

void test_dim_white_core_with_colored_halo()
{
    for (bool integral : {false, true}) {
        for (float exposure : {0.35F, 0.65F}) {
            for (int ambient_green : {14, 28}) {
                maix::image::Image image(width, height);
                for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
                    const float shadow = 0.65F + 0.35F * y / height;
                    image.set_pixel(x, y, static_cast<int>(8 * shadow),
                                    static_cast<int>(ambient_green * shadow),
                                    static_cast<int>(12 * shadow));
                }
                ellipse(image, 63, 45, 9, 1.1F, exposure, true);
                dart::GreenLightDetector detector(config(integral));
                dart::GreenLightDetection output;
                for (int frame = 0; frame < 5; ++frame) output = run(detector, image, frame);
                check(output.valid && !output.predicted &&
                      std::hypot(output.center_x - 63, output.center_y - 45) < 2.5F,
                      "dim white core retains genuine colored halo under mixed ambient light");
                ++positive_scenes;
            }
        }
    }
}

void test_dark_color_cannot_identify_a_bright_neutral_reflection()
{
    for (float diameter : {9.0F, 15.0F}) for (float exposure : {0.6F, 1.0F}) {
        auto reflection = background(exposure);
        auto lamp = background(exposure);
        ellipse(reflection, 61, 43, diameter, 0.7F, 1, true);
        ellipse(lamp, 61, 43, diameter, 0.7F, 1, true);
        for (auto *image : {&reflection, &lamp}) {
            auto *pixels = static_cast<uint8_t *>(image->data());
            for (int i = 0; i < width * height; ++i) {
                auto *p = pixels + 3 * i;
                if (p[1] == 252) {
                    p[0] = exposed(150, exposure); p[1] = exposed(160, exposure);
                    p[2] = exposed(151, exposure);
                } else if (p[1] == 240) {
                    p[0] = exposed(10, exposure);
                    p[1] = exposed(image == &reflection ? 55 : 145, exposure);
                    p[2] = exposed(16, exposure);
                }
            }
        }
        reject_persistent_scene(reflection,
            "a dark chromatic rim cannot identify a bright neutral reflection");
        for (bool integral : {false, true}) {
            dart::GreenLightDetector detector(config(integral));
            dart::GreenLightDetection output;
            for (int frame = 0; frame < 5; ++frame) output = run(detector, lamp, frame);
            check(output.valid && !output.predicted,
                "sufficiently bright colored halo still identifies a neutral lamp core");
            ++positive_scenes;
        }
    }
}

// A low-contrast halo may connect separate bright strokes into a compact, filled
// component. Shape evidence must describe the luminous interior as well as its
// faint perimeter. These are resolved glyphs, not ambiguous two-pixel lamps.
void test_luminous_interior_across_shape_exposure_and_rotation()
{
    struct Position { float x; float y; };
    const Position positions[]{{37, 29}, {89, 65}};
    int scene = 0;
    for (float diameter : {13.0F, 21.0F}) {
        for (float exposure : {0.55F, 1.0F}) {
            for (float degrees : {0.0F, 37.0F, 83.0F}) {
                const auto position = positions[scene++ % 2];
                const float angle = degrees * pi / 180.0F;
                const float cosine = std::cos(angle), sine = std::sin(angle);
                const auto label = "diameter=" + std::to_string(diameter) +
                    " exposure=" + std::to_string(exposure) +
                    " rotation=" + std::to_string(degrees);
                for (int pattern = 0; pattern < 3; ++pattern) {
                    auto image = background(exposure);
                    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
                        const float dx = x - position.x, dy = y - position.y;
                        const float u = (dx * cosine + dy * sine) / (diameter * 0.5F);
                        const float v = (-dx * sine + dy * cosine) / (diameter * 0.42F);
                        const float radius_squared = u * u + v * v;
                        if (radius_squared > 1.0F) continue;
                        // The connected halo is common to every glyph; bright
                        // strokes form a double bar, H, or open C respectively.
                        const bool vertical_strokes = std::fabs(std::fabs(u) - 0.48F) < 0.18F &&
                                                      std::fabs(v) < 0.78F;
                        bool bright = vertical_strokes;
                        if (pattern == 1)
                            bright = bright || (std::fabs(v) < 0.15F && std::fabs(u) < 0.62F);
                        if (pattern == 2)
                            bright = radius_squared > 0.22F && radius_squared < 0.78F &&
                                     (u < 0 || std::fabs(v) > 0.40F);
                        image.set_pixel(x, y, exposed(bright ? 25 : 12, exposure),
                                        exposed(bright ? 220 : 72, exposure),
                                        exposed(bright ? 40 : 20, exposure));
                    }
                    const char *names[]{"double bright strokes", "H-shaped strokes", "concave C"};
                    reject_persistent_scene(image,
                        std::string(names[pattern]) + " with a connected faint halo: " + label);
                }
                for (int pattern = 0; pattern < 3; ++pattern) {
                    auto image = background(exposure);
                    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
                        const float dx = x - position.x, dy = y - position.y;
                        const float u = (dx * cosine + dy * sine) / (diameter * 0.5F);
                        const float v = (-dx * sine + dy * cosine) / (diameter * 0.30F);
                        const float radius_squared = u * u + v * v;
                        if (radius_squared > 1.0F) continue;
                        if (pattern == 1 && radius_squared < 0.25F) {
                            image.set_pixel(x, y, exposed(235, exposure), exposed(240, exposure),
                                            exposed(234, exposure));
                        } else {
                            // A complete lamp may have small LED/diffuser
                            // texture and isolated brighter pixels. Neither
                            // texture nor one peak removes the filled lamp face.
                            const float texture = pattern == 2 ?
                                1.0F + 0.10F * std::sin(2.1F * u + 0.3F) * std::cos(5.3F * v) : 1.0F;
                            const int green = static_cast<int>(200 * texture);
                            image.set_pixel(x, y, exposed(24, exposure), exposed(green, exposure),
                                            exposed(36, exposure));
                        }
                    }
                    if (pattern == 2) {
                        image.set_pixel(static_cast<int>(position.x), static_cast<int>(position.y),
                                        exposed(30, exposure), exposed(250, exposure), exposed(42, exposure));
                        image.set_pixel(static_cast<int>(position.x) + 2, static_cast<int>(position.y),
                                        exposed(30, exposure), exposed(235, exposure), exposed(42, exposure));
                    }
                    for (bool integral : {false, true}) {
                        dart::GreenLightDetector detector(config(integral));
                        dart::GreenLightDetection output;
                        for (int frame = 0; frame < 5; ++frame) output = run(detector, image, frame);
                        check(output.valid && !output.predicted &&
                              std::hypot(output.center_x - position.x, output.center_y - position.y) < 2.5F,
                              "filled perspective lamp retains its luminous interior: pattern=" +
                              std::to_string(pattern) + " " + label + " integral=" + std::to_string(integral));
                        ++positive_scenes;
                    }
                }
            }
        }
    }
}

void test_source_roi_preserves_periodic_search_cadence()
{
    auto settings = config(true);
    settings.confirm_hits = 1;
    settings.gate_min_px = 10;
    settings.gate_max_px = 20;
    settings.multiscale_tracking_roi_radius_px = 20;
    settings.multiscale_full_refresh_interval = 5;
    auto first = background(), both = background();
    ellipse(first, 25, 45, 9, 0, 1, false);
    ellipse(both, 25, 45, 9, 0, 1, false);
    ellipse(both, 101, 45, 9, 0, 1, false);
    dart::GreenLightDetector detector(settings);
    for (int frame = 1; frame <= 6; ++frame) {
        auto &image = frame == 1 ? first : both;
        const auto result = detector.process_region(image, {0, 0, width, height},
            width, height, 1000000 + frame * 11112, nullptr, false, false);
        check(result.green.valid && !result.green.predicted,
            "source ROI retains the acquired lamp during periodic full searches");
        if (frame == 1) continue;
        bool remote_candidate = false;
        for (const auto &candidate : detector.last_candidates())
            remote_candidate = remote_candidate || candidate.center_x > 90;
        check(remote_candidate == (frame == 5),
            "source ROI adapter shares the persistent detector's full-search cadence");
    }
}

void test_tentative_acquisition_requires_uninterrupted_measured_evidence()
{
    for (bool integral : {false, true}) {
        auto visible = background();
        ellipse(visible, 67, 45, 9, 0.4F, 1, true);
        auto blank = background();
        const auto settings = config(integral);
        check(settings.confirm_hits == 3,
              "acquisition regression uses the normal three-hit requirement");
        dart::GreenLightDetector detector(settings);
        const auto first = run(detector, visible, 0);
        check(!first.valid && !first.predicted,
              "one real lamp measurement remains tentative");
        const auto missed = run(detector, blank, 1);
        check(!missed.valid && !missed.predicted && missed.missed_frames == 1,
              "an actual failed observation cannot predict an unconfirmed lamp");
        for (int frame = 2; frame <= 3; ++frame) {
            const auto output = run(detector, visible, frame);
            check(!output.valid && !output.predicted,
                  "an acquisition miss invalidates older hits; fewer than three new hits cannot confirm" +
                  std::string(" integral=") + std::to_string(integral) +
                  " frame=" + std::to_string(frame));
        }
        const auto confirmed = run(detector, visible, 4);
        check(confirmed.valid && !confirmed.predicted,
              "the third consecutive fresh observation confirms after an acquisition miss");
        const auto occluded = run(detector, blank, 5);
        check(occluded.valid && occluded.predicted && occluded.measurement_age_us > 0,
              "an already confirmed lamp still coasts through a brief real occlusion");
        const auto recovered = run(detector, visible, 6);
        check(recovered.valid && !recovered.predicted,
              "a confirmed lamp resumes direct tracking after brief occlusion");

        auto scheduled_settings = settings;
        scheduled_settings.classical_interval_frames = 2;
        dart::GreenLightDetector scheduled(scheduled_settings);
        for (int frame = 0; frame <= 4; ++frame) {
            // A blank image in a deliberately unprocessed slot must not be
            // treated as negative evidence. Actual measurements occur at 0,2,4.
            auto &image = frame % 2 == 0 ? visible : blank;
            const auto output = run(scheduled, image, frame);
            check(output.missed_frames == 0,
                  "intentional classical scheduling skips do not consume the miss budget");
            check(output.valid == (frame == 4) && !output.predicted,
                  "scheduled skips retain tentative evidence without adding a confirmation hit");
        }
        const auto scheduled_coast = run(scheduled, blank, 5);
        check(scheduled_coast.valid && scheduled_coast.predicted && scheduled_coast.missed_frames == 0,
              "confirmed scheduled coast remains predicted without a failed observation");
        const auto real_miss = run(scheduled, blank, 6);
        check(real_miss.valid && real_miss.predicted && real_miss.missed_frames == 1,
              "a confirmed scheduled detector still distinguishes a real occlusion from a skip");
        const auto next_skip = run(scheduled, blank, 7);
        check(next_skip.valid && next_skip.predicted && next_skip.missed_frames == 1,
              "a scheduling skip after occlusion adds no second miss");
        const auto resumed = run(scheduled, visible, 8);
        check(resumed.valid && !resumed.predicted && resumed.missed_frames == 0,
              "scheduled confirmed tracking recovers on its next fresh measurement");
    }
}
} // namespace

int main()
{
    test_lamps_across_scale_exposure_position_and_orientation();
    test_persistent_colored_backgrounds();
    test_occlusion_and_appearance_loss();
    test_lamps_on_chromatic_backgrounds();
    test_large_lamps_and_gray_mount();
    test_dim_white_core_with_colored_halo();
    test_dark_color_cannot_identify_a_bright_neutral_reflection();
    test_luminous_interior_across_shape_exposure_and_rotation();
    test_source_roi_preserves_periodic_search_cadence();
    test_tentative_acquisition_requires_uninterrupted_measured_evidence();
    std::cout << "Appearance regression: " << positive_scenes << " positive scenes, "
              << negative_scenes << " persistent negatives, " << failures << " failures\n";
    return failures == 0 ? 0 : 1;
}
