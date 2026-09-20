#include "dart/green_detector.hpp"
#include <stdexcept>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

namespace dart {
namespace {

constexpr float kPi = 3.14159265358979323846F;

float clamp01(float value)
{
    return std::max(0.0F, std::min(1.0F, value));
}

float distance(const Point2f &a, const Point2f &b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

Point2f point(float x, float y)
{
    return {x, y, true};
}

struct RawBar {
    Point2f center;
    float dir_x = 0.0F;
    float dir_y = 1.0F;
    float length = 0.0F;
    float width = 0.0F;
    float color_response = 0.0F;
    float confidence = 0.0F;
};

float color_response(uint8_t red,
                     uint8_t green,
                     uint8_t blue,
                     ArmorColor color)
{
    const float denominator = red + green + blue + 1.0F;
    if (color == ArmorColor::Blue) {
        return (2.0F * blue - red - green) / denominator;
    }
    return (2.0F * red - green - blue) / denominator;
}

std::vector<RawBar> collect_bars(maix::image::Image &frame,
                                 const ArmorConfig &config,
                                 const GreenLightDetection &green,
                                 ArmorColor color)
{
    const int width = frame.width();
    const int height = frame.height();
    const auto *rgb = static_cast<const uint8_t *>(frame.data());
    std::vector<RawBar> bars;
    if (rgb == nullptr || width <= 0 || height <= 0 ||
        frame.data_size() < width * height * 3) {
        return bars;
    }

    const std::size_t pixel_count =
        static_cast<std::size_t>(width) * height;
    std::vector<uint8_t> mask(pixel_count, 0);
    std::vector<float> responses(pixel_count, 0.0F);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const std::size_t index = static_cast<std::size_t>(y) * width + x;
            const std::size_t offset = index * 3U;
            const uint8_t red = rgb[offset];
            const uint8_t green = rgb[offset + 1];
            const uint8_t blue = rgb[offset + 2];
            const float brightness =
                (0.299F * red + 0.587F * green + 0.114F * blue) / 255.0F;
            const float response = color_response(red, green, blue, color);
            const int dominant = color == ArmorColor::Blue ? blue : red;
            const int other = color == ArmorColor::Blue
                ? std::max(red, green) : std::max(green, blue);
            // A warm/yellow or magenta edge can exceed the average of the
            // other channels without having the requested light-bar color.
            // Require resolved dominance over BOTH channels; neutral cores
            // contribute to the later brightness profile, never color seeds.
            const bool chromatic = dominant - other >= std::max(4.0F, 0.08F * dominant);
            if (chromatic && brightness >= config.min_brightness &&
                response >= config.min_color_response) {
                mask[index] = 1;
                responses[index] = response;
            }
        }
    }

    std::vector<int> queue;
    queue.reserve(1024);
    for (int seed_y = 0; seed_y < height; ++seed_y) {
        for (int seed_x = 0; seed_x < width; ++seed_x) {
            const std::size_t seed_index =
                static_cast<std::size_t>(seed_y) * width + seed_x;
            if (mask[seed_index] != 1) {
                continue;
            }
            queue.clear();
            queue.push_back(static_cast<int>(seed_index));
            mask[seed_index] = 2;
            std::size_t cursor = 0;
            int count = 0;
            double weight_sum = 0.0;
            double x_sum = 0.0;
            double y_sum = 0.0;
            double xx_sum = 0.0;
            double yy_sum = 0.0;
            double xy_sum = 0.0;
            double response_sum = 0.0;
            bool touches_boundary = false;
            while (cursor < queue.size()) {
                const int index = queue[cursor++];
                const int x = index % width;
                const int y = index / width;
                touches_boundary = touches_boundary || x == 0 || y == 0 ||
                    x == width - 1 || y == height - 1;
                const float weight = std::max(0.01F, responses[index]);
                ++count;
                weight_sum += weight;
                response_sum += responses[index];
                x_sum += weight * x;
                y_sum += weight * y;
                xx_sum += weight * x * x;
                yy_sum += weight * y * y;
                xy_sum += weight * x * y;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) {
                            continue;
                        }
                        const int nx = x + dx;
                        const int ny = y + dy;
                        if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                            continue;
                        }
                        const std::size_t neighbor =
                            static_cast<std::size_t>(ny) * width + nx;
                        if (mask[neighbor] == 1) {
                            mask[neighbor] = 2;
                            queue.push_back(static_cast<int>(neighbor));
                        }
                    }
                }
            }

            // A crop can turn a broad colored surface into an apparently thin
            // bar. Its extent/PCA are not observable until the component is
            // fully inside the image; do not rank that artificial geometry.
            if (touches_boundary || count < config.min_component_pixels ||
                count > config.max_component_pixels || weight_sum <= 0.0) {
                continue;
            }
            const float center_x = static_cast<float>(x_sum / weight_sum);
            const float center_y = static_cast<float>(y_sum / weight_sum);
            const float cov_xx = static_cast<float>(xx_sum / weight_sum -
                                                    center_x * center_x);
            const float cov_yy = static_cast<float>(yy_sum / weight_sum -
                                                    center_y * center_y);
            const float cov_xy = static_cast<float>(xy_sum / weight_sum -
                                                    center_x * center_y);
            const float trace = cov_xx + cov_yy;
            const float discriminant = std::sqrt(std::max(
                0.0F, (cov_xx - cov_yy) * (cov_xx - cov_yy) +
                          4.0F * cov_xy * cov_xy));
            const float lambda_max = std::max(0.0F, 0.5F * (trace + discriminant));
            const float lambda_min = std::max(0.0F, 0.5F * (trace - discriminant));
            const float length = std::sqrt(12.0F * lambda_max) + 1.0F;
            const float bar_width = std::sqrt(12.0F * lambda_min) + 1.0F;
            if (length < config.min_bar_length_px ||
                length > config.max_bar_length_px ||
                length / std::max(1.0F, bar_width) < config.min_elongation) {
                continue;
            }
            // Apply a necessary pair-geometry bound BEFORE the 24-bar budget.
            // Otherwise distant saturated clutter can evict both target bars.
            // The partner's length is at most length / min_length_ratio.
            const float max_pair_length = 0.5F * (length + std::min(
                config.max_bar_length_px, length / config.min_length_ratio));
            float max_pair_separation =
                config.max_separation_to_length * max_pair_length;
            if (config.max_separation_to_green_size > 0.0F) {
                max_pair_separation = std::min(max_pair_separation,
                    config.max_separation_to_green_size * green.apparent_size);
            }
            const float max_green_distance = std::hypot(
                config.max_green_offset_to_length * max_pair_length,
                config.max_green_lateral_to_separation * max_pair_separation)
                + 0.5F * max_pair_separation;
            if (std::hypot(center_x - green.center_x,
                           center_y - green.center_y) > max_green_distance) {
                continue;
            }
            const float angle = 0.5F * std::atan2(2.0F * cov_xy,
                                                  cov_xx - cov_yy);
            const float along_x = std::cos(angle), along_y = std::sin(angle);
            const float across_x = -along_y, across_y = along_x;
            // Emissive bars form a transverse luminance ridge. Compare their
            // central band with BOTH adjacent side bands in PCA coordinates,
            // independent of image roll or which end points toward the lamp.
            // Read actual RGB here: a white core inside a chromatic component
            // supplies valid brightness evidence without becoming a gray bar.
            const auto sample_rgb = [&](float x, float y, std::array<float, 3> &value) {
                if (x < 0 || y < 0 || x > width - 1 || y > height - 1) return false;
                const int x0 = static_cast<int>(std::floor(x));
                const int y0 = static_cast<int>(std::floor(y));
                const int x1 = std::min(x0 + 1, width - 1);
                const int y1 = std::min(y0 + 1, height - 1);
                const float fx = x - x0, fy = y - y0;
                const auto *p00 = rgb + (static_cast<std::size_t>(y0) * width + x0) * 3;
                const auto *p10 = rgb + (static_cast<std::size_t>(y0) * width + x1) * 3;
                const auto *p01 = rgb + (static_cast<std::size_t>(y1) * width + x0) * 3;
                const auto *p11 = rgb + (static_cast<std::size_t>(y1) * width + x1) * 3;
                for (int c = 0; c < 3; ++c)
                    value[c] = (1 - fy) * ((1 - fx) * p00[c] + fx * p10[c]) +
                        fy * ((1 - fx) * p01[c] + fx * p11[c]);
                return true;
            };
            const auto luma = [](const std::array<float, 3> &p) {
                return (77 * p[0] + 150 * p[1] + 29 * p[2]) / 256;
            };
            const auto color_or_core = [&](const std::array<float, 3> &p) {
                const float dominant = color == ArmorColor::Blue ? p[2] : p[0];
                const float other = color == ArmorColor::Blue
                    ? std::max(p[0], p[1]) : std::max(p[1], p[2]);
                return dominant - other >= std::max(4.0F, 0.08F * dominant) ||
                    std::min({p[0], p[1], p[2]}) >= 0.90F * std::max({p[0], p[1], p[2]}) ||
                    (std::min({p[0], p[1], p[2]}) >= 0.65F * std::max({p[0], p[1], p[2]}) &&
                     dominant + 4 >= other);
            };
            // Saturation may leave chroma on only one side of a white core.
            // Test a bounded set of COMMON transverse offsets at all stations,
            // nearest first. The ridge must stay connected to the color seed;
            // isolated neighboring highlights and open bright half-planes fail.
            constexpr int kProfileSamples = 5;
            const float search_extent = std::min(32.0F, std::max(6.0F, 1.5F * bar_width));
            const float shoulder = std::min(64.0F, std::max(6.0F, 1.5F * bar_width + 2));
            struct ProfileOrigin {
                float x = 0, y = 0, brightness = 0;
                bool compatible = false;
            };
            std::array<ProfileOrigin, kProfileSamples> origins{};
            int compatible_origins = 0;
            for (int sample = 0; sample < kProfileSamples; ++sample) {
                const float along = (sample - 2) * 0.15F * length;
                auto &origin = origins[sample];
                origin.x = center_x + along * along_x;
                origin.y = center_y + along * along_y;
                std::array<float, 3> value{};
                const bool valid = sample_rgb(origin.x, origin.y, value);
                origin.brightness = luma(value);
                origin.compatible = valid && color_or_core(value) && origin.brightness >= 4;
                compatible_origins += origin.compatible;
            }
            // Every allowed path includes its origin and has a >=4 noise
            // floor. If fewer than three origins qualify, no shift or shoulder
            // can satisfy the existing three-station requirement.
            if (compatible_origins < 3) continue;
            bool luminous_ridge = false;
            for (int trial = 0; trial < 13 && !luminous_ridge; ++trial) {
                const int step = (trial + 1) / 2;
                const float shift = trial == 0 ? 0 :
                    (trial % 2 ? -1.0F : 1.0F) * step * search_extent / 6;
                struct Station {
                    float x = 0, y = 0, central = 0;
                    bool complete_center = true, connected = false;
                };
                std::array<Station, kProfileSamples> stations{};
                // The central band and color/core connection do not depend
                // on shoulder distance. Compute them once for both tests.
                for (int sample = 0; sample < kProfileSamples; ++sample) {
                    const auto &origin = origins[sample];
                    const float original_x = origin.x, original_y = origin.y;
                    auto &station = stations[sample];
                    station.x = original_x + shift * across_x;
                    station.y = original_y + shift * across_y;
                    std::array<float, 3> peak{};
                    const bool peak_valid = sample_rgb(station.x, station.y, peak);
                    station.connected = peak_valid && color_or_core(peak) && origin.compatible;
                    for (float offset : {-0.20F * bar_width, 0.0F, 0.20F * bar_width}) {
                        std::array<float, 3> value{};
                        bool valid;
                        if (offset == 0) { value = peak; valid = peak_valid; }
                        else valid = sample_rgb(station.x + offset * across_x,
                            station.y + offset * across_y, value);
                        station.complete_center = valid && station.complete_center;
                        station.central += luma(value) / 3;
                    }
                    // Do not jump across a dark or differently colored gap to
                    // borrow another object's highlight as the lamp's core.
                    const int path_steps = std::max(1, static_cast<int>(std::ceil(std::fabs(shift))));
                    const float path_floor = std::max(4.0F,
                        0.75F * std::min(origin.brightness, luma(peak)));
                    for (int path = 1; path <= path_steps && station.connected; ++path) {
                        const float offset = shift * path / path_steps;
                        std::array<float, 3> value{};
                        station.connected = sample_rgb(original_x + offset * across_x,
                            original_y + offset * across_y, value) && color_or_core(value) &&
                            luma(value) >= path_floor;
                    }
                }
                for (int band = 0; band < 2 && !luminous_ridge; ++band) {
                    const float side_near = band ? shoulder : 0.50F * bar_width + 1;
                    const float side_far = band ? shoulder + std::max(1.0F, 0.25F * bar_width)
                        : 0.75F * bar_width + 1;
                    double center_sum = 0, left_sum = 0, right_sum = 0;
                    int supported_stations = 0;
                    bool complete_profile = true;
                    for (const auto &station : stations) {
                        complete_profile = station.complete_center && complete_profile;
                        float left = 0, right = 0;
                        for (float offset : {side_near, side_far}) {
                            std::array<float, 3> l{}, r{};
                            complete_profile = sample_rgb(station.x - offset * across_x,
                                station.y - offset * across_y, l) && complete_profile;
                            complete_profile = sample_rgb(station.x + offset * across_x,
                                station.y + offset * across_y, r) && complete_profile;
                            left += luma(l) / 2; right += luma(r) / 2;
                        }
                        const float minimum_contrast = std::max(4.0F, 0.12F * station.central);
                        supported_stations += station.connected && station.central - left >= minimum_contrast &&
                            station.central - right >= minimum_contrast;
                        center_sum += station.central; left_sum += left; right_sum += right;
                    }
                    const double minimum_mean_contrast = std::max(4.0,
                        0.12 * center_sum / kProfileSamples);
                    luminous_ridge = complete_profile && supported_stations >= 3 &&
                        (center_sum - left_sum) / kProfileSamples >= minimum_mean_contrast &&
                        (center_sum - right_sum) / kProfileSamples >= minimum_mean_contrast;
                }
            }
            if (!luminous_ridge) continue;
            const float elongation_score = clamp01(
                (length / std::max(1.0F, bar_width) - config.min_elongation) /
                4.0F);
            const float mean_response =
                static_cast<float>(response_sum / count);
            RawBar bar;
            bar.center = point(center_x, center_y);
            bar.dir_x = std::cos(angle);
            bar.dir_y = std::sin(angle);
            bar.length = length;
            bar.width = bar_width;
            bar.color_response = mean_response;
            bar.confidence = clamp01(0.55F * elongation_score +
                                     0.45F * mean_response);
            bars.push_back(bar);
        }
    }
    std::sort(bars.begin(), bars.end(), [](const RawBar &left, const RawBar &right) {
        return left.confidence > right.confidence;
    });
    if (bars.size() > 24U) {
        bars.resize(24U);
    }
    return bars;
}

LineSegment2f make_segment(const RawBar &bar, float down_x, float down_y)
{
    // A PCA eigenvector has an arbitrary sign.  The pair solver has already
    // oriented `down` toward the green lamp, so do not flip it back to the
    // component's arbitrary eigenvector sign here.
    LineSegment2f result;
    result.center = bar.center;
    result.top = point(bar.center.x - 0.5F * bar.length * down_x,
                       bar.center.y - 0.5F * bar.length * down_y);
    result.bottom = point(bar.center.x + 0.5F * bar.length * down_x,
                          bar.center.y + 0.5F * bar.length * down_y);
    result.length = bar.length;
    result.width = bar.width;
    result.angle_rad = std::atan2(down_y, down_x);
    result.confidence = bar.confidence;
    return result;
}

ArmorDetection detect_armor_for_color(maix::image::Image &frame,
                                      const ArmorConfig &config,
                                      const GreenLightDetection &green,
                                      ArmorColor color)
{
    ArmorDetection best;
    if (green.apparent_size <= 0.0F) {
        return best;
    }
    const auto bars = collect_bars(frame, config, green, color);
    if (bars.size() < 2U) {
        return best;
    }

    float best_score = 0.0F;
    for (std::size_t i = 0; i < bars.size(); ++i) {
        for (std::size_t j = i + 1; j < bars.size(); ++j) {
            const RawBar &first = bars[i];
            const RawBar &second = bars[j];
            const float dot = std::fabs(first.dir_x * second.dir_x +
                                        first.dir_y * second.dir_y);
            const float angle_difference = std::acos(clamp01(dot));
            if (angle_difference > config.max_pair_angle_deg * kPi / 180.0F) {
                continue;
            }
            const float average_length = 0.5F * (first.length + second.length);
            const float length_ratio = std::min(first.length, second.length) /
                                       std::max(first.length, second.length);
            if (length_ratio < config.min_length_ratio) {
                continue;
            }
            const float response_difference = std::fabs(
                first.color_response - second.color_response);
            if (response_difference > config.max_color_response_diff) {
                continue;
            }
            const float separation = distance(first.center, second.center);
            if (config.max_separation_to_green_size > 0.0F &&
                separation > config.max_separation_to_green_size *
                                 green.apparent_size) {
                continue;
            }
            const float separation_ratio = separation /
                                           std::max(1.0F, average_length);
            if (separation_ratio < config.min_separation_to_length ||
                separation_ratio > config.max_separation_to_length) {
                continue;
            }

            float down_x = first.dir_x;
            float down_y = first.dir_y;
            if (down_x * second.dir_x + down_y * second.dir_y < 0.0F) {
                down_x -= second.dir_x;
                down_y -= second.dir_y;
            } else {
                down_x += second.dir_x;
                down_y += second.dir_y;
            }
            const float down_norm = std::hypot(down_x, down_y);
            if (down_norm < 1.0e-4F) {
                continue;
            }
            down_x /= down_norm;
            down_y /= down_norm;
            // Parallel components also occur as fragments of ONE light bar.
            // A real pair must be side by side in its own (roll-invariant)
            // coordinates; angle and Euclidean separation alone do not prove it.
            const float pair_longitudinal = std::fabs(
                (second.center.x - first.center.x) * down_x +
                (second.center.y - first.center.y) * down_y) /
                std::max(1.0F, average_length);
            if (pair_longitudinal > config.max_pair_longitudinal_to_length) {
                continue;
            }
            const Point2f armor_center = point(
                0.5F * (first.center.x + second.center.x),
                0.5F * (first.center.y + second.center.y));
            const float green_dx = green.center_x - armor_center.x;
            const float green_dy = green.center_y - armor_center.y;
            if (green_dx * down_x + green_dy * down_y < 0.0F) {
                down_x = -down_x;
                down_y = -down_y;
            }
            const float right_x = down_y;
            const float right_y = -down_x;
            const float longitudinal =
                (green_dx * down_x + green_dy * down_y) /
                std::max(1.0F, average_length);
            const float lateral = std::fabs(green_dx * right_x +
                                            green_dy * right_y) /
                                  std::max(1.0F, separation);
            if (longitudinal < config.min_green_offset_to_length ||
                longitudinal > config.max_green_offset_to_length ||
                lateral > config.max_green_lateral_to_separation) {
                continue;
            }

            const float parallel_score = clamp01(
                1.0F - angle_difference /
                    (config.max_pair_angle_deg * kPi / 180.0F));
            const float lateral_score = clamp01(
                1.0F - lateral /
                    std::max(0.01F, config.max_green_lateral_to_separation));
            const float brightness_consistency = clamp01(
                1.0F - response_difference /
                    std::max(0.01F, config.max_color_response_diff));
            const float pair_score = clamp01(
                0.22F * parallel_score + 0.18F * length_ratio +
                0.18F * lateral_score + 0.12F * brightness_consistency +
                0.15F * first.confidence + 0.15F * second.confidence);
            if (pair_score < config.min_geometry_confidence ||
                pair_score <= best_score) {
                continue;
            }

            const float first_side =
                (first.center.x - armor_center.x) * right_x +
                (first.center.y - armor_center.y) * right_y;
            const RawBar &left = first_side < 0.0F ? first : second;
            const RawBar &right = first_side < 0.0F ? second : first;
            best.valid = true;
            best.color = color;
            best.left_bar = make_segment(left, down_x, down_y);
            best.right_bar = make_segment(right, down_x, down_y);
            best.center = armor_center;
            best.separation_px = separation;
            best.geometry_confidence = pair_score;
            best_score = pair_score;
        }
    }
    return best;
}

ArmorDetection detect_armor(maix::image::Image &frame,
                            const ArmorConfig &config,
                            const GreenLightDetection &green)
{
    if (config.expected_color == ArmorColor::Red ||
        config.expected_color == ArmorColor::Blue) {
        return detect_armor_for_color(frame, config, green,
                                      config.expected_color);
    }

    // Auto color is diagnostic-only. process_target() deliberately keeps
    // safe_for_control false while expected_color is Unknown.
    ArmorDetection red = detect_armor_for_color(frame, config, green,
                                                ArmorColor::Red);
    ArmorDetection blue = detect_armor_for_color(frame, config, green,
                                                 ArmorColor::Blue);
    return blue.geometry_confidence > red.geometry_confidence ? blue : red;
}

bool same_armor_geometry(const ArmorDetection &previous,
                         const Point2f &previous_green,
                         const ArmorDetection &current,
                         const Point2f &current_green,
                         const ArmorConfig &config)
{
    if (!previous.valid || !current.valid || !previous_green.valid ||
        !current_green.valid || previous.color != current.color) {
        return false;
    }
    const auto similar_scale = [&config](float a, float b) {
        return a > 0.0F && b > 0.0F &&
               std::min(a, b) / std::max(a, b) >= config.min_length_ratio;
    };
    const float previous_length =
        0.5F * (previous.left_bar.length + previous.right_bar.length);
    const float current_length =
        0.5F * (current.left_bar.length + current.right_bar.length);
    if (!similar_scale(previous.separation_px, current.separation_px) ||
        !similar_scale(previous_length, current_length)) {
        return false;
    }
    // Compare relative change, never an absolute image direction. The pair
    // solver orients both bar axes toward the green lamp, removing PCA sign
    // ambiguity while retaining targets at any static camera roll.
    const float angle_delta = current.left_bar.angle_rad - previous.left_bar.angle_rad;
    const float cosine = std::cos(angle_delta), sine = std::sin(angle_delta);
    if (cosine < std::cos(config.max_pair_angle_deg * kPi / 180.0F)) {
        return false;
    }
    const float scale = current.separation_px / previous.separation_px;
    const float dx = previous.center.x - previous_green.x;
    const float dy = previous.center.y - previous_green.y;
    const float expected_x = current_green.x + scale * (cosine * dx - sine * dy);
    const float expected_y = current_green.y + scale * (sine * dx + cosine * dy);
    return std::hypot(current.center.x - expected_x, current.center.y - expected_y) <=
        std::max(2.0F, config.max_green_lateral_to_separation * current.separation_px);
}

bool solve_8x8(float matrix[8][9], std::array<float, 8> &solution)
{
    for (int column = 0; column < 8; ++column) {
        int pivot = column;
        for (int row = column + 1; row < 8; ++row) {
            if (std::fabs(matrix[row][column]) >
                std::fabs(matrix[pivot][column])) {
                pivot = row;
            }
        }
        if (std::fabs(matrix[pivot][column]) < 1.0e-8F) {
            return false;
        }
        if (pivot != column) {
            for (int item = column; item <= 8; ++item) {
                std::swap(matrix[pivot][item], matrix[column][item]);
            }
        }
        const float divisor = matrix[column][column];
        for (int item = column; item <= 8; ++item) {
            matrix[column][item] /= divisor;
        }
        for (int row = 0; row < 8; ++row) {
            if (row == column) {
                continue;
            }
            const float factor = matrix[row][column];
            for (int item = column; item <= 8; ++item) {
                matrix[row][item] -= factor * matrix[column][item];
            }
        }
    }
    for (int row = 0; row < 8; ++row) {
        solution[row] = matrix[row][8];
    }
    return true;
}

std::array<float, 2> undistort_normalized(const Point2f &pixel,
                                          const CameraModel &camera)
{
    const float distorted_x = (pixel.x - camera.principal_x) / camera.fx;
    const float distorted_y = (pixel.y - camera.principal_y) / camera.fy;
    float x = distorted_x;
    float y = distorted_y;
    for (int iteration = 0; iteration < 6; ++iteration) {
        const float x2 = x * x;
        const float y2 = y * y;
        const float r2 = x2 + y2;
        const float radial = std::max(
            1.0e-6F,
            1.0F + camera.k1 * r2 + camera.k2 * r2 * r2 +
                camera.k3 * r2 * r2 * r2);
        const float tangential_x =
            2.0F * camera.p1 * x * y + camera.p2 * (r2 + 2.0F * x2);
        const float tangential_y =
            camera.p1 * (r2 + 2.0F * y2) + 2.0F * camera.p2 * x * y;
        x = (distorted_x - tangential_x) / radial;
        y = (distorted_y - tangential_y) / radial;
    }
    return {x, y};
}

Point2f project_distorted(float x,
                          float y,
                          float z,
                          const CameraModel &camera)
{
    if (z <= 1.0e-6F) {
        return {};
    }
    const float normalized_x = x / z;
    const float normalized_y = y / z;
    const float x2 = normalized_x * normalized_x;
    const float y2 = normalized_y * normalized_y;
    const float r2 = x2 + y2;
    const float radial = 1.0F + camera.k1 * r2 +
                         camera.k2 * r2 * r2 +
                         camera.k3 * r2 * r2 * r2;
    const float distorted_x = normalized_x * radial +
        2.0F * camera.p1 * normalized_x * normalized_y +
        camera.p2 * (r2 + 2.0F * x2);
    const float distorted_y = normalized_y * radial +
        camera.p1 * (r2 + 2.0F * y2) +
        2.0F * camera.p2 * normalized_x * normalized_y;
    return point(camera.fx * distorted_x + camera.principal_x,
                 camera.fy * distorted_y + camera.principal_y);
}

std::array<float, 4> rotation_to_quaternion(
    const std::array<std::array<float, 3>, 3> &rotation)
{
    std::array<float, 4> quaternion{};
    const float trace = rotation[0][0] + rotation[1][1] + rotation[2][2];
    if (trace > 0.0F) {
        const float s = 2.0F * std::sqrt(trace + 1.0F);
        quaternion[0] = 0.25F * s;
        quaternion[1] = (rotation[2][1] - rotation[1][2]) / s;
        quaternion[2] = (rotation[0][2] - rotation[2][0]) / s;
        quaternion[3] = (rotation[1][0] - rotation[0][1]) / s;
    } else {
        int i = rotation[1][1] > rotation[0][0] ? 1 : 0;
        if (rotation[2][2] > rotation[i][i]) {
            i = 2;
        }
        const int j = (i + 1) % 3;
        const int k = (i + 2) % 3;
        const float s = 2.0F * std::sqrt(
            std::max(1.0e-8F, 1.0F + rotation[i][i] -
                                  rotation[j][j] - rotation[k][k]));
        quaternion[i + 1] = 0.25F * s;
        quaternion[0] = (rotation[k][j] - rotation[j][k]) / s;
        quaternion[j + 1] = (rotation[j][i] + rotation[i][j]) / s;
        quaternion[k + 1] = (rotation[k][i] + rotation[i][k]) / s;
    }
    return quaternion;
}

TargetPose solve_planar_pose(const ArmorDetection &armor,
                             const GreenLightDetection &green,
                             const TargetGeometryConfig &geometry,
                             const CameraModel &camera)
{
    TargetPose result;
    if (!geometry.pose_enabled || !armor.valid ||
        armor.separation_px < geometry.min_pose_separation_px ||
        !armor.left_bar.top.valid || !armor.left_bar.bottom.valid ||
        !armor.right_bar.top.valid || !armor.right_bar.bottom.valid) {
        return result;
    }

    const float half_separation = 0.5F * geometry.bar_separation_m;
    const float half_length = 0.5F * geometry.bar_length_m;
    const std::array<std::array<float, 2>, 4> object_points{{
        {-half_separation, -half_length},
        {-half_separation, half_length},
        {half_separation, -half_length},
        {half_separation, half_length},
    }};
    const std::array<Point2f, 4> image_points{{
        armor.left_bar.top, armor.left_bar.bottom,
        armor.right_bar.top, armor.right_bar.bottom,
    }};
    float matrix[8][9]{};
    for (int index = 0; index < 4; ++index) {
        const float X = object_points[index][0];
        const float Y = object_points[index][1];
        const auto normalized = undistort_normalized(image_points[index], camera);
        const float x = normalized[0];
        const float y = normalized[1];
        matrix[2 * index][0] = X;
        matrix[2 * index][1] = Y;
        matrix[2 * index][2] = 1.0F;
        matrix[2 * index][6] = -x * X;
        matrix[2 * index][7] = -x * Y;
        matrix[2 * index][8] = x;
        matrix[2 * index + 1][3] = X;
        matrix[2 * index + 1][4] = Y;
        matrix[2 * index + 1][5] = 1.0F;
        matrix[2 * index + 1][6] = -y * X;
        matrix[2 * index + 1][7] = -y * Y;
        matrix[2 * index + 1][8] = y;
    }
    std::array<float, 8> h{};
    if (!solve_8x8(matrix, h)) {
        return result;
    }

    std::array<float, 3> r1{h[0], h[3], h[6]};
    std::array<float, 3> r2{h[1], h[4], h[7]};
    std::array<float, 3> translation{h[2], h[5], 1.0F};
    const auto norm = [](const std::array<float, 3> &value) {
        return std::sqrt(value[0] * value[0] + value[1] * value[1] +
                         value[2] * value[2]);
    };
    const float scale = 2.0F /
        std::max(1.0e-6F, norm(r1) + norm(r2));
    for (int axis = 0; axis < 3; ++axis) {
        r1[axis] *= scale;
        r2[axis] *= scale;
        translation[axis] *= scale;
    }
    if (translation[2] < 0.0F) {
        for (int axis = 0; axis < 3; ++axis) {
            r1[axis] = -r1[axis];
            r2[axis] = -r2[axis];
            translation[axis] = -translation[axis];
        }
    }
    const float r1_norm = std::max(1.0e-6F, norm(r1));
    for (float &value : r1) {
        value /= r1_norm;
    }
    const float dot = r1[0] * r2[0] + r1[1] * r2[1] + r1[2] * r2[2];
    for (int axis = 0; axis < 3; ++axis) {
        r2[axis] -= dot * r1[axis];
    }
    const float r2_norm = std::max(1.0e-6F, norm(r2));
    for (float &value : r2) {
        value /= r2_norm;
    }
    const std::array<float, 3> r3{
        r1[1] * r2[2] - r1[2] * r2[1],
        r1[2] * r2[0] - r1[0] * r2[2],
        r1[0] * r2[1] - r1[1] * r2[0],
    };
    const std::array<std::array<float, 3>, 3> rotation{{
        {r1[0], r2[0], r3[0]},
        {r1[1], r2[1], r3[1]},
        {r1[2], r2[2], r3[2]},
    }};

    float squared_error = 0.0F;
    int error_points = 0;
    const auto accumulate_error = [&](float X, float Y, const Point2f &observed) {
        const float px = r1[0] * X + r2[0] * Y + translation[0];
        const float py = r1[1] * X + r2[1] * Y + translation[1];
        const float pz = r1[2] * X + r2[2] * Y + translation[2];
        if (pz <= 1.0e-6F) {
            return;
        }
        const Point2f projected = project_distorted(px, py, pz, camera);
        if (!projected.valid) {
            return;
        }
        const float dx = projected.x - observed.x;
        const float dy = projected.y - observed.y;
        squared_error += dx * dx + dy * dy;
        ++error_points;
    };
    for (int index = 0; index < 4; ++index) {
        accumulate_error(object_points[index][0], object_points[index][1],
                         image_points[index]);
    }
    accumulate_error(0.0F, geometry.green_offset_m,
                     point(green.center_x, green.center_y));
    if (error_points < 4) {
        return result;
    }
    result.reprojection_error_px =
        std::sqrt(squared_error / error_points);
    if (result.reprojection_error_px > geometry.max_reprojection_error_px) {
        return result;
    }
    result.valid = true;
    result.translation_m = translation;
    result.orientation_wxyz = rotation_to_quaternion(rotation);
    result.distance_m = norm(translation);
    return result;
}

CandidateRoi make_roi(float center_x,
                      float center_y,
                      float apparent_size,
                      int image_width,
                      int image_height,
                      const NpuConfig &config)
{
    const int side = std::max(
        config.min_roi_size_px,
        std::min(config.max_roi_size_px,
                 static_cast<int>(std::lround(
                     config.roi_size_factor * std::max(1.0F, apparent_size)))));
    const int width = std::min(side, image_width);
    const int height = std::min(side, image_height);
    const int x = std::max(0, std::min(image_width - width,
        static_cast<int>(std::lround(center_x - 0.5F * width))));
    const int y = std::max(0, std::min(image_height - height,
        static_cast<int>(std::lround(center_y - 0.5F * height))));
    return {x, y, width, height};
}

ArmorDetection armor_from_validation(const PoseValidation &validation)
{
    ArmorDetection armor;
    if (!validation.valid || !validation.keypoints[1].valid ||
        !validation.keypoints[2].valid || !validation.keypoints[3].valid ||
        !validation.keypoints[4].valid) {
        return armor;
    }
    armor.valid = true;
    armor.model_validated = true;
    armor.model_confidence = validation.confidence;
    armor.left_bar.top = validation.keypoints[1];
    armor.left_bar.bottom = validation.keypoints[2];
    armor.right_bar.top = validation.keypoints[3];
    armor.right_bar.bottom = validation.keypoints[4];
    armor.left_bar.center = point(
        0.5F * (armor.left_bar.top.x + armor.left_bar.bottom.x),
        0.5F * (armor.left_bar.top.y + armor.left_bar.bottom.y));
    armor.right_bar.center = point(
        0.5F * (armor.right_bar.top.x + armor.right_bar.bottom.x),
        0.5F * (armor.right_bar.top.y + armor.right_bar.bottom.y));
    armor.left_bar.length = distance(armor.left_bar.top,
                                     armor.left_bar.bottom);
    armor.right_bar.length = distance(armor.right_bar.top,
                                      armor.right_bar.bottom);
    armor.left_bar.confidence = validation.confidence;
    armor.right_bar.confidence = validation.confidence;
    armor.center = point(
        0.5F * (armor.left_bar.center.x + armor.right_bar.center.x),
        0.5F * (armor.left_bar.center.y + armor.right_bar.center.y));
    armor.separation_px = distance(armor.left_bar.center,
                                   armor.right_bar.center);
    armor.geometry_confidence = validation.confidence;
    return armor;
}

}  // namespace

TargetEstimate GreenLightDetector::process_region(
    maix::image::Image &roi, const CandidateRoi &region, int width, int height,
    uint64_t timestamp, const MotionPrior *motion, bool force_armor_scan, bool run_armor_scan)
{
    if (region.x < 0 || region.y < 0 || region.width != roi.width() ||
        region.height != roi.height() || region.width <= 0 || region.height <= 0 ||
        region.x > width - region.width || region.y > height - region.height)
        throw std::invalid_argument("invalid source ROI");
    if (npu_config_.enabled || target_geometry_.pose_enabled)
        throw std::invalid_argument("ROI mode requires NPU and pose disabled pending calibrated model integration");
    region_force_armor_ = force_armor_scan; region_run_armor_ = run_armor_scan;
    region_active_ = true; region_ = region; source_width_ = width; source_height_ = height;
    try {
        auto result = process(roi, timestamp, motion);
        region_active_ = false;
        return result;
    } catch (...) { region_active_ = false; throw; }
}

TargetEstimate GreenLightDetector::process(
    maix::image::Image &frame,
    uint64_t timestamp_us,
    const MotionPrior *motion_prior)
{
    TargetEstimate result;
    result.timestamp_us = timestamp_us;
    result.green = process_green(frame, timestamp_us, motion_prior);
    result.classical_detection_ran = last_classical_detection_ran_;
    result.classical_detection_ms = last_classical_detection_ms_;
    result.measurement_age_us = result.green.measurement_age_us;
    result.predicted = result.green.predicted;

    const auto clear_armor_candidate = [this]() {
        armor_candidate_detection_ = ArmorDetection{};
        armor_candidate_green_center_ = Point2f{};
        armor_candidate_timestamp_us_ = 0;
        armor_candidate_hits_ = 0;
    };
    const bool green_present = result.green.state != TrackState::Lost &&
                               result.green.apparent_size > 0.0F;
    // Candidate acquisition is not a confirmed geometric anchor. Preserve its
    // green cadence instead of spending the frame budget scanning paired bars.
    // Explicit diagnostic scans may still run below, but cannot add armor hits.
    const bool usable_green_anchor = green_present && result.green.valid &&
        ((!result.green.predicted && result.classical_detection_ran) ||
         (result.green.valid && result.green.predicted &&
          result.green.measurement_age_us <=
              static_cast<uint64_t>(config_.prediction_max_age_ms) * 1000U));
    const bool forced_scan = region_active_ && region_force_armor_;
    if (armor_candidate_detection_.valid &&
        (timestamp_us < armor_candidate_timestamp_us_ ||
         timestamp_us - armor_candidate_timestamp_us_ >
             static_cast<uint64_t>(armor_config_.confirmation_max_gap_ms) * 1000U)) {
        clear_armor_candidate();
    }
    if (forced_scan || (usable_green_anchor &&
                       result.green.apparent_size >= armor_config_.min_green_size_px)) {
        const int effective_classical_interval = npu_config_.enabled
            ? std::max(2, config_.classical_interval_frames)
            : config_.classical_interval_frames;
        const bool armor_scheduler_slot = effective_classical_interval <= 1 ||
            (npu_config_.enabled ? last_classical_detection_ran_ : !last_classical_detection_ran_);
        // Preserve the existing interleaved schedule. A fresh pair can use a
        // short, valid green prediction as a geometric reference; prediction
        // alone or a skipped scan never adds an armor hit or renews its age.
        if (armor_scheduler_slot && (!region_active_ || region_run_armor_)) {
            result.armor_detection_ran = true;
            auto local_green = result.green;
            if (region_active_) {
                local_green.center_x -= region_.x; local_green.center_y -= region_.y;
                local_green.bbox_x -= region_.x; local_green.bbox_y -= region_.y;
            }
            auto detected = detect_armor(frame, armor_config_, local_green);
            if (region_active_) {
                auto translate = [this](Point2f &p) { if (p.valid) { p.x += region_.x; p.y += region_.y; } };
                auto &a = detected;
                translate(a.center);
                for (auto *bar : {&a.left_bar, &a.right_bar}) {
                    translate(bar->top); translate(bar->bottom); translate(bar->center);
                }
            }
            // A failed or inconsistent scan must not masquerade as a fresh
            // copy of the old cache: downstream source timestamps distinguish
            // actual scans from intentional cache-only scheduling slots.
            last_armor_detection_ = ArmorDetection{};
            last_armor_timestamp_us_ = 0;
            if (usable_green_anchor && detected.valid) {
                const auto green_center = point(result.green.center_x, result.green.center_y);
                const bool newer = !armor_candidate_detection_.valid ||
                                   timestamp_us > armor_candidate_timestamp_us_;
                if (newer) {
                    const bool consistent = same_armor_geometry(armor_candidate_detection_,
                        armor_candidate_green_center_, detected, green_center, armor_config_);
                    armor_candidate_hits_ = consistent
                        ? std::min(armor_candidate_hits_ + 1,
                            std::max(armor_config_.confirmation_hits, armor_config_.required_pose_hits))
                        : 1;
                    armor_candidate_detection_ = detected;
                    armor_candidate_green_center_ = green_center;
                    armor_candidate_timestamp_us_ = timestamp_us;
                    if (armor_candidate_hits_ >= armor_config_.confirmation_hits) {
                        last_armor_detection_ = detected;
                        last_armor_timestamp_us_ = timestamp_us;
                    }
                }
            } else {
                clear_armor_candidate();
            }
        }
        const uint64_t armor_cache_age_us =
            timestamp_us >= last_armor_timestamp_us_
                ? timestamp_us - last_armor_timestamp_us_
                : std::numeric_limits<uint64_t>::max();
        if (last_armor_detection_.valid &&
            armor_cache_age_us <= static_cast<uint64_t>(
                armor_config_.cache_max_age_ms) * 1000U) {
            result.armor = last_armor_detection_;
        }
    } else {
        clear_armor_candidate();
        last_armor_detection_ = ArmorDetection{};
        last_armor_timestamp_us_ = 0;
    }

    const int effective_classical_interval = npu_config_.enabled
        ? std::max(2, config_.classical_interval_frames)
        : config_.classical_interval_frames;
    if (pose_validator_ != nullptr && npu_config_.enabled &&
        (effective_classical_interval <= 1 ||
         !last_classical_detection_ran_) &&
        !last_candidates_.empty() &&
        (last_model_timestamp_us_ == 0 ||
         timestamp_us - last_model_timestamp_us_ >=
             static_cast<uint64_t>(npu_config_.interval_ms) * 1000U)) {
        std::size_t candidate_index = 0;
        if (!tracker_.tracking_confirmed()) {
            const std::size_t candidate_limit = std::min(
                last_candidates_.size(),
                static_cast<std::size_t>(npu_config_.search_candidates));
            candidate_index = search_model_cursor_ % std::max<std::size_t>(1, candidate_limit);
            ++search_model_cursor_;
        } else {
            for (std::size_t index = 0; index < last_candidates_.size(); ++index) {
                if (last_candidates_[index].selected) {
                    candidate_index = index;
                    break;
                }
            }
        }
        auto &candidate = last_candidates_[candidate_index];
        const CandidateRoi roi = make_roi(
            candidate.center_x, candidate.center_y,
            candidate.apparent_size, frame.width(), frame.height(), npu_config_);
        const auto model_started = std::chrono::steady_clock::now();
        last_pose_validation_ = pose_validator_->validate(frame, roi);
        const auto model_finished = std::chrono::steady_clock::now();
        last_model_inference_ms_ = static_cast<float>(
            std::chrono::duration<double, std::milli>(
                model_finished - model_started).count());
        result.model_ran = true;
        result.model_inference_ms = last_model_inference_ms_;
        last_model_roi_ = roi;
        last_model_timestamp_us_ = timestamp_us;
        candidate.model_validated = last_pose_validation_.valid;
        candidate.model_score = last_pose_validation_.confidence;
        if (last_pose_validation_.valid) {
            last_model_positive_timestamp_us_ = timestamp_us;
        }
    }

    const bool model_lamp_consistent = result.green.valid &&
        last_pose_validation_.keypoints[0].valid &&
        distance(last_pose_validation_.keypoints[0],
                 point(result.green.center_x, result.green.center_y)) <=
            std::max(12.0F, 2.0F * result.green.apparent_size);
    const bool model_is_recent = last_model_positive_timestamp_us_ > 0 &&
        timestamp_us >= last_model_positive_timestamp_us_ &&
        timestamp_us - last_model_positive_timestamp_us_ <= 100000U &&
        model_lamp_consistent;
    if (model_is_recent && last_pose_validation_.valid) {
        ArmorDetection model_armor = armor_from_validation(last_pose_validation_);
        model_armor.color = armor_config_.expected_color;
        if (model_armor.valid &&
            (!result.armor.valid ||
             model_armor.model_confidence >= result.armor.geometry_confidence)) {
            result.armor = model_armor;
        } else if (result.armor.valid) {
            result.armor.model_validated = true;
            result.armor.model_confidence = last_pose_validation_.confidence;
        }
    }

    if (result.armor.valid) {
        // Classical confirmation already counted the coherent observations;
        // do not impose a second identical hit delay before fusion/pose use.
        if (result.armor_detection_ran && armor_candidate_hits_ > 0)
            armor_pose_hits_ = armor_candidate_hits_;
        else if (result.model_ran)
            ++armor_pose_hits_;
    } else {
        armor_pose_hits_ = 0;
        armor_blend_start_us_ = 0;
    }
    result.pose = solve_planar_pose(result.armor, result.green,
                                    target_geometry_, config_.camera_model);

    switch (result.green.state) {
    case TrackState::Candidate:
        result.state = GuidanceTrackState::Acquiring;
        break;
    case TrackState::Tracking:
        if (!result.green.valid) {
            result.state = GuidanceTrackState::Reacquire;
        } else {
            result.state = result.green.predicted
                               ? GuidanceTrackState::Coasting
                               : GuidanceTrackState::Tracking;
        }
        break;
    case TrackState::Lost:
        result.state = last_output_timestamp_us_ > 0 &&
                               timestamp_us - last_output_timestamp_us_ < 500000U
                           ? GuidanceTrackState::Reacquire
                           : GuidanceTrackState::Search;
        break;
    }

    result.valid = result.green.valid;
    const bool prediction_is_control_fresh =
        !result.green.predicted ||
        (result.green.missed_frames <= config_.control_prediction_max_frames &&
         result.green.measurement_age_us <=
             static_cast<uint64_t>(config_.control_prediction_max_age_ms) * 1000U);
    const bool model_gate = !npu_config_.enabled ||
        (pose_validator_ != nullptr &&
         (model_is_recent || result.armor.valid));
    const bool known_armor_color =
        armor_config_.expected_color != ArmorColor::Unknown;
    result.safe_for_control = result.green.valid &&
                              prediction_is_control_fresh && model_gate &&
                              known_armor_color;

    if (result.green.valid) {
        result.aim_point = point(result.green.center_x, result.green.center_y);
    }
    if (result.green.valid && result.armor.valid &&
        armor_pose_hits_ >= armor_config_.required_pose_hits) {
        if (armor_blend_start_us_ == 0) {
            armor_blend_start_us_ = timestamp_us;
        }
        const uint64_t blend_duration_us =
            static_cast<uint64_t>(std::max(0, armor_config_.aim_blend_ms)) * 1000U;
        const float blend = blend_duration_us == 0
                                ? 1.0F
                                : clamp01(static_cast<float>(
                                      timestamp_us - armor_blend_start_us_) /
                                      blend_duration_us);
        result.aim_point.x = (1.0F - blend) * result.green.center_x +
                             blend * result.armor.center.x;
        result.aim_point.y = (1.0F - blend) * result.green.center_y +
                             blend * result.armor.center.y;
        result.guidance_mode = blend >= 1.0F
                                   ? GuidanceMode::ArmorImpact
                                   : GuidanceMode::Fused;
    }

    std::array<float, 2> aim_angles{};
    if (result.aim_point.valid) {
        aim_angles = detail::pixel_to_angles(
            result.aim_point.x, result.aim_point.y, config_.camera_model);
        result.yaw_rad = aim_angles[0];
        result.pitch_rad = aim_angles[1];
        const float ray_x = std::tan(result.yaw_rad);
        const float ray_y = -std::tan(result.pitch_rad);
        const float ray_norm = std::sqrt(ray_x * ray_x + ray_y * ray_y + 1.0F);
        result.line_of_sight_camera = {ray_x / ray_norm, ray_y / ray_norm,
                                       1.0F / ray_norm};
    }
    result.line_of_sight_rate_rad_s = tracker_.line_of_sight_rate();
    result.angular_covariance = tracker_.angular_covariance();
    result.confidence = clamp01(
        0.65F * result.green.confidence +
        0.20F * result.armor.geometry_confidence +
        0.15F * result.armor.model_confidence);

    if (result.green.valid) {
        last_output_timestamp_us_ = timestamp_us;
    }
    if (last_output_timestamp_us_ > 0 && timestamp_us > last_output_timestamp_us_ &&
        result.state == GuidanceTrackState::Reacquire &&
        timestamp_us - last_output_timestamp_us_ >= 500000U) {
        result.state = GuidanceTrackState::Search;
    }
    last_output_angles_ = aim_angles;
    return result;
}

TargetEstimate GreenLightDetector::process_target(
    maix::image::Image &frame,
    uint64_t timestamp_us,
    const MotionPrior *motion_prior)
{
    return process(frame, timestamp_us, motion_prior);
}

}  // namespace dart
