#include "dart/green_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace dart {
namespace {

struct Rect {
    int x0 = 0;
    int y0 = 0;
    int x1 = 0;
    int y1 = 0;
};

struct RgbStatistics {
    double brightness_sum = 0.0;
    double green_dominance_sum = 0.0;
    double positive_green_dominance_sum = 0.0;
    int samples = 0;
    int positive_green_samples = 0;

    void add(uint8_t red, uint8_t green, uint8_t blue)
    {
        brightness_sum += (0.299 * red + 0.587 * green + 0.114 * blue) / 255.0;
        const double green_dominance =
            (static_cast<int>(green) - std::max(static_cast<int>(red),
                                                static_cast<int>(blue))) /
            255.0;
        green_dominance_sum += green_dominance;
        if (green_dominance > 0.0) {
            positive_green_dominance_sum += green_dominance;
            ++positive_green_samples;
        }
        ++samples;
    }

    float brightness() const
    {
        return samples > 0 ? static_cast<float>(brightness_sum / samples) : 0.0F;
    }

    float green_dominance() const
    {
        if (samples <= 0) {
            return 0.0F;
        }

        const double full_box_mean = green_dominance_sum / samples;
        if (positive_green_samples <= 0) {
            return static_cast<float>(full_box_mean);
        }

        // A close lamp has a saturated white core surrounded by a green halo.
        // Averaging the full bounding box dilutes that halo until it fails the
        // color gate. Keep the full-box statistic for small/dim targets, but
        // also consider the mean positive-green response weighted by its
        // coverage so a few isolated green noise pixels cannot dominate.
        const double positive_mean =
            positive_green_dominance_sum / positive_green_samples;
        const double positive_fraction =
            static_cast<double>(positive_green_samples) / samples;
        const double coverage_weighted_positive =
            positive_mean * std::sqrt(positive_fraction);
        return static_cast<float>(
            std::max(full_box_mean, coverage_weighted_positive));
    }

    float positive_green_fraction() const
    {
        return samples > 0
                   ? static_cast<float>(positive_green_samples) / samples
                   : 0.0F;
    }
};

struct CoreBlob {
    float center_x = 0.0F;
    float center_y = 0.0F;
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;
    float density = 0.0F;
    float roundness = 0.0F;
};

template <typename Value>
struct IntegralPlane {
    int width = 0;
    int height = 0;
    std::vector<Value> values;

    explicit IntegralPlane(int image_width = 0, int image_height = 0)
        : width(image_width), height(image_height),
          values(static_cast<std::size_t>(image_width + 1) *
                 (image_height + 1), Value{})
    {
    }

    Value &at(int x, int y)
    {
        return values[static_cast<std::size_t>(y) * (width + 1) + x];
    }

    Value at(int x, int y) const
    {
        return values[static_cast<std::size_t>(y) * (width + 1) + x];
    }

    uint64_t sum(const Rect &rect) const
    {
        const int64_t result =
            static_cast<int64_t>(at(rect.x1, rect.y1)) +
            static_cast<int64_t>(at(rect.x0, rect.y0)) -
            static_cast<int64_t>(at(rect.x0, rect.y1)) -
            static_cast<int64_t>(at(rect.x1, rect.y0));
        return static_cast<uint64_t>(std::max<int64_t>(0, result));
    }
};

struct ScalePeak {
    int x = 0;
    int y = 0;
    int diameter = 0;
    float response = 0.0F;
    float green_mean = 0.0F;
    float brightness_mean = 0.0F;
    float brightness_contrast = 0.0F;
    float contrast_z = 0.0F;
};

float clamp01(float value)
{
    return std::max(0.0F, std::min(1.0F, value));
}

float normalized_green_response(float red, float green, float blue)
{
    static const std::array<float, 256> signal_lut = [] {
        std::array<float, 256> values{};
        for (std::size_t index = 0; index < values.size(); ++index) {
            values[index] = std::sqrt(static_cast<float>(index) / 255.0F);
        }
        return values;
    }();
    const float excess = std::max(0.0F, 2.0F * green - red - blue);
    const float ratio = excess / (red + green + blue + 1.0F);
    const float absolute = excess / 510.0F;
    const int peak = std::max(0, std::min(
        255, static_cast<int>(std::max(red, std::max(green, blue)))));
    const float signal = signal_lut[static_cast<std::size_t>(peak)];
    // A ratio alone assigns near-unit "greenness" to dark codec noise. The
    // absolute term and signal-weighted ratio preserve dim real lamps while
    // suppressing chromatic noise whose RGB energy is close to zero.
    return clamp01(0.65F * absolute + 0.35F * ratio * signal);
}

uint8_t fast_normalized_green_response(uint8_t red,
                                       uint8_t green,
                                       uint8_t blue)
{
    // Keep the floating-point definition's two terms, but evaluate them with
    // fixed-point arithmetic and tiny L1-resident tables. The previous
    // 6-bit/channel LUT occupied 256 KiB and caused effectively random L2
    // reads for every pixel on Cortex-A53.
    static const std::array<uint32_t, 766> reciprocal_lut = [] {
        std::array<uint32_t, 766> values{};
        for (std::size_t sum = 0; sum < values.size(); ++sum) {
            values[sum] = static_cast<uint32_t>(
                ((255ULL << 16U) + (sum + 1U) / 2U) / (sum + 1U));
        }
        return values;
    }();
    static const std::array<uint8_t, 256> signal_lut = [] {
        std::array<uint8_t, 256> values{};
        for (std::size_t peak = 0; peak < values.size(); ++peak) {
            values[peak] = static_cast<uint8_t>(std::lround(
                255.0F * std::sqrt(static_cast<float>(peak) / 255.0F)));
        }
        return values;
    }();

    const int excess = 2 * static_cast<int>(green) - red - blue;
    if (excess <= 0) {
        return 0U;
    }
    const int sum = static_cast<int>(red) + green + blue;
    const int peak = std::max(static_cast<int>(red),
                              std::max(static_cast<int>(green),
                                       static_cast<int>(blue)));
    const uint32_t absolute = static_cast<uint32_t>(
        std::min(255, (excess + 1) / 2));
    const uint32_t ratio = std::min<uint32_t>(
        255U, (static_cast<uint32_t>(excess) * reciprocal_lut[sum] +
               (1U << 15U)) >> 16U);
    const uint32_t ratio_signal =
        (ratio * signal_lut[static_cast<std::size_t>(peak)] + 127U) / 255U;
    return static_cast<uint8_t>(
        std::min<uint32_t>(255U,
                           (65U * absolute + 35U * ratio_signal + 50U) /
                               100U));
}

Rect clip_rect(int x, int y, int width, int height, int image_width, int image_height)
{
    Rect result;
    result.x0 = std::max(0, std::min(image_width, x));
    result.y0 = std::max(0, std::min(image_height, y));
    result.x1 = std::max(result.x0, std::min(image_width, x + width));
    result.y1 = std::max(result.y0, std::min(image_height, y + height));
    return result;
}

struct ResponseComponent {
    static constexpr std::size_t kMaxLocalPeaks = 4;
    float center_x = 0.0F;
    float center_y = 0.0F;
    float priority = 0.0F;
    int pixels = 0;
    std::array<int, kMaxLocalPeaks> local_peak_x{};
    std::array<int, kMaxLocalPeaks> local_peak_y{};
    std::array<uint8_t, kMaxLocalPeaks> local_peak_response{};
    std::size_t local_peak_count = 0;
};

bool has_bright_green_evidence(const uint8_t *rgb,
                               int image_width,
                               const Rect &inner,
                               const DetectorConfig &config)
{
    if (config.normalized_min_peak_green == 0) {
        return true;
    }
    const float minimum_dominance = 255.0F * config.min_green_dominance;
    for (int y = inner.y0; y < inner.y1; ++y) {
        for (int x = inner.x0; x < inner.x1; ++x) {
            const std::size_t offset =
                (static_cast<std::size_t>(y) * image_width + x) * 3U;
            const int green = rgb[offset + 1];
            if (green >= config.normalized_min_peak_green &&
                green - std::max(rgb[offset], rgb[offset + 2]) >= minimum_dominance) {
                return true;
            }
        }
    }
    return false;
}

struct LampAppearance {
    bool valid = false;
    float color_fraction = 0.0F;
    float relative_contrast = 0.0F;
    float radial_support = 0.0F;
    float axis_ratio = 0.0F;
    float shape = 0.0F;
    float score = 0.0F;
    float center_x = 0.0F, center_y = 0.0F;
    int x = 0, y = 0, width = 0, height = 0;
};

// Appearance is measured anew on every frame, including confirmed tracks.
// A green maximum alone also describes edges, tape and broad colored surfaces.
// Require a compact, locally brighter patch supported by genuinely green
// pixels. White cores are allowed: color support may come from their halo.
LampAppearance lamp_appearance(const uint8_t *rgb, int width, int height,
                               int cx, int cy, int diameter,
                               const DetectorConfig &config)
{
    LampAppearance result;
    const int radius = std::max(1, diameter / 2);
    const int outer = std::min(std::max(radius + 3, diameter),
        std::min({cx, cy, width - 1 - cx, height - 1 - cy}));
    if (outer > 128) return result; // bound component-validation work
    // Incomplete surroundings cannot establish a closed lamp appearance.
    if (outer < radius + 3) return result;
    std::array<double, 8> sector_sum{};
    std::array<int, 8> sector_count{};
    double luminance_sum = 0, mass = 0, sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;
    double boundary_margin = 0;
    int count = 0, colored = 0, boundary_count = 0, peak_margin = 0, peak_luma = 0, peak_green = 0;
    for (int dy = -outer; dy <= outer; ++dy) {
        for (int dx = -outer; dx <= outer; ++dx) {
            const auto *p = rgb + (static_cast<std::size_t>(cy + dy) * width + cx + dx) * 3;
            const int luminance = (77 * p[0] + 150 * p[1] + 29 * p[2]) >> 8;
            const int margin = static_cast<int>(p[1]) - std::max(p[0], p[2]);
            if (std::abs(dx) == outer || std::abs(dy) == outer) {
                boundary_margin += std::max(0, margin);
                ++boundary_count;
            }
            if (std::abs(dx) <= radius && std::abs(dy) <= radius) {
                ++count;
                luminance_sum += luminance;
                peak_luma = std::max(peak_luma, luminance);
                peak_green = std::max(peak_green, static_cast<int>(p[1]));
                if (margin >= std::max(4.0F, config.lamp_min_green_margin * p[1])) {
                    ++colored;
                    peak_margin = std::max(peak_margin, margin);
                }
            } else {
                int sector;
                if (std::abs(dx) > 2 * std::abs(dy)) sector = dx > 0 ? 0 : 4;
                else if (std::abs(dy) > 2 * std::abs(dx)) sector = dy > 0 ? 2 : 6;
                else if (dx > 0) sector = dy > 0 ? 1 : 7;
                else sector = dy > 0 ? 3 : 5;
                sector_sum[sector] += luminance;
                ++sector_count[sector];
            }
        }
    }
    result.color_fraction = static_cast<float>(colored) / count;
    if (colored < 2 || result.color_fraction < config.lamp_min_color_fraction)
        return result;
    // A white reflection on a broad green surface has a bright core but no
    // localized green support. Require its chromatic halo to stand out too.
    if (peak_margin - boundary_margin / std::max(1, boundary_count) <
        std::max(4.0F, config.lamp_min_relative_contrast * peak_margin)) return result;
    const float inside = static_cast<float>(luminance_sum / count);
    float background = 0;
    int directions = 0;
    for (std::size_t i = 0; i < sector_sum.size(); ++i) {
        if (!sector_count[i]) return result;
        const float surrounding = static_cast<float>(sector_sum[i] / sector_count[i]);
        background += surrounding / 8;
        if (inside - surrounding >= std::max(4.0F, config.lamp_min_relative_contrast * inside))
            ++directions;
    }
    result.radial_support = directions / 8.0F;
    result.relative_contrast = (inside - background) / std::max(inside, 1.0F);
    // Two contaminated sectors tolerate adjacent structure without admitting
    // a half-plane edge. Covariance separately rejects narrow colored lines.
    if (directions < 6 || result.relative_contrast < config.lamp_min_relative_contrast)
        return result;
    // Follow connected support beyond the scoring window. A line endpoint
    // and one lobe of a larger halo must not masquerade as separate lamps.
    const int side = 2 * outer + 1;
    const float background_margin = static_cast<float>(boundary_margin / std::max(1, boundary_count));
    const float support_margin = background_margin + 0.20F * (peak_margin - background_margin);
    const float support_luma = background + std::max(2.0F, 0.15F * (peak_luma - background));
    std::vector<uint8_t> support(static_cast<std::size_t>(side) * side, 0);
    int seed = -1, seed_margin = -1;
    for (int dy = -outer; dy <= outer; ++dy) for (int dx = -outer; dx <= outer; ++dx) {
        const auto *p = rgb + (static_cast<std::size_t>(cy + dy) * width + cx + dx) * 3;
        const int margin = static_cast<int>(p[1]) - std::max(p[0], p[2]);
        const float luma = static_cast<float>((77 * p[0] + 150 * p[1] + 29 * p[2]) >> 8);
        const bool color = margin >= std::max({4.0F, config.lamp_min_green_margin * p[1], support_margin}) &&
            luma >= support_luma;
        const bool core = std::min({p[0], p[1], p[2]}) >= 0.65F * std::max({p[0], p[1], p[2]}) &&
            p[1] >= 0.85F * peak_green && p[1] + 4 >= std::max(p[0], p[2]) && luma > std::max(0.8F * inside,
                background + std::max(4.0F, config.lamp_min_relative_contrast * inside));
        const int index = (dy + outer) * side + dx + outer;
        support[index] = color || core;
        if (color && std::abs(dx) <= radius && std::abs(dy) <= radius && margin > seed_margin) {
            seed = index; seed_margin = margin;
        }
    }
    if (seed < 0) return result;
    std::vector<int> component; component.reserve(support.size()); component.push_back(seed);
    support[seed] = 0;
    int min_x = side, min_y = side, max_x = 0, max_y = 0;
    int component_colored = 0, component_strong_colored = 0;
    int component_peak_green = 0, component_peak_luma = 0;
    std::array<int, 256> component_luma_histogram{};
    std::array<int, 2> strongest_color_green{};
    double center_mass = 0, center_x = 0, center_y = 0;
    for (std::size_t next = 0; next < component.size(); ++next) {
        const int index = component[next], px = index % side, py = index / side;
        if (px == 0 || py == 0 || px == side - 1 || py == side - 1) return result;
        min_x = std::min(min_x, px); min_y = std::min(min_y, py);
        max_x = std::max(max_x, px); max_y = std::max(max_y, py);
        const int dx = px - outer, dy = py - outer;
        mass += 1; sx += dx; sy += dy;
        sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
        const auto *p = rgb + (static_cast<std::size_t>(cy + dy) * width + cx + dx) * 3;
        component_peak_green = std::max(component_peak_green, static_cast<int>(p[1]));
        const int color_margin = static_cast<int>(p[1]) - std::max(p[0], p[2]);
        const float color_luma = static_cast<float>((77 * p[0] + 150 * p[1] + 29 * p[2]) >> 8);
        component_peak_luma = std::max(component_peak_luma, static_cast<int>(color_luma));
        ++component_luma_histogram[static_cast<int>(color_luma)];
        if (color_margin >= std::max({4.0F, config.lamp_min_green_margin * p[1], support_margin}) &&
            color_luma >= support_luma) {
            ++component_colored;
            // Hysteresis: weak halo pixels reconstruct the shape, but a
            // neutral highlight with only a slight green cast cannot establish
            // color identity. Require resolved stronger chromatic support too.
            if (color_margin >= std::max(8.0F, 2 * config.lamp_min_green_margin * p[1])) {
                ++component_strong_colored;
                if (p[1] > strongest_color_green[0]) {
                    strongest_color_green[1] = strongest_color_green[0];
                    strongest_color_green[0] = p[1];
                } else strongest_color_green[1] = std::max(strongest_color_green[1], static_cast<int>(p[1]));
            }
        }
        const double weight = std::max(1.0F,
            static_cast<float>((77 * p[0] + 150 * p[1] + 29 * p[2]) >> 8) - background);
        center_mass += weight; center_x += weight * dx; center_y += weight * dy;
        for (int oy = -1; oy <= 1; ++oy) for (int ox = -1; ox <= 1; ++ox) {
            const int neighbor = index + oy * side + ox;
            if (support[neighbor]) { support[neighbor] = 0; component.push_back(neighbor); }
        }
    }
    // Dark colored edges must not lend their hue to a bright neutral surface.
    // Relate both supporting pixels to this COMPLETE component's signal peak,
    // not the seed window: a small seed on the dark rim can omit its white core.
    if (component_strong_colored < 2 || strongest_color_green[1] < 0.5F * component_peak_green ||
        component_colored / mass < config.lamp_min_color_fraction)
        return result;
    // A blurred glyph can have a connected, almost elliptical halo while its
    // luminous body consists of separated strokes. Check the FULL component's
    // mid-brightness sections, including white cores, independently of hue.
    // Use moderate levels for filled-area evidence. At the bright-core level,
    // LED texture may split the disk, so only its axis ratio is constrained.
    // Unresolved sections (<8 pixels) cannot establish either shape property.
    struct CoreMoments {
        double n = 0, x = 0, y = 0, xx = 0, yy = 0, xy = 0;
    };
    std::array<CoreMoments, 3> cores{};
    const std::array<float, 3> levels{{0.40F, 0.50F, 0.80F}};
    // An isolated glint must not raise the bright-core threshold until only
    // an arbitrary stripe of an otherwise filled textured lamp remains.
    // Use P95 for this high level only; retain the actual peak for mid-level
    // structure and for the independent chromatic-identity checks above.
    int robust_peak_luma = component_peak_luma, cumulative = 0;
    const int peak_rank = static_cast<int>(std::ceil(0.95 * mass));
    for (int luma = 0; luma < 256; ++luma) {
        cumulative += component_luma_histogram[luma];
        if (cumulative >= peak_rank) { robust_peak_luma = luma; break; }
    }
    for (const int index : component) {
        const int dx = index % side - outer, dy = index / side - outer;
        const auto *p = rgb + (static_cast<std::size_t>(cy + dy) * width + cx + dx) * 3;
        const int luma = (77 * p[0] + 150 * p[1] + 29 * p[2]) >> 8;
        for (std::size_t level = 0; level < cores.size(); ++level) {
            const int peak = level == 2 ? robust_peak_luma : component_peak_luma;
            if (luma < background + levels[level] * (peak - background)) continue;
            auto &core = cores[level];
            ++core.n; core.x += dx; core.y += dy;
            core.xx += dx * dx; core.yy += dy * dy; core.xy += dx * dy;
        }
    }
    for (std::size_t level = 0; level < cores.size(); ++level) {
        const auto &core = cores[level];
        if (core.n < 8) continue;
        const double ux = core.x / core.n, uy = core.y / core.n;
        const double vx = std::max(0.0, core.xx / core.n - ux * ux) + 1.0 / 12;
        const double vy = std::max(0.0, core.yy / core.n - uy * uy) + 1.0 / 12;
        const double vxy = core.xy / core.n - ux * uy;
        // A uniformly filled ellipse has area = 4*pi*sqrt(det(covariance)).
        // Concave strokes, holes and separated lobes spread the second moments
        // without filling that area. This needs no hull sort or extra flood.
        const double fill = core.n / (4.0 * 3.14159265358979323846 *
            std::sqrt(std::max(1.0e-12, vx * vy - vxy * vxy)));
        const double delta = std::hypot(vx - vy, 2 * vxy);
        const double axis = std::sqrt((vx + vy + delta) /
            std::max(1.0e-6, vx + vy - delta));
        if ((level < 2 && fill < config.lamp_min_core_fill) || axis > config.lamp_max_axis_ratio)
            return result;
    }
    const double mx = sx / mass, my = sy / mass;
    // Pixel-footprint variance makes the shape of small resolved patches
    // well-defined without allowing a single color-noise pixel to qualify.
    const double xx = std::max(0.0, sxx / mass - mx * mx) + 1.0 / 12;
    const double yy = std::max(0.0, syy / mass - my * my) + 1.0 / 12;
    const double xy = sxy / mass - mx * my;
    const double delta = std::hypot(xx - yy, 2 * xy);
    result.axis_ratio = static_cast<float>(std::sqrt((xx + yy + delta) /
        std::max(1.0e-6, xx + yy - delta)));
    if (result.axis_ratio > config.lamp_max_axis_ratio) return result;
    // A resolved filled ellipse has squared Mahalanobis radius <=4 under
    // its own uniform-area covariance. Allow pixelization/segmentation error,
    // but reject the extended corners of a large flat rectangular surface.
    if (mass >= 64) {
        const double determinant = xx * yy - xy * xy;
        int outside_ellipse = 0;
        for (const int index : component) {
            const double dx = index % side - outer - mx, dy = index / side - outer - my;
            if ((yy * dx * dx - 2 * xy * dx * dy + xx * dy * dy) /
                determinant > 4.5) ++outside_ellipse;
        }
        if (outside_ellipse > 0.03 * mass) return result;
    }
    result.shape = 1.0F / result.axis_ratio;
    result.center_x = cx + static_cast<float>(center_x / center_mass);
    result.center_y = cy + static_cast<float>(center_y / center_mass);
    result.x = cx + min_x - outer; result.y = cy + min_y - outer;
    result.width = max_x - min_x + 1; result.height = max_y - min_y + 1;
    result.score = 0.35F * clamp01(result.color_fraction / 0.5F) +
                   0.35F * clamp01(result.relative_contrast / 0.6F) +
                   0.30F * result.shape;
    result.valid = true;
    return result;
}

ScalePeak evaluate_component_scale(const uint8_t *rgb,
                                   int image_width,
                                   int image_height,
                                   int center_x,
                                   int center_y,
                                   int diameter,
                                   const DetectorConfig &config,
                                   bool tracking_confirmed,
                                   const IntegralPlane<uint32_t> *response_plane = nullptr,
                                   const IntegralPlane<uint32_t> *brightness_plane = nullptr,
                                   const IntegralPlane<uint64_t> *squared_plane = nullptr,
                                   const Rect *plane_region = nullptr)
{
    ScalePeak rejected;
    const int radius = std::max(1, diameter / 2);
    const int outer_radius = std::max(radius + 2, diameter);
    const Rect inner = clip_rect(center_x - radius, center_y - radius,
                                 2 * radius + 1, 2 * radius + 1,
                                 image_width, image_height);
    const Rect outer = clip_rect(center_x - outer_radius,
                                 center_y - outer_radius,
                                 2 * outer_radius + 1,
                                 2 * outer_radius + 1,
                                 image_width, image_height);
    const int inner_area = (inner.x1 - inner.x0) * (inner.y1 - inner.y0);
    const int outer_area = (outer.x1 - outer.x0) * (outer.y1 - outer.y0);
    const int ring_area = outer_area - inner_area;
    if (inner_area <= 0 || ring_area <= 0) {
        return rejected;
    }

    uint64_t inner_green_sum = 0;
    uint64_t inner_brightness_sum = 0;
    uint64_t ring_green_sum = 0;
    uint64_t ring_brightness_sum = 0;
    uint64_t ring_brightness_squared_sum = 0;
    const bool cached = plane_region && response_plane && brightness_plane && squared_plane &&
        outer.x0 >= plane_region->x0 && outer.y0 >= plane_region->y0 &&
        outer.x1 <= plane_region->x1 && outer.y1 <= plane_region->y1;
    if (cached) {
        auto local = [plane_region](Rect r) {
            r.x0-=plane_region->x0; r.x1-=plane_region->x0;
            r.y0-=plane_region->y0; r.y1-=plane_region->y0; return r;
        };
        const auto a=local(inner), b=local(outer);
        inner_green_sum=response_plane->sum(a);
        inner_brightness_sum=brightness_plane->sum(a);
        ring_green_sum=response_plane->sum(b)-inner_green_sum;
        ring_brightness_sum=brightness_plane->sum(b)-inner_brightness_sum;
        ring_brightness_squared_sum=squared_plane->sum(b)-squared_plane->sum(a);
    } else {
    for (int y = outer.y0; y < outer.y1; ++y) {
        for (int x = outer.x0; x < outer.x1; ++x) {
            const std::size_t offset =
                (static_cast<std::size_t>(y) * image_width + x) * 3U;
            const uint8_t red = rgb[offset];
            const uint8_t green = rgb[offset + 1];
            const uint8_t blue = rgb[offset + 2];
            const uint32_t green_response =
                fast_normalized_green_response(red, green, blue);
            const uint32_t brightness =
                (77U * red + 150U * green + 29U * blue) >> 8U;
            if (x >= inner.x0 && x < inner.x1 &&
                y >= inner.y0 && y < inner.y1) {
                inner_green_sum += green_response;
                inner_brightness_sum += brightness;
            } else {
                ring_green_sum += green_response;
                ring_brightness_sum += brightness;
                ring_brightness_squared_sum +=
                    static_cast<uint64_t>(brightness) * brightness;
            }
        }
    }

    }

    const float inner_normalizer =
        1.0F / (255.0F * static_cast<float>(inner_area));
    const float ring_normalizer =
        1.0F / (255.0F * static_cast<float>(ring_area));
    const float inner_green =
        static_cast<float>(inner_green_sum) * inner_normalizer;
    const float inner_brightness =
        static_cast<float>(inner_brightness_sum) * inner_normalizer;
    const float ring_green =
        static_cast<float>(ring_green_sum) * ring_normalizer;
    const float ring_brightness =
        static_cast<float>(ring_brightness_sum) * ring_normalizer;
    const float brightness_contrast = inner_brightness - ring_brightness;
    const float response = inner_green - ring_green +
        config.normalized_brightness_weight *
            std::max(-0.10F, brightness_contrast);
    if (response < config.min_normalized_green_response ||
        inner_green < config.min_normalized_green_response ||
        inner_brightness < config.min_normalized_inner_brightness) {
        return rejected;
    }

    const float ring_brightness_squared =
        static_cast<float>(ring_brightness_squared_sum) /
        (255.0F * 255.0F * static_cast<float>(ring_area));
    const float ring_variance = std::max(
        0.0F, ring_brightness_squared - ring_brightness * ring_brightness);
    const float contrast_z = brightness_contrast /
        std::sqrt(ring_variance + 0.0025F);
    const float minimum_contrast_z = tracking_confirmed
        ? config.min_tracking_normalized_contrast_z
        : config.min_normalized_contrast_z;
    if (contrast_z < minimum_contrast_z) {
        return rejected;
    }
    // Evaluate native RGB evidence only after the existing cheap gates. This
    // also applies when response/contrast statistics came from integral planes.
    if (!has_bright_green_evidence(rgb, image_width, inner, config)) {
        return rejected;
    }

    return ScalePeak{center_x, center_y, diameter, response, inner_green,
                     inner_brightness, brightness_contrast, contrast_z};
}

std::vector<ScalePeak> collect_sparse_multiscale_peaks(
    const uint8_t *rgb,
    int image_width,
    int image_height,
    const Rect &search_region,
    const DetectorConfig &config,
    bool tracking_confirmed)
{
    const int search_width = search_region.x1 - search_region.x0;
    const int search_height = search_region.y1 - search_region.y0;
    std::vector<ScalePeak> peaks;
    if (search_width <= 0 || search_height <= 0) {
        return peaks;
    }
    std::vector<int> diameters(config.multiscale_diameters_px.begin(), config.multiscale_diameters_px.end());
    if (config.enable_lamp_appearance) {
        // Complete-component validation must also cover close-range lamps,
        // not merely fragments at the old small peak scales. Extend by octaves
        // while leaving room for surrounding evidence in the current ROI.
        const int limit = std::min(128, 3 * std::min(search_width, search_height) / 4);
        for (int64_t next = static_cast<int64_t>(diameters.back()) * 2; next <= limit; next *= 2)
            diameters.push_back(static_cast<int>(next));
    }

    {
        // Divide the capture cone into tiny tiles and retain the strongest
        // qualifying green pixel in each tile. Unlike connected components,
        // this remains O(pixels) even when a cyan wall or LED screen forms one
        // huge component, and it naturally preserves several spatially
        // distinct hypotheses on a motion-blurred lamp.
        constexpr int kTileSize = 4;
        // The 256/128 budgets are required by the long-range clips: reducing
        // either budget loses valid 5-8 px lamps in LED-heavy frames. Use
        // nth_element below to keep the budget without fully sorting thousands
        // of provisional peaks.
        constexpr std::size_t kCoarseHypotheses = 256;
        constexpr std::size_t kExactHypotheses = 128;
        struct TileSeed {
            uint8_t response = 0;
            int x = 0;
            int y = 0;
            uint8_t brightness = 0;
            int bright_x = 0;
            int bright_y = 0;
        };

        const uint8_t response_threshold = static_cast<uint8_t>(std::max(
            1, static_cast<int>(std::ceil(
                   255.0F * config.min_normalized_green_response))));
        const uint8_t brightness_threshold = static_cast<uint8_t>(std::max(
            1, static_cast<int>(std::ceil(
                   255.0F * config.min_normalized_inner_brightness))));
        const int tile_columns =
            (search_width + kTileSize - 1) / kTileSize;
        const int tile_rows =
            (search_height + kTileSize - 1) / kTileSize;
        std::vector<TileSeed> tile_seeds(
            static_cast<std::size_t>(tile_columns) * tile_rows);
        // Coarse hypotheses keep the original search-region clipping. Exact
        // refinement may later need surrounding pixels; grow its cache only
        // after the retained hypotheses establish the necessary support.
        Rect statistics_region = search_region;
        const int statistics_width = statistics_region.x1 - statistics_region.x0;
        const int statistics_height = statistics_region.y1 - statistics_region.y0;
        IntegralPlane<uint32_t> response_integral(statistics_width, statistics_height);
        // Coarse ranking must use the same brightness contribution as exact
        // refinement; otherwise a pale bright core is pruned by green alone.
        IntegralPlane<uint32_t> brightness_integral(statistics_width, statistics_height);
        IntegralPlane<uint64_t> squared_integral(config.integral_peak_statistics ? statistics_width : 0,
                                                config.integral_peak_statistics ? statistics_height : 0);
        for (int local_y = 0; local_y < statistics_height; ++local_y) {
            uint32_t response_row_sum = 0, brightness_row_sum = 0;
            uint64_t squared_row_sum = 0;
            const int image_y = statistics_region.y0 + local_y;
            for (int local_x = 0; local_x < statistics_width; ++local_x) {
                const int image_x = statistics_region.x0 + local_x;
                const std::size_t image_offset =
                    (static_cast<std::size_t>(image_y) * image_width +
                     image_x) * 3U;
                const uint8_t red = rgb[image_offset];
                const uint8_t green = rgb[image_offset + 1];
                const uint8_t blue = rgb[image_offset + 2];
                const uint8_t response =
                    fast_normalized_green_response(red, green, blue);
                response_row_sum += response;
                response_integral.at(local_x + 1, local_y + 1) =
                    response_integral.at(local_x + 1, local_y) +
                    response_row_sum;
                const uint8_t brightness = static_cast<uint8_t>(
                    (77U * red + 150U * green + 29U * blue) >> 8U);
                brightness_row_sum+=brightness;
                brightness_integral.at(local_x+1,local_y+1)=brightness_integral.at(local_x+1,local_y)+brightness_row_sum;
                if (config.integral_peak_statistics) {
                    squared_row_sum+=static_cast<uint64_t>(brightness)*brightness;
                    squared_integral.at(local_x+1,local_y+1)=squared_integral.at(local_x+1,local_y)+squared_row_sum;
                }
                if (image_x < search_region.x0 || image_x >= search_region.x1 ||
                    image_y < search_region.y0 || image_y >= search_region.y1) continue;
                TileSeed &seed = tile_seeds[
                    static_cast<std::size_t>((image_y - search_region.y0) / kTileSize) *
                        tile_columns +
                    (image_x - search_region.x0) / kTileSize];
                if (green >= red && green >= blue && brightness > seed.brightness) {
                    seed.brightness = brightness; seed.bright_x = image_x; seed.bright_y = image_y;
                }
                if (response < response_threshold || brightness < brightness_threshold) continue;
                if (response > seed.response) {
                    seed.response = response;
                    seed.x = image_x;
                    seed.y = image_y;
                }
            }
        }

        const auto rank_peak = [&](int image_center_x,
                                   int image_center_y,
                                   int diameter) {
            ScalePeak result;
            const int local_center_x = image_center_x - search_region.x0;
            const int local_center_y = image_center_y - search_region.y0;
            const int radius = std::max(1, diameter / 2);
            const int outer_radius = std::max(radius + 2, diameter);
            const Rect inner = clip_rect(
                local_center_x - radius, local_center_y - radius,
                2 * radius + 1, 2 * radius + 1,
                search_width, search_height);
            const Rect outer = clip_rect(
                local_center_x - outer_radius,
                local_center_y - outer_radius,
                2 * outer_radius + 1, 2 * outer_radius + 1,
                search_width, search_height);
            const int inner_area =
                (inner.x1 - inner.x0) * (inner.y1 - inner.y0);
            const int outer_area =
                (outer.x1 - outer.x0) * (outer.y1 - outer.y0);
            const int ring_area = outer_area - inner_area;
            if (inner_area <= 0 || ring_area <= 0) {
                return result;
            }
            // Preserve the coarse search-window clipping and ranking exactly;
            // only the integral-plane origin changes when padding is cached.
            const auto statistics_rect = [&](Rect rect) {
                const int dx = search_region.x0 - statistics_region.x0;
                const int dy = search_region.y0 - statistics_region.y0;
                rect.x0 += dx; rect.x1 += dx;
                rect.y0 += dy; rect.y1 += dy;
                return rect;
            };
            const Rect cached_inner = statistics_rect(inner);
            const Rect cached_outer = statistics_rect(outer);
            const uint64_t inner_sum = response_integral.sum(cached_inner);
            const uint64_t outer_sum = response_integral.sum(cached_outer);
            const float inner_green = static_cast<float>(inner_sum) /
                (255.0F * inner_area);
            if (inner_green <
                0.5F * config.min_normalized_green_response) {
                return result;
            }
            const float ring_green =
                static_cast<float>(outer_sum - inner_sum) /
                (255.0F * ring_area);
            const auto inner_luma_sum = brightness_integral.sum(cached_inner);
            const float inner_luma = static_cast<float>(inner_luma_sum) / (255.0F * inner_area);
            const float ring_luma = static_cast<float>(brightness_integral.sum(cached_outer) - inner_luma_sum) /
                (255.0F * ring_area);
            result.x = image_center_x;
            result.y = image_center_y;
            result.diameter = diameter;
            result.response = inner_green - ring_green +
                config.normalized_brightness_weight * std::max(-0.10F, inner_luma - ring_luma);
            result.green_mean = inner_green;
            return result;
        };

        std::vector<ScalePeak> coarse_peaks;
        coarse_peaks.reserve(2U * tile_seeds.size());
        for (const auto &seed : tile_seeds) {
            if (seed.response == 0U) {
                continue;
            }
            // Keep a bright seed as well as a chromatic seed in tiles that
            // contain green evidence. This preserves white-core/green-halo
            // lamps without giving every neutral highlight a search budget.
            for (int seed_kind = 0; seed_kind < 2; ++seed_kind) {
            if (seed_kind == 1 && (seed.brightness == 0 ||
                (seed.bright_x == seed.x && seed.bright_y == seed.y))) continue;
            const int seed_x = seed_kind ? seed.bright_x : seed.x;
            const int seed_y = seed_kind ? seed.bright_y : seed.y;
            ScalePeak best;
            ScalePeak second_best;
            for (const int diameter : diameters) {
                const ScalePeak candidate =
                    rank_peak(seed_x, seed_y, diameter);
                if (candidate.diameter == 0) {
                    continue;
                }
                if (best.diameter == 0 ||
                    candidate.response > best.response) {
                    second_best = best;
                    best = candidate;
                } else if (second_best.diameter == 0 ||
                           candidate.response > second_best.response) {
                    second_best = candidate;
                }
            }
            if (best.diameter > 0) {
                coarse_peaks.push_back(best);
            }
            if (second_best.diameter > 0) {
                coarse_peaks.push_back(second_best);
            }
            }
        }
        const auto stronger_peak =
            [](const ScalePeak &left, const ScalePeak &right) {
                return left.response > right.response;
            };
        if (coarse_peaks.size() > kCoarseHypotheses) {
            std::nth_element(coarse_peaks.begin(),
                             coarse_peaks.begin() + kCoarseHypotheses,
                             coarse_peaks.end(), stronger_peak);
            coarse_peaks.resize(kCoarseHypotheses);
        }

        std::vector<ScalePeak> refined_peaks = coarse_peaks;
        refined_peaks.reserve(
            coarse_peaks.size() * (1U + 8U * 3U));
        for (const auto &coarse : coarse_peaks) {
            for (int offset_y = -1; offset_y <= 1; ++offset_y) {
                for (int offset_x = -1; offset_x <= 1; ++offset_x) {
                    if (offset_x == 0 && offset_y == 0) {
                        continue;
                    }
                    for (std::size_t scale = 0;
                         scale < std::min<std::size_t>(
                             3U, config.multiscale_diameters_px.size());
                         ++scale) {
                        const ScalePeak candidate = rank_peak(
                            coarse.x + offset_x, coarse.y + offset_y,
                            config.multiscale_diameters_px[scale]);
                        if (candidate.diameter > 0) {
                            refined_peaks.push_back(candidate);
                        }
                    }
                }
            }
        }
        // Neighbour refinement can visit the same location and scale from
        // several seeds. Their integral statistics and rank are identical;
        // spending several slots on them can remove the full-lamp hypothesis
        // while retaining many copies of small kernels on its colored rim.
        // Preserve the first occurrence and its order before applying the
        // existing fixed budget; no additional exact evaluations are allowed.
        struct PeakLocationHash {
            std::size_t operator()(const std::array<int, 3> &location) const
            {
                std::size_t hash = 0;
                for (const int value : location)
                    hash ^= std::hash<int>{}(value) + 0x9e3779b9U + (hash << 6) + (hash >> 2);
                return hash;
            }
        };
        std::unordered_set<std::array<int, 3>, PeakLocationHash> seen_locations;
        seen_locations.reserve(refined_peaks.size());
        auto unique_end = refined_peaks.begin();
        for (const auto &peak : refined_peaks) {
            if (seen_locations.insert({peak.x, peak.y, peak.diameter}).second)
                *unique_end++ = peak;
        }
        refined_peaks.erase(unique_end, refined_peaks.end());
        if (refined_peaks.size() > kExactHypotheses) {
            std::nth_element(refined_peaks.begin(),
                             refined_peaks.begin() + kExactHypotheses,
                             refined_peaks.end(), stronger_peak);
            refined_peaks.resize(kExactHypotheses);
        }
        if (config.integral_peak_statistics) {
            Rect required = statistics_region;
            for (const auto &ranked : refined_peaks) {
                const int radius = std::max(1, ranked.diameter / 2);
                const int outer = std::max(radius + 2, ranked.diameter);
                const Rect support = clip_rect(ranked.x - outer, ranked.y - outer,
                    2 * outer + 1, 2 * outer + 1, image_width, image_height);
                required.x0 = std::min(required.x0, support.x0);
                required.y0 = std::min(required.y0, support.y0);
                required.x1 = std::max(required.x1, support.x1);
                required.y1 = std::max(required.y1, support.y1);
            }
            if (required.x0 != statistics_region.x0 || required.y0 != statistics_region.y0 ||
                required.x1 != statistics_region.x1 || required.y1 != statistics_region.y1) {
                // Only genuine cache misses pay for an expanded plane. Without
                // this, large optional scales would enlarge every tracking
                // cache even when all selected small peaks already fit.
                statistics_region = required;
                const int w = required.x1 - required.x0, h = required.y1 - required.y0;
                response_integral = IntegralPlane<uint32_t>(w, h);
                brightness_integral = IntegralPlane<uint32_t>(w, h);
                squared_integral = IntegralPlane<uint64_t>(w, h);
                for (int y = 0; y < h; ++y) {
                    uint32_t response_row = 0, brightness_row = 0;
                    uint64_t squared_row = 0;
                    for (int x = 0; x < w; ++x) {
                        const auto *p = rgb + (static_cast<std::size_t>(y + required.y0) *
                            image_width + x + required.x0) * 3;
                        const uint32_t brightness = (77U * p[0] + 150U * p[1] + 29U * p[2]) >> 8U;
                        response_row += fast_normalized_green_response(p[0], p[1], p[2]);
                        brightness_row += brightness;
                        squared_row += static_cast<uint64_t>(brightness) * brightness;
                        response_integral.at(x + 1, y + 1) = response_integral.at(x + 1, y) + response_row;
                        brightness_integral.at(x + 1, y + 1) = brightness_integral.at(x + 1, y) + brightness_row;
                        squared_integral.at(x + 1, y + 1) = squared_integral.at(x + 1, y) + squared_row;
                    }
                }
            }
        }
        peaks.reserve(refined_peaks.size());
        for (const auto &ranked : refined_peaks) {
            const ScalePeak exact = evaluate_component_scale(
                rgb, image_width, image_height, ranked.x, ranked.y,
                ranked.diameter, config, tracking_confirmed,
                config.integral_peak_statistics ? &response_integral : nullptr,
                config.integral_peak_statistics ? &brightness_integral : nullptr,
                config.integral_peak_statistics ? &squared_integral : nullptr,
                config.integral_peak_statistics ? &statistics_region : nullptr);
            if (exact.diameter > 0) {
                peaks.push_back(exact);
            }
        }
        return peaks;
    }

#if 0  // Disabled connected-component A/B reference; production uses tiles.

    // Only one byte per searched pixel is retained. A pixel becomes a seed
    // when it could contribute to a passing local mean. A later ring test is
    // still authoritative, so this first pass deliberately favors recall.
    const uint8_t response_threshold = static_cast<uint8_t>(std::max(
        1, static_cast<int>(std::ceil(
               255.0F * config.min_normalized_green_response))));
    const uint8_t brightness_threshold = static_cast<uint8_t>(std::max(
        1, static_cast<int>(std::ceil(
               255.0F * config.min_normalized_inner_brightness))));
    std::vector<uint8_t> response_map(
        static_cast<std::size_t>(search_width) * search_height);
    IntegralPlane<uint32_t> response_integral(search_width, search_height);
    for (int local_y = 0; local_y < search_height; ++local_y) {
        uint32_t response_row_sum = 0;
        const int image_y = search_region.y0 + local_y;
        for (int local_x = 0; local_x < search_width; ++local_x) {
            const int image_x = search_region.x0 + local_x;
            const std::size_t image_offset =
                (static_cast<std::size_t>(image_y) * image_width + image_x) *
                3U;
            const uint8_t red = rgb[image_offset];
            const uint8_t green = rgb[image_offset + 1];
            const uint8_t blue = rgb[image_offset + 2];
            const uint8_t response =
                fast_normalized_green_response(red, green, blue);
            const uint8_t brightness = static_cast<uint8_t>(
                (77U * red + 150U * green + 29U * blue) >> 8U);
            response_row_sum += response;
            response_integral.at(local_x + 1, local_y + 1) =
                response_integral.at(local_x + 1, local_y) +
                response_row_sum;
            response_map[static_cast<std::size_t>(local_y) * search_width +
                         local_x] =
                response >= response_threshold &&
                        brightness >= brightness_threshold
                    ? response
                    : 0U;
        }
    }

    int largest_diameter = 1;
    for (const int diameter : config.multiscale_diameters_px) {
        largest_diameter = std::max(largest_diameter, diameter);
    }
    const int max_component_span = std::max(32, 6 * largest_diameter);
    std::vector<ResponseComponent> components;
    components.reserve(128);
    std::vector<uint32_t> stack;
    stack.reserve(256);
    for (int seed_y = 0; seed_y < search_height; ++seed_y) {
        for (int seed_x = 0; seed_x < search_width; ++seed_x) {
            const std::size_t seed_index =
                static_cast<std::size_t>(seed_y) * search_width + seed_x;
            const uint8_t seed_response = response_map[seed_index];
            if (seed_response == 0U) {
                continue;
            }

            stack.clear();
            response_map[seed_index] = 0U;
            stack.push_back((static_cast<uint32_t>(seed_index) << 8U) |
                            seed_response);
            uint64_t response_sum = 0;
            uint64_t weighted_x_sum = 0;
            uint64_t weighted_y_sum = 0;
            int pixels = 0;
            int min_x = seed_x;
            int max_x = seed_x;
            int min_y = seed_y;
            int max_y = seed_y;
            std::array<int, ResponseComponent::kMaxLocalPeaks> local_peak_x{};
            std::array<int, ResponseComponent::kMaxLocalPeaks> local_peak_y{};
            std::array<uint8_t, ResponseComponent::kMaxLocalPeaks>
                local_peak_response{};
            std::size_t local_peak_count = 0;
            while (!stack.empty()) {
                const uint32_t packed = stack.back();
                stack.pop_back();
                const uint8_t response =
                    static_cast<uint8_t>(packed & 0xffU);
                const uint32_t index = packed >> 8U;
                const int local_y = static_cast<int>(index / search_width);
                const int local_x = static_cast<int>(index) -
                    local_y * search_width;
                response_sum += response;
                weighted_x_sum += static_cast<uint64_t>(response) * local_x;
                weighted_y_sum += static_cast<uint64_t>(response) * local_y;
                ++pixels;
                min_x = std::min(min_x, local_x);
                max_x = std::max(max_x, local_x);
                min_y = std::min(min_y, local_y);
                max_y = std::max(max_y, local_y);
                std::size_t nearby_peak = local_peak_count;
                for (std::size_t peak_index = 0;
                     peak_index < local_peak_count; ++peak_index) {
                    const int dx = local_x - local_peak_x[peak_index];
                    const int dy = local_y - local_peak_y[peak_index];
                    if (dx * dx + dy * dy <= 9) {
                        nearby_peak = peak_index;
                        break;
                    }
                }
                if (nearby_peak < local_peak_count) {
                    if (response > local_peak_response[nearby_peak]) {
                        local_peak_x[nearby_peak] = local_x;
                        local_peak_y[nearby_peak] = local_y;
                        local_peak_response[nearby_peak] = response;
                    }
                } else if (local_peak_count <
                           ResponseComponent::kMaxLocalPeaks) {
                    local_peak_x[local_peak_count] = local_x;
                    local_peak_y[local_peak_count] = local_y;
                    local_peak_response[local_peak_count] = response;
                    ++local_peak_count;
                } else {
                    const auto weakest = std::min_element(
                        local_peak_response.begin(),
                        local_peak_response.end());
                    if (response > *weakest) {
                        const std::size_t peak_index =
                            static_cast<std::size_t>(
                                weakest - local_peak_response.begin());
                        local_peak_x[peak_index] = local_x;
                        local_peak_y[peak_index] = local_y;
                        local_peak_response[peak_index] = response;
                    }
                }

                for (int neighbor_y = std::max(0, local_y - 1);
                     neighbor_y <= std::min(search_height - 1, local_y + 1);
                     ++neighbor_y) {
                    for (int neighbor_x = std::max(0, local_x - 1);
                         neighbor_x <= std::min(search_width - 1,
                                                local_x + 1);
                         ++neighbor_x) {
                        const std::size_t neighbor_index =
                            static_cast<std::size_t>(neighbor_y) *
                                search_width +
                            neighbor_x;
                        const uint8_t neighbor_response =
                            response_map[neighbor_index];
                        if (neighbor_response == 0U) {
                            continue;
                        }
                        response_map[neighbor_index] = 0U;
                        stack.push_back(
                            (static_cast<uint32_t>(neighbor_index) << 8U) |
                            neighbor_response);
                    }
                }
            }

            if (pixels <= 0 || max_x - min_x + 1 > max_component_span ||
                max_y - min_y + 1 > max_component_span) {
                continue;
            }
            const float inverse_response = 1.0F /
                static_cast<float>(std::max<uint64_t>(1, response_sum));
            const float center_x = search_region.x0 +
                static_cast<float>(weighted_x_sum) * inverse_response;
            const float center_y = search_region.y0 +
                static_cast<float>(weighted_y_sum) * inverse_response;
            const float mean_response =
                static_cast<float>(response_sum) / (255.0F * pixels);
            const float priority = mean_response *
                std::sqrt(static_cast<float>(std::min(pixels, 64)));
            ResponseComponent component;
            component.center_x = center_x;
            component.center_y = center_y;
            component.priority = priority;
            component.pixels = pixels;
            component.local_peak_count = local_peak_count;
            component.local_peak_response = local_peak_response;
            for (std::size_t peak_index = 0;
                 peak_index < local_peak_count; ++peak_index) {
                component.local_peak_x[peak_index] =
                    search_region.x0 + local_peak_x[peak_index];
                component.local_peak_y[peak_index] =
                    search_region.y0 + local_peak_y[peak_index];
            }
            components.push_back(component);
        }
    }

    std::sort(components.begin(), components.end(),
              [](const ResponseComponent &left,
                 const ResponseComponent &right) {
                  return left.priority > right.priority;
              });
    if (components.size() > 64U) {
        components.resize(64U);
    }
    std::vector<ScalePeak> preliminary_peaks;
    preliminary_peaks.reserve(64U * components.size());
    const auto add_preliminary_peak =
        [&](int image_center_x, int image_center_y, int diameter) {
            const int local_center_x = image_center_x - search_region.x0;
            const int local_center_y = image_center_y - search_region.y0;
            const int radius = std::max(1, diameter / 2);
            const int outer_radius = std::max(radius + 2, diameter);
            const Rect inner = clip_rect(
                local_center_x - radius, local_center_y - radius,
                2 * radius + 1, 2 * radius + 1,
                search_width, search_height);
            const Rect outer = clip_rect(
                local_center_x - outer_radius,
                local_center_y - outer_radius,
                2 * outer_radius + 1, 2 * outer_radius + 1,
                search_width, search_height);
            const int inner_area =
                (inner.x1 - inner.x0) * (inner.y1 - inner.y0);
            const int outer_area =
                (outer.x1 - outer.x0) * (outer.y1 - outer.y0);
            const int ring_area = outer_area - inner_area;
            if (inner_area <= 0 || ring_area <= 0) {
                return;
            }
            const uint64_t inner_sum = response_integral.sum(inner);
            const uint64_t outer_sum = response_integral.sum(outer);
            const float inner_green = static_cast<float>(inner_sum) /
                (255.0F * inner_area);
            if (inner_green <
                0.5F * config.min_normalized_green_response) {
                return;
            }
            const float ring_green =
                static_cast<float>(outer_sum - inner_sum) /
                (255.0F * ring_area);
            // This is only a cheap ranking score. The exact RGB brightness,
            // variance and configured gates are evaluated after global
            // pruning below.
            const float rank_score =
                inner_green - ring_green + 0.05F * inner_green;
            preliminary_peaks.push_back(
                ScalePeak{image_center_x, image_center_y, diameter,
                          rank_score, inner_green, 0.0F, 0.0F, 0.0F});
        };

    for (const auto &component : components) {
        std::vector<std::array<int, 2>> centers;
        centers.reserve(1U + component.local_peak_count);
        centers.push_back({
            static_cast<int>(std::lround(component.center_x)),
            static_cast<int>(std::lround(component.center_y)),
        });
        std::size_t strongest_peak_index = 0;
        for (std::size_t peak_index = 0;
             peak_index < component.local_peak_count; ++peak_index) {
            centers.push_back({component.local_peak_x[peak_index],
                               component.local_peak_y[peak_index]});
            if (component.local_peak_response[peak_index] >
                component.local_peak_response[strongest_peak_index]) {
                strongest_peak_index = peak_index;
            }
        }
        for (const auto &center : centers) {
            for (const int diameter : config.multiscale_diameters_px) {
                add_preliminary_peak(center[0], center[1], diameter);
            }
        }
        // A distant lamp may be embedded in a much larger weak-green
        // component (for example a cyan wall). Its strongest pixel remains a
        // good seed, but a one-pixel shift changes a 3x3 ring response
        // materially. Refine only the three small scales in a 3x3
        // neighborhood; this recovers the dense scan's centering precision at
        // a bounded cost of 24 tiny windows per component.
        for (int offset_y = -1; offset_y <= 1; ++offset_y) {
            for (int offset_x = -1; offset_x <= 1; ++offset_x) {
                if (offset_x == 0 && offset_y == 0) {
                    continue;
                }
                for (std::size_t scale = 0;
                     scale < std::min<std::size_t>(
                         3U, config.multiscale_diameters_px.size());
                     ++scale) {
                    add_preliminary_peak(
                        component.local_peak_x[strongest_peak_index] +
                            offset_x,
                        component.local_peak_y[strongest_peak_index] +
                            offset_y,
                        config.multiscale_diameters_px[scale]);
                }
            }
        }
    }

    std::sort(preliminary_peaks.begin(), preliminary_peaks.end(),
              [](const ScalePeak &left, const ScalePeak &right) {
                  return left.response > right.response;
              });
    if (preliminary_peaks.size() > 128U) {
        preliminary_peaks.resize(128U);
    }
    peaks.reserve(preliminary_peaks.size());
    for (const auto &preliminary : preliminary_peaks) {
        const ScalePeak exact = evaluate_component_scale(
            rgb, image_width, image_height, preliminary.x, preliminary.y,
            preliminary.diameter, config, tracking_confirmed);
        if (exact.diameter > 0) {
            peaks.push_back(exact);
        }
    }
    return peaks;
#endif
}

RgbStatistics sample_rect(const uint8_t *rgb,
                          int image_width,
                          const Rect &rect,
                          int grid,
                          const Rect *excluded = nullptr)
{
    RgbStatistics statistics;
    const int width = rect.x1 - rect.x0;
    const int height = rect.y1 - rect.y0;
    if (rgb == nullptr || width <= 0 || height <= 0) {
        return statistics;
    }

    const int step_x = std::max(1, (width + grid - 1) / grid);
    const int step_y = std::max(1, (height + grid - 1) / grid);
    for (int y = rect.y0; y < rect.y1; y += step_y) {
        for (int x = rect.x0; x < rect.x1; x += step_x) {
            if (excluded != nullptr && x >= excluded->x0 && x < excluded->x1 &&
                y >= excluded->y0 && y < excluded->y1) {
                continue;
            }
            const std::size_t offset =
                (static_cast<std::size_t>(y) * image_width + x) * 3U;
            statistics.add(rgb[offset], rgb[offset + 1], rgb[offset + 2]);
        }
    }
    return statistics;
}

int intersection_area(int ax, int ay, int aw, int ah,
                      int bx, int by, int bw, int bh)
{
    const int left = std::max(ax, bx);
    const int top = std::max(ay, by);
    const int right = std::min(ax + aw, bx + bw);
    const int bottom = std::min(ay + ah, by + bh);
    return std::max(0, right - left) * std::max(0, bottom - top);
}

bool point_inside_capture_cone(float x,
                               float y,
                               int image_width,
                               int image_height,
                               const DetectorConfig &config)
{
    if (!config.enable_capture_cone) {
        return true;
    }
    const bool calibration_matches_frame =
        config.camera_model.principal_x >= 0.0F &&
        config.camera_model.principal_x < image_width &&
        config.camera_model.principal_y >= 0.0F &&
        config.camera_model.principal_y < image_height;
    if (!calibration_matches_frame) {
        return true;
    }
    const auto angles = detail::pixel_to_angles(x, y, config.camera_model);
    constexpr float kPi = 3.14159265358979323846F;
    const float radius_rad = config.capture_cone_deg * kPi / 180.0F;
    return std::hypot(angles[0], angles[1]) <= radius_rad;
}

}  // namespace

struct GreenLightDetector::Candidate {
    detail::CandidateObservation observation;
    GreenLightCandidateDebug debug;
};

GreenLightDetector::GreenLightDetector(
    const DetectorConfig &config,
    const ArmorConfig &armor_config,
    const TargetGeometryConfig &target_geometry,
    const NpuConfig &npu_config,
    std::shared_ptr<TargetPoseValidator> pose_validator)
    : config_(config),
      armor_config_(armor_config),
      target_geometry_(target_geometry),
      npu_config_(npu_config),
      tracker_(config),
      pose_validator_(std::move(pose_validator))
{
}

void GreenLightDetector::reset()
{
    tracker_.reset();
    last_candidates_.clear();
    candidate_hypotheses_.fill(CandidateHypothesis{});
    last_model_timestamp_us_ = 0;
    last_model_positive_timestamp_us_ = 0;
    last_model_inference_ms_ = 0.0F;
    last_pose_validation_ = PoseValidation{};
    last_model_roi_ = CandidateRoi{};
    last_output_timestamp_us_ = 0;
    last_output_angles_.fill(0.0F);
    armor_pose_hits_ = 0;
    armor_blend_start_us_ = 0;
    last_armor_timestamp_us_ = 0;
    last_armor_detection_ = ArmorDetection{};
    armor_candidate_detection_ = ArmorDetection{};
    armor_candidate_green_center_ = Point2f{};
    armor_candidate_timestamp_us_ = 0;
    armor_candidate_hits_ = 0;
    search_model_cursor_ = 0;
    frame_counter_ = 0;
    classical_detection_count_ = 0;
    last_classical_detection_ran_ = false;
    last_classical_detection_ms_ = 0.0F;
}

const std::vector<GreenLightCandidateDebug> &GreenLightDetector::last_candidates() const
{
    return last_candidates_;
}

std::vector<GreenLightDetector::Candidate>
GreenLightDetector::collect_candidates(maix::image::Image &frame)
{
    const int image_width = frame.width();
    const int image_height = frame.height();
    const auto *rgb = static_cast<const uint8_t *>(frame.data());
    if (rgb == nullptr || frame.data_size() < image_width * image_height * 3) {
        throw std::runtime_error("RGB888 frame has an invalid data buffer");
    }

    std::vector<maix::image::Blob> halo_blobs;
    std::vector<maix::image::Blob> core_blobs;
    if (config_.enable_legacy_lab_candidates) {
        const std::vector<std::vector<int>> halo_thresholds{
            config_.halo_lab.as_vector(),
        };
        const std::vector<std::vector<int>> core_thresholds{
            config_.core_lab.as_vector(),
        };
        halo_blobs = frame.find_blobs(halo_thresholds, false, {}, 1, 1, 1, 1,
                                      config_.merge_blobs,
                                      config_.merge_margin_px);
        core_blobs = frame.find_blobs(core_thresholds, false, {}, 1, 1, 1, 1,
                                      config_.merge_blobs,
                                      config_.merge_margin_px);
    }

    std::vector<CoreBlob> cores;
    cores.reserve(core_blobs.size());
    for (auto &blob : core_blobs) {
        if (blob.w() > 0 && blob.h() > 0) {
            cores.push_back({blob.cxf(), blob.cyf(), blob.x(), blob.y(),
                             blob.w(), blob.h(), blob.density(),
                             blob.roundness()});
        }
    }

    std::vector<Candidate> candidates;
    candidates.reserve(halo_blobs.size() + cores.size());
    const float frame_area = static_cast<float>(image_width) * image_height;
    const bool acquiring = !tracker_.tracking_confirmed();
    const float initial_size_weight =
        acquiring ? config_.weight_initial_size : 0.0F;
    const float center_prior_weight =
        acquiring ? config_.weight_center_prior : 0.0F;
    const float temporal_weight =
        tracker_.has_prediction() ? config_.weight_temporal : 0.0F;
    const float total_weight = config_.weight_color + config_.weight_contrast +
                               config_.weight_density + config_.weight_shape +
                               config_.weight_core + temporal_weight +
                               center_prior_weight + initial_size_weight;

    for (auto &halo : halo_blobs) {
        const int x = halo.x();
        const int y = halo.y();
        const int width = halo.w();
        const int height = halo.h();
        if (width <= 0 || height <= 0) {
            continue;
        }

        const float density = clamp01(halo.density());
        if (density < config_.min_density) {
            continue;
        }

        const Rect inner = clip_rect(x, y, width, height, image_width, image_height);
        const Rect outer = clip_rect(x - config_.ring_margin_px,
                                     y - config_.ring_margin_px,
                                     width + 2 * config_.ring_margin_px,
                                     height + 2 * config_.ring_margin_px,
                                     image_width,
                                     image_height);
        const RgbStatistics inside =
            sample_rect(rgb, image_width, inner, config_.sample_grid);
        const RgbStatistics ring =
            sample_rect(rgb, image_width, outer, config_.sample_grid, &inner);
        const float green_dominance = inside.green_dominance();
        const float local_contrast = ring.samples > 0
                                         ? inside.brightness() - ring.brightness()
                                         : 0.0F;
        if (green_dominance < config_.min_green_dominance ||
            local_contrast < config_.min_tracking_local_contrast) {
            continue;
        }

        const float halo_center_x = halo.cxf();
        const float halo_center_y = halo.cyf();
        const float geometric_center_x = x + 0.5F * width;
        const float geometric_center_y = y + 0.5F * height;
        const float apparent_size =
            std::sqrt(std::max(1.0F, static_cast<float>(width) * height));
        if (apparent_size < config_.min_halo_size_px) {
            continue;
        }

        const CoreBlob *matched_core = nullptr;
        float matched_core_score = 0.0F;
        for (const auto &core : cores) {
            const int overlap = intersection_area(x, y, width, height,
                                                  core.x, core.y, core.w, core.h);
            if (overlap <= 0) {
                continue;
            }
            const float dx = core.center_x - halo_center_x;
            const float dy = core.center_y - halo_center_y;
            const float center_offset = std::sqrt(dx * dx + dy * dy) /
                                        std::max(1.0F, apparent_size);
            const float center_score = clamp01(
                1.0F - center_offset /
                           std::max(0.01F, config_.core_center_max_fraction));
            const float overlap_score = clamp01(
                static_cast<float>(overlap) /
                std::max(1.0F, static_cast<float>(core.w) * core.h));
            const float core_score = 0.75F * center_score + 0.25F * overlap_score;
            if (core_score > matched_core_score) {
                matched_core_score = core_score;
                matched_core = &core;
            }
        }

        const float area_fraction =
            static_cast<float>(width) * height / std::max(1.0F, frame_area);
        const float large_blob_factor =
            clamp01((std::sqrt(area_fraction) - 0.03F) / 0.32F);
        float center_x = halo_center_x;
        float center_y = halo_center_y;
        if (matched_core != nullptr) {
            const float core_weight = 0.65F * large_blob_factor;
            const float geometry_weight = 0.20F * large_blob_factor;
            const float halo_weight = 1.0F - core_weight - geometry_weight;
            center_x = halo_weight * halo_center_x +
                       core_weight * matched_core->center_x +
                       geometry_weight * geometric_center_x;
            center_y = halo_weight * halo_center_y +
                       core_weight * matched_core->center_y +
                       geometry_weight * geometric_center_y;
        } else {
            const float geometry_weight = 0.40F * large_blob_factor;
            center_x = (1.0F - geometry_weight) * halo_center_x +
                       geometry_weight * geometric_center_x;
            center_y = (1.0F - geometry_weight) * halo_center_y +
                       geometry_weight * geometric_center_y;
        }

        const float aspect_score = static_cast<float>(std::min(width, height)) /
                                   std::max(width, height);
        float roundness = halo.roundness();
        if (!std::isfinite(roundness)) {
            roundness = 0.0F;
        }
        const float shape_score = width * height <= 4
                                      ? 0.8F
                                      : clamp01(0.5F * aspect_score +
                                                0.5F * clamp01(roundness));
        const float color_score = clamp01((green_dominance + 0.02F) / 0.48F);
        const float contrast_score = clamp01((local_contrast + 0.05F) / 0.40F);
        const float density_score = clamp01(
            (density - config_.min_density) /
            std::max(0.01F, 1.0F - config_.min_density));
        const float center_prior_dx =
            center_x - config_.camera_model.principal_x;
        const float center_prior_dy =
            center_y - config_.camera_model.principal_y;
        const float center_prior_distance_squared =
            center_prior_dx * center_prior_dx + center_prior_dy * center_prior_dy;
        const float center_prior_radius =
            std::max(1.0F, config_.center_prior_radius_px);
        const float center_prior_score = std::exp(
            -0.5F * center_prior_distance_squared /
            (center_prior_radius * center_prior_radius));
        const float initial_size_score = clamp01(
            std::log1p(apparent_size) /
            std::log1p(std::max(1.0F, config_.initial_size_reference_px)));

        detail::CandidateObservation observation;
        observation.center_x = center_x;
        observation.center_y = center_y;
        observation.bbox_x = x;
        observation.bbox_y = y;
        observation.bbox_w = width;
        observation.bbox_h = height;
        observation.apparent_size = apparent_size;
        observation.source = detail::CandidateSource::Halo;

        const float temporal_score = tracker_.association_score(observation);
        observation.association_score = temporal_score;
        if (tracker_.tracking_confirmed() && tracker_.missed_frames() == 0 &&
            !tracker_.passes_association_gate(observation)) {
            continue;
        }
        // Bright structures immediately behind a green lamp can make the
        // lamp's bounding box darker than its surrounding ring even though
        // its color and motion remain unambiguous. Keep the strict contrast
        // gate while acquiring. Once tracking is confirmed, relax it only for
        // a candidate that strongly agrees with the predicted position and
        // size; local contrast remains a soft score below.
        const bool contrast_relaxation_allowed =
            tracker_.tracking_confirmed() &&
            temporal_score >= config_.contrast_relax_min_association;
        if (local_contrast < config_.min_local_contrast &&
            !contrast_relaxation_allowed) {
            continue;
        }

        observation.score = clamp01(
            (config_.weight_color * color_score +
             config_.weight_contrast * contrast_score +
             config_.weight_density * density_score +
             config_.weight_shape * shape_score +
             config_.weight_core * matched_core_score +
             temporal_weight * temporal_score +
             center_prior_weight * center_prior_score +
             initial_size_weight * initial_size_score) /
            std::max(1.0e-6F, total_weight));

        Candidate candidate;
        candidate.observation = observation;
        candidate.debug.center_x = center_x;
        candidate.debug.center_y = center_y;
        candidate.debug.bbox_x = x;
        candidate.debug.bbox_y = y;
        candidate.debug.bbox_w = width;
        candidate.debug.bbox_h = height;
        candidate.debug.apparent_size = apparent_size;
        candidate.debug.density = density;
        candidate.debug.green_dominance = green_dominance;
        candidate.debug.green_fraction = inside.positive_green_fraction();
        candidate.debug.local_contrast = local_contrast;
        candidate.debug.shape_score = shape_score;
        candidate.debug.core_score = matched_core_score;
        candidate.debug.temporal_score = temporal_score;
        candidate.debug.center_prior_score = center_prior_score;
        candidate.debug.initial_size_score = initial_size_score;
        candidate.debug.score = observation.score;
        candidate.debug.inside_capture_cone = point_inside_capture_cone(
            center_x, center_y, image_width, image_height, config_);
        candidates.push_back(candidate);
    }

    if (config_.enable_normalized_multiscale) {
        Rect search_region{0, 0, image_width, image_height};
        if (config_.multiscale_capture_cone_only &&
            config_.enable_capture_cone &&
            config_.camera_model.principal_x >= 0.0F &&
            config_.camera_model.principal_x < image_width &&
            config_.camera_model.principal_y >= 0.0F &&
            config_.camera_model.principal_y < image_height) {
            constexpr float kPi = 3.14159265358979323846F;
            const float cone_tangent = std::tan(
                config_.capture_cone_deg * kPi / 180.0F);
            const int radius_x = std::max(
                1, static_cast<int>(std::ceil(
                       cone_tangent * config_.camera_model.fx)));
            const int radius_y = std::max(
                1, static_cast<int>(std::ceil(
                       cone_tangent * config_.camera_model.fy)));
            search_region = clip_rect(
                static_cast<int>(std::floor(
                    config_.camera_model.principal_x)) - radius_x,
                static_cast<int>(std::floor(
                    config_.camera_model.principal_y)) - radius_y,
                2 * radius_x + 1, 2 * radius_y + 1,
                image_width, image_height);
        }

        const bool refresh_full_cone =
            classical_detection_count_ == 0 ||
            classical_detection_count_ % static_cast<uint64_t>(
                config_.multiscale_full_refresh_interval) == 0;
        if (tracker_.tracking_confirmed() && tracker_.missed_frames() == 0 &&
            config_.multiscale_tracking_roi_radius_px > 0.0F &&
            !refresh_full_cone) {
            const int radius = static_cast<int>(std::ceil(std::max({
                config_.multiscale_tracking_roi_radius_px, config_.gate_max_px,
                config_.enable_lamp_appearance ? 1.5F * tracker_.predicted_size() : 0.0F})));
            const Rect tracking_region = clip_rect(
                static_cast<int>(std::floor(tracker_.predicted_x())) - radius,
                static_cast<int>(std::floor(tracker_.predicted_y())) - radius,
                2 * radius + 1, 2 * radius + 1,
                image_width, image_height);
            search_region.x0 = std::max(search_region.x0, tracking_region.x0);
            search_region.y0 = std::max(search_region.y0, tracking_region.y0);
            search_region.x1 = std::min(search_region.x1, tracking_region.x1);
            search_region.y1 = std::min(search_region.y1, tracking_region.y1);
        }

        std::vector<ScalePeak> peaks;
        peaks.reserve(64);
        if (config_.enable_sparse_component_search) {
            peaks = collect_sparse_multiscale_peaks(
                rgb, image_width, image_height, search_region, config_,
                tracker_.tracking_confirmed());
        } else {
        const int source_search_width =
            std::max(0, search_region.x1 - search_region.x0);
        const int source_search_height =
            std::max(0, search_region.y1 - search_region.y0);
        const int sampling_stride = std::max(1, config_.multiscale_downsample);
        const int search_width =
            (source_search_width + sampling_stride - 1) / sampling_stride;
        const int search_height =
            (source_search_height + sampling_stride - 1) / sampling_stride;
        IntegralPlane<uint32_t> green_integral(search_width, search_height);
        IntegralPlane<uint32_t> brightness_integral(search_width, search_height);
        IntegralPlane<uint64_t> brightness_squared_integral(
            search_width, search_height);
        for (int y = 0; y < search_height; ++y) {
            uint32_t green_row_sum = 0;
            uint32_t brightness_row_sum = 0;
            uint64_t brightness_squared_row_sum = 0;
            const int image_y = std::min(
                search_region.y1 - 1,
                search_region.y0 + y * sampling_stride + sampling_stride / 2);
            for (int x = 0; x < search_width; ++x) {
                const int image_x = std::min(
                    search_region.x1 - 1,
                    search_region.x0 + x * sampling_stride +
                        sampling_stride / 2);
                const std::size_t offset =
                    (static_cast<std::size_t>(image_y) * image_width +
                     image_x) * 3U;
                const uint8_t red = rgb[offset];
                const uint8_t green = rgb[offset + 1];
                const uint8_t blue = rgb[offset + 2];
                green_row_sum += fast_normalized_green_response(
                    red, green, blue);
                const uint32_t brightness =
                    (77U * red + 150U * green + 29U * blue) >> 8U;
                brightness_row_sum += brightness;
                brightness_squared_row_sum +=
                    static_cast<uint64_t>(brightness) * brightness;
                green_integral.at(x + 1, y + 1) =
                    green_integral.at(x + 1, y) + green_row_sum;
                brightness_integral.at(x + 1, y + 1) =
                    brightness_integral.at(x + 1, y) + brightness_row_sum;
                brightness_squared_integral.at(x + 1, y + 1) =
                    brightness_squared_integral.at(x + 1, y) +
                    brightness_squared_row_sum;
            }
        }

        for (const int configured_diameter : config_.multiscale_diameters_px) {
            const int diameter = std::max(
                2, (configured_diameter + sampling_stride - 1) /
                       sampling_stride);
            const int radius = std::max(1, diameter / 2);
            const int ring_margin = std::max(
                1, (2 + sampling_stride - 1) / sampling_stride);
            const int outer_radius = std::max(radius + ring_margin, diameter);
            const int inner_side = 2 * radius + 1;
            const int outer_side = 2 * outer_radius + 1;
            const int inner_area = inner_side * inner_side;
            const int outer_area = outer_side * outer_side;
            const int ring_area = std::max(1, outer_area - inner_area);
            const float inner_normalizer =
                1.0F / (255.0F * static_cast<float>(inner_area));
            const float ring_normalizer =
                1.0F / (255.0F * static_cast<float>(ring_area));
            const float ring_squared_normalizer =
                1.0F / (255.0F * 255.0F *
                        static_cast<float>(ring_area));
            const int minimum_step =
                tracker_.tracking_confirmed() && tracker_.missed_frames() == 0
                    ? config_.multiscale_tracking_min_scan_step_px
                    : config_.multiscale_min_scan_step_px;
            const int sampled_minimum_step = std::max(
                1, (minimum_step + sampling_stride - 1) / sampling_stride);
            const int step = std::max(sampled_minimum_step, diameter / 3);
            std::vector<ScalePeak> scale_peaks;
            scale_peaks.reserve(16);
            for (int y = outer_radius; y < search_height - outer_radius;
                 y += step) {
                for (int x = outer_radius; x < search_width - outer_radius;
                     x += step) {
                    // The loop bounds already keep both windows inside the
                    // sampled image. Avoid clipping and its min/max branches
                    // in this hottest loop.
                    const Rect inner{x - radius, y - radius,
                                     x + radius + 1, y + radius + 1};
                    const Rect outer{x - outer_radius, y - outer_radius,
                                     x + outer_radius + 1,
                                     y + outer_radius + 1};
                    const float inner_green =
                        static_cast<float>(green_integral.sum(inner)) *
                        inner_normalizer;
                    const float inner_brightness =
                        static_cast<float>(brightness_integral.sum(inner)) *
                        inner_normalizer;
                    const float ring_green =
                        static_cast<float>(green_integral.sum(outer) -
                                           green_integral.sum(inner)) *
                        ring_normalizer;
                    const float ring_brightness =
                        static_cast<float>(brightness_integral.sum(outer) -
                                           brightness_integral.sum(inner)) *
                        ring_normalizer;
                    const float brightness_contrast =
                        inner_brightness - ring_brightness;
                    const float response = inner_green - ring_green +
                        config_.normalized_brightness_weight *
                            std::max(-0.10F, brightness_contrast);
                    if (response < config_.min_normalized_green_response ||
                        inner_green < config_.min_normalized_green_response ||
                        inner_brightness <
                            config_.min_normalized_inner_brightness) {
                        continue;
                    }

                    // Variance and sqrt are needed only for the small number
                    // of positions that pass the cheap green/brightness gate.
                    const float ring_brightness_squared =
                        static_cast<float>(
                            brightness_squared_integral.sum(outer) -
                            brightness_squared_integral.sum(inner)) *
                        ring_squared_normalizer;
                    const float ring_variance = std::max(
                        0.0F, ring_brightness_squared -
                                  ring_brightness * ring_brightness);
                    const float contrast_z = brightness_contrast /
                        std::sqrt(ring_variance + 0.0025F);
                    const float minimum_contrast_z = tracker_.tracking_confirmed()
                        ? config_.min_tracking_normalized_contrast_z
                        : config_.min_normalized_contrast_z;
                    if (contrast_z < minimum_contrast_z) {
                        continue;
                    }
                    const int peak_x = std::min(
                        search_region.x1 - 1,
                        search_region.x0 + x * sampling_stride +
                            sampling_stride / 2);
                    const int peak_y = std::min(
                        search_region.y1 - 1,
                        search_region.y0 + y * sampling_stride +
                            sampling_stride / 2);
                    const int native_radius = std::max(1, configured_diameter / 2);
                    const Rect native_inner = clip_rect(
                        peak_x - native_radius, peak_y - native_radius,
                        2 * native_radius + 1, 2 * native_radius + 1,
                        image_width, image_height);
                    if (!has_bright_green_evidence(rgb, image_width, native_inner, config_)) {
                        continue;
                    }
                    ScalePeak peak{peak_x, peak_y,
                                   configured_diameter, response, inner_green,
                                   inner_brightness, brightness_contrast,
                                   contrast_z};
                    scale_peaks.push_back(peak);
                }
            }
            std::sort(scale_peaks.begin(), scale_peaks.end(),
                      [](const ScalePeak &left, const ScalePeak &right) {
                          return left.response > right.response;
                      });
            if (scale_peaks.size() > 12U) {
                scale_peaks.resize(12U);
            }
            peaks.insert(peaks.end(), scale_peaks.begin(), scale_peaks.end());
        }
        }

        std::sort(peaks.begin(), peaks.end(),
                  [](const ScalePeak &left, const ScalePeak &right) {
                      return left.response > right.response;
                  });
        std::vector<ScalePeak> accepted_peaks;
        accepted_peaks.reserve(config_.max_green_candidates);
        for (const auto &peak : peaks) {
            bool overlaps_existing = false;
            for (const auto &candidate : candidates) {
                const float dx = peak.x - candidate.observation.center_x;
                const float dy = peak.y - candidate.observation.center_y;
                const float nms_radius = std::max(
                    2.0F, 0.65F * std::max(
                        static_cast<float>(peak.diameter),
                        candidate.observation.apparent_size));
                if (dx * dx + dy * dy <= nms_radius * nms_radius) {
                    overlaps_existing = true;
                    break;
                }
            }
            for (const auto &accepted : accepted_peaks) {
                const float dx = static_cast<float>(peak.x - accepted.x);
                const float dy = static_cast<float>(peak.y - accepted.y);
                const float nms_radius =
                    0.65F * std::max(peak.diameter, accepted.diameter);
                if (dx * dx + dy * dy <= nms_radius * nms_radius) {
                    overlaps_existing = true;
                    break;
                }
            }
            if (overlaps_existing) {
                continue;
            }

            LampAppearance appearance;
            if (config_.enable_lamp_appearance) {
                appearance = lamp_appearance(rgb, image_width, image_height,
                    peak.x, peak.y, peak.diameter, config_);
                if (!appearance.valid) continue;
            }

            const int radius = std::max(1, peak.diameter / 2);
            const Rect inner = clip_rect(peak.x - radius, peak.y - radius,
                                         2 * radius + 1, 2 * radius + 1,
                                         image_width, image_height);
            float weighted_x = 0.0F;
            float weighted_y = 0.0F;
            float weight_sum = 0.0F;
            if (!config_.enable_lamp_appearance) {
            for (int y = inner.y0; y < inner.y1; ++y) {
                for (int x = inner.x0; x < inner.x1; ++x) {
                    const std::size_t offset =
                        (static_cast<std::size_t>(y) * image_width + x) * 3U;
                    const float red = rgb[offset];
                    const float green = rgb[offset + 1];
                    const float blue = rgb[offset + 2];
                    const float response =
                        normalized_green_response(red, green, blue);
                    weighted_x += response * x;
                    weighted_y += response * y;
                    weight_sum += response;
                }
            }
            }
            const float center_x = config_.enable_lamp_appearance ? appearance.center_x : weight_sum > 1.0e-5F
                                       ? weighted_x / weight_sum
                                       : static_cast<float>(peak.x);
            const float center_y = config_.enable_lamp_appearance ? appearance.center_y : weight_sum > 1.0e-5F
                                       ? weighted_y / weight_sum
                                       : static_cast<float>(peak.y);
            detail::CandidateObservation observation;
            observation.center_x = center_x;
            observation.center_y = center_y;
            observation.bbox_x = config_.enable_lamp_appearance ? appearance.x : inner.x0;
            observation.bbox_y = config_.enable_lamp_appearance ? appearance.y : inner.y0;
            observation.bbox_w = config_.enable_lamp_appearance ? appearance.width : inner.x1 - inner.x0;
            observation.bbox_h = config_.enable_lamp_appearance ? appearance.height : inner.y1 - inner.y0;
            observation.apparent_size = config_.enable_lamp_appearance
                ? std::sqrt(static_cast<float>(appearance.width * appearance.height))
                : static_cast<float>(peak.diameter);
            observation.source = detail::CandidateSource::NormalizedResponse;
            observation.association_score = tracker_.association_score(observation);
            const float response_score = clamp01(
                (peak.response - config_.min_normalized_green_response) /
                std::max(0.02F, 0.30F - config_.min_normalized_green_response));
            observation.score = config_.enable_lamp_appearance
                ? clamp01(0.30F * response_score + 0.55F * appearance.score +
                          0.15F * observation.association_score)
                : clamp01(0.55F * response_score + 0.20F * clamp01(peak.green_mean / 0.45F) +
                          0.25F * observation.association_score);

            Candidate candidate;
            candidate.observation = observation;
            candidate.debug.center_x = center_x;
            candidate.debug.center_y = center_y;
            candidate.debug.bbox_x = observation.bbox_x;
            candidate.debug.bbox_y = observation.bbox_y;
            candidate.debug.bbox_w = observation.bbox_w;
            candidate.debug.bbox_h = observation.bbox_h;
            candidate.debug.apparent_size = observation.apparent_size;
            candidate.debug.green_dominance = peak.green_mean;
            candidate.debug.green_fraction = config_.enable_lamp_appearance ? appearance.color_fraction : 1.0F;
            candidate.debug.local_contrast = peak.brightness_contrast;
            candidate.debug.shape_score = config_.enable_lamp_appearance ? appearance.shape : 1.0F;
            candidate.debug.appearance_score = appearance.score;
            candidate.debug.radial_support = appearance.radial_support;
            candidate.debug.axis_ratio = appearance.axis_ratio;
            candidate.debug.temporal_score = observation.association_score;
            candidate.debug.score = observation.score;
            candidate.debug.normalized_response = peak.response;
            candidate.debug.inside_capture_cone = point_inside_capture_cone(
                center_x, center_y, image_width, image_height, config_);
            candidates.push_back(candidate);
            accepted_peaks.push_back(peak);
            if (accepted_peaks.size() >=
                static_cast<std::size_t>(config_.max_green_candidates)) {
                break;
            }
        }
    }

    if (!config_.enable_saturated_core_candidates) {
        std::sort(candidates.begin(), candidates.end(),
                  [](const Candidate &left, const Candidate &right) {
                      return left.observation.score > right.observation.score;
                  });
        if (candidates.size() >
            static_cast<std::size_t>(config_.max_green_candidates)) {
            candidates.resize(config_.max_green_candidates);
        }
        return candidates;
    }
    const float core_size_weight = config_.weight_core_size;
    const float core_total_weight =
        total_weight - initial_size_weight + core_size_weight;

    // When the lamp approaches the camera, its center clips to white and the
    // surrounding green halo can become connected to the green structure
    // behind it. A halo-only detector then follows tiny edge fragments. Treat
    // compact saturated cores as a second candidate source, but only when the
    // immediately surrounding ring is green. This rejects ordinary white
    // lights while preserving an accurate lamp center and apparent size.
    for (const auto &core : cores) {
        const int width = core.w;
        const int height = core.h;
        if (width <= 0 || height <= 0) {
            continue;
        }

        const float density = clamp01(core.density);
        if (density < config_.min_density) {
            continue;
        }

        const int adaptive_margin = std::max(
            config_.ring_margin_px,
            static_cast<int>(std::lround(
                config_.core_ring_scale * std::max(width, height))));
        const Rect inner = clip_rect(core.x, core.y, width, height,
                                     image_width, image_height);
        const Rect outer = clip_rect(core.x - adaptive_margin,
                                     core.y - adaptive_margin,
                                     width + 2 * adaptive_margin,
                                     height + 2 * adaptive_margin,
                                     image_width,
                                     image_height);
        const RgbStatistics inside =
            sample_rect(rgb, image_width, inner, config_.sample_grid);
        const RgbStatistics ring =
            sample_rect(rgb, image_width, outer, config_.sample_grid, &inner);
        const float core_brightness = inside.brightness();
        const float ring_green_dominance = ring.green_dominance();
        const float ring_green_fraction = ring.positive_green_fraction();
        const float local_contrast = ring.samples > 0
                                         ? core_brightness - ring.brightness()
                                         : 0.0F;
        if (core_brightness < config_.min_core_brightness ||
            ring_green_dominance <
                config_.min_core_ring_green_dominance ||
            ring_green_fraction < config_.min_core_ring_green_fraction) {
            continue;
        }

        const float geometric_center_x = core.x + 0.5F * width;
        const float geometric_center_y = core.y + 0.5F * height;
        const float center_x = 0.75F * core.center_x +
                               0.25F * geometric_center_x;
        const float center_y = 0.75F * core.center_y +
                               0.25F * geometric_center_y;
        const float apparent_size =
            std::sqrt(std::max(1.0F, static_cast<float>(width) * height));
        if (apparent_size < config_.min_core_size_px) {
            continue;
        }
        const float aspect_score = static_cast<float>(std::min(width, height)) /
                                   std::max(width, height);
        const float roundness = std::isfinite(core.roundness)
                                    ? clamp01(core.roundness)
                                    : 0.0F;
        const float shape_score = width * height <= 4
                                      ? 0.8F
                                      : clamp01(0.5F * aspect_score +
                                                0.5F * roundness);
        const float color_score = clamp01(
            (ring_green_dominance + 0.02F) / 0.48F);
        const float contrast_score = clamp01(
            (local_contrast + 0.05F) / 0.40F);
        const float density_score = clamp01(
            (density - config_.min_density) /
            std::max(0.01F, 1.0F - config_.min_density));
        const float core_score = clamp01(
            (core_brightness - config_.min_core_brightness) /
            std::max(0.01F, 1.0F - config_.min_core_brightness));
        const float center_prior_dx =
            center_x - config_.camera_model.principal_x;
        const float center_prior_dy =
            center_y - config_.camera_model.principal_y;
        const float center_prior_distance_squared =
            center_prior_dx * center_prior_dx + center_prior_dy * center_prior_dy;
        const float center_prior_radius =
            std::max(1.0F, config_.center_prior_radius_px);
        const float center_prior_score = std::exp(
            -0.5F * center_prior_distance_squared /
            (center_prior_radius * center_prior_radius));
        const float initial_size_score = clamp01(
            std::log1p(apparent_size) /
            std::log1p(std::max(1.0F, config_.initial_size_reference_px)));

        detail::CandidateObservation observation;
        observation.center_x = center_x;
        observation.center_y = center_y;
        observation.bbox_x = core.x;
        observation.bbox_y = core.y;
        observation.bbox_w = width;
        observation.bbox_h = height;
        observation.apparent_size = apparent_size;
        observation.source = detail::CandidateSource::SaturatedCore;

        const float temporal_score = tracker_.association_score(observation);
        observation.association_score = temporal_score;
        if (tracker_.tracking_confirmed() && tracker_.missed_frames() == 0 &&
            !tracker_.passes_association_gate(observation)) {
            continue;
        }

        observation.score = clamp01(
            (config_.weight_color * color_score +
             config_.weight_contrast * contrast_score +
             config_.weight_density * density_score +
             config_.weight_shape * shape_score +
             config_.weight_core * core_score +
             temporal_weight * temporal_score +
             center_prior_weight * center_prior_score +
             core_size_weight * initial_size_score) /
            std::max(1.0e-6F, core_total_weight));

        Candidate candidate;
        candidate.observation = observation;
        candidate.debug.center_x = center_x;
        candidate.debug.center_y = center_y;
        candidate.debug.bbox_x = core.x;
        candidate.debug.bbox_y = core.y;
        candidate.debug.bbox_w = width;
        candidate.debug.bbox_h = height;
        candidate.debug.apparent_size = apparent_size;
        candidate.debug.density = density;
        candidate.debug.green_dominance = ring_green_dominance;
        candidate.debug.green_fraction = ring_green_fraction;
        candidate.debug.local_contrast = local_contrast;
        candidate.debug.shape_score = shape_score;
        candidate.debug.core_score = core_score;
        candidate.debug.temporal_score = temporal_score;
        candidate.debug.center_prior_score = center_prior_score;
        candidate.debug.initial_size_score = initial_size_score;
        candidate.debug.score = observation.score;
        candidate.debug.saturated_core = true;
        candidate.debug.inside_capture_cone = point_inside_capture_cone(
            center_x, center_y, image_width, image_height, config_);
        candidates.push_back(candidate);
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate &left, const Candidate &right) {
                  return left.observation.score > right.observation.score;
              });
    if (candidates.size() >
        static_cast<std::size_t>(config_.max_green_candidates)) {
        candidates.resize(config_.max_green_candidates);
    }

    return candidates;
}

void GreenLightDetector::update_candidate_hypotheses(
    std::vector<Candidate> &candidates,
    uint64_t timestamp_us,
    const MotionPrior *motion_prior)
{
    if (motion_prior != nullptr && motion_prior->image_transform_valid &&
        motion_prior->confidence > 0.0F) {
        const float cx = config_.camera_model.principal_x;
        const float cy = config_.camera_model.principal_y;
        const float cosine = std::cos(motion_prior->image_rotation_rad);
        const float sine = std::sin(motion_prior->image_rotation_rad);
        const float scale = std::max(0.5F, std::min(2.0F,
                                                   motion_prior->image_scale));
        for (auto &hypothesis : candidate_hypotheses_) {
            if (!hypothesis.active) {
                continue;
            }
            const float x = hypothesis.x - cx;
            const float y = hypothesis.y - cy;
            hypothesis.x = scale * (cosine * x - sine * y) + cx +
                           motion_prior->image_dx_px;
            hypothesis.y = scale * (sine * x + cosine * y) + cy +
                           motion_prior->image_dy_px;
            hypothesis.apparent_size *= scale;
        }
    }

    std::vector<int> candidate_assignment(candidates.size(), -1);
    for (std::size_t hypothesis_index = 0;
         hypothesis_index < candidate_hypotheses_.size(); ++hypothesis_index) {
        auto &hypothesis = candidate_hypotheses_[hypothesis_index];
        if (!hypothesis.active) {
            continue;
        }
        const float dt = hypothesis.timestamp_us > 0 &&
                                 timestamp_us > hypothesis.timestamp_us
                             ? std::min(0.1F, static_cast<float>(
                                   timestamp_us - hypothesis.timestamp_us) *
                                   1.0e-6F)
                             : 0.0F;
        const float predicted_x = hypothesis.x + hypothesis.velocity_x * dt;
        const float predicted_y = hypothesis.y + hypothesis.velocity_y * dt;
        const float gate = std::max(
            config_.confirmation_gate_px,
            config_.gate_size_factor * hypothesis.apparent_size);
        int best_candidate = -1;
        float best_cost = std::numeric_limits<float>::max();
        for (std::size_t candidate_index = 0;
             candidate_index < candidates.size(); ++candidate_index) {
            if (candidate_assignment[candidate_index] >= 0) {
                continue;
            }
            const auto &observation = candidates[candidate_index].observation;
            const float distance = std::hypot(observation.center_x - predicted_x,
                                              observation.center_y - predicted_y);
            const float size_delta = std::fabs(std::log(
                std::max(1.0F, observation.apparent_size) /
                std::max(1.0F, hypothesis.apparent_size)));
            const float appearance = std::max(
                candidates[candidate_index].debug.normalized_response,
                candidates[candidate_index].debug.green_dominance);
            const float appearance_delta =
                std::fabs(appearance - hypothesis.appearance);
            if (distance > gate ||
                size_delta > config_.max_cross_source_log_size_jump) {
                continue;
            }
            const float cost = distance / gate + 0.25F * size_delta +
                               0.20F * appearance_delta;
            if (cost < best_cost) {
                best_cost = cost;
                best_candidate = static_cast<int>(candidate_index);
            }
        }
        if (best_candidate < 0) {
            ++hypothesis.misses;
            hypothesis.timestamp_us = timestamp_us;
            hypothesis.x = predicted_x;
            hypothesis.y = predicted_y;
            hypothesis.confidence *= 0.75F;
            if (hypothesis.misses > 5) {
                hypothesis = CandidateHypothesis{};
            }
            continue;
        }

        auto &candidate = candidates[best_candidate];
        const float inverse_dt = dt > 1.0e-4F ? 1.0F / dt : 0.0F;
        const float measured_velocity_x =
            (candidate.observation.center_x - hypothesis.x) * inverse_dt;
        const float measured_velocity_y =
            (candidate.observation.center_y - hypothesis.y) * inverse_dt;
        hypothesis.velocity_x = 0.65F * hypothesis.velocity_x +
                                0.35F * measured_velocity_x;
        hypothesis.velocity_y = 0.65F * hypothesis.velocity_y +
                                0.35F * measured_velocity_y;
        hypothesis.x = candidate.observation.center_x;
        hypothesis.y = candidate.observation.center_y;
        hypothesis.apparent_size = 0.70F * hypothesis.apparent_size +
                                   0.30F * candidate.observation.apparent_size;
        const float appearance = std::max(candidate.debug.normalized_response,
                                          candidate.debug.green_dominance);
        hypothesis.appearance = 0.70F * hypothesis.appearance +
                                0.30F * appearance;
        hypothesis.timestamp_us = timestamp_us;
        hypothesis.misses = 0;
        ++hypothesis.hits;
        hypothesis.confidence = clamp01(
            0.65F * hypothesis.confidence +
            0.35F * candidate.observation.score);
        const float track_consistency = clamp01(
            (1.0F - std::min(1.0F, best_cost)) *
            std::min(1.0F, hypothesis.hits / 3.0F));
        candidate.observation.association_score = std::max(
            candidate.observation.association_score, track_consistency);
        candidate.observation.score = clamp01(
            0.85F * candidate.observation.score +
            0.15F * track_consistency);
        candidate.debug.temporal_score =
            candidate.observation.association_score;
        candidate.debug.score = candidate.observation.score;
        candidate_assignment[best_candidate] =
            static_cast<int>(hypothesis_index);
    }

    for (std::size_t candidate_index = 0;
         candidate_index < candidates.size(); ++candidate_index) {
        if (candidate_assignment[candidate_index] >= 0) {
            continue;
        }
        auto free_hypothesis = std::find_if(
            candidate_hypotheses_.begin(), candidate_hypotheses_.end(),
            [](const CandidateHypothesis &hypothesis) {
                return !hypothesis.active;
            });
        if (free_hypothesis == candidate_hypotheses_.end()) {
            break;
        }
        const auto &candidate = candidates[candidate_index];
        free_hypothesis->active = true;
        free_hypothesis->x = candidate.observation.center_x;
        free_hypothesis->y = candidate.observation.center_y;
        free_hypothesis->apparent_size = candidate.observation.apparent_size;
        free_hypothesis->appearance = std::max(
            candidate.debug.normalized_response,
            candidate.debug.green_dominance);
        free_hypothesis->confidence = candidate.observation.score;
        free_hypothesis->timestamp_us = timestamp_us;
        free_hypothesis->hits = 1;
    }
}

GreenLightDetection GreenLightDetector::process_green(
    maix::image::Image &frame,
    uint64_t timestamp_us,
    const MotionPrior *motion_prior)
{
    if (frame.format() != maix::image::Format::FMT_RGB888) {
        throw std::invalid_argument("GreenLightDetector requires an RGB888 frame");
    }

    tracker_.predict(timestamp_us);
    if (motion_prior != nullptr) {
        tracker_.apply_motion_prior(*motion_prior);
    }

    const uint64_t current_frame = frame_counter_++;
    const int effective_classical_interval = npu_config_.enabled
        ? std::max(2, config_.classical_interval_frames)
        : config_.classical_interval_frames;
    const bool run_classical_detection =
        current_frame % static_cast<uint64_t>(
            effective_classical_interval) == 0;
    last_classical_detection_ran_ = run_classical_detection;
    last_classical_detection_ms_ = 0.0F;
    if (!run_classical_detection) {
        // This is an intentional scheduler coast, not an observation failure.
        // Preserve candidates for the NPU slot and do not consume the miss
        // budget used by the REACQUIRE state machine.
        return tracker_.update(nullptr, timestamp_us, config_.camera_model,
                               false);
    }

    const auto detection_started = std::chrono::steady_clock::now();
    ++classical_detection_count_;
    std::vector<Candidate> candidates;
    if (region_active_) {
        auto local_config = config_;
        local_config.camera_model.principal_x -= region_.x;
        local_config.camera_model.principal_y -= region_.y;
        // Cone gating happens in source coordinates after collection.
        local_config.enable_capture_cone = false;
        local_config.integral_peak_statistics = true;
        GreenLightDetector local(local_config, armor_config_, target_geometry_);
        local.tracker_ = tracker_.roi_view(region_.x, region_.y);
        // The adapter is recreated per frame, but full-search cadence belongs
        // to the persistent detector. Leaving this at zero forced a full ROI
        // search on every frame and disabled the configured tracking window.
        local.classical_detection_count_ = classical_detection_count_;
        candidates = local.collect_candidates(frame);
        for (auto &c : candidates) {
            c.observation.center_x += region_.x; c.observation.center_y += region_.y;
            c.observation.bbox_x += region_.x; c.observation.bbox_y += region_.y;
            c.debug.center_x += region_.x; c.debug.center_y += region_.y;
            c.debug.bbox_x += region_.x; c.debug.bbox_y += region_.y;
            c.debug.inside_capture_cone = point_inside_capture_cone(
                c.observation.center_x, c.observation.center_y,
                source_width_, source_height_, config_);
        }
    } else candidates = collect_candidates(frame);
    update_candidate_hypotheses(candidates, timestamp_us, motion_prior);
    last_candidates_.clear();
    last_candidates_.reserve(candidates.size());
    for (const auto &candidate : candidates) {
        last_candidates_.push_back(candidate.debug);
    }

    const float minimum_score = tracker_.tracking_confirmed()
                                    ? config_.min_tracking_score
                                    : config_.min_candidate_score;
    int selected_index = -1;
    float best_score = minimum_score;
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        if (!candidates[index].debug.inside_capture_cone) {
            continue;
        }
        if (candidates[index].observation.score >= best_score) {
            best_score = candidates[index].observation.score;
            selected_index = static_cast<int>(index);
        }
    }

    GreenLightDetection result;
    if (selected_index < 0) {
        result = tracker_.update(nullptr, timestamp_us, config_.camera_model);
    } else {
        last_candidates_[selected_index].selected = true;
    // After any missed frame the next plausible full-cone candidate is allowed
    // to start a fresh confirmation sequence. This provides true REACQUIRE
    // behavior instead of keeping the previous narrow association gate until
    // max_missed_frames expires.
        if (tracker_.tracking_confirmed() && tracker_.missed_frames() > 0 &&
            !tracker_.passes_association_gate(
                candidates[selected_index].observation)) {
            tracker_.reset();
        }
        result = tracker_.update(&candidates[selected_index].observation,
                                 timestamp_us,
                                 config_.camera_model);
    }
    const auto detection_finished = std::chrono::steady_clock::now();
    last_classical_detection_ms_ = static_cast<float>(
        std::chrono::duration<double, std::milli>(
            detection_finished - detection_started).count());
    return result;
}

GreenLightDetection GreenLightDetector::process(maix::image::Image &frame,
                                                 uint64_t timestamp_us)
{
    return process_green(frame, timestamp_us, nullptr);
}

}  // namespace dart
