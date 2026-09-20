#include "dart/green_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dart {
namespace {

constexpr float kMinSize = 1.0F;
constexpr float kMinDt = 0.001F;
constexpr float kMaxDt = 0.100F;

float clamp01(float value)
{
    return std::max(0.0F, std::min(1.0F, value));
}

bool invert_3x3(const float input[3][3], float output[3][3])
{
    // The position states are normalized by focal length, so a perfectly
    // healthy innovation covariance can have a determinant far below 1e-9.
    // Scaled partial pivoting avoids using an absolute determinant threshold
    // and remains stable when position and log-size use different units.
    double augmented[3][6]{};
    double row_scale[3]{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            augmented[row][column] = input[row][column];
            row_scale[row] = std::max(row_scale[row],
                                      std::fabs(augmented[row][column]));
        }
        augmented[row][row + 3] = 1.0;
        if (row_scale[row] <= std::numeric_limits<double>::epsilon()) {
            return false;
        }
    }

    for (int pivot_column = 0; pivot_column < 3; ++pivot_column) {
        int pivot_row = pivot_column;
        double best_scaled_pivot = 0.0;
        for (int row = pivot_column; row < 3; ++row) {
            const double scaled =
                std::fabs(augmented[row][pivot_column]) / row_scale[row];
            if (scaled > best_scaled_pivot) {
                best_scaled_pivot = scaled;
                pivot_row = row;
            }
        }
        if (best_scaled_pivot <= 1.0e-12) {
            return false;
        }
        if (pivot_row != pivot_column) {
            for (int column = 0; column < 6; ++column) {
                std::swap(augmented[pivot_row][column],
                          augmented[pivot_column][column]);
            }
            std::swap(row_scale[pivot_row], row_scale[pivot_column]);
        }

        const double pivot = augmented[pivot_column][pivot_column];
        for (int column = 0; column < 6; ++column) {
            augmented[pivot_column][column] /= pivot;
        }
        for (int row = 0; row < 3; ++row) {
            if (row == pivot_column) {
                continue;
            }
            const double factor = augmented[row][pivot_column];
            for (int column = 0; column < 6; ++column) {
                augmented[row][column] -=
                    factor * augmented[pivot_column][column];
            }
        }
    }

    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            output[row][column] =
                static_cast<float>(augmented[row][column + 3]);
        }
    }
    return true;
}

}  // namespace

const char *track_state_name(TrackState state)
{
    switch (state) {
    case TrackState::Lost:
        return "LOST";
    case TrackState::Candidate:
        return "CANDIDATE";
    case TrackState::Tracking:
        return "TRACKING";
    }
    return "UNKNOWN";
}

const char *guidance_track_state_name(GuidanceTrackState state)
{
    switch (state) {
    case GuidanceTrackState::Search:
        return "SEARCH";
    case GuidanceTrackState::Acquiring:
        return "ACQUIRING";
    case GuidanceTrackState::Tracking:
        return "TRACKING";
    case GuidanceTrackState::Coasting:
        return "COASTING";
    case GuidanceTrackState::Reacquire:
        return "REACQUIRE";
    }
    return "UNKNOWN";
}

const char *guidance_mode_name(GuidanceMode mode)
{
    switch (mode) {
    case GuidanceMode::LampApproach:
        return "LAMP_APPROACH";
    case GuidanceMode::Fused:
        return "FUSED";
    case GuidanceMode::ArmorImpact:
        return "ARMOR_IMPACT";
    }
    return "UNKNOWN";
}

const char *armor_color_name(ArmorColor color)
{
    switch (color) {
    case ArmorColor::Red:
        return "red";
    case ArmorColor::Blue:
        return "blue";
    case ArmorColor::Unknown:
        return "unknown";
    }
    return "unknown";
}

MotionPrior interpolate_motion_prior(const MotionPrior &before,
                                     const MotionPrior &after,
                                     uint64_t timestamp_us)
{
    MotionPrior result;
    result.timestamp_us = timestamp_us;
    if (!before.valid || !after.valid ||
        after.timestamp_us <= before.timestamp_us ||
        timestamp_us < before.timestamp_us ||
        timestamp_us > after.timestamp_us) {
        return result;
    }
    const float alpha = static_cast<float>(timestamp_us - before.timestamp_us) /
                        static_cast<float>(after.timestamp_us -
                                           before.timestamp_us);
    result.valid = true;
    float quaternion_dot = 0.0F;
    for (std::size_t index = 0; index < 4; ++index) {
        quaternion_dot += before.orientation_wxyz[index] *
                          after.orientation_wxyz[index];
    }
    float quaternion_norm = 0.0F;
    for (std::size_t index = 0; index < 4; ++index) {
        const float after_value = quaternion_dot < 0.0F
                                      ? -after.orientation_wxyz[index]
                                      : after.orientation_wxyz[index];
        result.orientation_wxyz[index] =
            (1.0F - alpha) * before.orientation_wxyz[index] +
            alpha * after_value;
        quaternion_norm += result.orientation_wxyz[index] *
                           result.orientation_wxyz[index];
    }
    quaternion_norm = std::sqrt(std::max(1.0e-12F, quaternion_norm));
    for (float &value : result.orientation_wxyz) {
        value /= quaternion_norm;
    }
    for (std::size_t index = 0; index < 3; ++index) {
        result.angular_velocity_rad_s[index] =
            (1.0F - alpha) * before.angular_velocity_rad_s[index] +
            alpha * after.angular_velocity_rad_s[index];
    }
    result.image_transform_valid = before.image_transform_valid &&
                                   after.image_transform_valid;
    if (result.image_transform_valid) {
        result.image_dx_px = (1.0F - alpha) * before.image_dx_px +
                             alpha * after.image_dx_px;
        result.image_dy_px = (1.0F - alpha) * before.image_dy_px +
                             alpha * after.image_dy_px;
        result.image_rotation_rad =
            (1.0F - alpha) * before.image_rotation_rad +
            alpha * after.image_rotation_rad;
        result.image_scale = (1.0F - alpha) * before.image_scale +
                             alpha * after.image_scale;
    }
    result.confidence = std::min(before.confidence, after.confidence);
    return result;
}

std::vector<int> LabThreshold::as_vector() const
{
    return {l_min, l_max, a_min, a_max, b_min, b_max};
}

namespace detail {

std::array<float, 2> pixel_to_angles(float pixel_x,
                                     float pixel_y,
                                     const CameraModel &model)
{
    if (model.fx <= 0.0F || model.fy <= 0.0F) {
        return {0.0F, 0.0F};
    }

    const float distorted_x = (pixel_x - model.principal_x) / model.fx;
    const float distorted_y = (pixel_y - model.principal_y) / model.fy;
    float undistorted_x = distorted_x;
    float undistorted_y = distorted_y;

    for (int iteration = 0; iteration < 6; ++iteration) {
        const float x2 = undistorted_x * undistorted_x;
        const float y2 = undistorted_y * undistorted_y;
        const float xy = undistorted_x * undistorted_y;
        const float r2 = x2 + y2;
        const float radial = 1.0F + model.k1 * r2 + model.k2 * r2 * r2 +
                             model.k3 * r2 * r2 * r2;
        if (std::fabs(radial) < 1.0e-6F) {
            break;
        }

        const float tangential_x = 2.0F * model.p1 * xy + model.p2 * (r2 + 2.0F * x2);
        const float tangential_y = model.p1 * (r2 + 2.0F * y2) + 2.0F * model.p2 * xy;
        undistorted_x = (distorted_x - tangential_x) / radial;
        undistorted_y = (distorted_y - tangential_y) / radial;
    }

    return {std::atan(undistorted_x), -std::atan(undistorted_y)};
}

TemporalTracker::TemporalTracker(const DetectorConfig &config)
    : config_(config)
{
    reset();
}

void TemporalTracker::reset()
{
    initialized_ = false;
    last_timestamp_us_ = 0;
    last_measurement_timestamp_us_ = 0;
    state_.fill(0.0F);
    for (auto &row : covariance_) {
        row.fill(0.0F);
    }
    hit_history_.clear();
    missed_frames_ = 0;
    track_state_ = TrackState::Lost;
    last_candidate_ = CandidateObservation{};
}

void TemporalTracker::initialize(const CandidateObservation &candidate,
                                 uint64_t timestamp_us)
{
    initialized_ = true;
    last_timestamp_us_ = timestamp_us;
    last_measurement_timestamp_us_ = timestamp_us;
    const float fx = std::max(1.0F, config_.camera_model.fx);
    const float fy = std::max(1.0F, config_.camera_model.fy);
    state_ = {(candidate.center_x - config_.camera_model.principal_x) / fx,
              (candidate.center_y - config_.camera_model.principal_y) / fy,
              0.0F,
              0.0F,
              std::log(std::max(candidate.apparent_size, kMinSize)),
              0.0F};

    for (auto &row : covariance_) {
        row.fill(0.0F);
    }
    covariance_[0][0] = 25.0F / (fx * fx);
    covariance_[1][1] = 25.0F / (fy * fy);
    covariance_[2][2] = 400.0F / (fx * fx);
    covariance_[3][3] = 400.0F / (fy * fy);
    covariance_[4][4] = 0.25F;
    covariance_[5][5] = 1.0F;
}

TemporalTracker TemporalTracker::roi_view(int x, int y) const
{
    auto view = *this;
    view.config_.camera_model.principal_x -= x;
    view.config_.camera_model.principal_y -= y;
    view.last_candidate_.center_x -= x; view.last_candidate_.center_y -= y;
    view.last_candidate_.bbox_x -= x; view.last_candidate_.bbox_y -= y;
    return view;
}

void TemporalTracker::predict(uint64_t timestamp_us)
{
    if (!initialized_) {
        return;
    }
    if (timestamp_us <= last_timestamp_us_) {
        return;
    }

    float dt = static_cast<float>(timestamp_us - last_timestamp_us_) * 1.0e-6F;
    dt = std::max(kMinDt, std::min(kMaxDt, dt));
    last_timestamp_us_ = timestamp_us;

    std::array<std::array<float, 6>, 6> transition{};
    for (int i = 0; i < 6; ++i) {
        transition[i][i] = 1.0F;
    }
    transition[0][2] = dt;
    transition[1][3] = dt;
    transition[4][5] = dt;

    state_[0] += state_[2] * dt;
    state_[1] += state_[3] * dt;
    state_[4] += state_[5] * dt;

    std::array<std::array<float, 6>, 6> intermediate{};
    std::array<std::array<float, 6>, 6> predicted_covariance{};
    for (int row = 0; row < 6; ++row) {
        for (int column = 0; column < 6; ++column) {
            for (int k = 0; k < 6; ++k) {
                intermediate[row][column] += transition[row][k] * covariance_[k][column];
            }
        }
    }
    for (int row = 0; row < 6; ++row) {
        for (int column = 0; column < 6; ++column) {
            for (int k = 0; k < 6; ++k) {
                predicted_covariance[row][column] += intermediate[row][k] * transition[column][k];
            }
        }
    }

    const float fx2 = std::max(1.0F, config_.camera_model.fx * config_.camera_model.fx);
    const float fy2 = std::max(1.0F, config_.camera_model.fy * config_.camera_model.fy);
    predicted_covariance[0][0] += config_.process_noise_position * dt / fx2;
    predicted_covariance[1][1] += config_.process_noise_position * dt / fy2;
    predicted_covariance[2][2] += config_.process_noise_velocity * dt / fx2;
    predicted_covariance[3][3] += config_.process_noise_velocity * dt / fy2;
    predicted_covariance[4][4] += config_.process_noise_log_size * dt;
    predicted_covariance[5][5] += config_.process_noise_size_rate * dt;
    covariance_ = predicted_covariance;
}

bool TemporalTracker::has_prediction() const
{
    return initialized_;
}

bool TemporalTracker::tracking_confirmed() const
{
    return initialized_ && track_state_ == TrackState::Tracking;
}

float TemporalTracker::predicted_x() const
{
    return config_.camera_model.principal_x +
           state_[0] * config_.camera_model.fx;
}

float TemporalTracker::predicted_y() const
{
    return config_.camera_model.principal_y +
           state_[1] * config_.camera_model.fy;
}

float TemporalTracker::predicted_size() const
{
    return initialized_ ? std::exp(state_[4]) : kMinSize;
}

uint64_t TemporalTracker::measurement_age_us(uint64_t timestamp_us) const
{
    if (!initialized_ || last_measurement_timestamp_us_ == 0 ||
        timestamp_us < last_measurement_timestamp_us_) {
        return 0;
    }
    return timestamp_us - last_measurement_timestamp_us_;
}

int TemporalTracker::missed_frames() const
{
    return missed_frames_;
}

std::array<float, 2> TemporalTracker::line_of_sight_rate() const
{
    if (!initialized_) {
        return {0.0F, 0.0F};
    }
    const float yaw_rate = state_[2] / (1.0F + state_[0] * state_[0]);
    const float pitch_rate = -state_[3] / (1.0F + state_[1] * state_[1]);
    return {yaw_rate, pitch_rate};
}

std::array<float, 2> TemporalTracker::angular_covariance() const
{
    if (!initialized_) {
        return {0.0F, 0.0F};
    }
    const float yaw_scale = 1.0F / (1.0F + state_[0] * state_[0]);
    const float pitch_scale = 1.0F / (1.0F + state_[1] * state_[1]);
    return {std::max(0.0F, covariance_[0][0] * yaw_scale * yaw_scale),
            std::max(0.0F, covariance_[1][1] * pitch_scale * pitch_scale)};
}

void TemporalTracker::apply_motion_prior(const MotionPrior &prior)
{
    if (!initialized_ || !prior.image_transform_valid ||
        prior.confidence <= 0.0F) {
        return;
    }

    const float cx = config_.camera_model.principal_x;
    const float cy = config_.camera_model.principal_y;
    const float x = predicted_x() - cx;
    const float y = predicted_y() - cy;
    const float scale = std::max(0.5F, std::min(2.0F, prior.image_scale));
    const float cosine = std::cos(prior.image_rotation_rad);
    const float sine = std::sin(prior.image_rotation_rad);
    const float transformed_x = scale * (cosine * x - sine * y) +
                                cx + prior.image_dx_px;
    const float transformed_y = scale * (sine * x + cosine * y) +
                                cy + prior.image_dy_px;
    state_[0] = (transformed_x - cx) / std::max(1.0F, config_.camera_model.fx);
    state_[1] = (transformed_y - cy) / std::max(1.0F, config_.camera_model.fy);
    state_[4] += std::log(scale);

    const float confidence = clamp01(prior.confidence);
    covariance_[0][0] += (1.0F - confidence) * 4.0F /
                         std::max(1.0F, config_.camera_model.fx * config_.camera_model.fx);
    covariance_[1][1] += (1.0F - confidence) * 4.0F /
                         std::max(1.0F, config_.camera_model.fy * config_.camera_model.fy);
}

float TemporalTracker::association_score(const CandidateObservation &candidate) const
{
    if (!initialized_) {
        return 0.5F;
    }

    const float dx = candidate.center_x - predicted_x();
    const float dy = candidate.center_y - predicted_y();
    const float distance = std::sqrt(dx * dx + dy * dy);
    const float predicted_apparent_size = std::max(predicted_size(), kMinSize);
    const float gate = std::min(
        config_.gate_max_px,
        std::max(config_.gate_min_px,
                 predicted_apparent_size * config_.gate_size_factor));
    const float spatial_score = std::exp(-0.5F * (distance / gate) * (distance / gate));

    const float log_size = std::log(std::max(candidate.apparent_size, kMinSize));
    const float log_difference = std::fabs(log_size - state_[4]);
    const float size_jump_limit =
        candidate.source == last_candidate_.source
            ? config_.max_log_size_jump
            : config_.max_cross_source_log_size_jump;
    const float size_scale = std::max(0.25F, size_jump_limit * 0.5F);
    const float size_score = std::exp(-0.5F * (log_difference / size_scale) *
                                     (log_difference / size_scale));
    return clamp01(spatial_score * size_score);
}

bool TemporalTracker::passes_association_gate(
    const CandidateObservation &candidate) const
{
    if (!initialized_) {
        return true;
    }

    const float dx = candidate.center_x - predicted_x();
    const float dy = candidate.center_y - predicted_y();
    const float distance = std::sqrt(dx * dx + dy * dy);
    const float gate = std::min(
        config_.gate_max_px,
        std::max(config_.gate_min_px,
                 std::max(predicted_size(), kMinSize) *
                     config_.gate_size_factor));
    if (distance > gate) {
        return false;
    }

    const float log_difference = std::fabs(
        std::log(std::max(candidate.apparent_size, kMinSize)) - state_[4]);
    const float size_jump_limit =
        candidate.source == last_candidate_.source
            ? config_.max_log_size_jump
            : config_.max_cross_source_log_size_jump;
    if (log_difference > size_jump_limit) {
        return false;
    }
    return association_score(candidate) >= config_.min_association_score;
}

bool TemporalTracker::matches_tentative_candidate(
    const CandidateObservation &candidate) const
{
    if (!initialized_) {
        return false;
    }
    const float dx = candidate.center_x - predicted_x();
    const float dy = candidate.center_y - predicted_y();
    if (std::sqrt(dx * dx + dy * dy) > config_.confirmation_gate_px) {
        return false;
    }

    const float log_difference = std::fabs(
        std::log(std::max(candidate.apparent_size, kMinSize)) - state_[4]);
    const float size_jump_limit =
        candidate.source == last_candidate_.source
            ? config_.max_log_size_jump
            : config_.max_cross_source_log_size_jump;
    return log_difference <= size_jump_limit;
}

void TemporalTracker::correct(const CandidateObservation &candidate)
{
    const float measurement[3] = {
        (candidate.center_x - config_.camera_model.principal_x) /
            std::max(1.0F, config_.camera_model.fx),
        (candidate.center_y - config_.camera_model.principal_y) /
            std::max(1.0F, config_.camera_model.fy),
        std::log(std::max(candidate.apparent_size, kMinSize)),
    };
    const int observed_state_index[3] = {0, 1, 4};

    float innovation[3]{};
    for (int i = 0; i < 3; ++i) {
        innovation[i] = measurement[i] - state_[observed_state_index[i]];
    }

    float innovation_covariance[3][3]{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            innovation_covariance[row][column] =
                covariance_[observed_state_index[row]][observed_state_index[column]];
        }
    }
    innovation_covariance[0][0] += config_.measurement_noise_position /
        std::max(1.0F, config_.camera_model.fx * config_.camera_model.fx);
    innovation_covariance[1][1] += config_.measurement_noise_position /
        std::max(1.0F, config_.camera_model.fy * config_.camera_model.fy);
    innovation_covariance[2][2] += config_.measurement_noise_log_size;

    float inverse_innovation_covariance[3][3]{};
    if (!invert_3x3(innovation_covariance, inverse_innovation_covariance)) {
        return;
    }

    float kalman_gain[6][3]{};
    for (int row = 0; row < 6; ++row) {
        for (int column = 0; column < 3; ++column) {
            for (int k = 0; k < 3; ++k) {
                kalman_gain[row][column] +=
                    covariance_[row][observed_state_index[k]] *
                    inverse_innovation_covariance[k][column];
            }
        }
    }

    for (int row = 0; row < 6; ++row) {
        for (int i = 0; i < 3; ++i) {
            state_[row] += kalman_gain[row][i] * innovation[i];
        }
    }

    const auto previous_covariance = covariance_;
    for (int row = 0; row < 6; ++row) {
        for (int column = 0; column < 6; ++column) {
            float correction = 0.0F;
            for (int k = 0; k < 3; ++k) {
                correction += kalman_gain[row][k] *
                              previous_covariance[observed_state_index[k]][column];
            }
            covariance_[row][column] = previous_covariance[row][column] - correction;
        }
    }
}

void TemporalTracker::push_hit(bool hit)
{
    hit_history_.push_back(hit);
    while (static_cast<int>(hit_history_.size()) > config_.confirm_window) {
        hit_history_.pop_front();
    }
}

int TemporalTracker::recent_hits() const
{
    return static_cast<int>(std::count(hit_history_.begin(), hit_history_.end(), true));
}

GreenLightDetection TemporalTracker::update(const CandidateObservation *candidate,
                                            uint64_t timestamp_us,
                                            const CameraModel &camera_model,
                                            bool count_as_miss)
{
    predict(timestamp_us);

    GreenLightDetection result;
    result.timestamp_us = timestamp_us;
    result.measurement_age_us = measurement_age_us(timestamp_us);
    result.missed_frames = missed_frames_;

    if (candidate != nullptr && tracking_confirmed() &&
        !passes_association_gate(*candidate)) {
        candidate = nullptr;
    }

    if (candidate == nullptr) {
        const bool was_tracking = initialized_ &&
                                  track_state_ == TrackState::Tracking;
        const uint64_t prediction_age_us =
            timestamp_us >= last_measurement_timestamp_us_
                ? timestamp_us - last_measurement_timestamp_us_
                : std::numeric_limits<uint64_t>::max();
        if (count_as_miss) {
            // Acquisition must be supported by uninterrupted real evidence.
            // Otherwise a glyph intermittently passing appearance checks can
            // pool isolated hits across rejected frames and become a target.
            // Keep its position for reacquisition, but restart confirmation.
            // Scheduled skips and an already confirmed track retain their
            // existing prediction/occlusion behaviour.
            if (!was_tracking) hit_history_.clear();
            push_hit(false);
            ++missed_frames_;
            if (missed_frames_ >= config_.max_missed_frames) {
                reset();
            } else if (initialized_ && track_state_ != TrackState::Tracking) {
                track_state_ = TrackState::Candidate;
            }
        }
        result.state = track_state_;
        const uint64_t maximum_prediction_age_us =
            static_cast<uint64_t>(config_.prediction_max_age_ms) * 1000U;
        if (was_tracking && initialized_ &&
            config_.prediction_max_age_ms > 0 &&
            prediction_age_us <= maximum_prediction_age_us) {
            const float apparent_size = std::exp(state_[4]);
            const float previous_size =
                std::max(kMinSize, last_candidate_.apparent_size);
            const float bbox_scale = apparent_size / previous_size;
            const int bbox_width = std::max(
                1, static_cast<int>(std::lround(
                       last_candidate_.bbox_w * bbox_scale)));
            const int bbox_height = std::max(
                1, static_cast<int>(std::lround(
                       last_candidate_.bbox_h * bbox_scale)));
            const float output_x = predicted_x();
            const float output_y = predicted_y();
            const auto angles = pixel_to_angles(output_x, output_y,
                                                camera_model);
            const float hit_ratio = config_.confirm_window > 0
                                        ? static_cast<float>(recent_hits()) /
                                              config_.confirm_window
                                        : 1.0F;
            const float age_fraction =
                static_cast<float>(prediction_age_us) /
                std::max(1.0F,
                         static_cast<float>(maximum_prediction_age_us));
            const float confidence_decay = std::exp(-age_fraction);

            result.valid = true;
            result.predicted = true;
            result.center_x = output_x;
            result.center_y = output_y;
            result.bbox_x = static_cast<int>(
                std::lround(output_x - 0.5F * bbox_width));
            result.bbox_y = static_cast<int>(
                std::lround(output_y - 0.5F * bbox_height));
            result.bbox_w = bbox_width;
            result.bbox_h = bbox_height;
            result.apparent_size = apparent_size;
            result.yaw_rad = angles[0];
            result.pitch_rad = angles[1];
            result.confidence = clamp01(
                confidence_decay *
                (0.60F * last_candidate_.score +
                 0.25F * last_candidate_.association_score +
                 0.15F * hit_ratio));
        }
        result.measurement_age_us = prediction_age_us;
        result.missed_frames = missed_frames_;
        return result;
    }

    if (!initialized_) {
        initialize(*candidate, timestamp_us);
        hit_history_.clear();
    } else if (track_state_ != TrackState::Tracking &&
               !matches_tentative_candidate(*candidate)) {
        initialize(*candidate, timestamp_us);
        hit_history_.clear();
    } else {
        correct(*candidate);
    }

    last_candidate_ = *candidate;
    last_measurement_timestamp_us_ = timestamp_us;
    missed_frames_ = 0;
    push_hit(true);
    if (track_state_ != TrackState::Tracking) {
        track_state_ = recent_hits() >= config_.confirm_hits
                           ? TrackState::Tracking
                           : TrackState::Candidate;
    }

    const float output_x = predicted_x();
    const float output_y = predicted_y();
    const auto angles = pixel_to_angles(output_x, output_y, camera_model);
    const float hit_ratio = config_.confirm_window > 0
                                ? static_cast<float>(recent_hits()) /
                                      static_cast<float>(config_.confirm_window)
                                : 1.0F;

    // A tentative observation is useful for visualization, but must not be
    // consumed by aiming/control until the same physical candidate has been
    // confirmed across multiple frames.
    result.valid = track_state_ == TrackState::Tracking;
    result.state = track_state_;
    result.center_x = output_x;
    result.center_y = output_y;
    result.bbox_x = candidate->bbox_x;
    result.bbox_y = candidate->bbox_y;
    result.bbox_w = candidate->bbox_w;
    result.bbox_h = candidate->bbox_h;
    result.apparent_size = std::exp(state_[4]);
    result.yaw_rad = angles[0];
    result.pitch_rad = angles[1];
    result.confidence = result.valid
                            ? clamp01(0.60F * candidate->score +
                                      0.25F * candidate->association_score +
                                      0.15F * hit_ratio)
                            : 0.0F;
    result.measurement_age_us = 0;
    result.missed_frames = 0;
    return result;
}

}  // namespace detail
}  // namespace dart
