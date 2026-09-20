#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "maix_image.hpp"

namespace dart {

enum class TrackState {
    Lost,
    Candidate,
    Tracking,
};

enum class GuidanceTrackState {
    Search,
    Acquiring,
    Tracking,
    Coasting,
    Reacquire,
};

enum class GuidanceMode {
    LampApproach,
    Fused,
    ArmorImpact,
};

enum class ArmorColor {
    Unknown,
    Red,
    Blue,
};

const char *guidance_track_state_name(GuidanceTrackState state);
const char *guidance_mode_name(GuidanceMode mode);
const char *armor_color_name(ArmorColor color);

struct Point2f {
    float x = 0.0F;
    float y = 0.0F;
    bool valid = false;
};

struct LineSegment2f {
    Point2f top;
    Point2f bottom;
    Point2f center;
    float length = 0.0F;
    float width = 0.0F;
    float angle_rad = 0.0F;
    float confidence = 0.0F;
};

struct MotionPrior {
    bool valid = false;
    uint64_t timestamp_us = 0;
    std::array<float, 4> orientation_wxyz{1.0F, 0.0F, 0.0F, 0.0F};
    std::array<float, 3> angular_velocity_rad_s{};

    // Optional image-space delta supplied by the visual motion estimator.
    bool image_transform_valid = false;
    float image_dx_px = 0.0F;
    float image_dy_px = 0.0F;
    float image_rotation_rad = 0.0F;
    float image_scale = 1.0F;
    float confidence = 0.0F;
};

MotionPrior interpolate_motion_prior(const MotionPrior &before,
                                     const MotionPrior &after,
                                     uint64_t timestamp_us);

struct TargetPose {
    bool valid = false;
    std::array<float, 3> translation_m{};
    std::array<float, 4> orientation_wxyz{1.0F, 0.0F, 0.0F, 0.0F};
    float distance_m = 0.0F;
    float reprojection_error_px = 0.0F;
};

struct ArmorDetection {
    bool valid = false;
    ArmorColor color = ArmorColor::Unknown;
    LineSegment2f left_bar;
    LineSegment2f right_bar;
    Point2f center;
    float separation_px = 0.0F;
    float geometry_confidence = 0.0F;
    bool model_validated = false;
    float model_confidence = 0.0F;
};

const char *track_state_name(TrackState state);

struct GreenLightDetection {
    bool valid = false;
    bool predicted = false;
    uint64_t timestamp_us = 0;
    TrackState state = TrackState::Lost;
    float center_x = 0.0F;
    float center_y = 0.0F;
    int bbox_x = 0;
    int bbox_y = 0;
    int bbox_w = 0;
    int bbox_h = 0;
    float apparent_size = 0.0F;
    float yaw_rad = 0.0F;
    float pitch_rad = 0.0F;
    float confidence = 0.0F;
    uint64_t measurement_age_us = 0;
    int missed_frames = 0;
};

struct TargetEstimate {
    int schema_version = 2;
    bool angles_valid = true; // legacy path; high-fps uncalibrated path clears this
    uint64_t source_sequence = 0, source_pts_raw = 0, source_received_us = 0;
    uint64_t armor_source_received_us = 0; // high-fps cached geometry observation
    uint64_t application_dropped = 0, upstream_missing = 0;
    uint64_t upstream_duplicate = 0, upstream_reversed = 0;
    bool source_metadata_valid = false;
    uint64_t timestamp_us = 0;
    uint64_t measurement_age_us = 0;
    GuidanceTrackState state = GuidanceTrackState::Search;
    GuidanceMode guidance_mode = GuidanceMode::LampApproach;
    bool valid = false;
    bool safe_for_control = false;
    bool predicted = false;
    bool armor_detection_ran = false;
    bool classical_detection_ran = false;
    float classical_detection_ms = 0.0F;
    bool model_ran = false;
    float model_inference_ms = 0.0F;

    GreenLightDetection green;
    ArmorDetection armor;
    TargetPose pose;

    Point2f aim_point;
    std::array<float, 3> line_of_sight_camera{0.0F, 0.0F, 1.0F};
    std::array<float, 2> line_of_sight_rate_rad_s{};
    std::array<float, 2> angular_covariance{};
    float yaw_rad = 0.0F;
    float pitch_rad = 0.0F;
    float confidence = 0.0F;
};

struct GreenLightCandidateDebug {
    float center_x = 0.0F;
    float center_y = 0.0F;
    int bbox_x = 0;
    int bbox_y = 0;
    int bbox_w = 0;
    int bbox_h = 0;
    float apparent_size = 0.0F;
    float density = 0.0F;
    float green_dominance = 0.0F;
    float green_fraction = 0.0F;
    float local_contrast = 0.0F;
    float shape_score = 0.0F;
    float core_score = 0.0F;
    float temporal_score = 0.0F;
    float center_prior_score = 0.0F;
    float initial_size_score = 0.0F;
    float score = 0.0F;
    float normalized_response = 0.0F;
    float model_score = 0.0F;
    bool inside_capture_cone = true;
    bool model_validated = false;
    bool saturated_core = false;
    bool selected = false;
    float appearance_score = 0.0F;
    float radial_support = 0.0F;
    float axis_ratio = 0.0F;
};

struct LabThreshold {
    int l_min = 0;
    int l_max = 100;
    int a_min = -128;
    int a_max = 127;
    int b_min = -128;
    int b_max = 127;

    std::vector<int> as_vector() const;
};

struct CameraModel {
    float fx = 4096.0F;
    float fy = 4096.0F;
    float principal_x = 640.0F;
    float principal_y = 360.0F;
    float k1 = 0.0F;
    float k2 = 0.0F;
    float p1 = 0.0F;
    float p2 = 0.0F;
    float k3 = 0.0F;
};

struct DetectorConfig {
    LabThreshold core_lab{70, 100, -90, -5, -40, 90};
    LabThreshold halo_lab{25, 100, -90, -3, -60, 100};

    float min_candidate_score = 0.42F;
    float min_tracking_score = 0.35F;
    float min_density = 0.03F;
    float min_green_dominance = 0.05F;
    float min_local_contrast = -0.03F;
    float min_tracking_local_contrast = -0.08F;
    float contrast_relax_min_association = 0.75F;
    float min_halo_size_px = 1.0F;
    float core_center_max_fraction = 0.45F;
    float center_prior_radius_px = 160.0F;
    float initial_size_reference_px = 24.0F;
    bool enable_saturated_core_candidates = false;
    float min_core_brightness = 0.88F;
    float min_core_ring_green_dominance = 0.04F;
    float min_core_ring_green_fraction = 0.0F;
    float core_ring_scale = 0.35F;
    float min_core_size_px = 3.0F;

    float weight_color = 0.25F;
    float weight_contrast = 0.20F;
    float weight_density = 0.15F;
    float weight_shape = 0.10F;
    float weight_core = 0.10F;
    float weight_temporal = 0.20F;
    float weight_center_prior = 0.0F;
    float weight_initial_size = 0.0F;
    float weight_core_size = 0.0F;

    int confirm_window = 5;
    int confirm_hits = 3;
    int max_missed_frames = 5;
    int prediction_max_age_ms = 110;
    float confirmation_gate_px = 48.0F;
    float gate_min_px = 32.0F;
    float gate_max_px = 96.0F;
    float gate_size_factor = 4.0F;
    float max_log_size_jump = 2.3F;
    float max_cross_source_log_size_jump = 2.3F;
    float min_association_score = 0.02F;

    float process_noise_position = 20.0F;
    float process_noise_velocity = 100.0F;
    float process_noise_log_size = 0.6F;
    float process_noise_size_rate = 2.0F;
    float measurement_noise_position = 4.0F;
    float measurement_noise_log_size = 0.08F;

    int sample_grid = 20;
    int ring_margin_px = 8;
    int merge_margin_px = 2;
    bool merge_blobs = true;

    bool enable_normalized_multiscale = true;
    // Compatibility fallback. Two full-frame RGB-to-LAB passes are too slow
    // for the MaixCAM2 Cortex-A53 and may be disabled when the normalized RGB
    // candidate path is active.
    bool enable_legacy_lab_candidates = true;
    // Use a single full-resolution response pass to find compact components,
    // then run the five-scale ring test only around those components. This
    // preserves 3-5 px targets without the cost of a dense multi-scale scan.
    bool enable_sparse_component_search = false;
    bool integral_peak_statistics = false; // exact sums; enabled by the source-ROI adapter
    std::array<int, 5> multiscale_diameters_px{3, 5, 8, 12, 18};
    int classical_interval_frames = 1;
    int multiscale_downsample = 1;
    int multiscale_min_scan_step_px = 1;
    int multiscale_tracking_min_scan_step_px = 1;
    bool multiscale_capture_cone_only = false;
    float multiscale_tracking_roi_radius_px = 0.0F;
    int multiscale_full_refresh_interval = 1;
    float min_normalized_green_response = 0.045F;
    float min_normalized_inner_brightness = 0.06F;
    // Optional bright green pixel evidence for normalized candidates; 0 disables.
    int normalized_min_peak_green = 0;
    // Independent per-frame evidence for the fast normalized path. Temporal
    // association must never bypass these appearance checks.
    bool enable_lamp_appearance = true;
    float lamp_min_green_margin = 0.08F;
    float lamp_min_color_fraction = 0.12F;
    float lamp_min_relative_contrast = 0.12F;
    float lamp_max_axis_ratio = 2.5F;
    float lamp_min_core_fill = 0.80F;
    float min_normalized_contrast_z = -0.50F;
    float min_tracking_normalized_contrast_z = -2.0F;
    float normalized_brightness_weight = 0.20F;
    int max_green_candidates = 5;
    bool enable_capture_cone = true;
    float capture_cone_deg = 6.5F;

    int control_prediction_max_frames = 2;
    int control_prediction_max_age_ms = 35;

    CameraModel camera_model;
};

struct ArmorConfig {
    ArmorColor expected_color = ArmorColor::Red;
    float min_color_response = 0.16F;
    float min_brightness = 0.18F;
    float min_green_size_px = 16.0F;
    int min_component_pixels = 2;
    int max_component_pixels = 12000;
    float min_bar_length_px = 2.0F;
    float max_bar_length_px = 180.0F;
    float min_elongation = 1.45F;
    float max_pair_angle_deg = 20.0F;
    // Offset of the two centers along their mean bar axis / mean bar length.
    float max_pair_longitudinal_to_length = 0.6F;
    float min_length_ratio = 0.50F;
    float max_color_response_diff = 0.35F;
    float min_separation_to_length = 0.8F;
    float max_separation_to_length = 6.0F;
    // Optional scale-consistency gate; zero preserves uncalibrated setups.
    float max_separation_to_green_size = 0.0F;
    float min_green_offset_to_length = 0.55F;
    float max_green_offset_to_length = 5.0F;
    float max_green_lateral_to_separation = 0.75F;
    float min_geometry_confidence = 0.55F;
    int required_pose_hits = 3;
    // Consecutive consistent fresh pair observations, not cached outputs.
    int confirmation_hits = 3;
    int confirmation_max_gap_ms = 100;
    int aim_blend_ms = 100;
    int cache_max_age_ms = 50;
};

struct TargetGeometryConfig {
    bool pose_enabled = false;
    float bar_separation_m = 0.0F;
    float bar_length_m = 0.0F;
    float green_offset_m = 0.0F;
    float max_reprojection_error_px = 2.0F;
    float min_pose_separation_px = 16.0F;
};

struct NpuConfig {
    bool enabled = false;
    bool required = false;
    std::string model_path = "models/dart_target_pose.mud";
    int input_size = 256;
    int interval_ms = 33;
    int search_candidates = 3;
    float confidence_threshold = 0.50F;
    float keypoint_threshold = 0.35F;
    int min_roi_size_px = 64;
    int max_roi_size_px = 384;
    float roi_size_factor = 8.0F;
};

struct VisualMotionConfig {
    bool enabled = true;
    // Global motion is useful only after a target has been acquired. Keeping
    // it idle during SEARCH leaves the whole frame budget to reacquisition.
    bool tracking_only = false;
    int interval_frames = 1;
    int grid_width = 80;
    int grid_height = 60;
    int max_shift_px = 5;
    float max_rotation_deg = 6.0F;
    float rotation_step_deg = 2.0F;
    float min_response = 0.20F;
};

struct CameraSettings {
    int width = 640;
    int height = 480;
    int fps = 60;
    int buffer_count = 3;
    int warmup_frames = 30;
    int exposure_us = 500;
    int gain = 0;
    bool manual_white_balance = true;
    std::array<float, 4> white_balance_gain{0.0682F, 0.0F, 0.0F, 0.04897F};
};

struct DebugSettings {
    bool enabled = false;
    std::string directory = "/tmp/dart_green_debug";
    int json_log_every_n_frames = 6;
    bool log_candidates = false;
    int save_every_n_frames = 0;
    bool save_on_state_change = true;
    bool save_failed_frames = false;
    int max_saved_frames = 1000;
};

struct HighFpsScheduleConfig {
    int green_hz = 90;
    int armor_hz = 60;
    int search_hz = 30;
    int motion_hz = 30;
};

struct ApplicationConfig {
    DetectorConfig detector;
    ArmorConfig armor;
    TargetGeometryConfig target_geometry;
    NpuConfig npu;
    VisualMotionConfig visual_motion;
    HighFpsScheduleConfig highfps;
    CameraSettings camera;
    DebugSettings debug;
};

ApplicationConfig load_application_config(const std::string &path);

namespace detail {

enum class CandidateSource {
    Halo,
    SaturatedCore,
    NormalizedResponse,
};

struct CandidateObservation {
    float center_x = 0.0F;
    float center_y = 0.0F;
    int bbox_x = 0;
    int bbox_y = 0;
    int bbox_w = 0;
    int bbox_h = 0;
    float apparent_size = 1.0F;
    float score = 0.0F;
    float association_score = 0.5F;
    CandidateSource source = CandidateSource::Halo;
};

class TemporalTracker {
public:
    explicit TemporalTracker(const DetectorConfig &config);

    void reset();
    // Copy-only view of the same normalized state in an integer-offset ROI.
    TemporalTracker roi_view(int x, int y) const;
    void predict(uint64_t timestamp_us);
    bool has_prediction() const;
    bool tracking_confirmed() const;
    float predicted_x() const;
    float predicted_y() const;
    float predicted_size() const;
    uint64_t measurement_age_us(uint64_t timestamp_us) const;
    int missed_frames() const;
    std::array<float, 2> line_of_sight_rate() const;
    std::array<float, 2> angular_covariance() const;
    void apply_motion_prior(const MotionPrior &prior);
    float association_score(const CandidateObservation &candidate) const;
    bool passes_association_gate(const CandidateObservation &candidate) const;
    GreenLightDetection update(const CandidateObservation *candidate,
                               uint64_t timestamp_us,
                               const CameraModel &camera_model,
                               bool count_as_miss = true);

private:
    DetectorConfig config_;
    bool initialized_ = false;
    uint64_t last_timestamp_us_ = 0;
    uint64_t last_measurement_timestamp_us_ = 0;
    std::array<float, 6> state_{};
    std::array<std::array<float, 6>, 6> covariance_{};
    std::deque<bool> hit_history_;
    int missed_frames_ = 0;
    TrackState track_state_ = TrackState::Lost;
    CandidateObservation last_candidate_{};

    void initialize(const CandidateObservation &candidate, uint64_t timestamp_us);
    void correct(const CandidateObservation &candidate);
    bool matches_tentative_candidate(const CandidateObservation &candidate) const;
    void push_hit(bool hit);
    int recent_hits() const;
};

std::array<float, 2> pixel_to_angles(float pixel_x,
                                     float pixel_y,
                                     const CameraModel &model);

}  // namespace detail

struct CandidateRoi {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
};

struct PoseValidation {
    bool valid = false;
    float confidence = 0.0F;
    // green, left-top, left-bottom, right-top, right-bottom
    std::array<Point2f, 5> keypoints{};
};

class TargetPoseValidator {
public:
    virtual ~TargetPoseValidator() = default;
    virtual PoseValidation validate(maix::image::Image &frame,
                                    const CandidateRoi &roi) = 0;
};

std::shared_ptr<TargetPoseValidator>
create_yolo_pose_validator(const NpuConfig &config);

class GreenLightDetector {
public:
    explicit GreenLightDetector(
        const DetectorConfig &config = DetectorConfig(),
        const ArmorConfig &armor_config = ArmorConfig(),
        const TargetGeometryConfig &target_geometry = TargetGeometryConfig(),
        const NpuConfig &npu_config = NpuConfig(),
        std::shared_ptr<TargetPoseValidator> pose_validator = nullptr);

    GreenLightDetection process(maix::image::Image &frame,
                                uint64_t timestamp_us);
    TargetEstimate process(maix::image::Image &frame,
                           uint64_t timestamp_us,
                           const MotionPrior *motion_prior);
    // Full-resolution RGB ROI, with all tracker/output coordinates in source pixels.
    TargetEstimate process_region(maix::image::Image &roi, const CandidateRoi &region,
                                  int source_width, int source_height,
                                  uint64_t timestamp_us, const MotionPrior *motion = nullptr,
                                  bool force_armor_scan = false, bool run_armor_scan = true);
    detail::TemporalTracker tracker_snapshot() const { return tracker_; }
    // Compatibility alias for early v0.2 callers.
    TargetEstimate process_target(maix::image::Image &frame,
                                  uint64_t timestamp_us,
                                  const MotionPrior *motion_prior = nullptr);
    void reset();
    const std::vector<GreenLightCandidateDebug> &last_candidates() const;

private:
    bool region_active_ = false, region_force_armor_ = false, region_run_armor_ = true;
    CandidateRoi region_{};
    int source_width_ = 0, source_height_ = 0;
    struct Candidate;
    struct CandidateHypothesis {
        bool active = false;
        float x = 0.0F;
        float y = 0.0F;
        float velocity_x = 0.0F;
        float velocity_y = 0.0F;
        float apparent_size = 1.0F;
        float appearance = 0.0F;
        float confidence = 0.0F;
        uint64_t timestamp_us = 0;
        int hits = 0;
        int misses = 0;
    };

    DetectorConfig config_;
    ArmorConfig armor_config_;
    TargetGeometryConfig target_geometry_;
    NpuConfig npu_config_;
    detail::TemporalTracker tracker_;
    std::shared_ptr<TargetPoseValidator> pose_validator_;
    std::vector<GreenLightCandidateDebug> last_candidates_;
    std::array<CandidateHypothesis, 3> candidate_hypotheses_{};

    uint64_t last_model_timestamp_us_ = 0;
    uint64_t last_model_positive_timestamp_us_ = 0;
    float last_model_inference_ms_ = 0.0F;
    PoseValidation last_pose_validation_{};
    CandidateRoi last_model_roi_{};
    uint64_t last_output_timestamp_us_ = 0;
    std::array<float, 2> last_output_angles_{};
    int armor_pose_hits_ = 0;
    uint64_t armor_blend_start_us_ = 0;
    uint64_t last_armor_timestamp_us_ = 0;
    ArmorDetection last_armor_detection_{};
    ArmorDetection armor_candidate_detection_{};
    Point2f armor_candidate_green_center_{};
    uint64_t armor_candidate_timestamp_us_ = 0;
    int armor_candidate_hits_ = 0;
    std::size_t search_model_cursor_ = 0;
    uint64_t frame_counter_ = 0;
    uint64_t classical_detection_count_ = 0;
    bool last_classical_detection_ran_ = false;
    float last_classical_detection_ms_ = 0.0F;

    std::vector<Candidate> collect_candidates(maix::image::Image &frame);
    void update_candidate_hypotheses(std::vector<Candidate> &candidates,
                                     uint64_t timestamp_us,
                                     const MotionPrior *motion_prior);
    GreenLightDetection process_green(maix::image::Image &frame,
                                      uint64_t timestamp_us,
                                      const MotionPrior *motion_prior);
};

}  // namespace dart
