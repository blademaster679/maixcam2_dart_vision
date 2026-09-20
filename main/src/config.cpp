#include "dart/green_detector.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace dart {
namespace {

std::string trim(const std::string &value)
{
    const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    });
    if (first == value.end()) {
        return {};
    }
    const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    }).base();
    return std::string(first, last);
}

std::vector<std::string> split(const std::string &value, char delimiter)
{
    std::vector<std::string> parts;
    std::stringstream stream(value);
    std::string part;
    while (std::getline(stream, part, delimiter)) {
        parts.push_back(trim(part));
    }
    return parts;
}

int parse_int(const std::string &key, const std::string &value)
{
    std::size_t consumed = 0;
    const int result = std::stoi(value, &consumed);
    if (consumed != value.size()) {
        throw std::runtime_error("invalid integer for " + key + ": " + value);
    }
    return result;
}

float parse_float(const std::string &key, const std::string &value)
{
    std::size_t consumed = 0;
    const float result = std::stof(value, &consumed);
    if (consumed != value.size() || !std::isfinite(result)) {
        throw std::runtime_error("invalid float for " + key + ": " + value);
    }
    return result;
}

bool parse_bool(const std::string &key, std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    if (value == "true" || value == "1" || value == "yes" || value == "on") {
        return true;
    }
    if (value == "false" || value == "0" || value == "no" || value == "off") {
        return false;
    }
    throw std::runtime_error("invalid boolean for " + key + ": " + value);
}

LabThreshold parse_lab(const std::string &key, const std::string &value)
{
    const auto parts = split(value, ',');
    if (parts.size() != 6) {
        throw std::runtime_error(key + " must contain six comma-separated integers");
    }
    return {parse_int(key, parts[0]),
            parse_int(key, parts[1]),
            parse_int(key, parts[2]),
            parse_int(key, parts[3]),
            parse_int(key, parts[4]),
            parse_int(key, parts[5])};
}

std::array<float, 4> parse_four_floats(const std::string &key,
                                       const std::string &value)
{
    const auto parts = split(value, ',');
    if (parts.size() != 4) {
        throw std::runtime_error(key + " must contain four comma-separated numbers");
    }
    return {parse_float(key, parts[0]),
            parse_float(key, parts[1]),
            parse_float(key, parts[2]),
            parse_float(key, parts[3])};
}

std::array<int, 5> parse_five_ints(const std::string &key,
                                   const std::string &value)
{
    const auto parts = split(value, ',');
    if (parts.size() != 5) {
        throw std::runtime_error(key + " must contain five comma-separated integers");
    }
    return {parse_int(key, parts[0]), parse_int(key, parts[1]),
            parse_int(key, parts[2]), parse_int(key, parts[3]),
            parse_int(key, parts[4])};
}

ArmorColor parse_armor_color(const std::string &key, std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    if (value == "red") {
        return ArmorColor::Red;
    }
    if (value == "blue") {
        return ArmorColor::Blue;
    }
    if (value == "unknown" || value == "auto") {
        return ArmorColor::Unknown;
    }
    throw std::runtime_error("invalid armor color for " + key + ": " + value);
}

void validate_lab(const std::string &name, const LabThreshold &threshold)
{
    if (threshold.l_min < 0 || threshold.l_max > 100 ||
        threshold.a_min < -128 || threshold.a_max > 127 ||
        threshold.b_min < -128 || threshold.b_max > 127 ||
        threshold.l_min > threshold.l_max ||
        threshold.a_min > threshold.a_max ||
        threshold.b_min > threshold.b_max) {
        throw std::runtime_error(name + " is outside the MaixCDK LAB ranges");
    }
}

void validate(const ApplicationConfig &config)
{
    const auto &h = config.highfps;
    if (h.green_hz < 1 || h.green_hz > 180 || h.armor_hz < 1 ||
        h.armor_hz > h.green_hz || h.search_hz < 1 || h.search_hz > h.green_hz ||
        h.motion_hz < 1 || h.motion_hz > h.green_hz) {
        throw std::runtime_error("invalid full180 stage rates");
    }
    validate_lab("lab.core", config.detector.core_lab);
    validate_lab("lab.halo", config.detector.halo_lab);

    if (config.camera.width <= 0 || config.camera.height <= 0 ||
        config.camera.fps <= 0 || config.camera.buffer_count <= 0 ||
        config.camera.warmup_frames < 0 || config.camera.exposure_us < 0 ||
        config.camera.gain < -1) {
        throw std::runtime_error("camera dimensions, timing, exposure or gain is invalid");
    }
    if (config.detector.confirm_window <= 0 ||
        config.detector.confirm_hits <= 0 ||
        config.detector.confirm_hits > config.detector.confirm_window ||
        config.detector.max_missed_frames <= 0 ||
        config.detector.prediction_max_age_ms < 0) {
        throw std::runtime_error("tracking confirmation settings are inconsistent");
    }
    if (config.detector.camera_model.fx <= 0.0F ||
        config.detector.camera_model.fy <= 0.0F ||
        config.detector.camera_model.principal_x < 0.0F ||
        config.detector.camera_model.principal_x >= config.camera.width ||
        config.detector.camera_model.principal_y < 0.0F ||
        config.detector.camera_model.principal_y >= config.camera.height) {
        throw std::runtime_error("camera intrinsics are outside the configured image");
    }
    if (config.detector.sample_grid <= 0 || config.detector.ring_margin_px < 0) {
        throw std::runtime_error("sampling settings are invalid");
    }
    if (config.detector.max_green_candidates <= 0 ||
        config.detector.classical_interval_frames <= 0 ||
        config.detector.multiscale_downsample <= 0 ||
        config.detector.multiscale_downsample > 4 ||
        config.detector.multiscale_min_scan_step_px <= 0 ||
        config.detector.multiscale_tracking_min_scan_step_px <= 0 ||
        config.detector.multiscale_tracking_roi_radius_px < 0.0F ||
        config.detector.multiscale_full_refresh_interval <= 0 ||
        config.detector.capture_cone_deg <= 0.0F ||
        config.detector.capture_cone_deg >= 90.0F ||
        config.detector.min_normalized_green_response < 0.0F ||
        config.detector.min_normalized_green_response > 1.0F ||
        config.detector.min_normalized_inner_brightness < 0.0F ||
        config.detector.min_normalized_inner_brightness > 1.0F ||
        config.detector.normalized_min_peak_green < 0 ||
        config.detector.normalized_min_peak_green > 255 ||
        config.detector.lamp_min_green_margin <= 0.0F ||
        config.detector.lamp_min_green_margin >= 1.0F ||
        config.detector.lamp_min_color_fraction <= 0.0F ||
        config.detector.lamp_min_color_fraction > 1.0F ||
        config.detector.lamp_min_relative_contrast <= 0.0F ||
        config.detector.lamp_min_relative_contrast >= 1.0F ||
        config.detector.lamp_max_axis_ratio < 1.0F ||
        config.detector.lamp_max_axis_ratio > 10.0F ||
        config.detector.lamp_min_core_fill <= 0.0F ||
        config.detector.lamp_min_core_fill > 1.0F ||
        config.detector.min_normalized_contrast_z < -10.0F ||
        config.detector.min_normalized_contrast_z > 10.0F ||
        config.detector.min_tracking_normalized_contrast_z < -10.0F ||
        config.detector.min_tracking_normalized_contrast_z >
            config.detector.min_normalized_contrast_z ||
        config.detector.control_prediction_max_frames < 0 ||
        config.detector.control_prediction_max_age_ms < 0 ||
        config.detector.normalized_brightness_weight < 0.0F ||
        std::any_of(config.detector.multiscale_diameters_px.begin(),
                    config.detector.multiscale_diameters_px.end(),
                    [](int diameter) { return diameter <= 0; }) ||
        !std::is_sorted(config.detector.multiscale_diameters_px.begin(),
                        config.detector.multiscale_diameters_px.end())) {
        throw std::runtime_error("multiscale detector and control prediction settings are invalid");
    }
    if (config.detector.center_prior_radius_px <= 0.0F ||
        config.detector.initial_size_reference_px <= 0.0F ||
        config.detector.min_halo_size_px <= 0.0F ||
        config.detector.core_ring_scale < 0.0F ||
        config.detector.min_core_size_px <= 0.0F) {
        throw std::runtime_error("initial candidate prior scales must be positive");
    }
    if (config.detector.confirmation_gate_px <= 0.0F ||
        config.detector.gate_min_px <= 0.0F ||
        config.detector.gate_max_px < config.detector.gate_min_px ||
        config.detector.gate_size_factor < 0.0F ||
        config.detector.max_log_size_jump <= 0.0F ||
        config.detector.max_cross_source_log_size_jump <= 0.0F) {
        throw std::runtime_error("tracking association gates are invalid");
    }
    if (config.detector.min_candidate_score < 0.0F ||
        config.detector.min_candidate_score > 1.0F ||
        config.detector.min_tracking_score < 0.0F ||
        config.detector.min_tracking_score > 1.0F ||
        config.detector.min_local_contrast < -1.0F ||
        config.detector.min_local_contrast > 1.0F ||
        config.detector.min_tracking_local_contrast < -1.0F ||
        config.detector.min_tracking_local_contrast >
            config.detector.min_local_contrast ||
        config.detector.contrast_relax_min_association < 0.0F ||
        config.detector.contrast_relax_min_association > 1.0F ||
        config.detector.min_density < 0.0F ||
        config.detector.min_density > 1.0F ||
        config.detector.min_core_brightness < 0.0F ||
        config.detector.min_core_brightness > 1.0F ||
        config.detector.min_core_ring_green_dominance < -1.0F ||
        config.detector.min_core_ring_green_dominance > 1.0F ||
        config.detector.min_core_ring_green_fraction < 0.0F ||
        config.detector.min_core_ring_green_fraction > 1.0F ||
        config.detector.min_association_score < 0.0F ||
        config.detector.min_association_score > 1.0F) {
        throw std::runtime_error("detector score, contrast, and density limits are invalid");
    }
    if (config.debug.json_log_every_n_frames < 0 ||
        config.debug.save_every_n_frames < 0 ||
        config.debug.max_saved_frames < 0) {
        throw std::runtime_error("debug frame limits must not be negative");
    }
    if (config.armor.min_component_pixels <= 0 ||
        config.armor.max_component_pixels < config.armor.min_component_pixels ||
        config.armor.min_color_response < -1.0F ||
        config.armor.min_color_response > 1.0F ||
        config.armor.min_brightness < 0.0F ||
        config.armor.min_brightness > 1.0F ||
        config.armor.min_green_size_px <= 0.0F ||
        config.armor.min_bar_length_px <= 0.0F ||
        config.armor.max_bar_length_px < config.armor.min_bar_length_px ||
        config.armor.min_elongation < 1.0F ||
        config.armor.max_pair_angle_deg <= 0.0F ||
        config.armor.max_pair_angle_deg >= 90.0F ||
        config.armor.max_pair_longitudinal_to_length < 0.0F ||
        config.armor.min_length_ratio <= 0.0F ||
        config.armor.min_length_ratio > 1.0F ||
        config.armor.max_color_response_diff <= 0.0F ||
        config.armor.min_separation_to_length <= 0.0F ||
        config.armor.max_separation_to_length <
            config.armor.min_separation_to_length ||
        config.armor.max_separation_to_green_size < 0.0F ||
        config.armor.min_green_offset_to_length < 0.0F ||
        config.armor.max_green_offset_to_length <
            config.armor.min_green_offset_to_length ||
        config.armor.max_green_lateral_to_separation < 0.0F ||
        config.armor.min_geometry_confidence < 0.0F ||
        config.armor.min_geometry_confidence > 1.0F ||
        config.armor.required_pose_hits <= 0 || config.armor.aim_blend_ms < 0 ||
        config.armor.confirmation_hits <= 0 || config.armor.confirmation_hits > 100 ||
        config.armor.confirmation_max_gap_ms <= 0 || config.armor.confirmation_max_gap_ms > 1000 ||
        config.armor.cache_max_age_ms < 0) {
        throw std::runtime_error("armor geometry settings are invalid");
    }
    if (config.target_geometry.max_reprojection_error_px <= 0.0F ||
        config.target_geometry.min_pose_separation_px <= 0.0F ||
        (config.target_geometry.pose_enabled &&
         (config.target_geometry.bar_separation_m <= 0.0F ||
         config.target_geometry.bar_length_m <= 0.0F ||
         config.target_geometry.green_offset_m <= 0.0F))) {
        throw std::runtime_error("pose geometry dimensions must be measured before pose is enabled");
    }
    if ((config.npu.input_size != 192 && config.npu.input_size != 256 &&
         config.npu.input_size != 320) ||
        (config.npu.required && !config.npu.enabled) ||
        config.npu.interval_ms <= 0 ||
        config.npu.search_candidates <= 0 ||
        config.npu.confidence_threshold < 0.0F ||
        config.npu.confidence_threshold > 1.0F ||
        config.npu.keypoint_threshold < 0.0F ||
        config.npu.keypoint_threshold > 1.0F ||
        config.npu.min_roi_size_px <= 0 ||
        config.npu.max_roi_size_px < config.npu.min_roi_size_px ||
        config.npu.roi_size_factor <= 0.0F) {
        throw std::runtime_error("NPU validator settings are invalid");
    }
    if (config.visual_motion.interval_frames <= 0 ||
        config.visual_motion.grid_width < 16 ||
        config.visual_motion.grid_height < 16 ||
        config.visual_motion.max_shift_px < 0 ||
        config.visual_motion.max_rotation_deg < 0.0F ||
        config.visual_motion.rotation_step_deg <= 0.0F ||
        config.visual_motion.min_response < 0.0F ||
        config.visual_motion.min_response > 1.0F) {
        throw std::runtime_error("visual motion settings are invalid");
    }
    const float weight_sum = config.detector.weight_color +
                             config.detector.weight_contrast +
                             config.detector.weight_density +
                             config.detector.weight_shape +
                             config.detector.weight_core +
                             config.detector.weight_temporal +
                             config.detector.weight_center_prior +
                             config.detector.weight_initial_size +
                             config.detector.weight_core_size;
    if (weight_sum <= 0.0F || config.detector.weight_color < 0.0F ||
        config.detector.weight_contrast < 0.0F ||
        config.detector.weight_density < 0.0F ||
        config.detector.weight_shape < 0.0F ||
        config.detector.weight_core < 0.0F ||
        config.detector.weight_temporal < 0.0F ||
        config.detector.weight_center_prior < 0.0F ||
        config.detector.weight_initial_size < 0.0F ||
        config.detector.weight_core_size < 0.0F) {
        throw std::runtime_error("candidate score weights must be non-negative and not all zero");
    }
}

}  // namespace

ApplicationConfig load_application_config(const std::string &path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("cannot open configuration file: " + path);
    }

    ApplicationConfig config;
    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        const auto comment = line.find('#');
        if (comment != std::string::npos) {
            line.erase(comment);
        }
        line = trim(line);
        if (line.empty()) {
            continue;
        }

        const auto separator = line.find('=');
        if (separator == std::string::npos) {
            throw std::runtime_error("missing '=' at " + path + ":" +
                                     std::to_string(line_number));
        }
        const std::string key = trim(line.substr(0, separator));
        const std::string value = trim(line.substr(separator + 1));
        if (key.empty() || value.empty()) {
            throw std::runtime_error("empty key or value at " + path + ":" +
                                     std::to_string(line_number));
        }

        if (key == "camera.width") config.camera.width = parse_int(key, value);
        else if (key == "camera.height") config.camera.height = parse_int(key, value);
        else if (key == "camera.fps") config.camera.fps = parse_int(key, value);
        else if (key == "camera.buffer_count") config.camera.buffer_count = parse_int(key, value);
        else if (key == "camera.warmup_frames") config.camera.warmup_frames = parse_int(key, value);
        else if (key == "camera.exposure_us") config.camera.exposure_us = parse_int(key, value);
        else if (key == "camera.gain") config.camera.gain = parse_int(key, value);
        else if (key == "camera.manual_white_balance") config.camera.manual_white_balance = parse_bool(key, value);
        else if (key == "camera.white_balance_gain") config.camera.white_balance_gain = parse_four_floats(key, value);
        else if (key == "lab.core") config.detector.core_lab = parse_lab(key, value);
        else if (key == "lab.halo") config.detector.halo_lab = parse_lab(key, value);
        else if (key == "detector.min_candidate_score") config.detector.min_candidate_score = parse_float(key, value);
        else if (key == "detector.min_tracking_score") config.detector.min_tracking_score = parse_float(key, value);
        else if (key == "detector.min_density") config.detector.min_density = parse_float(key, value);
        else if (key == "detector.min_green_dominance") config.detector.min_green_dominance = parse_float(key, value);
        else if (key == "detector.min_local_contrast") config.detector.min_local_contrast = parse_float(key, value);
        else if (key == "detector.min_tracking_local_contrast") config.detector.min_tracking_local_contrast = parse_float(key, value);
        else if (key == "detector.contrast_relax_min_association") config.detector.contrast_relax_min_association = parse_float(key, value);
        else if (key == "detector.min_halo_size_px") config.detector.min_halo_size_px = parse_float(key, value);
        else if (key == "detector.core_center_max_fraction") config.detector.core_center_max_fraction = parse_float(key, value);
        else if (key == "detector.center_prior_radius_px") config.detector.center_prior_radius_px = parse_float(key, value);
        else if (key == "detector.initial_size_reference_px") config.detector.initial_size_reference_px = parse_float(key, value);
        else if (key == "detector.enable_saturated_core_candidates") config.detector.enable_saturated_core_candidates = parse_bool(key, value);
        else if (key == "detector.min_core_brightness") config.detector.min_core_brightness = parse_float(key, value);
        else if (key == "detector.min_core_ring_green_dominance") config.detector.min_core_ring_green_dominance = parse_float(key, value);
        else if (key == "detector.min_core_ring_green_fraction") config.detector.min_core_ring_green_fraction = parse_float(key, value);
        else if (key == "detector.core_ring_scale") config.detector.core_ring_scale = parse_float(key, value);
        else if (key == "detector.min_core_size_px") config.detector.min_core_size_px = parse_float(key, value);
        else if (key == "score.weight_color") config.detector.weight_color = parse_float(key, value);
        else if (key == "score.weight_contrast") config.detector.weight_contrast = parse_float(key, value);
        else if (key == "score.weight_density") config.detector.weight_density = parse_float(key, value);
        else if (key == "score.weight_shape") config.detector.weight_shape = parse_float(key, value);
        else if (key == "score.weight_core") config.detector.weight_core = parse_float(key, value);
        else if (key == "score.weight_temporal") config.detector.weight_temporal = parse_float(key, value);
        else if (key == "score.weight_center_prior") config.detector.weight_center_prior = parse_float(key, value);
        else if (key == "score.weight_initial_size") config.detector.weight_initial_size = parse_float(key, value);
        else if (key == "score.weight_core_size") config.detector.weight_core_size = parse_float(key, value);
        else if (key == "tracking.confirm_window") config.detector.confirm_window = parse_int(key, value);
        else if (key == "tracking.confirm_hits") config.detector.confirm_hits = parse_int(key, value);
        else if (key == "tracking.max_missed_frames") config.detector.max_missed_frames = parse_int(key, value);
        else if (key == "tracking.prediction_max_age_ms") config.detector.prediction_max_age_ms = parse_int(key, value);
        else if (key == "tracking.confirmation_gate_px") config.detector.confirmation_gate_px = parse_float(key, value);
        else if (key == "tracking.gate_min_px") config.detector.gate_min_px = parse_float(key, value);
        else if (key == "tracking.gate_max_px") config.detector.gate_max_px = parse_float(key, value);
        else if (key == "tracking.gate_size_factor") config.detector.gate_size_factor = parse_float(key, value);
        else if (key == "tracking.max_log_size_jump") config.detector.max_log_size_jump = parse_float(key, value);
        else if (key == "tracking.max_cross_source_log_size_jump") config.detector.max_cross_source_log_size_jump = parse_float(key, value);
        else if (key == "tracking.min_association_score") config.detector.min_association_score = parse_float(key, value);
        else if (key == "kalman.process_position") config.detector.process_noise_position = parse_float(key, value);
        else if (key == "kalman.process_velocity") config.detector.process_noise_velocity = parse_float(key, value);
        else if (key == "kalman.process_log_size") config.detector.process_noise_log_size = parse_float(key, value);
        else if (key == "kalman.process_size_rate") config.detector.process_noise_size_rate = parse_float(key, value);
        else if (key == "kalman.measurement_position") config.detector.measurement_noise_position = parse_float(key, value);
        else if (key == "kalman.measurement_log_size") config.detector.measurement_noise_log_size = parse_float(key, value);
        else if (key == "sampling.grid") config.detector.sample_grid = parse_int(key, value);
        else if (key == "sampling.ring_margin_px") config.detector.ring_margin_px = parse_int(key, value);
        else if (key == "sampling.merge_margin_px") config.detector.merge_margin_px = parse_int(key, value);
        else if (key == "sampling.merge_blobs") config.detector.merge_blobs = parse_bool(key, value);
        else if (key == "multiscale.enabled") config.detector.enable_normalized_multiscale = parse_bool(key, value);
        else if (key == "performance.legacy_lab_candidates_enabled") config.detector.enable_legacy_lab_candidates = parse_bool(key, value);
        else if (key == "performance.sparse_component_search_enabled") config.detector.enable_sparse_component_search = parse_bool(key, value);
        else if (key == "multiscale.diameters_px") config.detector.multiscale_diameters_px = parse_five_ints(key, value);
        else if (key == "performance.classical_interval_frames") config.detector.classical_interval_frames = parse_int(key, value);
        else if (key == "performance.multiscale_downsample") config.detector.multiscale_downsample = parse_int(key, value);
        else if (key == "performance.multiscale_min_scan_step_px") config.detector.multiscale_min_scan_step_px = parse_int(key, value);
        else if (key == "performance.multiscale_tracking_min_scan_step_px") config.detector.multiscale_tracking_min_scan_step_px = parse_int(key, value);
        else if (key == "performance.multiscale_capture_cone_only") config.detector.multiscale_capture_cone_only = parse_bool(key, value);
        else if (key == "performance.multiscale_tracking_roi_radius_px") config.detector.multiscale_tracking_roi_radius_px = parse_float(key, value);
        else if (key == "performance.multiscale_full_refresh_interval") config.detector.multiscale_full_refresh_interval = parse_int(key, value);
        else if (key == "multiscale.min_green_response") config.detector.min_normalized_green_response = parse_float(key, value);
        else if (key == "multiscale.min_inner_brightness") config.detector.min_normalized_inner_brightness = parse_float(key, value);
        else if (key == "multiscale.min_peak_green") config.detector.normalized_min_peak_green = parse_int(key, value);
        else if (key == "appearance.enabled") config.detector.enable_lamp_appearance = parse_bool(key, value);
        else if (key == "appearance.min_green_margin") config.detector.lamp_min_green_margin = parse_float(key, value);
        else if (key == "appearance.min_color_fraction") config.detector.lamp_min_color_fraction = parse_float(key, value);
        else if (key == "appearance.min_relative_contrast") config.detector.lamp_min_relative_contrast = parse_float(key, value);
        else if (key == "appearance.max_axis_ratio") config.detector.lamp_max_axis_ratio = parse_float(key, value);
        else if (key == "appearance.min_core_fill") config.detector.lamp_min_core_fill = parse_float(key, value);
        else if (key == "multiscale.min_contrast_z") config.detector.min_normalized_contrast_z = parse_float(key, value);
        else if (key == "multiscale.min_tracking_contrast_z") config.detector.min_tracking_normalized_contrast_z = parse_float(key, value);
        else if (key == "multiscale.brightness_weight") config.detector.normalized_brightness_weight = parse_float(key, value);
        else if (key == "multiscale.max_candidates") config.detector.max_green_candidates = parse_int(key, value);
        else if (key == "capture.enabled") config.detector.enable_capture_cone = parse_bool(key, value);
        else if (key == "capture.cone_deg") config.detector.capture_cone_deg = parse_float(key, value);
        else if (key == "control.prediction_max_frames") config.detector.control_prediction_max_frames = parse_int(key, value);
        else if (key == "control.prediction_max_age_ms") config.detector.control_prediction_max_age_ms = parse_int(key, value);
        else if (key == "armor.expected_color") config.armor.expected_color = parse_armor_color(key, value);
        else if (key == "armor.min_color_response") config.armor.min_color_response = parse_float(key, value);
        else if (key == "armor.min_brightness") config.armor.min_brightness = parse_float(key, value);
        else if (key == "armor.min_green_size_px") config.armor.min_green_size_px = parse_float(key, value);
        else if (key == "armor.min_component_pixels") config.armor.min_component_pixels = parse_int(key, value);
        else if (key == "armor.max_component_pixels") config.armor.max_component_pixels = parse_int(key, value);
        else if (key == "armor.min_bar_length_px") config.armor.min_bar_length_px = parse_float(key, value);
        else if (key == "armor.max_bar_length_px") config.armor.max_bar_length_px = parse_float(key, value);
        else if (key == "armor.min_elongation") config.armor.min_elongation = parse_float(key, value);
        else if (key == "armor.max_pair_angle_deg") config.armor.max_pair_angle_deg = parse_float(key, value);
        else if (key == "armor.max_pair_longitudinal_to_length") config.armor.max_pair_longitudinal_to_length = parse_float(key, value);
        else if (key == "armor.min_length_ratio") config.armor.min_length_ratio = parse_float(key, value);
        else if (key == "armor.max_color_response_diff") config.armor.max_color_response_diff = parse_float(key, value);
        else if (key == "armor.min_separation_to_length") config.armor.min_separation_to_length = parse_float(key, value);
        else if (key == "armor.max_separation_to_length") config.armor.max_separation_to_length = parse_float(key, value);
        else if (key == "armor.max_separation_to_green_size") config.armor.max_separation_to_green_size = parse_float(key, value);
        else if (key == "armor.min_green_offset_to_length") config.armor.min_green_offset_to_length = parse_float(key, value);
        else if (key == "armor.max_green_offset_to_length") config.armor.max_green_offset_to_length = parse_float(key, value);
        else if (key == "armor.max_green_lateral_to_separation") config.armor.max_green_lateral_to_separation = parse_float(key, value);
        else if (key == "armor.min_geometry_confidence") config.armor.min_geometry_confidence = parse_float(key, value);
        else if (key == "armor.required_pose_hits") config.armor.required_pose_hits = parse_int(key, value);
        else if (key == "armor.confirmation_hits") config.armor.confirmation_hits = parse_int(key, value);
        else if (key == "armor.confirmation_max_gap_ms") config.armor.confirmation_max_gap_ms = parse_int(key, value);
        else if (key == "armor.aim_blend_ms") config.armor.aim_blend_ms = parse_int(key, value);
        else if (key == "armor.cache_max_age_ms") config.armor.cache_max_age_ms = parse_int(key, value);
        else if (key == "target_geometry.pose_enabled") config.target_geometry.pose_enabled = parse_bool(key, value);
        else if (key == "target_geometry.bar_separation_m") config.target_geometry.bar_separation_m = parse_float(key, value);
        else if (key == "target_geometry.bar_length_m") config.target_geometry.bar_length_m = parse_float(key, value);
        else if (key == "target_geometry.green_offset_m") config.target_geometry.green_offset_m = parse_float(key, value);
        else if (key == "target_geometry.max_reprojection_error_px") config.target_geometry.max_reprojection_error_px = parse_float(key, value);
        else if (key == "target_geometry.min_pose_separation_px") config.target_geometry.min_pose_separation_px = parse_float(key, value);
        else if (key == "npu.enabled") config.npu.enabled = parse_bool(key, value);
        else if (key == "npu.required") config.npu.required = parse_bool(key, value);
        else if (key == "npu.model_path") config.npu.model_path = value;
        else if (key == "npu.input_size") config.npu.input_size = parse_int(key, value);
        else if (key == "npu.interval_ms") config.npu.interval_ms = parse_int(key, value);
        else if (key == "npu.search_candidates") config.npu.search_candidates = parse_int(key, value);
        else if (key == "npu.confidence_threshold") config.npu.confidence_threshold = parse_float(key, value);
        else if (key == "npu.keypoint_threshold") config.npu.keypoint_threshold = parse_float(key, value);
        else if (key == "npu.min_roi_size_px") config.npu.min_roi_size_px = parse_int(key, value);
        else if (key == "npu.max_roi_size_px") config.npu.max_roi_size_px = parse_int(key, value);
        else if (key == "npu.roi_size_factor") config.npu.roi_size_factor = parse_float(key, value);
        else if (key == "highfps.green_hz") config.highfps.green_hz = parse_int(key, value);
        else if (key == "highfps.armor_hz") config.highfps.armor_hz = parse_int(key, value);
        else if (key == "highfps.search_hz") config.highfps.search_hz = parse_int(key, value);
        else if (key == "highfps.motion_hz") config.highfps.motion_hz = parse_int(key, value);
        else if (key == "visual_motion.enabled") config.visual_motion.enabled = parse_bool(key, value);
        else if (key == "visual_motion.tracking_only") config.visual_motion.tracking_only = parse_bool(key, value);
        else if (key == "visual_motion.interval_frames") config.visual_motion.interval_frames = parse_int(key, value);
        else if (key == "visual_motion.grid_width") config.visual_motion.grid_width = parse_int(key, value);
        else if (key == "visual_motion.grid_height") config.visual_motion.grid_height = parse_int(key, value);
        else if (key == "visual_motion.max_shift_px") config.visual_motion.max_shift_px = parse_int(key, value);
        else if (key == "visual_motion.max_rotation_deg") config.visual_motion.max_rotation_deg = parse_float(key, value);
        else if (key == "visual_motion.rotation_step_deg") config.visual_motion.rotation_step_deg = parse_float(key, value);
        else if (key == "visual_motion.min_response") config.visual_motion.min_response = parse_float(key, value);
        else if (key == "calibration.fx") config.detector.camera_model.fx = parse_float(key, value);
        else if (key == "calibration.fy") config.detector.camera_model.fy = parse_float(key, value);
        else if (key == "calibration.principal_x") config.detector.camera_model.principal_x = parse_float(key, value);
        else if (key == "calibration.principal_y") config.detector.camera_model.principal_y = parse_float(key, value);
        else if (key == "calibration.k1") config.detector.camera_model.k1 = parse_float(key, value);
        else if (key == "calibration.k2") config.detector.camera_model.k2 = parse_float(key, value);
        else if (key == "calibration.p1") config.detector.camera_model.p1 = parse_float(key, value);
        else if (key == "calibration.p2") config.detector.camera_model.p2 = parse_float(key, value);
        else if (key == "calibration.k3") config.detector.camera_model.k3 = parse_float(key, value);
        else if (key == "debug.enabled") config.debug.enabled = parse_bool(key, value);
        else if (key == "debug.directory") config.debug.directory = value;
        else if (key == "debug.json_log_every_n_frames") config.debug.json_log_every_n_frames = parse_int(key, value);
        else if (key == "debug.log_candidates") config.debug.log_candidates = parse_bool(key, value);
        else if (key == "debug.save_every_n_frames") config.debug.save_every_n_frames = parse_int(key, value);
        else if (key == "debug.save_on_state_change") config.debug.save_on_state_change = parse_bool(key, value);
        else if (key == "debug.save_failed_frames") config.debug.save_failed_frames = parse_bool(key, value);
        else if (key == "debug.max_saved_frames") config.debug.max_saved_frames = parse_int(key, value);
        else {
            throw std::runtime_error("unknown configuration key at " + path + ":" +
                                     std::to_string(line_number) + ": " + key);
        }
    }

    validate(config);
    return config;
}

}  // namespace dart
