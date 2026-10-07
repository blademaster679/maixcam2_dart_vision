#include "dart/green_detector.hpp"

#include "maix_basic.hpp"
#include "maix_nn_yolo11.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace dart {
namespace {

class YoloPoseValidator final : public TargetPoseValidator {
public:
    explicit YoloPoseValidator(const NpuConfig &config)
        : config_(config), detector_(config.model_path, false)
    {
        if (detector_.type() != "pose") {
            throw std::runtime_error("NPU model metadata type must be pose");
        }
        if (detector_.input_width() != config_.input_size ||
            detector_.input_height() != config_.input_size) {
            throw std::runtime_error(
                "NPU model input size does not match npu.input_size");
        }
        if (detector_.labels.size() != 1U ||
            detector_.labels.front() != "dart_target") {
            throw std::runtime_error(
                "NPU model must contain only the dart_target class");
        }
    }

    PoseValidation validate(maix::image::Image &frame,
                            const CandidateRoi &roi) override
    {
        PoseValidation result;
        if (roi.width <= 0 || roi.height <= 0) {
            return result;
        }
        std::unique_ptr<maix::image::Image> crop(
            frame.crop(roi.x, roi.y, roi.width, roi.height));
        if (!crop) {
            return result;
        }
        double gain = 1.0;
        int left = 0, top = 0;
        std::unique_ptr<maix::image::Image> prepared;
        if (config_.center_only) {
            // Match Ultralytics letterbox explicitly. SDK FIT_CONTAIN defaults
            // to nearest-neighbor and can fail when padding is only one pixel.
            gain = std::min(static_cast<double>(config_.input_size) / roi.width,
                            static_cast<double>(config_.input_size) / roi.height);
            const int width = static_cast<int>(std::nearbyint(roi.width * gain));
            const int height = static_cast<int>(std::nearbyint(roi.height * gain));
            left = static_cast<int>(std::lround((config_.input_size - width) / 2.0 - 0.1));
            top = static_cast<int>(std::lround((config_.input_size - height) / 2.0 - 0.1));
            std::unique_ptr<maix::image::Image> resized(crop->resize(
                width, height, maix::image::FIT_FILL, maix::image::ResizeMethod::BILINEAR));
            prepared = std::make_unique<maix::image::Image>(
                config_.input_size, config_.input_size, maix::image::FMT_RGB888,
                maix::image::Color::from_rgb(114, 114, 114));
            prepared->draw_image(left, top, *resized);
        }
        std::unique_ptr<maix::nn::Objects> objects(detector_.detect(
            prepared ? *prepared : *crop, config_.confidence_threshold, 0.45F,
            prepared ? maix::image::FIT_FILL : maix::image::FIT_CONTAIN,
            config_.keypoint_threshold, 0));
        if (!objects || objects->size() == 0U) {
            return result;
        }
        const maix::nn::Object *best = nullptr;
        for (const auto *object : *objects) {
            if (object != nullptr &&
                (best == nullptr || object->score > best->score)) {
                best = object;
            }
        }
        if (best == nullptr || best->points.size() != 10U) {
            return result;
        }
        result.confidence = best->score;
        for (std::size_t index = 0; index < result.keypoints.size(); ++index) {
            if (config_.center_only && index > 0) {
                continue;
            }
            const int x = best->points[2 * index];
            const int y = best->points[2 * index + 1];
            result.keypoints[index].valid = x >= 0 && y >= 0;
            if (result.keypoints[index].valid) {
                const double local_x = (x - left) / gain;
                const double local_y = (y - top) / gain;
                result.keypoints[index].valid = local_x >= 0 && local_y >= 0 &&
                    local_x < roi.width && local_y < roi.height;
                result.keypoints[index].x = static_cast<float>(roi.x + local_x);
                result.keypoints[index].y = static_cast<float>(roi.y + local_y);
            }
        }
        result.valid = result.keypoints[0].valid &&
                       result.confidence >= config_.confidence_threshold;
        return result;
    }

private:
    NpuConfig config_;
    maix::nn::YOLO11 detector_;
};

}  // namespace

std::shared_ptr<TargetPoseValidator>
create_yolo_pose_validator(const NpuConfig &config)
{
    if (!config.enabled) {
        return nullptr;
    }
    try {
        return std::make_shared<YoloPoseValidator>(config);
    } catch (const std::exception &exception) {
        if (config.required) {
            throw;
        }
        maix::log::warn("NPU pose validator disabled: %s", exception.what());
        return nullptr;
    }
}

}  // namespace dart
