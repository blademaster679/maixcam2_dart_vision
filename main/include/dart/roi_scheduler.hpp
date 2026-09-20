#pragma once
#include "dart/nv21_pipeline.hpp"
#include <algorithm>
#include <cmath>

namespace dart {
enum class RoiMode { Searching, Tentative, Tracking, Coasting };
struct RoiSelection {
    CandidateRoi roi;
    RoiMode mode = RoiMode::Searching;
    bool reset_tracker = false;
};

// Keep observations of the same tentative target together without weakening
// the tracker's confirmation or miss rules. All budgets count real detections.
class RoiScheduler {
public:
    explicit RoiScheduler(const DetectorConfig &config, int measurement_hz=90): config_(config) {
        // Acquisition needs several real frames even when prediction output
        // is disabled. Its finite lease must not inherit a zero coast budget.
        tentative_limit_us_ = std::max(
            static_cast<uint64_t>(std::max(0, config.prediction_max_age_ms))*2000U,
            static_cast<uint64_t>(std::max(2, 2*config.confirm_window))*1000000U /
                static_cast<uint64_t>(std::max(1, measurement_hz)));
    }

    bool needs_global_search(uint64_t now, const detail::TemporalTracker &tracker) const {
        return tracker.has_prediction() && !usable(now, tracker);
    }

    RoiSelection select(const std::vector<Point2f> &proposals,
                        const detail::TemporalTracker &tracker,
                        uint64_t now, int width, int height) {
        RoiSelection result;
        Point2f center{width / 2.0F, height / 2.0F, true};
        int size = 96;
        if (usable(now, tracker)) {
            center = {tracker.predicted_x(), tracker.predicted_y(), true};
            const bool confirmed = tracker.tracking_confirmed();
            result.mode = confirmed
                ? (tracker.missed_frames() ? RoiMode::Coasting : RoiMode::Tracking)
                : RoiMode::Tentative;
            // First confirm the lamp with its complete body and surrounding
            // background. Searching the whole board before confirmation makes
            // large candidates expensive enough to miss the next association.
            // Once confirmed, retain the full board coverage for paired bars.
            const float scale = confirmed ? 12.0F : 3.0F;
            size = static_cast<int>(std::clamp(tracker.predicted_size() * scale,
                                              confirmed ? 128.0F : 96.0F, 384.0F));
        } else {
            result.reset_tracker = tracker.has_prediction();
            tentative_ = false;
            attempts_ = 0;
            auto unseen = [this](const Point2f &p) {
                if (!p.valid || !std::isfinite(p.x) || !std::isfinite(p.y)) return false;
                return std::none_of(visited_.begin(), visited_.end(), [&p](const Point2f &v) {
                    // Half the proposal extractor's 48-pixel suppression radius.
                    return std::hypot(v.x-p.x, v.y-p.y) < 24.0F;
                });
            };
            auto next = std::find_if(proposals.begin(), proposals.end(), unseen);
            if (next == proposals.end()) {
                visited_.clear();
                next = std::find_if(proposals.begin(), proposals.end(), unseen);
            }
            if (next != proposals.end()) {
                center = *next;
                visited_.push_back(center);
                if (visited_.size() > 32) visited_.erase(visited_.begin());
            }
        }
        result.roi = source_roi(center.x, center.y, size, width, height);
        return result;
    }

    void observe(const GreenLightDetection &green, uint64_t now, bool measurement_ran=true) {
        if (!measurement_ran) return;
        if (green.state == TrackState::Tracking || green.state == TrackState::Lost) {
            tentative_ = false;
            attempts_ = 0;
        } else {
            if (!tentative_) { tentative_ = true; started_ = now; attempts_ = 0; }
            ++attempts_;
        }
    }

private:
    bool usable(uint64_t now, const detail::TemporalTracker &tracker) const {
        if (!tracker.has_prediction() ||
            tracker.missed_frames() >= config_.max_missed_frames)
            return false;
        if (tracker.tracking_confirmed()) {
            const uint64_t age_limit = config_.prediction_max_age_ms == 0 &&
                    tracker.missed_frames() == 0 ? tentative_limit_us_ :
                    static_cast<uint64_t>(std::max(0, config_.prediction_max_age_ms))*1000U;
            return tracker.measurement_age_us(now) <= age_limit;
        }
        if (tracker.measurement_age_us(now) > tentative_limit_us_) return false;
        return !tentative_ || (now >= started_ &&
            now-started_ <= tentative_limit_us_ &&
            attempts_ < std::max(2, 2*config_.confirm_window));
    }
    DetectorConfig config_;
    bool tentative_ = false;
    uint64_t started_ = 0;
    uint64_t tentative_limit_us_ = 0;
    int attempts_ = 0;
    std::vector<Point2f> visited_;
};
} // namespace dart
