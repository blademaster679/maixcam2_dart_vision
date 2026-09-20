#pragma once
#include "dart/async_frame.hpp"
#include "dart/green_detector.hpp"
#include <atomic>
#include <thread>
#include <fstream>

namespace dart {
struct Nv21View {
    const uint8_t *y=nullptr, *vu=nullptr;
    int width=0, height=0, y_stride=0, vu_stride=0;
    void validate() const;
};
// Lease owns DMA storage and mappings until the last reader finishes.
class Nv21Frame {
public:
    FrameMetadata metadata;
    virtual Nv21View map() = 0;
    virtual ~Nv21Frame() = default;
};
CandidateRoi source_roi(float x, float y, int size, int width, int height);
std::vector<Point2f> nv21_green_proposals(const Nv21View &view);
std::unique_ptr<maix::image::Image> nv21_rgb_region(const Nv21View &view,
                                                  const CandidateRoi &roi, int step=1);
void invalidate_uncalibrated(TargetEstimate &target);
TargetEstimate predict_full180_output(const TargetEstimate &source,
    detail::TemporalTracker tracker, const ApplicationConfig &config, uint64_t now);
enum class PipelineInputSource { Vin, CachedVideo };
class HighFpsPipeline {
public:
    explicit HighFpsPipeline(const ApplicationConfig &config, bool idle=false, bool stress=false,
                             PipelineInputSource source=PipelineInputSource::Vin);
    ~HighFpsPipeline();
    void submit(std::shared_ptr<Nv21Frame> frame);
    void finish();
    bool failed() const { return failed_; }
private:
    struct Snapshot {
        TargetEstimate target;
        detail::TemporalTracker tracker;
        explicit Snapshot(const DetectorConfig &c):tracker(c){}
    };
    ApplicationConfig config_;
    bool idle_, stress_;
    PipelineInputSource input_source_;
    LatestFrameSlot<Nv21Frame> frames_;
    LatestFrameSlot<Snapshot> estimates_;
    std::atomic<bool> stopped_{false}, failed_{false};
    struct MotionFrame { std::shared_ptr<maix::image::Image> image; uint64_t timestamp=0; };
    LatestFrameSlot<MotionFrame> motion_frames_;
    LatestFrameSlot<MotionPrior> motion_results_;
    std::thread vision_, control_, motion_;
    std::mutex stats_mutex_;
    SequenceStats sequence_;
    uint64_t first_received_=0, last_received_=0;
    uint64_t vision_count_=0, green_count_=0, armor_count_=0, output_count_=0;
    std::atomic<uint64_t> scheduled_skipped_{0};
    bool finished_=false;
    void vision_loop();
    void control_loop();
    void motion_loop();
};
} // namespace dart
