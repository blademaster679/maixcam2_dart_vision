#include "dart/roi_scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace {

int failures = 0;
constexpr int width = 1344;
constexpr int height = 760;

void check(bool condition, const std::string &message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

bool contains(const dart::CandidateRoi &roi, const dart::Point2f &point)
{
    return point.x >= roi.x && point.x < roi.x + roi.width &&
           point.y >= roi.y && point.y < roi.y + roi.height;
}

bool bounded(const dart::CandidateRoi &roi)
{
    return roi.x >= 0 && roi.y >= 0 && roi.width > 0 && roi.height > 0 &&
           roi.x + roi.width <= width && roi.y + roi.height <= height;
}

dart::detail::CandidateObservation observation(dart::Point2f point, float size = 6.0F)
{
    dart::detail::CandidateObservation result;
    result.center_x = point.x;
    result.center_y = point.y;
    result.bbox_x = static_cast<int>(point.x - size / 2);
    result.bbox_y = static_cast<int>(point.y - size / 2);
    result.bbox_w = result.bbox_h = std::max(1, static_cast<int>(size));
    result.apparent_size = size;
    result.score = result.association_score = 0.9F;
    return result;
}

std::vector<dart::Point2f> proposals(int count)
{
    const std::vector<dart::Point2f> points = {
        {80, 90, true}, {390, 90, true}, {700, 90, true}, {1120, 90, true},
        {80, 580, true}, {390, 580, true}, {700, 580, true}, {1120, 580, true},
    };
    return {points.begin(), points.begin() + count};
}

struct Fixture {
    dart::DetectorConfig config;
    dart::detail::TemporalTracker tracker;
    dart::RoiScheduler scheduler;
    uint64_t now = 1000000;

    explicit Fixture(dart::DetectorConfig settings = {}, int measurement_hz = 90)
        : config(settings), tracker(settings), scheduler(settings, measurement_hz) {}

    dart::RoiSelection select(const std::vector<dart::Point2f> &points)
    {
        tracker.predict(now);
        const auto result = scheduler.select(points, tracker, now, width, height);
        check(bounded(result.roi), "all selected ROIs stay inside the source frame");
        if (result.reset_tracker) tracker.reset();
        return result;
    }

    dart::GreenLightDetection measure(const dart::detail::CandidateObservation *value,
                                      bool measurement_ran = true)
    {
        const auto result = tracker.update(value, now, config.camera_model, measurement_ran);
        scheduler.observe(result, now, measurement_ran);
        return result;
    }

    void acquire(dart::Point2f point)
    {
        auto candidate = observation(point);
        for (int frame = 0; frame < config.confirm_hits; ++frame) {
            const auto selected = select({point});
            check(contains(selected.roi, point), "confirmation ROI includes its measurement");
            measure(&candidate);
            now += 10000;
        }
        check(tracker.tracking_confirmed(), "fixture acquires a real TemporalTracker track");
    }
};

void test_round_robin_regression_and_proposal_order()
{
    // Reproduce the integration failure using the real confirmation rule:
    // one eligible measurement every five calls never gives three of five.
    dart::DetectorConfig config;
    dart::detail::TemporalTracker old_tracker(config);
    const auto old_candidate = observation(proposals(5).back());
    bool old_confirmed = false;
    for (int frame = 0; frame < 50; ++frame) {
        old_confirmed |= old_tracker.update(frame % 5 == 4 ? &old_candidate : nullptr,
            1000000 + frame * 10000, config.camera_model).valid;
    }
    check(!old_confirmed, "legacy five-way rotation reproduces confirmation starvation");

    for (const int count : {1, 3, 5, 8}) {
        for (int target_index = 0; target_index < count; ++target_index) {
            Fixture fixture;
            const auto points = proposals(count);
            const auto target = points[target_index];
            auto candidate = observation(target);
            bool confirmed = false;
            for (int frame = 0; frame < 160; ++frame) {
                auto changed_order = points;
                std::rotate(changed_order.begin(), changed_order.begin() + frame % count,
                            changed_order.end());
                if (frame % 2) std::reverse(changed_order.begin(), changed_order.end());
                const auto selected = fixture.select(changed_order);
                const auto result = fixture.measure(contains(selected.roi, target) ? &candidate : nullptr);
                fixture.now += 10000;
                if (result.valid && !result.predicted) {
                    confirmed = true;
                    break;
                }
            }
            check(confirmed, "target confirms across proposal counts/order/positions: count=" +
                             std::to_string(count) + " target=" + std::to_string(target_index));
        }
    }
}

void test_interrupted_confirmation_and_refresh_without_target_proposal()
{
    Fixture fixture;
    const auto points = proposals(5);
    const auto target = points.front();
    auto candidate = observation(target);
    for (int frame = 0; frame < 5; ++frame) {
        // A weak coarse proposal can disappear while the full RGB detector
        // still sees the lamp; this must not erase its tentative association.
        std::vector<dart::Point2f> refreshed = frame == 0
            ? points : std::vector<dart::Point2f>(points.begin() + 1, points.end());
        const auto selected = fixture.select(refreshed);
        check(contains(selected.roi, target), "tentative ROI survives proposal refresh and brief misses");
        const bool hit = frame % 2 == 0;
        const auto result = fixture.measure(hit ? &candidate : nullptr);
        if (!hit) check(result.missed_frames == 1, "a real tentative miss consumes the tracker miss budget");
        fixture.now += 10000;
    }
    check(!fixture.tracker.tracking_confirmed(),
          "retaining a tentative ROI does not pool hits across real misses");
    for (int hit = 0; hit < 2; ++hit) {
        const auto selected = fixture.select(std::vector<dart::Point2f>(points.begin()+1, points.end()));
        check(contains(selected.roi, target), "restarting confirmation retains the target's tentative ROI");
        const auto result = fixture.measure(&candidate);
        check(result.valid == (hit == 1), "the third new consecutive observation confirms in the retained ROI");
        fixture.now += 10000;
    }
}

void test_false_candidates_cannot_starve_other_proposals()
{
    // A persistent *proposal* need not be an accepted image measurement.
    // The pulse case is stronger: occasional false image measurements keep
    // resetting consecutive misses, but never provide consecutive confirmation.
    for (const bool persistent_pulses : {false, true}) {
        Fixture fixture;
        const auto points = proposals(3);
        const auto clutter = points.front();
        const auto target = points.back();
        auto false_candidate = observation(clutter);
        auto true_candidate = observation(target);
        int clutter_visits = 0;
        bool confirmed = false;
        for (int frame = 0; frame < 160; ++frame) {
            auto refreshed = points;
            if (frame % 2) std::reverse(refreshed.begin(), refreshed.end());
            const auto selected = fixture.select(refreshed);
            const dart::detail::CandidateObservation *value = nullptr;
            if (contains(selected.roi, target)) value = &true_candidate;
            else if (contains(selected.roi, clutter)) {
                const bool pulse = clutter_visits == 0 ||
                                   (persistent_pulses && clutter_visits % 4 == 0);
                ++clutter_visits;
                if (pulse) value = &false_candidate;
            }
            const auto result = fixture.measure(value);
            if (result.valid && !result.predicted) {
                check(std::hypot(result.center_x - target.x, result.center_y - target.y) < 3.0F,
                      "transient/under-confirmed clutter does not become the final confirmed target");
                confirmed = true;
                break;
            }
            fixture.now += 10000;
        }
        check(confirmed, "bounded tentative lease releases intermittent clutter and visits a stable target");
        check(clutter_visits > 0, "clutter regression exercises an initial false image measurement");
        check(clutter_visits <= 2 * fixture.config.confirm_window,
              "intermittent accepted clutter cannot extend its tentative observation lease indefinitely");
    }
}

void test_prediction_coast_and_real_miss_limit()
{
    Fixture fixture;
    const auto target = proposals(5).front();
    fixture.acquire(target);
    const std::vector<dart::Point2f> distractors{proposals(5).back()};
    for (int miss = 1; miss <= fixture.config.max_missed_frames; ++miss) {
        const auto selected = fixture.select(distractors);
        check(contains(selected.roi, target), "one dropout does not move the ROI onto a distant proposal");
        if (miss > 1) check(selected.mode == dart::RoiMode::Coasting,
                            "subsequent dropout ROIs are explicitly coasting");
        const auto result = fixture.measure(nullptr);
        if (miss < fixture.config.max_missed_frames) {
            check(result.valid && result.predicted, "coasting result remains explicitly predicted");
            check(fixture.tracker.missed_frames() == miss, "every attempted but failed measurement counts as a miss");
        } else {
            check(!fixture.tracker.has_prediction() && !result.valid,
                  "real misses eventually reset the tracker even while its ROI is retained");
        }
        fixture.now += 10000;
    }
    check(!fixture.scheduler.needs_global_search(fixture.now, fixture.tracker),
          "ordinary lost state preserves the configured periodic global-search rate");
    const auto fresh = fixture.select(distractors);
    check(contains(fresh.roi, distractors.front()), "miss exhaustion returns selection to the full-field proposals");
}

void test_age_expiry_and_intentional_skips()
{
    Fixture fixture;
    const auto points = proposals(3);
    const auto target = points.front();
    auto candidate = observation(target);
    fixture.select(points);
    fixture.measure(&candidate);
    // Many scheduler-only calls must not spend an observation-count lease.
    for (int skip = 0; skip < 30; ++skip) {
        fixture.now += 100;
        const auto selected = fixture.select({points.back()});
        check(contains(selected.roi, target), "intentional skips preserve the tentative observation budget");
        fixture.measure(nullptr, false);
        check(fixture.tracker.missed_frames() == 0, "intentional stage skip is not a failed observation");
    }
    for (int hit = 1; hit < fixture.config.confirm_hits; ++hit) {
        fixture.now += 10000;
        fixture.select({points.back()});
        fixture.measure(&candidate);
    }
    check(fixture.tracker.tracking_confirmed(), "intentional skips do not destroy valid confirmation history");
    const auto last_measurement = fixture.now;
    fixture.now += static_cast<uint64_t>(fixture.config.prediction_max_age_ms) * 1000;
    check(!fixture.scheduler.needs_global_search(fixture.now, fixture.tracker),
          "prediction remains eligible at the configured inclusive age limit");
    ++fixture.now;
    check(fixture.tracker.missed_frames() == 0 && fixture.tracker.tracking_confirmed(),
          "age-expiry regression starts with a confirmed internal state and no counted misses");
    check(fixture.scheduler.needs_global_search(fixture.now, fixture.tracker),
          "elapsed-time expiry requests full search even without exhausting the miss counter");
    const auto selected = fixture.select({points.back()});
    check(selected.reset_tracker, "expired prediction cannot carry stale confirmation into reacquisition");
    check(contains(selected.roi, points.back()), "prediction expiry releases the stale position");
    check(fixture.now > last_measurement, "age-expiry test advances real observation timestamps");

    Fixture tentative;
    tentative.select(points);
    tentative.measure(&candidate);
    tentative.now += static_cast<uint64_t>(tentative.config.prediction_max_age_ms) * 2000 + 1;
    check(tentative.scheduler.needs_global_search(tentative.now, tentative.tracker),
          "tentative wall-clock lease expires when no more observations arrive");
    check(contains(tentative.select({points.back()}).roi, points.back()),
          "timed-out tentative target does not block the next candidate");
}

void test_motion_scale_and_image_boundaries()
{
    Fixture moving;
    for (int frame = 0; frame < 75; ++frame) {
        dart::Point2f target{400.0F + 4.0F * frame, 320.0F + frame, true};
        auto candidate = observation(target, 4.0F + frame * 0.3F);
        const auto selected = moving.select({target, {1100, 100, true}, {100, 600, true}});
        check(contains(selected.roi, target), "ROI follows target motion while apparent scale grows");
        const auto result = moving.measure(contains(selected.roi, target) ? &candidate : nullptr);
        if (frame >= moving.config.confirm_hits - 1)
            check(result.valid && !result.predicted, "motion and gradual scale change retain direct confirmed observations");
        moving.now += 10000;
    }
    for (const dart::Point2f target : {dart::Point2f{2, 2, true},
                                     dart::Point2f{1341, 757, true}}) {
        Fixture edge;
        edge.acquire(target);
        const auto selected = edge.select({});
        check(bounded(selected.roi) && contains(selected.roi, target),
              "confirmed edge target remains covered by a clipped valid ROI");
    }
    Fixture empty;
    check(bounded(empty.select({}).roi), "empty proposal set has a valid fallback ROI");
}

void test_confirmation_when_prediction_output_is_disabled()
{
    dart::DetectorConfig config;
    config.prediction_max_age_ms = 0; // Valid: suppress predicted output, retain image confirmation.
    for (const int measurement_hz : {10, 90, 180}) {
        Fixture fixture(config, measurement_hz);
        const auto points = proposals(5);
        const auto target = points.front();
        auto candidate = observation(target);
        for (int frame = 0; frame < config.confirm_hits; ++frame) {
            const auto selected = fixture.select(points);
            fixture.measure(contains(selected.roi, target) ? &candidate : nullptr);
            fixture.now += (1000000U + measurement_hz - 1) / measurement_hz;
        }
        check(fixture.tracker.tracking_confirmed(),
              "disabling prediction output does not disable consecutive-image confirmation at " +
              std::to_string(measurement_hz) + " Hz");
        check(contains(fixture.select(points).roi, target),
              "prediction-disabled track still gets the next direct measurement opportunity");
        const auto missed = fixture.measure(nullptr);
        check(!missed.valid && !missed.predicted,
              "zero prediction budget does not publish a synthetic observation after a real miss");
        check(fixture.scheduler.needs_global_search(fixture.now, fixture.tracker),
              "prediction-disabled track returns to search after a real miss");
    }
}

void test_large_candidates_confirm_before_board_roi_expands()
{
    for (float diameter : {20.0F, 40.0F, 70.0F}) {
        Fixture fixture;
        const dart::Point2f target{600, 350, true};
        auto candidate = observation(target, diameter);
        fixture.select({target});
        const auto first = fixture.measure(&candidate);
        check(!first.valid, "a single large lamp candidate is still unconfirmed");
        fixture.now += 10000;
        const auto tentative = fixture.select({target});
        const float expected = std::clamp(3 * diameter, 96.0F, 384.0F);
        check(tentative.mode == dart::RoiMode::Tentative &&
              std::fabs(tentative.roi.width - expected) <= 2,
              "tentative search covers the full lamp plus context without premature board expansion");
        check(contains(tentative.roi, {target.x - diameter, target.y, true}) &&
              contains(tentative.roi, {target.x + diameter, target.y, true}),
              "tentative search retains surrounding background on both sides of a large lamp");
        for (int hit = 1; hit < fixture.config.confirm_hits; ++hit) {
            fixture.select({target}); fixture.measure(&candidate); fixture.now += 10000;
        }
        check(fixture.tracker.tracking_confirmed(), "all normal confirmation hits are still required");
        const auto confirmed = fixture.select({target});
        check(confirmed.mode == dart::RoiMode::Tracking &&
              std::fabs(confirmed.roi.width - std::clamp(12 * diameter, 128.0F, 384.0F)) <= 2,
              "confirmed lamp restores original full-board search coverage");
    }
}

} // namespace

int main()
{
    test_round_robin_regression_and_proposal_order();
    test_interrupted_confirmation_and_refresh_without_target_proposal();
    test_false_candidates_cannot_starve_other_proposals();
    test_prediction_coast_and_real_miss_limit();
    test_age_expiry_and_intentional_skips();
    test_motion_scale_and_image_boundaries();
    test_confirmation_when_prediction_output_is_disabled();
    test_large_candidates_confirm_before_board_roi_expands();
    if (failures) {
        std::cerr << failures << " ROI scheduler assertions failed\n";
        return 1;
    }
    std::cout << "All ROI scheduler behavioral tests passed\n";
    return 0;
}
