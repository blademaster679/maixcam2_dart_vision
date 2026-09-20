#!/usr/bin/env python3
"""Regression checks for sparse reference evaluation, using synthetic data only."""

import importlib.util
from pathlib import Path
import unittest


SPEC = importlib.util.spec_from_file_location(
    "evaluate_target_reference", Path(__file__).with_name("evaluate_target_reference.py")
)
EVALUATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EVALUATOR)


def reference(frame=0, green_status="visible", armor_status="visible"):
    return {
        "frame": frame,
        "split": "development",
        "green_status": green_status,
        "green_center": [50, 80] if green_status == "visible" else None,
        "green_radius": 4 if green_status == "visible" else None,
        "armor_status": armor_status,
        "armor_center": [50, 30] if armor_status == "visible" else None,
        "armor_bbox": [20, 20, 80, 40] if armor_status == "visible" else None,
    }


def detection(green_x=50, predicted=False, armor_scan=True, bar_x=(22, 78)):
    return {
        "green": {
            "valid": True,
            "predicted": predicted,
            "center_x": green_x,
            "center_y": 80,
        },
        "armor": {
            "valid": True,
            "center": {"x": 50, "y": 30},
            "left_bar": {"center": {"x": bar_x[0], "y": 30}},
            "right_bar": {"center": {"x": bar_x[1], "y": 30}},
        },
        "armor_detection_ran": armor_scan,
    }


class ReferenceEvaluationTests(unittest.TestCase):
    def evaluate(self, rows, refs):
        return EVALUATOR.evaluate(rows, refs)["development"]

    def test_missing_replay_frame_rejects_incomplete_comparison(self):
        with self.assertRaisesRegex(ValueError, "missing 1 reference frames"):
            self.evaluate({0: detection()}, [reference(0), reference(360)])

    def test_green_hits_require_correct_location_and_direct_observation(self):
        rows = {
            0: detection(green_x=53),
            1: detection(green_x=65),
            2: detection(predicted=True),
        }
        stats = self.evaluate(rows, [reference(i) for i in rows])["green"]
        self.assertEqual(stats["visible_reference_frames"], 3)
        self.assertEqual(stats["matched_direct_detections"], 1)
        self.assertEqual(stats["wrong_location_detections"], 1)
        self.assertEqual(stats["missed_or_wrong"], 2)
        self.assertEqual(stats["matched_center_error_median_px"], 3)
        self.assertEqual(stats["matched_center_error_max_px"], 3)

    def test_cached_armor_is_not_a_direct_detection(self):
        stats = self.evaluate(
            {0: detection(), 1: detection(armor_scan=False)},
            [reference(0), reference(1)],
        )["armor"]
        self.assertEqual(stats["visible_reference_frames"], 2)
        self.assertEqual(stats["matched_direct_detections"], 1)
        self.assertEqual(stats["missed_or_wrong"], 1)
        self.assertEqual(stats["wrong_location_detections"], 0)

    def test_near_center_fragments_do_not_match_two_widely_separated_bars(self):
        stats = self.evaluate(
            {0: detection(), 1: detection(bar_x=(49, 51))},
            [reference(0), reference(1)],
        )["armor"]
        self.assertEqual(stats["matched_direct_detections"], 1)
        self.assertEqual(stats["wrong_location_detections"], 1)
        self.assertEqual(stats["missed_or_wrong"], 1)

    def test_uncertain_references_do_not_enter_positive_or_negative_counts(self):
        refs = [reference(0), reference(1, "absent", "absent"),
                reference(2, "uncertain", "uncertain")]
        stats = self.evaluate({i: detection() for i in range(3)}, refs)
        for kind in ("green", "armor"):
            with self.subTest(kind=kind):
                self.assertEqual(stats[kind]["visible_reference_frames"], 1)
                self.assertEqual(stats[kind]["absent_reference_frames"], 1)
                self.assertEqual(stats[kind]["matched_direct_detections"], 1)
                self.assertEqual(stats[kind]["false_detections_on_absent"], 1)
                self.assertEqual(stats[kind]["uncertain_excluded"], 1)

    def test_predicted_or_cached_outputs_are_not_direct_false_detections(self):
        rows = {0: detection(predicted=True, armor_scan=False)}
        stats = self.evaluate(rows, [reference(0, "absent", "absent")])
        for kind in ("green", "armor"):
            with self.subTest(kind=kind):
                self.assertEqual(stats[kind]["absent_reference_frames"], 1)
                self.assertEqual(stats[kind]["false_detections_on_absent"], 0)


if __name__ == "__main__":
    unittest.main()
