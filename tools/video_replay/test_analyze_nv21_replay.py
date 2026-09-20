"""Small synthetic-log tests for the replay measurement contract."""

import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


_SPEC = importlib.util.spec_from_file_location(
    "analyze_nv21_replay", Path(__file__).with_name("analyze_nv21_replay.py"))
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
analyze = _MODULE.analyze


class ReplayAnalysisTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.origin = 1000000

    def write_fixture(self, missing=(), replaced=0):
        """Ten seconds, 180 Hz producer, 60 Hz vision; actual KLT is 6 Hz."""
        feed, vision, motion, targets = [], [], [], []
        previous_tick = -1
        for tick in range(1800):
            if tick in missing:
                continue
            relative = (tick * 1000000 + 179) // 180
            received = self.origin + relative
            index = tick % 90
            frame = dict(sequence=tick + 1, pts_raw=tick * 1000000 // 180,
                         source_frame=2400 + index, source_pts_raw=index * 512,
                         source_timestamp_us=13333333 + index * 5556,
                         source_loop=tick // 90, cache_index=index,
                         scheduled_us=received, received_us=received,
                         submit_finished_us=received + 10,
                         skipped_ticks_before=tick - previous_tick - 1)
            feed.append(frame)
            previous_tick = tick
            if tick % 3:
                continue
            row = dict(sequence=tick + 1, pts_raw=frame["pts_raw"], received_us=received,
                       started_us=received + 20, finished_us=received + 520,
                       search_ran=int(tick % 18 == 0), green_ran=1,
                       armor_ran=int(tick % 12 == 0), motion_ran=int(tick % 30 == 0),
                       search_us=40 if tick % 18 == 0 else 1, roi_convert_us=10,
                       detect_us=300, motion_us=20 if tick % 30 == 0 else 1,
                       direct_green=int(tick % 6 == 0), green_us=100,
                       roi_x=500, roi_y=300, roi_width=128, roi_height=128,
                       proposal_count=3, green_state=2, green_valid=1,
                       green_predicted=int(tick % 6 != 0), green_x=550,
                       green_y=350, green_size=5, armor_valid=1)
            vision.append(row)
            if row["motion_ran"]:
                motion.append(dict(timestamp_us=received, started_us=received + 80,
                                   finished_us=received + 1080, valid=1))
            targets.append(dict(timestamp_us=received + 1000, safe_for_control=False,
                                angles_valid=False, pose={"valid": False},
                                source_metadata_valid=True, source_sequence=tick + 1,
                                source_received_us=received, source_pts_raw=frame["pts_raw"]))
        replay = dict(failed=False, interrupted=False, input_mode="cached_video_nv21",
                      feed_started_us=self.origin, feed_stopped_us=self.origin + 10000000,
                      finished_us=self.origin + 10000100, requested_seconds=10,
                      feed_hz=180, cache_frames=90, planned_ticks=1800,
                      submitted_frames=len(feed), skipped_ticks=len(missing),
                      unsubmitted_ticks_after_last=0, safe_for_control=False,
                      angles_valid=False, hardware_capture_included=False)
        business = dict(failed=False, input_source="nv21_cached_video", received=len(feed),
                        first_received_us=feed[0]["received_us"], last_received_us=feed[-1]["received_us"],
                        upstream_missing=len(missing) - feed[0]["sequence"] + 1,
                        upstream_duplicate=0, upstream_reversed=0, pts_nonincreasing=0,
                        slot_taken=len(feed) - replaced, slot_replaced=replaced,
                        shutdown_discarded=0, vision_frames=len(vision),
                        scheduled_skipped=len(feed) - replaced - len(vision),
                        green_detection_calls=len(vision),
                        armor_detection_calls=sum(row["armor_ran"] for row in vision),
                        control_outputs=len(targets))
        self.write_json("replay.json", replay)
        self.write_json("business.json", business)
        self.write_csv("feed.csv", feed)
        self.write_csv("vision.csv", vision)
        self.write_csv("motion.csv", motion)
        self.write_targets(targets)
        for name in ("vision.csv", "motion.csv", "targets.jsonl"):
            self.write_json(name + ".io.json", {"failed": False})
        return feed, vision, motion, targets

    def write_json(self, name, value):
        (self.root / name).write_text(json.dumps(value))

    def update_json(self, name, **changes):
        value = json.loads((self.root / name).read_text())
        value.update(changes)
        self.write_json(name, value)

    def write_csv(self, name, rows):
        with (self.root / name).open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)

    def write_targets(self, rows):
        (self.root / "targets.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))

    def test_common_wall_window_and_real_worker_timing(self):
        self.write_fixture()
        result = analyze(self.root)
        self.assertTrue(result["validation_pass"], result["errors"])
        self.assertEqual(result["window"]["seconds"], 6.5)
        self.assertEqual(result["counts"]["feed"], 1170)
        self.assertEqual(result["rates"]["feed_hz"], 180)
        self.assertEqual(result["rates"]["vision_hz"], 60)
        self.assertEqual(result["rates"]["direct_green_outputs_hz"], 30)
        self.assertEqual(result["rates"]["armor_calls_hz"], 15.076923076923077)
        self.assertEqual(result["stage_ms"]["search_call"]["count"], 65)
        self.assertAlmostEqual(result["stage_ms"]["search_call"]["mean"], .04)
        self.assertAlmostEqual(result["stage_ms"]["motion_dispatch_prepare"]["mean"], .02)
        self.assertAlmostEqual(result["stage_ms"]["motion_worker"]["mean"], 1)
        self.assertEqual(result["roi_size_counts"], {"128x128": 390})
        self.assertEqual(result["green_state_counts"], {"tracking": 390})
        self.assertGreater(result["detect_duration_reciprocal_hz_not_throughput"], result["rates"]["vision_hz"])

    def test_producer_gaps_separate_from_slot_replacements_and_rate_limit(self):
        self.write_fixture(missing={1, 100, 539, 541, 700, 1709, 1711}, replaced=7)
        result = analyze(self.root)
        self.assertTrue(result["validation_pass"], result["errors"])
        self.assertEqual(result["source_skipped_ticks_full_run"], 7)
        self.assertEqual(result["source_skipped_ticks_window"], 3)
        self.assertEqual(result["counts"]["feed"], 1167)
        self.assertEqual(result["drop_accounting_full_run"]["slot_replaced"], 7)
        self.assertEqual(result["drop_accounting_full_run"]["scheduled_skipped"], 1186)
        self.assertEqual(result["source_unsubmitted_tail_ticks_window"], 0)

    def test_calls_straddling_window_are_excluded(self):
        _, vision, motion, _ = self.write_fixture()
        # Last pre-warmup call finishes after start; last in-window call finishes after end.
        vision[179]["finished_us"] = self.origin + 3000001
        vision[569]["finished_us"] = self.origin + 9500001
        motion[56]["finished_us"] = self.origin + 9500001
        self.write_csv("vision.csv", vision)
        self.write_csv("motion.csv", motion)
        result = analyze(self.root)
        self.assertTrue(result["validation_pass"], result["errors"])
        self.assertEqual(result["counts"]["vision"], 389)
        self.assertEqual(result["counts"]["motion_worker_calls"], 38)

    def test_missing_or_misattributed_rows_are_rejected(self):
        _, vision, _, targets = self.write_fixture()
        self.write_csv("vision.csv", vision[:-1])
        result = analyze(self.root)
        self.assertFalse(result["validation_pass"])
        self.assertIn("business vision_frames differs from vision rows", result["errors"])
        self.write_csv("vision.csv", vision)
        targets[200]["source_received_us"] += 1
        self.write_targets(targets)
        result = analyze(self.root)
        self.assertFalse(result["validation_pass"])
        self.assertTrue(any("control source_received_us mismatch" in error for error in result["errors"]))

    def test_failed_interrupted_and_unsafe_runs_are_not_accepted(self):
        _, _, _, targets = self.write_fixture()
        self.update_json("replay.json", interrupted=True)
        self.update_json("business.json", failed=True)
        self.update_json("vision.csv.io.json", failed=True)
        self.write_json("run.json", {"exit_code": -9, "timed_out": True})
        targets[-1]["safe_for_control"] = True
        self.write_targets(targets)
        result = analyze(self.root)
        self.assertFalse(result["validation_pass"])
        self.assertEqual(result["safety"]["safe_for_control_true_full_run"], 1)
        self.assertIn("business reports failed=true", result["errors"])
        self.assertIn("replay was interrupted", result["errors"])
        self.assertIn("telemetry failed: vision.csv.io.json", result["errors"])
        self.assertIn("run process exit_code is nonzero", result["errors"])
        self.assertIn("run process timed out", result["errors"])

    def test_empty_window_and_broken_csv_fail_loudly(self):
        self.write_fixture()
        with self.assertRaisesRegex(ValueError, "no wall-clock window"):
            analyze(self.root, warmup_s=10)
        with (self.root / "vision.csv").open("a") as handle:
            handle.write("1,2\n")
        with self.assertRaisesRegex(ValueError, "incomplete CSV row"):
            analyze(self.root)

    def test_even_empty_motion_log_requires_correct_schema(self):
        self.write_fixture()
        (self.root / "motion.csv").write_text("wrong_header\n")
        with self.assertRaisesRegex(ValueError, "missing CSV columns"):
            analyze(self.root)

    def test_roi_stage_distributions_keep_different_sizes_separate(self):
        _, vision, _, _ = self.write_fixture()
        row = vision[200]  # Inside the measurement window.
        row.update(roi_width=384, roi_height=384, detect_us=2000,
                   finished_us=row["started_us"] + 4000)
        self.write_csv("vision.csv", vision)
        result = analyze(self.root)
        self.assertTrue(result["validation_pass"], result["errors"])
        groups = result["roi_stage_ms"]
        self.assertEqual(groups["384x384"]["count"], 1)
        self.assertEqual(groups["384x384"]["detect_and_fusion"]["mean"], 2)
        self.assertEqual(groups["384x384"]["vision_call_wall"]["p95"], 4)
        self.assertEqual(groups["128x128"]["count"], 389)
        self.assertAlmostEqual(groups["128x128"]["detect_and_fusion"]["mean"], .3)
        self.assertAlmostEqual(groups["128x128"]["vision_call_wall"]["mean"], .5)
        self.assertEqual(sum(group["count"] for group in groups.values()), result["counts"]["vision"])


if __name__ == "__main__":
    unittest.main()
