#!/usr/bin/env python3
"""Exercise replay frame identity, output pacing and diagnostic control flags."""

import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
BINARY = ROOT / "build/video-replay/dart_video_replay"


class VideoReplayCliTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="dart-replay-test-")
        cls.folder = Path(cls.tmp.name)
        cls.video = cls.folder / "source.avi"
        writer = cv2.VideoWriter(str(cls.video), cv2.VideoWriter_fourcc(*"MJPG"),
                                 180, (320, 240))
        if not writer.isOpened():
            raise RuntimeError("cannot create synthetic replay fixture")
        for _ in range(19):
            frame = np.full((240, 320, 3), 10, dtype=np.uint8)
            cv2.circle(frame, (160, 145), 9, (30, 250, 30), -1)
            cv2.rectangle(frame, (138, 95), (141, 115), (30, 30, 250), -1)
            cv2.rectangle(frame, (179, 95), (182, 115), (30, 30, 250), -1)
            writer.write(frame)
        writer.release()
        base = (ROOT / "config/green_detector.conf").read_text()
        overrides = """
camera.width=320
camera.height=240
camera.fps=60
calibration.fx=320
calibration.fy=320
calibration.principal_x=160
calibration.principal_y=120
visual_motion.enabled=false
"""
        cls.legacy_config = cls.folder / "legacy.conf"
        cls.legacy_config.write_text(base + overrides)
        cls.diagnostic_config = cls.folder / "diagnostic.conf"
        cls.diagnostic_config.write_text(base + overrides + "\ncapture.enabled=false\n")

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def replay(self, name, *arguments, config=None):
        jsonl = self.folder / f"{name}.jsonl"
        result = subprocess.run(
            [str(BINARY), "--input", str(self.video), "--config",
             str(config or self.legacy_config), "--jsonl", str(jsonl), *arguments],
            text=True, capture_output=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        summary = json.loads(result.stdout)
        rows = [json.loads(line) for line in jsonl.read_text().splitlines()]
        return summary, rows

    def test_frame_step_preserves_source_indices_and_timestamps(self):
        summary, rows = self.replay("step", "--frame-step", "6",
                                    "--max-frames", "14", "--no-overlay-video")
        self.assertEqual([row["frame"] for row in rows], [0, 6, 12])
        self.assertEqual([row["timestamp_us"] for row in rows], [0, 33333, 66667])
        self.assertEqual(summary["frames"], 3)
        self.assertEqual(summary["source_frames_read"], 14)
        self.assertEqual(summary["output_fps"], 30)
        self.assertFalse(summary["overlay_video"])
        self.assertTrue(all(row["frame_step"] == 6 for row in rows))
        self.assertTrue(all(row["replay_pipeline"] == "rgb" for row in rows))
        expected_fps = 1000 * len(rows) / sum(row["processing_ms"] for row in rows)
        self.assertAlmostEqual(summary["average_detector_fps"], expected_fps, delta=0.1)

    def test_overlay_preserves_playback_rate(self):
        output = self.folder / "stepped.mp4"
        summary, rows = self.replay("overlay", "--frame-step", "6",
                                    "--output", str(output))
        capture = cv2.VideoCapture(str(output))
        self.assertTrue(capture.isOpened())
        self.assertAlmostEqual(capture.get(cv2.CAP_PROP_FPS), 30, places=3)
        self.assertEqual(capture.get(cv2.CAP_PROP_FRAME_COUNT), 4)
        capture.release()
        self.assertEqual([row["frame"] for row in rows], [0, 6, 12, 18])
        self.assertEqual(summary["source_frames_read"], 19)

    def test_default_behavior_is_one_output_per_source_frame(self):
        output = self.folder / "legacy.mp4"
        summary, rows = self.replay("default", "--max-frames", "5",
                                    "--output", str(output))
        self.assertEqual([row["frame"] for row in rows], list(range(5)))
        self.assertEqual(summary["frames"], 5)
        self.assertFalse(summary["replay_diagnostic_only"])
        self.assertTrue(all(row["angles_valid"] for row in rows))
        capture = cv2.VideoCapture(str(output))
        self.assertAlmostEqual(capture.get(cv2.CAP_PROP_FPS), 180, places=3)
        self.assertEqual(capture.get(cv2.CAP_PROP_FRAME_COUNT), 5)
        capture.release()

    def test_diagnostic_profiles_invalidate_angles_pose_and_control(self):
        configs = [self.diagnostic_config, ROOT / "config/green_detector_full180.conf"]
        for index, config in enumerate(configs):
            with self.subTest(config=config.name):
                summary, rows = self.replay(f"diagnostic-{index}",
                                            "--native-resolution", "--no-overlay-video",
                                            config=config)
                self.assertTrue(summary["replay_diagnostic_only"])
                self.assertEqual(summary["safe_frames"], 0)
                self.assertTrue(any(row["green"]["valid"] for row in rows),
                                "diagnostic gating must preserve pixel detections")
                for row in rows:
                    self.assertFalse(row["angles_valid"])
                    self.assertFalse(row["safe_for_control"])
                    self.assertFalse(row["pose"]["valid"])
                    self.assertEqual(row["line_of_sight_camera"], [0, 0, 0])
                    self.assertEqual(row["green"]["yaw_rad"], 0)

    def test_invalid_frame_steps_and_missing_output_are_rejected(self):
        for value in ["0", "-1", "1.5", "18446744073709551616"]:
            with self.subTest(value=value):
                result = subprocess.run(
                    [str(BINARY), "--input", str(self.video), "--no-overlay-video",
                     "--jsonl", str(self.folder / "invalid.jsonl"), "--frame-step", value],
                    text=True, capture_output=True, timeout=10)
                self.assertNotEqual(result.returncode, 0)
        result = subprocess.run([str(BINARY), "--input", str(self.video),
                                 "--no-overlay-video"], text=True,
                                capture_output=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, default=BINARY)
    arguments, remaining = parser.parse_known_args()
    BINARY = arguments.binary.resolve()
    unittest.main(argv=[__file__, *remaining])
