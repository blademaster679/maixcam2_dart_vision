#!/usr/bin/env python3
"""Portable CLI tests for a host build of board_replay_benchmark.

Requires Python OpenCV/NumPy. Tests create their own short video; no board access.
"""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
BINARY = Path("/tmp/dart_board_replay_host")


class BoardReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="dart-board-replay-test-")
        cls.folder = Path(cls.temporary.name)
        cls.video = cls.folder / "source.avi"
        writer = cv2.VideoWriter(str(cls.video), cv2.VideoWriter_fourcc(*"MJPG"), 180, (320, 240))
        if not writer.isOpened():
            raise RuntimeError("unable to generate synthetic video")
        for index in range(19):
            frame = np.full((240, 320, 3), 10, np.uint8)
            cv2.circle(frame, (160 + index, 145), 8, (0, 255, 0), -1)
            writer.write(frame)
        writer.release()
        cls.config = cls.folder / "detector.conf"
        cls.config.write_text((ROOT / "config/green_detector_full180.conf").read_text() + """
camera.width=320
camera.height=240
calibration.fx=320
calibration.fy=320
calibration.principal_x=160
calibration.principal_y=120
visual_motion.enabled=false
""")

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def run_benchmark(self, name, *options, video=None, config=None, expected_success=True):
        jsonl = self.folder / (name + ".jsonl")
        result = subprocess.run(
            [str(BINARY), "--input", str(video or self.video),
             "--config", str(config or self.config), "--native-resolution",
             "--jsonl", str(jsonl), *options],
            capture_output=True, text=True, timeout=30)
        if expected_success:
            self.assertEqual(result.returncode, 0, result.stderr)
            return json.loads(result.stdout), [json.loads(row) for row in jsonl.read_text().splitlines()]
        self.assertNotEqual(result.returncode, 0)
        return result.stderr

    def test_warmup_keeps_source_identity_and_excludes_only_statistics(self):
        summary, rows = self.run_benchmark("step", "--frame-step", "3",
                                          "--max-frames", "14", "--warmup-frames", "2")
        self.assertEqual([row["frame"] for row in rows], [0, 3, 6, 9, 12])
        self.assertEqual([row["timestamp_us"] for row in rows], [0, 16667, 33333, 50000, 66667])
        self.assertEqual([row["warmup"] for row in rows], [True, True, False, False, False])
        self.assertEqual((summary["source_frames_read"], summary["processed_frames"],
                          summary["measured_frames"], summary["measured_source_frames"]),
                         (14, 5, 3, 10))
        self.assertEqual(summary["detector"]["count"], 3)
        self.assertEqual(summary["decode"]["count"], 10)
        for row in rows:
            self.assertFalse(row["safe_for_control"])
            self.assertFalse(row["angles_valid"])
            self.assertFalse(row["pose"]["valid"])
        _, without_warmup = self.run_benchmark("no-warmup", "--frame-step", "3",
                                               "--max-frames", "14", "--warmup-frames", "0")
        # Warmup changes statistics, not detector/tracker progression.
        for warm, cold in zip(rows, without_warmup):
            self.assertEqual(warm["green"], cold["green"])
            self.assertEqual(warm["armor"], cold["armor"])

    def test_complete_decode_and_rgb_dump(self):
        raw = self.folder / "first.rgb"
        summary, rows = self.run_benchmark("complete", "--warmup-frames", "1", "--first-rgb", str(raw))
        self.assertEqual(summary["source_frames_read"], 19)
        self.assertEqual(len(rows), 19)
        self.assertTrue(summary["reached_eof"])
        self.assertEqual(raw.stat().st_size, 320 * 240 * 3)
        capture = cv2.VideoCapture(str(self.video))
        valid, bgr = capture.read()
        capture.release()
        self.assertTrue(valid)
        reference = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.int16)
        actual = np.frombuffer(raw.read_bytes(), np.uint8).reshape(reference.shape).astype(np.int16)
        # Different host FFmpeg builds may differ slightly in chroma rounding.
        self.assertLessEqual(np.abs(actual - reference).max(), 3)

    def test_unsupported_npu_and_all_warmup_are_rejected(self):
        npu = self.folder / "npu.conf"
        npu.write_text(self.config.read_text() + "\nnpu.enabled=true\n")
        error = self.run_benchmark("npu", "--warmup-frames", "0", config=npu, expected_success=False)
        self.assertIn("NPU", error)
        error = self.run_benchmark("warmup", "--max-frames", "1", "--warmup-frames", "1",
                                   expected_success=False)
        self.assertIn("no measured frames", error)

    def test_truncated_video_is_not_reported_as_complete(self):
        truncated = self.folder / "truncated.avi"
        data = self.video.read_bytes()
        truncated.write_bytes(data[:len(data) * 2 // 3])
        self.run_benchmark("truncated", "--warmup-frames", "0",
                           video=truncated, expected_success=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    options, remaining = parser.parse_known_args()
    BINARY = options.binary.resolve()
    unittest.main(argv=[__file__, *remaining])
