#!/usr/bin/env python3
"""Host integration checks for the camera-free cached-NV21 replay executable."""
import argparse
import csv
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[2]
BINARY = Path("/tmp/dart_nv21_replay_host")


class Nv21ReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which("ffmpeg"):
            raise RuntimeError("FFmpeg is required to generate the test video")
        cls.temporary = tempfile.TemporaryDirectory(prefix="dart-nv21-replay-test-")
        cls.folder = Path(cls.temporary.name)
        cls.video = cls.folder / "source.mp4"
        cls.config = ROOT / "config/green_detector_full180.conf"
        cls.make_video(cls.video, "1344x760", 180, 108)
        cls.wrong_size = cls.folder / "wrong_size.mp4"
        cls.make_video(cls.wrong_size, "320x240", 180, 3)
        cls.wrong_fps = cls.folder / "wrong_fps.mp4"
        cls.make_video(cls.wrong_fps, "1344x760", 60, 3)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    @staticmethod
    def make_video(path, dimensions, fps, frames):
        # The H.264 fixture explicitly carries full-range BT.601-family metadata.
        filters = ("drawbox=x=666:y=474:w=12:h=12:color=0x00ff00:t=fill,"
                   "drawbox=x=646:y=424:w=4:h=20:color=0xff0000:t=fill,"
                   "drawbox=x=694:y=424:w=4:h=20:color=0xff0000:t=fill,"
                   "scale=in_range=tv:out_range=pc:in_color_matrix=bt601:out_color_matrix=bt601")
        subprocess.run(
            ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
             "-f", "lavfi", "-i", f"color=c=0x101010:s={dimensions}:r={fps}",
             "-vf", filters, "-frames:v", str(frames), "-c:v", "libx264",
             "-threads", "1", "-preset", "ultrafast", "-crf", "18",
             "-pix_fmt", "yuvj420p", "-color_range", "pc", "-colorspace", "bt470bg",
             "-color_primaries", "bt470bg", "-color_trc", "bt709", "-movflags", "+faststart", str(path)],
            check=True, capture_output=True, timeout=30)

    def directory(self, name):
        path = self.folder / name
        path.mkdir()
        return path

    def command(self, *options, video=None):
        return [str(BINARY), "--input", str(video or self.video), "--config", str(self.config),
                "--cache-frames", "9", "--seconds", "0.4", *options]

    def run_replay(self, directory, *options, video=None, success=True):
        environment = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
        result = subprocess.run(self.command(*options, video=video), cwd=directory,
                                env=environment, text=True, capture_output=True, timeout=30)
        if success:
            self.assertEqual(result.returncode, 0, result.stderr)
            return json.loads((directory / "replay.json").read_text())
        self.assertNotEqual(result.returncode, 0)
        return result.stderr

    @staticmethod
    def rows(path):
        with path.open(newline="") as source:
            return list(csv.DictReader(source))

    def test_input_dimensions_fps_and_resource_limits_are_enforced(self):
        for index, options in enumerate([
                ("--cache-frames", "0"), ("--cache-frames", "110"),
                ("--seconds", "0"), ("--seconds", "121"), ("--start-seconds", "-1")]):
            with self.subTest(options=options):
                directory = self.directory(f"invalid-option-{index}")
                self.run_replay(directory, *options, success=False)
                self.assertEqual(list(directory.iterdir()), [])
        for name, video in (("size", self.wrong_size), ("fps", self.wrong_fps)):
            with self.subTest(video=name):
                directory = self.directory("invalid-" + name)
                error = self.run_replay(directory, video=video, success=False)
                self.assertIn("requires", error)
                self.assertFalse((directory / "feed.csv").exists())

    def test_existing_outputs_are_rejected_without_modification(self):
        sentinel = b"previous experiment must survive\n"
        for name in ("feed.csv", "vision.csv", "targets.jsonl", "business.json", "motion.csv", "replay.json"):
            with self.subTest(name=name):
                directory = self.directory("existing-" + name)
                existing = directory / name
                existing.write_bytes(sentinel)
                self.assertIn("refusing to overwrite", self.run_replay(directory, success=False))
                self.assertEqual(existing.read_bytes(), sentinel)
                self.assertEqual(list(directory.iterdir()), [existing])

    def test_seek_mapping_metadata_and_shared_pipeline_outputs(self):
        directory = self.directory("seek")
        summary = self.run_replay(directory, "--start-seconds", "0.1")
        feed = self.rows(directory / "feed.csv")
        business = json.loads((directory / "business.json").read_text())
        targets = [json.loads(line) for line in (directory / "targets.jsonl").read_text().splitlines()]
        vision = self.rows(directory / "vision.csv")
        self.assertFalse(summary["failed"])
        self.assertFalse(summary["interrupted"])
        self.assertEqual((summary["source_first_frame"], summary["source_last_frame"]), (18, 26))
        self.assertEqual(summary["source_frame_index_method"], "derived_from_pts")
        self.assertEqual(summary["cache_bytes"], 1344 * 760 * 3 // 2 * 9)
        self.assertEqual(summary["source_color_range"], "pc")
        self.assertEqual(summary["source_colorspace"], "bt470bg")
        self.assertEqual(summary["output_pixel_format"], "nv21")
        self.assertEqual(summary["output_color_range"], "limited")
        self.assertEqual(summary["output_colorspace"], "bt601")
        self.assertFalse(summary["source_range_assumed"])
        self.assertFalse(summary["source_space_assumed"])
        self.assertFalse(summary["decode_in_timed_feed"])
        self.assertEqual(business["input_source"], "nv21_cached_video")
        self.assertFalse(business["failed"])
        self.assertEqual(business["received"], summary["submitted_frames"])
        self.assertEqual(len(feed), summary["submitted_frames"])
        self.assertGreater(len(feed), 1)
        self.assertTrue(vision, "the real shared pipeline must run vision work")
        self.assertTrue(any(int(row["source_loop"]) > 0 for row in feed))
        previous = None
        by_sequence = {}
        for row in feed:
            sequence = int(row["sequence"])
            tick = sequence - 1
            received = int(row["received_us"])
            self.assertEqual(int(row["cache_index"]), tick % 9)
            self.assertEqual(int(row["source_loop"]), tick // 9)
            self.assertEqual(int(row["source_frame"]), 18 + tick % 9)
            self.assertEqual(int(row["pts_raw"]), tick * 1_000_000 // 180)
            scheduled = summary["feed_started_us"] + (tick * 1_000_000 + 179) // 180
            self.assertEqual(int(row["scheduled_us"]), scheduled)
            self.assertGreaterEqual(received, scheduled)
            self.assertGreaterEqual(int(row["submit_finished_us"]), received)
            source_us = int(row["source_timestamp_us"])
            self.assertLessEqual(abs(source_us - int(row["source_frame"]) * 1_000_000 / 180), 0.6)
            if previous is not None:
                self.assertGreater(sequence, int(previous["sequence"]))
                self.assertGreater(int(row["pts_raw"]), int(previous["pts_raw"]))
                self.assertGreater(received, int(previous["received_us"]))
                self.assertEqual(int(row["skipped_ticks_before"]), sequence - int(previous["sequence"]) - 1)
            previous = row
            by_sequence[sequence] = row
        self.assertEqual(summary["skipped_ticks"], sum(int(row["skipped_ticks_before"]) for row in feed))
        observed = [row for row in targets if row["source_metadata_valid"]]
        self.assertTrue(observed)
        for row in targets:
            self.assertFalse(row["safe_for_control"])
            self.assertFalse(row["angles_valid"])
            self.assertFalse(row["pose"]["valid"])
            self.assertEqual(row["measurement_timestamp_source"], "cached_video_submit_host_monotonic")
            self.assertEqual(row["pts_event"], "synthetic_monotonic_replay_tick_us")
        for row in observed:
            origin = by_sequence[row["source_sequence"]]
            self.assertEqual(row["source_pts_raw"], int(origin["pts_raw"]))
            self.assertEqual(row["source_received_us"], int(origin["received_us"]))

    def test_seek_beyond_complete_cache_window_fails_before_feeding(self):
        directory = self.directory("short-window")
        error = self.run_replay(directory, "--start-seconds", "0.59", success=False)
        self.assertIn("EOF before complete cache window", error)
        self.assertFalse((directory / "feed.csv").exists())
        self.assertFalse((directory / "business.json").exists())

    @unittest.skipUnless(hasattr(signal, "SIGTERM"), "SIGTERM requires a POSIX host")
    def test_sigterm_finishes_pipeline_and_preserves_feed_evidence(self):
        directory = self.directory("interrupted")
        environment = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
        process = subprocess.Popen(self.command("--seconds", "10"), cwd=directory,
                                   env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            deadline = time.monotonic() + 10
            # File creation means cache preloading finished and vision thread started.
            while not (directory / "vision.csv").exists():
                self.assertIsNone(process.poll(), "process exited before pipeline startup")
                self.assertLess(time.monotonic(), deadline, "pipeline startup timed out")
                time.sleep(0.01)
            time.sleep(0.10)
            process.send_signal(signal.SIGTERM)
            stdout, stderr = process.communicate(timeout=10)
            self.assertEqual(process.returncode, 130, stderr)
            summary = json.loads((directory / "replay.json").read_text())
            self.assertTrue(summary["interrupted"])
            self.assertFalse(summary["failed"])
            self.assertGreater(summary["submitted_frames"], 0)
            self.assertGreater(summary["unsubmitted_ticks_after_last"], 0)
            self.assertEqual(len(self.rows(directory / "feed.csv")), summary["submitted_frames"])
            business = json.loads((directory / "business.json").read_text())
            self.assertFalse(business["failed"])
            self.assertEqual(business["received"], summary["submitted_frames"])
            self.assertEqual(json.loads(stdout)["submitted_frames"], summary["submitted_frames"])
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    args, remaining = parser.parse_known_args()
    BINARY = args.binary.resolve()
    unittest.main(argv=[__file__, *remaining])
