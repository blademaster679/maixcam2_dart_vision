#!/usr/bin/env python3
"""Audit cached-video NV21 pipeline throughput; call rates are not accuracy."""

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path


def flag(value):
    if value in (True, 1, "1", "true", "True"):
        return True
    if value in (False, 0, "0", "false", "False"):
        return False
    raise ValueError(f"invalid boolean: {value!r}")


def number(value):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"nonfinite measurement: {value!r}")
    return result


def integer(value):
    result = int(value)
    if result < 0 or number(value) != result:
        raise ValueError(f"invalid nonnegative integer: {value!r}")
    return result


def table(path, required=()):
    with path.open() as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise ValueError(f"missing CSV header: {path}")
        if len(set(reader.fieldnames)) != len(reader.fieldnames):
            raise ValueError(f"duplicate CSV header: {path}")
        missing = set(required) - set(reader.fieldnames)
        if missing:
            raise ValueError(f"missing CSV columns {sorted(missing)}: {path}")
        rows = list(reader)
    if any(None in row or any(value is None for value in row.values()) for row in rows):
        raise ValueError(f"incomplete CSV row: {path}")
    return rows


def distribution(values):
    values = sorted(number(value) for value in values)
    if not values:
        return {"count": 0, "mean": None, "p50": None, "p95": None, "max": None}
    def percentile(q):
        position = (len(values) - 1) * q
        lower = int(position)
        return values[lower] + (values[min(lower + 1, len(values) - 1)] - values[lower]) * (position - lower)
    return {"count": len(values), "mean": sum(values) / len(values),
            "p50": percentile(.5), "p95": percentile(.95), "max": values[-1]}


def analyze(root, warmup_s=3.0, tail_s=.5):
    root = Path(root)
    warmup_s, tail_s = number(warmup_s), number(tail_s)
    if warmup_s < 0 or tail_s < 0:
        raise ValueError("warmup and tail must be nonnegative")
    replay = json.loads((root / "replay.json").read_text())
    business = json.loads((root / "business.json").read_text())
    feed = table(root / "feed.csv", ("sequence", "pts_raw", "source_frame", "source_pts_raw", "source_timestamp_us",
                                   "source_loop", "cache_index", "scheduled_us", "received_us", "submit_finished_us", "skipped_ticks_before"))
    vision = table(root / "vision.csv", ("sequence", "pts_raw", "received_us", "started_us", "finished_us",
                                       "search_ran", "green_ran", "armor_ran", "motion_ran", "search_us",
                                       "roi_convert_us", "detect_us", "motion_us", "direct_green", "green_us"))
    motion = table(root / "motion.csv", ("timestamp_us", "started_us", "finished_us", "valid"))
    targets = [json.loads(line) for line in (root / "targets.jsonl").read_text().splitlines() if line.strip()]
    if not feed:
        raise ValueError("no submitted source frames")
    errors, warnings = [], []
    def check(condition, message):
        if not condition:
            errors.append(message)
    run_path = root / "run.json"
    run_manifest = json.loads(run_path.read_text()) if run_path.exists() else None
    if run_manifest is not None:
        check(number(run_manifest["exit_code"]) == 0, "run process exit_code is nonzero")
        check(not flag(run_manifest.get("timed_out", False)), "run process timed out")
    check(not flag(replay["failed"]), "replay reports failed=true")
    check(not flag(business["failed"]), "business reports failed=true")
    check(not flag(replay.get("interrupted", False)), "replay was interrupted")
    if "exit_code" in replay:
        check(integer(replay["exit_code"]) == 0, "replay exit_code is nonzero")
    check(replay.get("input_mode") == "cached_video_nv21", "input_mode is not cached_video_nv21")
    check(business.get("input_source") == "nv21_cached_video", "business input_source is not nv21_cached_video")
    for key in ("safe_for_control", "angles_valid", "hardware_capture_included", "hardware_decode_included", "npu_included", "decode_in_timed_feed"):
        if key in replay:
            check(not flag(replay[key]), f"unexpected replay {key}=true")
    begin_feed, end_feed = integer(replay["feed_started_us"]), integer(replay["feed_stopped_us"])
    check(integer(replay["finished_us"]) >= end_feed, "finished_us precedes feed stop")
    begin = begin_feed + round(warmup_s * 1e6)
    end = end_feed - round(tail_s * 1e6)
    if end <= begin:
        raise ValueError("no wall-clock window remains after warmup/tail exclusion")
    seconds = (end - begin) / 1e6
    within = lambda timestamp: begin <= integer(timestamp) < end
    complete = lambda row: within(row["started_us"]) and integer(row["finished_us"]) < end
    feed_hz = integer(replay.get("feed_hz", 180))
    if not feed_hz:
        raise ValueError("feed_hz must be positive")
    if "feed_hz" not in replay:
        warnings.append("feed_hz absent; assumed 180 from HighFpsPipeline contract")
    cache_frames = integer(replay["cache_frames"])
    if not cache_frames:
        raise ValueError("cache_frames must be positive")

    feed_by_sequence = {}
    cache_sources = {}
    previous_sequence = previous_received = previous_pts = None
    skipped_total = skipped_window = 0
    skipped_window_exact = True
    for row in feed:
        seq, received, pts = (integer(row[k]) for k in ("sequence", "received_us", "pts_raw"))
        check(seq >= 1, "feed sequence must start at 1 or later")
        check(seq not in feed_by_sequence, f"duplicate feed sequence {seq}")
        check(begin_feed <= received <= end_feed, f"feed sequence {seq} outside feed interval")
        if previous_sequence is not None:
            check(seq > previous_sequence, f"feed sequence does not increase at {seq}")
            check(received > previous_received, f"feed received timestamp does not increase at {seq}")
            check(pts > previous_pts, f"feed synthetic PTS does not increase at {seq}")
        skipped = seq - (previous_sequence or 0) - 1
        check(skipped >= 0, f"negative source tick gap at {seq}")
        skipped_total += max(0, skipped)
        check(pts == (seq - 1) * 1000000 // feed_hz, f"synthetic PTS/tick mismatch at {seq}")
        if "skipped_ticks_before" in row:
            check(integer(row["skipped_ticks_before"]) == skipped, f"skipped_ticks_before inconsistent at {seq}")
        if "scheduled_us" in row:
            scheduled = integer(row["scheduled_us"])
            expected = begin_feed + ((seq - 1) * 1000000 + feed_hz - 1) // feed_hz
            check(abs(scheduled - expected) <= 1, f"feed schedule/origin mismatch at {seq}")
            if abs(scheduled - expected) > 1:
                skipped_window_exact = False
            # Missing integer ticks whose scheduled wall times fall inside the window.
            first_tick = (previous_sequence or 0)
            last_tick = seq - 2
            window_first = ((begin - begin_feed - 1) * feed_hz) // 1000000 + 1
            window_last = ((end - begin_feed - 1) * feed_hz) // 1000000
            skipped_window += max(0, min(last_tick, window_last) - max(first_tick, window_first, 0) + 1)
        else:
            skipped_window_exact = False
        if "submit_finished_us" in row:
            check(integer(row["submit_finished_us"]) >= received, f"negative submit duration at {seq}")
        if "cache_index" in row:
            index = integer(row["cache_index"])
            check(index < cache_frames, f"cache index outside buffer at {seq}")
            check(index == (seq - 1) % cache_frames, f"cache index/tick mismatch at {seq}")
            if "source_loop" in row:
                check(integer(row["source_loop"]) == (seq - 1) // cache_frames, f"loop/tick mismatch at {seq}")
            source = tuple(row.get(key) for key in ("source_frame", "source_pts_raw", "source_timestamp_us"))
            if index in cache_sources:
                check(cache_sources[index] == source, f"cached source metadata changes at index {index}")
            cache_sources[index] = source
        feed_by_sequence[seq] = row
        previous_sequence, previous_received, previous_pts = seq, received, pts
    check(integer(replay["submitted_frames"]) == len(feed), "replay submitted_frames differs from feed rows")
    check(integer(business["received"]) == len(feed), "business received differs from feed rows")
    check(integer(replay["skipped_ticks"]) == skipped_total, "replay skipped_ticks differs from sequence gaps")
    check(integer(business["first_received_us"]) == integer(feed[0]["received_us"]), "first_received_us differs")
    check(integer(business["last_received_us"]) == integer(feed[-1]["received_us"]), "last_received_us differs")
    check(integer(business["upstream_missing"]) == skipped_total - integer(feed[0]["sequence"]) + 1,
          "business upstream_missing differs from gaps after first received frame")
    for key in ("upstream_duplicate", "upstream_reversed", "pts_nonincreasing"):
        check(integer(business[key]) == 0, f"unexpected business {key}")
    tail_window = None
    if "planned_ticks" in replay:
        planned = integer(replay["planned_ticks"])
        last_sequence = integer(feed[-1]["sequence"])
        check(last_sequence <= planned, "feed sequence exceeds planned ticks")
        check(integer(replay["unsubmitted_ticks_after_last"]) == planned - last_sequence,
              "unsubmitted tail count differs from planned ticks")
        if skipped_window_exact:
            tail_window = max(0, min(planned - 1, window_last) - max(last_sequence, window_first) + 1)

    last_finished = last_sequence = None
    for row in vision:
        seq, received, started, finished = (integer(row[k]) for k in ("sequence", "received_us", "started_us", "finished_us"))
        check(seq in feed_by_sequence, f"vision sequence {seq} has no feed row")
        if seq in feed_by_sequence:
            check(received == integer(feed_by_sequence[seq]["received_us"]), f"vision received_us mismatch at {seq}")
            check(integer(row["pts_raw"]) == integer(feed_by_sequence[seq]["pts_raw"]), f"vision PTS mismatch at {seq}")
        check(received <= started <= finished, f"vision timestamp order invalid at {seq}")
        if last_finished is not None:
            check(started >= last_finished, "vision worker time overlaps/regresses")
            check(seq > last_sequence, "vision source sequence does not increase")
        last_finished = finished
        last_sequence = seq
        for field in ("search_us", "roi_convert_us", "detect_us", "motion_us", "green_us"):
            if field in row:
                check(number(row[field]) >= 0, f"negative {field} at {seq}")
        for field in ("search_ran", "green_ran", "armor_ran", "motion_ran", "direct_green"):
            flag(row[field])
    motion_sources = {integer(row["received_us"]) for row in vision if flag(row["motion_ran"])}
    last_finished = None
    for row in motion:
        check(integer(row["timestamp_us"]) <= integer(row["started_us"]) <= integer(row["finished_us"]),
              "motion worker timestamp order invalid")
        check(integer(row["timestamp_us"]) in motion_sources, "motion worker has no matching dispatch")
        if last_finished is not None:
            check(integer(row["started_us"]) >= last_finished, "motion worker time overlaps/regresses")
        last_finished = integer(row["finished_us"])
    check(integer(business["vision_frames"]) == len(vision), "business vision_frames differs from vision rows")
    for key, column in (("green_detection_calls", "green_ran"), ("armor_detection_calls", "armor_ran")):
        check(integer(business[key]) == sum(flag(row[column]) for row in vision), f"business {key} differs from vision rows")
    check(integer(business["control_outputs"]) == len(targets), "business control_outputs differs from target rows")
    check(integer(business["slot_taken"]) + integer(business["slot_replaced"]) + integer(business["shutdown_discarded"]) == len(feed),
          "frame slot conservation failed")
    exit_taken = integer(business["slot_taken"]) - len(vision) - integer(business["scheduled_skipped"])
    check(exit_taken in (0, 1), "slot_taken cannot be explained by vision, scheduled skip and one exit race")
    previous_output = None
    for row in targets:
        timestamp = integer(row["timestamp_us"])
        if previous_output is not None:
            check(timestamp > previous_output, "control output timestamp does not increase")
        previous_output = timestamp
        check(not flag(row["safe_for_control"]) and not flag(row["angles_valid"]) and not flag(row["pose"]["valid"]),
              "cached-video diagnostic unexpectedly enabled control, angles or pose")
        if flag(row["source_metadata_valid"]):
            seq = integer(row["source_sequence"])
            check(seq in feed_by_sequence, f"control references unknown sequence {seq}")
            check(integer(row["source_received_us"]) <= timestamp, "negative control source age")
            if seq in feed_by_sequence:
                check(integer(row["source_received_us"]) == integer(feed_by_sequence[seq]["received_us"]),
                      f"control source_received_us mismatch at {seq}")
                check(integer(row["source_pts_raw"]) == integer(feed_by_sequence[seq]["pts_raw"]),
                      f"control source PTS mismatch at {seq}")
    telemetry = {}
    for path in sorted(root.glob("*.io.json")):
        item = json.loads(path.read_text())
        telemetry[path.name] = item
        check(not flag(item["failed"]), f"telemetry failed: {path.name}")
    for name in ("vision.csv.io.json", "motion.csv.io.json", "targets.jsonl.io.json"):
        if name not in telemetry:
            warnings.append(f"missing telemetry audit: {name}")

    selected_feed = [row for row in feed if within(row["received_us"])]
    selected_vision = [row for row in vision if complete(row)]
    selected_motion = [row for row in motion if complete(row)]
    selected_targets = [row for row in targets if within(row["timestamp_us"])]
    check(len(selected_feed) >= 2, "fewer than two feed samples in measurement window")
    check(bool(selected_vision), "no completed vision calls in measurement window")
    def count(column):
        return sum(flag(row[column]) for row in selected_vision)
    def stage(column, ran=None):
        return distribution(number(row[column]) / 1000 for row in selected_vision
                            if column in row and (ran is None or flag(row[ran])))
    counts = {"feed": len(selected_feed), "vision": len(selected_vision), "green_calls": count("green_ran"),
              "direct_green_outputs": count("direct_green"), "armor_calls": count("armor_ran"),
              "search_calls": count("search_ran"), "motion_dispatches": count("motion_ran"),
              "motion_worker_calls": len(selected_motion), "control_outputs": len(selected_targets)}
    if selected_vision and "armor_valid" in selected_vision[0]:
        counts["armor_direct_valid_outputs"] = sum(flag(row["armor_ran"]) and flag(row["armor_valid"]) for row in selected_vision)
    states = dict(Counter({"0": "lost", "1": "candidate", "2": "tracking"}.get(row["green_state"], row["green_state"])
                          for row in selected_vision if "green_state" in row))
    roi_sizes = dict(Counter(f"{integer(row['roi_width'])}x{integer(row['roi_height'])}" for row in selected_vision
                             if "roi_width" in row and "roi_height" in row))
    roi_groups = {}
    for row in selected_vision:
        if "roi_width" in row and "roi_height" in row:
            size = f"{integer(row['roi_width'])}x{integer(row['roi_height'])}"
            roi_groups.setdefault(size, []).append(row)
    roi_stage_ms = {
        size: {"count": len(rows),
               "detect_and_fusion": distribution(number(row["detect_us"]) / 1000 for row in rows),
               "vision_call_wall": distribution((integer(row["finished_us"]) - integer(row["started_us"])) / 1000
                                                for row in rows)}
        for size, rows in roi_groups.items()
    }
    recv = [integer(row["received_us"]) for row in selected_feed]
    rates = {name + "_hz": value / seconds for name, value in counts.items()}
    stages = {"search_call": stage("search_us", "search_ran"), "roi_convert": stage("roi_convert_us"),
              "detect_and_fusion": stage("detect_us"), "green_call": stage("green_us", "green_ran"),
              "motion_dispatch_prepare": stage("motion_us", "motion_ran"),
              "motion_worker": distribution((integer(row["finished_us"]) - integer(row["started_us"])) / 1000 for row in selected_motion),
              "vision_call_wall": distribution((integer(row["finished_us"]) - integer(row["started_us"])) / 1000 for row in selected_vision),
              "source_age_at_vision_start": distribution((integer(row["started_us"]) - integer(row["received_us"])) / 1000 for row in selected_vision)}
    detect_mean = stages["detect_and_fusion"]["mean"]
    return {
        "run": str(root), "validation_pass": not errors, "errors": list(dict.fromkeys(errors)), "warnings": warnings,
        "scope": "Cached NV21 source window loops through the real HighFpsPipeline; not a complete live-video/camera test. All call/output rates are workload/throughput, not accuracy.",
        "window": {"start_us": begin, "end_us_exclusive": end, "seconds": seconds,
                   "warmup_s": warmup_s, "tail_guard_s": tail_s,
                   "selection": "Feed/control by own timestamp; vision/motion calls must start and finish inside window; finish/drain excluded."},
        "counts": counts, "rates": rates,
        "feed_interarrival_hz": (len(recv) - 1) * 1e6 / (recv[-1] - recv[0]) if len(recv) > 1 and recv[-1] > recv[0] else None,
        "source_skipped_ticks_window": skipped_window if skipped_window_exact else None,
        "source_skipped_ticks_full_run": skipped_total,
        "source_unsubmitted_tail_ticks_window": tail_window,
        "source_unsubmitted_tail_ticks_full_run": replay.get("unsubmitted_ticks_after_last"),
        "stage_ms": stages, "roi_size_counts": roi_sizes or None, "roi_stage_ms": roi_stage_ms or None,
        "green_state_counts": states or None,
        "detect_duration_reciprocal_hz_not_throughput": 1000 / detect_mean if detect_mean else None,
        "drop_accounting_full_run": {key: business.get(key) for key in
            ("received", "slot_taken", "slot_replaced", "scheduled_skipped", "shutdown_discarded", "upstream_missing", "upstream_duplicate", "upstream_reversed", "pts_nonincreasing")},
        "drop_accounting_note": "slot_replaced is latest-slot replacement; scheduled_skipped is intentional pipeline rate limiting; replay skipped_ticks are producer deadlines. Full-run counters cannot be assigned exactly to the trimmed window.",
        "safety": {"safe_for_control_true_full_run": sum(flag(row["safe_for_control"]) for row in targets),
                   "angles_valid_true_full_run": sum(flag(row["angles_valid"]) for row in targets),
                   "pose_valid_true_full_run": sum(flag(row["pose"]["valid"]) for row in targets)},
        "replay": replay, "business": business, "telemetry_io": telemetry, "run_manifest": run_manifest,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--warmup-s", type=float, default=3.0)
    parser.add_argument("--tail-s", type=float, default=.5)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = args.output or args.run / "analysis.json"
    try:
        result = analyze(args.run, args.warmup_s, args.tail_s)
    except (OSError, ValueError, KeyError, TypeError) as error:
        result = {"run": str(args.run), "validation_pass": False, "errors": [str(error)]}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    print(output)
    print(json.dumps({key: result[key] for key in ("validation_pass", "window", "rates", "errors") if key in result},
                     ensure_ascii=False))
    return 0 if result["validation_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
