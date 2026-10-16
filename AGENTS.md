# MaixCAM2 Tracking and Vision repository instructions

## Work entry

Read `README.md` first, then read the task-relevant document in `docs/`.
For the current high-frame-rate path, use
`docs/GREEN_DETECTION_PLAN_FULL180.md` as the active design boundary and
`docs/IMPLEMENTATION_STATUS_V0.2.md` as the evidence/status record.
Historical conversation exports and dated reports provide context only; verify
claims against the current source, configuration and raw measurements.

## Common engineering standard

Apply `.agents/docs/engineering.md` for the shared engineering process. This
file is the Tracking-specific adapter: it defines the target-vision ownership,
board and dataset boundaries, qualification gates and hardware authorization.
Keep component requirements here or in the existing project documents; do not
duplicate them into the common standard.

## Component scope and contracts

- This repository owns MaixCAM2 green-light and red/blue armor observation,
  candidate detection, target tracking, motion compensation and the versioned
  `TargetEstimate` JSON v2 output. It does not own PX4/ArduPilot control,
  flight planning or VIO/SLAM estimation.
- Preserve the fail-closed defaults until the required evidence exists:
  `npu.enabled=false`, `target_geometry.pose_enabled=false`, and
  `safe_for_control=false` for uncalibrated or incomplete outputs.
- Keep target state, timestamps, source-clock meaning, camera calibration,
  frame conventions and model provenance explicit. A generated estimate or a
  replay output is not automatically a control-safe observation.
- Preserve the distinction between host unit tests, offline replay, board
  performance measurements and real target/flight qualification. Unlabelled
  frame rate or output rate is not recall, precision or end-to-end latency.
- Keep recordings, datasets, downloaded SDKs, model binaries, board builds,
  credentials and generated reports outside Git unless an existing repository
  policy explicitly tracks a small manifest or template. Preserve hashes and
  provenance for external inputs.

## Device and publication boundary

Do not access a camera, install an app, change firmware/system libraries,
change device parameters, enable a self-start entry, or run a flight/control
integration merely because a helper command exists. Such actions need current
explicit authorization. Do not push the `slam_drone` branch or publish local
board/model evidence without an explicit request.

## Verification

For C++ changes, use the existing host test path:

```bash
cmake -S tests -B build/tests -DCMAKE_BUILD_TYPE=Release
cmake --build build/tests -j
ctest --test-dir build/tests --output-on-failure
```

For Python tool changes, run the affected tests and the relevant replay or
log evaluator. Report host, replay and board results separately. Do not claim
real-camera timing, target accuracy, flight safety or competition qualification
from host tests alone.

## Working state

Simple changes do not need a plan. For a complex task, maintain one local plan
at `.agents/plans/plan_<task>_01.md` and put bounded diagnostic evidence under
`.agents/runs/<task-id>/`; these directories are local and ignored. Preserve
unrelated changes, do not reset the checkout, and update the existing status or
handoff document instead of creating a new report for every iteration.
