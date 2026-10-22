# P7.6 Yan Paper Pipeline Implementation Plan

> Execute inline with superpowers:executing-plans. The user's detailed P0–P13 specification and explicit instruction to complete the work autonomously authorize implementation and diagnosed repairs. No delegation, routine design approval, automatic push or master merge.

**Goal:** Integrate one YOLO detector, exclusive spatial/image delayed filters and a sole-Tracker paper PNG/IBVS Body-rate path, then qualify only the capabilities supported by real model and flight evidence.

**Architecture:** Preserve legacy P7 by opt-in configuration. A checked contract generates the complete ROS profile and session model calibration. A common acquisition-stamped detection feeds RGB-D and image bearing consumers. One mode coordinator in the Tracker owns FOLLOW → HANDOVER → PAPER_ACTIVE → CAPTURED/RECOVERY. Filter nodes subscribe to that durable mode epoch and run mutually exclusively. Body-rate outputs bypass legacy P7 velocity shaping while retaining a calibrated physical sea barrier and PX4 safety recovery.

**Tech Stack:** ROS 2 Humble, px4_msgs, NumPy/SciPy, ONNX Runtime CPU, YOLO11n-seg, PX4/Gazebo session copies, pytest.

**Spec:** `/home/qin/.codex/attachments/c2075042-81e9-41fd-a0a4-06fbce758235/已粘贴的文本.txt`.

## Global constraints

- Branch `codex/p76-paper-faithful-yolo-dkf` starts exactly at `944c684ef36389743e856bad19cdabb7868b4655`. Preserve master/P74/P75 and original evidence. No changes in `/home/qin/data/uav_usv`.
- Default P7 configuration and source SDF remain preserved; all new behavior is explicitly selected. No truth online, no color fallback in P76, one detector and one PX4 publisher owner.
- Original capture stamp, native navigation sample epoch, causal 50 ms bracket and 125 ms observation authority remain. Prediction does not renew an image's authority.
- Paper values: Ky/Kz 3, kp .03, kd .01, ka 2, camera20/DKF50/controller target200 Hz, target TWR3, maneuver capability5m/s². Candidate limits and mass/inertia/throttle are separately identified.
- Main layout65°/(-.25,0,.55) model-origin FLU/FOLLOW5×4 m; only the specified60°/5.5×4.5m backup if audit supports it. No unbounded geometry or gain searches.
- Body-rate .25m capture and pre-capture sea-contact failure remain. Candidate7m/s XY,2m/s descent,1.2 yaw/1.5 roll-pitch rad/s,3rad/s² angular slew require actual calibration.
- Missing models, failed timing/geometry/dynamics or unqualified Body-rate stop the dependent test. Mathematical tests and empty interfaces do not constitute a closed-loop implementation or flight qualification.

## Review focus

- Late/repeated/out-of-order images and clock reset must not create a fresh measurement, replay twice, or cross a mode epoch.
- A transient-local mode transition cannot leave two filters active or two actual PX4 modes set, including Y failure and recovery.
- Mask/letterbox coordinates must map to the original640×480 image; localizer/bearing/monitor consume the identical validated mask.
- Model-origin versus base_link translation and optical/FLU/FRD/NED transforms need independent axis and full-pose witnesses.
- A failed Python callback or process must not leave an uninitialized or stale thrust command; recovery preserves a legal heartbeat without simultaneously publishing velocity/rate control.

## T0: baseline, paper and dependency audit

Files: `docs/current/p76_yan_paper_faithful_validation.md`, evidence under `data/experiments/20261020_p76_audit_01/`.

- [x] Verify clean checkout, exact start, branch, remotes; freeze default/SDF/P75 hashes.
- [x] Read local Yan2025 SectionsII–V and Yang2025[28] SectionIV/Algorithm2; inspect rendered formulas and author/public materials.
- [x] Save equation-to-function table and explicit ambiguities14/19/20/23. Paper parameters, engineering interpretation and measured quantities remain distinct.
- [x] Generate full parameter/pub/sub/phase dependency matrix; audit actual composite model and installed package paths.
- [x] Establish model/data/backend availability; install only isolated project dependencies where possible, recording any failure.

## T1: common contract and session geometry

Files: `config/p76_contract.yaml`, `config/p76_yan_paper.yaml`, `scripts/p76_config_audit.py`, `scripts/prepare_gz_camera_model.py`; `test_p76_config_contract.py`.

- [x] Write failing tests for global geometry/limits, source-SDF preservation, base_link/model-origin offset, disagreement and model-hash rejection.
- [x] Generate full profile from one versioned contract. Extend only the opt-in model preparation for translation/calibrated propulsion copies.
- [x] Audit actual prepared SDF and effective node parameters before process launch. Current unqualified data must fail closed.
- [x] Independently check startup/acquisition,5×4 FOLLOW, partial near-field, heave and bounded full attitude; record main/backup decision and structural blockage.

## T2: real YOLO shared detection

Files: `msg/TargetImageDetection.msg`, interface CMake, `perception/yolo_inference.py`, `perception/yolo_target_detector.py`, `scripts/p76_collect_yolo_dataset.py`, `p76_train_yolo.py`, `p76_export_yolo.py`; detection tests.

- [x] Test acquisition/raw/completion stamps, strict weight SHA, RLE mask, no detection, negative frames, letterbox reversal and duplicate frame rejection with independent fixture outputs.
- [x] Implement real ONNX YOLO11n-seg preprocessing/output decode/NMS/mask generation; one image subscriber and one authoritative detection publisher.
- [x] Collect/label real simulator images into distinct sessions, split by session, train the single class, export416/optional512 and save model/dataset/software hashes. Synthetic unit fixtures do not substitute for a trained detector.
- [x] Replay held-out images, benchmark inference/end-to-end age under relevant load, and gate readiness.
- [x] Connect the same detection into localizer, bearing and monitor via opt-in branches; P76 never calls the color detector online.

## T3: delayed filters and exclusive phases

Files: `tracking/delayed_spatial_kalman.py`, `spatial_dkf_node.py`, `delayed_image_kalman.py`, `image_dkf_node.py`; mode/DKF tests.

- [x] Red tests for delayed update/replay against chronological reference, covariance PSD, source freshness, reset, duplicate/order rejection and attitude rotation covariance.
- [x] Implement timestamped history checkpoints and reproducible prediction replay. 3D CV state and 2D pixel state are explicit engineering reductions of[28]'s larger inertial EKF, not identical-state reproduction.
- [x] Spatial state publishes existing `/tracking/target_state` only duringFOLLOW; image state publishes50Hz only after warm-start from recent real detections and handover epoch.
- [x] Wire BCTRA/MINCO/mission-lock inputs; stop optimization in Y with durable mode receipt. Y active has no continuous spatial-state dependency.

## T4: paper control mathematics

Files: `control/p76_paper_png.py`, `p76_paper_ibvs.py`, `p76_paper_attitude.py`, `p76_paper_thrust.py`; separate LOS/PNG/IBVS/attitude tests.

- [x] Red tests for axis/sign transforms, full quaternion mount, paper atan conventions/branch continuity, repeat images, low speed, direction unit length.
- [x] Implement equations3,5,7–10 using dual-plane angle updates and actual pixel error. No quaternion-P7 guidance reuse.
- [x] Implement equation13 and FOV-holding11–16 with an explicit dimensionally consistent equation14 interpretation; no2m/s per200Hz tick. Record empirical status.
- [x] Implement SO(3) orthonormal Rodrigues/180° handling, equations17–23, physical mass/gravity and bounded angular output, documenting print corrections.
- [ ] Derive thrust mapping from actual simulator/PX4 motor model, exposing calibration prerequisites and saturation; never write newtons to normalized thrust.

## T5: sole-Tracker handover and safety

Files: `control/p76_mode_coordinator.py`, `p76_bodyrate_safety.py`, minimal `trajectory_tracker_node.py`, MissionManager/visibility/predictor phase wiring; safety/handover tests.

- [ ] Red tests for authorizedY, estimator activation ACK, fresh image/native attitude/rates, offboard mode bits, one output, invalid input and capture/recovery.
- [x] Run a200Hz timer only for the opt-in paper path; legacy20Hz logic returns while paper owns output. Maintain sole publishers in Tracker.
- [ ] Initialize from actual velocity, attitude, angular rate and measured thrust; retain heartbeat during guarded transition and explicit mode acceptance.
- [x] Add a Body-rate sea barrier using calibrated collision extent/braking/response; fail qualification on missing calibration.
- [x] Ensure PAPER_ACTIVE bypasses old_fast_step/precision helpers/FlightGuidance velocity shaping and does not depend on3D DKF updates.

## T6: complete launch and qualification tooling

Files: `modular_intercept.launch.py`, entry/setup/package files, `scripts/p76_sitl.py`, calibration tooling and old runner audit; integration tests.

- [x] Add an explicit P76 launch selection with unique YOLO, spatial/image modes, BCTRA/MINCO, soleTracker and native evaluator. Defaults retain old graph.
- [x] Reject hardware/external simulations; preserve owned-process cleanup and complete source/config/model snapshots.
- [x] Separate dataset/calibration/FOLLOW/capture eligibility. Require current source/model/weights/calibration hashes for each successor.
- [x] Build interfaces plus control/bringup and verify installed prefixes; current-code necessary regressions, flake8, shell and diff checks.

## T7: bounded evidence and report

- [ ] Deterministic dynamics, SO(3), both delayed filters and no-truth-feedback offline control first; replay real YOLO images with actual delays.
- [ ] A: actual PX4 Body-rate/thrust/hover/maneuver/descent braking qualification only after offline timing/safety gates. FailedA prohibits capture.
- [ ] B: current-source25s actualYOLO/3D-DKF/BCTRA/MINCO variable-S FOLLOW with qualified model. FailedB prohibitsY.
- [ ] C only afterA/B: native.25m capture with2D-DKF/paper/body-rate commands, pre-capture sea/vision/limit evidence and independently paused world.
- [ ] D only afterC success: frozen same source/model/weights, distinct lawful S phase. No mechanical repeats or unlimited tuning.
- [x] Final report identifies implemented/wired vs tested/qualified vs unresolved; include all failures, exact gate reasons, paths/hashes and command/interface proof. No fake success or automatic push/merge.

## Execution boundary at handoff

- Candidate ROS wiring and mathematical/contract tests are implemented. Real weights, independent images, two bounded static-layout audits and actual no-PX4 ROS latency/graph probes are retained.
- The checked implementation boxes do not qualify Body-rate flight. Current model/feedback/calibration gates fail closed.
- Actual thrust normalization/hover/rate/5m/s²/descent braking calibration is incomplete. The dedicated A calibration driver and automatic C/D flight runner are not implemented or qualified; the legacy speed runner explicitly rejects P76 rather than falsely accepting it.
- A/B/C/D are NOT_RUN. Both permitted mounts have complete airframe occlusion; the current DDS configuration does not publish executed thrust feedback. See final report and eligibility artifact. No capture, mode-transition latency or flight-control-frequency result is invented.
- Source-SDF and default P7 hashes remain preserved. No push/merge is performed.
