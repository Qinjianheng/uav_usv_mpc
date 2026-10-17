# P7 FOLLOW and visual guidance implementation plan

> Execute inline, sequentially, as explicitly requested by the user. No subagents or repeated historical audits.

**Goal:** X starts stable KF/BCTRA/MINCO FOLLOW; Y transfers the sole Tracker to bounded IBVS/PNG approach.

**Architecture:** Restore BCTRA selectively from the local R11 source snapshot. Keep current shared safety fixes, P6 modules and five motion profiles. Add an independent P7 YAML and a small pure guidance controller only after P7.0 passes actual F2 and F3.

**Tech stack:** ROS 2 Humble, Python, PX4 SITL, Gazebo, existing MINCO and velocity setpoints.

**Spec:** User-provided task `/home/qin/.codex/attachments/d2af4297-3ffa-40d7-a8d3-cd15c6b5f0e9/已粘贴的文本.txt` (2026-10-17).

## Constraints

- Online targets come only from vision/KF; truth is evaluation-only.
- Preserve acquisition timestamps, native navigation epochs, 125 ms freshness and bounded execution leases.
- Preserve Q/R, physical limits, camera geometry and all P5/P6 evidence.
- One bounded attempt per experiment; failures require a specific diagnosis before any retest. F2 failure blocks F3; P7.0 failure blocks PNG implementation.
- Explicit stage commits, no push, no `git add -A`. Work in the user-specified checkout; each experiment gets an exclusive directory.

## Review focus

- A KF heartbeat must not reset acquisition age or create a derivative observation.
- Exactly one main predictor must publish; choosing IMM must still remain possible.
- Y must invalidate FOLLOW execution and prevent old intercept plans from reacquiring ownership.
- Image loss, navigation loss, reversed/gapped time must not reuse guidance commands indefinitely.
- Actual PX4 motion and setpoints must substantiate ownership and approach claims.

## Tasks

### 1. Restore P7.0 BCTRA and R11 configuration

Files: restore `tracking/maneuvering_target_predictor.py`, `tracking/target_predictor_node.py`; extend `tracking/target_prediction.py`; update `setup.py`, `modular_intercept.launch.py`; add `config/p7_guidance.yaml`; targeted tests in `test/test_p7_follow.py`.

Interface: existing `PredictionEngine.update/generate`, `TargetPrediction` with BCTRA position/velocity/acceleration, source and acquisition epochs, 4 s/0.1 s series and 125 ms acquisition validity. Launch selector `prediction_model=bctra|imm` enables only its selected node.

- [ ] Write and run failing timestamp/interface and launch-selection tests.
- [ ] Restore from R11 snapshot; preserve current dataclass/IMM compatibility; use accurate fractional-second conversion.
- [ ] Add P7 YAML with R11 P47/C H1.6, qva, adaptive, no refinement, P44/D fallback, 5 Hz, acceleration feedforward, original admission values and nominal P6 policy.
- [ ] Run affected prediction, launch, motion and execution tests, static checks and one integration build. Confirm new workspace package prefix.
- [ ] Commit the stage with explicit paths.

### 2. Validate P7.0 actual FOLLOW

Files: add a small P7 bounded SITL entry using existing owned-session, health, recording and cleanup helpers; new ignored experiment directory and concise `docs/current/p7_progress.md`.

Interface: run current P7 config with actual sole Tracker; record source/config SHA256, native navigation and PX4 setpoint evidence, fixed F2 25 s and F3 60 s ownership windows.

- [ ] Verify environment exclusion and current source/config checks before launch; do not reuse historical readiness as current evidence.
- [ ] Run A once: linear F2. Analyze ownership, position/velocity errors, visibility and safety events.
- [ ] If A is healthy, run B once: original figure-eight F3; otherwise diagnose one main cause and apply a minimal targeted fix.
- [ ] Record the outcome and commit the runnable FOLLOW stage. Do not start Task 3 until A and B meet P7.0.

### 3. Implement IBVS and bounded PNG

Files: `control/p7_visual_guidance.py`, existing Tracker and P7 YAML; direct math/limit tests.

Interface: pure controller consumes current image error, fresh KF/PX4 position and velocity plus actual dt, initialized from previous velocity command; returns bounded horizontal velocity and IBVS yaw rate with LOS/closure diagnostics. Height remains under existing Tracker sea-safety logic.

- [ ] Test IBVS signs, deadband, bounds, invalid/stale inputs; implement only visual alignment first.
- [ ] Test independent PNG crossing/closing examples, small range, non-closing cases, speed/acceleration/jerk limits and switch continuity; implement PNG after IBVS is stable.
- [ ] Ensure one yaw owner and one final PX4 publisher. Run directly affected regression tests.

### 4. Integrate X/Y ownership and recovery

Files: existing mission manager, Tracker, intercept planner and launch, P7 YAML; focused state/ownership tests.

Interface: P7 `FAR_GUIDANCE` accepts Y only with valid mission, lock and navigation; stops FOLLOW planning, revokes old lease and blocks legacy intercept transitions. Existing Q/R/recovery/Offboard protections remain.

- [ ] Test X FOLLOW, rejected unsafe Y, accepted Y and lease revocation, stale plans, loss/recovery and reset.
- [ ] Integrate via opt-in P7 parameters without changing legacy configurations.
- [ ] Run affected tests, static checks, one integration build and commit.

### 5. Validate guidance and deliver

Files: bounded P7 runner, metrics/plots in new experiment directories, concise P7 usage/progress documentation.

- [ ] Run C once with safe initial distance: image alignment and controlled horizontal approach.
- [ ] Run D once: complete X to MINCO FOLLOW to Y guidance. Failure triggers diagnosis, never automatic retry.
- [ ] Plot actual UAV/USV motion, image errors, guidance outputs and Y continuity; retain source/config SHA256 and cleanup results.
- [ ] Review the full change inline (user prohibits agents), run required affected checks and commit. Report completed and unverified stages honestly.

## Execution notes

- Initial HEAD `e1a7666`; clean `master`; both remotes correct.
- R11 snapshot exists at `data/experiments/20261013_p53_closed_loop_11/f2_h16_01/source_snapshot/`; actual F2/F3 configs also exist. KF, direct-reference MINCO and MINCO trajectory math match R11; BCTRA engine/node were removed. Shared safety code will not be rolled back.
- Tasks 1–2 completed: `ade1bf4` restoration, `e115f15` observer distinction; 145 initial affected tests and 50 observer tests passed. F2 100% / 24.948 s and diagnostic F3 100% / 59.950 s, zero fallbacks. First failed F3 preserved. See `docs/current/p7_progress.md` for exact evidence and limitations. Tasks 3–5 now active in sequence.
