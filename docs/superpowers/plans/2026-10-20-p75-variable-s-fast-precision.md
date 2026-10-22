# P7.5 Variable S Fast Precision Implementation Plan

> **For agentic workers:** Execute inline with superpowers:executing-plans. User requires one agent and no parallel review. Existing authorization covers ordinary diagnosis and bounded fixes; do not ask for routine approvals.

**Goal:** A continuously observable, safe 0.25 m variable_speed_s interception, with auditable speed limits and native capture evidence if offline gates pass.

**Architecture:** Retain the sole Tracker and P7SpatialGuidance opt-in path. First attribute the frozen P7.4 limits without changing guidance, then separate actual-range PNG blending, geometry and speed allocation. Add shared exact RGB-D sphere-observability checks and a bounded dynamic braking preview. Default 0.50 m and truth boundaries remain unchanged.

**Tech Stack:** Python/NumPy, ROS 2 Humble, existing sphere localizer, pytest, PX4/Gazebo.

**Spec:** `/home/qin/.codex/attachments/d00124e3-7cc7-4873-b897-342bff96e915/已粘贴的文本.txt`

## Global Constraints

- Branch codex/p75-variable-s-fast-precision from 72eb61a; preserve master and P7.4. Clean tree required before switching.
- Only variable_speed_s for new dynamic probes/formal runs: unchanged 2–4 m/s, 24 s speed cycle, 8 s startup, S turns, ±0.15 m heave, 5 m FOLLOW.
- Default p7_guidance.yaml remains byte-identical; strict candidate 0.25 m native 3D radius, sea floor .33 m, existing stricter downstream limits.
- XY speed 6.2 m/s, XY acceleration 2 m/s², vz .6 m/s, yaw .6 rad/s; freshness .125 s, causal bracket .05 s. No timestamp offsets/nav extrapolation/truth correction/evaluator edits.
- Initial P6 allowed one offline correction; it was used and the 28° gate failed. The user subsequently authorized autonomous analysis/adjustment for problems outside the plan. Revised scope below permits one geometry-derived 65° layout and diagnosed scheduling fixes, without random searches or relaxed safety/time/capture constraints.
- SITL maximum five sessions: A before B, C only after B success; source changes require renewed qualification. No extra scene flights.
- One final full available regression, known missing R11-input exceptions recorded; explicit stage paths, no push/merge.

## Review Focus

- A sphere can intersect RGB rays but have no usable clipped depth; classify separately and stop closure.
- Partial-sphere geometry can fit even with an offscreen center; do not require center containment at the endpoint.
- Current-state safety cannot certify the response/braking interval; test attitude changes and sea clearance over the preview.
- Repeated/stale image or feedback epochs cannot grant new PNG, target acceleration or closing authority.
- Vector target acceleration, cross-track demand and reference derivative must not silently remove or amplify radial closure.

## P0 — frozen attribution

Files: `scripts/p75_offline_probe.py`, independent evidence directory; report.

- [x] Verify clean tree, exact start commit, branch and default/candidate SHA.
- [x] Verify P7.4 45-file manifest and available historical S inputs. Record missing derived inputs without fabricating replacements.
- [x] Run one unchanged-P7.4 variable S diagnostic probe. Trace internal limit stages without changing online behavior; also retain original P7.4 counterexamples.
- [x] Report limit occupancy, distance ranges, direction/reference effects, P7/final Tracker shaping and plant lag separately.

## P1–P5 — candidate with TDD

Files: `guidance/rgbd_visibility.py` (new), `control/p7_spatial_guidance.py`, minimal Tracker diagnostics, candidate YAML; `test_p75_precision.py` (new).

- [x] Red tests: actual-range blending despite small internal error; radial/lateral acceleration allocation; derivative sign and total closure limit; P7.4 near-clip and historical S lower-edge cases; partial-sphere fit; infeasible heave and input-time rejection.
- [x] Reuse existing 3D quaternion PNG and yaw; use actual capture-relative range for continuous terminal blending. Keep low-range position/speed priority.
- [x] Allocate longitudinal braking from the acceleration disk after lateral demand and uncertainty, derive speed budget from target matching. Expose all independent limits, not an opaque product.
- [x] Derive a continuous relative reference with a well-defined derivative. Keep target matching unattenuated; depth modulates damping/extra protection rather than unconditionally replacing the safe stopping envelope.
- [x] Evaluate rectangular pixel rays, positive front surface, true axial depth clipping and the existing fitter/count/ratio gates. Reuse full pose/extrinsics; no center-only gate.
- [x] Add bounded response/braking corridor checks, causal heave trend with uncertainty and visible waiting. No extra mission arbiter.
- [x] Record limit stages, controller/final command, transition reason and preview categories in JSONL.

## P6 — offline acceptance gate

- [x] Run focused regression; fixed variable S phases 32 s and 35 s only, plus saved historical state counterexamples (no new figure-eight run).
- [x] Record actual clipping pixels/ratio/fit, min surface/sea gaps, endpoint intervals, time to 1/.5/.25 m, limits and smoothness.
- [x] At most one diagnosed corrective round; if pre-capture observability or safety still fails, stop SITL and report structural/candidate blockage, not universal impossibility without proof.

## P7–P9 — conditional flight and delivery

- [ ] Freeze software, one available full regression, project flake8, shell/diff, build two packages and verify prefix.
- [ ] A: ~25 s current-source actual MINCO variable S FOLLOW, exact profile and fingerprint.
- [ ] B only after A and offline success: native .25 m capture, actual first-entry/relative-speed/safety/ownership/pause evidence.
- [ ] C only after B success: identical frozen software, distinct traceable lawful phase; ≤5 total sessions.
- [ ] Report `docs/current/p75_variable_s_fast_precision_validation.md`: attribution, algorithm, clipping/phase, offline and A/B/C status, time/precision, ToF requirements and unresolved limits.
- [ ] Hash independent evidence, explicit stage/commit only necessary source/tests/report/plan; no push or merge.

## Ledger

- Start: workspace was clean on master@52e7aab; requested P7.4 commit 72eb61a existed. New branch created directly at 72eb61a. No files overwritten.

## User-authorized continuation after the initial blocked gate

- Initial 28° S32/S35 remain archived; no P75 SITL has run. The user said: “对于计划无法解决的问题，你可以自行分析调整”.
- [x] Derive one 65° candidate from .24 m interior, .06 m axial reserve and atan(2/9.81) thrust tilt; keep 5 m FOLLOW and separate configuration/session SDF.
- [x] Finite full-cycle geometry: 480/480 far FOLLOW valid, 138 near endpoints and 12 nominal .3 s windows; not a dynamic admission yet.
- [x] First 65° S32 rejected at 6.35 s: descent/outward FOV coupling and excessive cross-demand. Saved evidence reveals the stored PNG direction is overwritten by the independently allocated velocity, rotating it near vertical and distorting its horizontal direction. Added failing direction-independence regression and removed that overwrite in precision mode only.
- [ ] Re-evaluate fixed S32, then S35 if corridor succeeds; inspect remaining failures before any further bounded correction.
- [ ] Verify coherent five-node geometry/session SDF, current-source tests/build/fingerprint. Flight still conditional on an executable safe capture corridor and current A; total SITL budget remains 5.
