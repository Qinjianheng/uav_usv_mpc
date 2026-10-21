# P76 32-degree camera, physical sea clearance and PX4 interface alignment

**Goal:** Apply the user's 32-degree revision globally, remove the inherited 0.33m origin floor from Y, and complete the PX4 feedback/interface changes with current evidence.

**Architecture:** Preserve FOLLOW's existing 3D-DKF/BCTRA/MINCO path. Y remains 2D-DKF -> dual-plane PNG/pixel PD/FOV holding -> attitude/rate/physical thrust -> sole Tracker -> PX4 Body-rate. The paper has no absolute origin-altitude floor; the water application retains only a model-derived physical collision/braking barrier and native SEA_CONTACT evaluation. No target truth feeds control.

**Authorization:** This is an in-place revision of the approved P76 design, directly requested by the user on 2026-10-21. The requested 32-degree candidate supersedes the earlier two-angle restriction; the fixed 0.33m policy is explicitly superseded. Keep the old default P7 and frozen diagnostic evidence unchanged.

- [x] 1. Freeze workspace/PX4 dirty-source hashes, re-read paper II-V, reproduce old origin-floor rejection and stale camera constraint with failing tests.
- [x] 2. Build full merged-model collision vertices in the UAV model-origin FRD frame; replace the origin floor with physical lower-point clearance plus stopping distance. Feed the same native-attitude geometry to evaluation without changing radius or disabling sea failure.
- [x] 3. Set 32 degrees in the single contract and regenerate all consumers. Prepare and independently audit session SDF/LOS/FOLLOW geometry. Test Y filter ACK, mode ACK and stale-image behavior; preserve initialized Body-rate outputs while fresh; stale imagery latches recovery before changing interfaces.
- [x] 4. Add only required PX4 DDS publications, preserve unrelated existing external edits, save a reproducible patch and rebuild SITL. Verify generated message compatibility and perform a bounded disarmed SITL mode/feedback probe; never call this dynamics/capture qualification.
- [x] 5. Run current focused/full available regression, builds, shell/flake8/diff checks and retain fresh diagnostics. Report remaining physical calibration/capture limitations without inventing results.

**Tech stack:** ROS 2 Humble, Gazebo Harmonic, local PX4 SITL, NumPy/SciPy, ONNX Runtime; isolated existing P76 CPU runtime.

**Outcome:** Current available regression 2363 passed, 2 skipped, 3 warnings; PX4 build and disarmed real mode/feedback probe passed. Static actual rendering retains 3/15 occluded FOLLOW cases. Formal armed P76 remains blocked by missing measured dynamics and current visibility qualification; no capture success is claimed. See docs/current/p76_32deg_physical_clearance_px4.md.
