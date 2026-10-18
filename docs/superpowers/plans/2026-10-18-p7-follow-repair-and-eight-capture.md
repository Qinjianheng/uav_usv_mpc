# P7 FOLLOW repair and figure-eight capture

**Goal:** Capture the original 4 m/s figure-eight at the existing 0.50 m radius, repair the three variable-speed FOLLOW failures, and verify actual control.

**Evidence:** `data/experiments/20261018_p7_follow_repair_audit_01/` uses raw image/Gazebo pose epochs with <=50 ms bracketing, full attitude and real mount. S: 46/47 matched rejected frames have the entire sphere beyond 25 m camera depth. Z: 69/69. Valid-frame camera-depth residual p95 is 0.052/0.078 m. Takeoff visual tracking is implemented and tested but disabled in P7. Variable-eight logged steady ~0.21 rad bearing error, a fresh-KF recovery, zero centered frames and no FOLLOW LOS feedforward in target-reference mode.

**Design:** Reuse the existing bounded, fresh/locked takeoff-follow option. Extend existing common-epoch FOLLOW yaw feedforward to P7 target-reference FOLLOW, including visual reacquisition with fresh pairs; retain the original unlocked-search yaw cap and all freshness checks. PNG/IBVS after Y remains the sole guidance yaw path. No predictor/KF, motion-profile, sensor-range, capture-radius or physical limit changes.

**Execution:** Single agent, sequential, autonomous per user instruction. Preserve raw failures, source/config snapshots, independent analysis, explicit stage commits, no push.

- [x] Reproduce and attribute historical failures offline without changing stamps or using online truth.
- [x] Test the P7 takeoff setting against stale/invalid/unlocked input and the existing height blend; enable the existing option.
- [x] Add failing yaw tests for P7 FOLLOW active/fallback/reacquisition, stale pairs, caps and Y separation; implement the narrow routing fix.
- [x] Run affected tests, lint, shell checks and build, then one current-source straight F2 to check for regressions and qualify F3. 323 passed; actual MINCO 100% for 24.9501 s, no fallback, current runtime qualified.
- [x] Run original 4 m/s figure-eight X/MINCO/Y capture once. Diagnose an observed failure before a bounded retest. First capture failed; capture-only measured velocity-change feedforward then passed both left/right turn tests and actual figure-eight 02: 18.8201 s, 0.498785 m, world/target pause verified. Fresh final-runtime F2 03 qualified F3 at 100% actual MINCO for 24.9000 s.
- [x] Verify variable-speed eight/S/Z with actual MINCO FOLLOW; record recoveries and causes, preserve failures and fix one cause at a time if needed. Eight 24.9502 s, S 24.9572 s, Z 24.9708 s; each 100% actual MINCO, no fallback.
- [x] Save measured curves and source-bound outcomes, self-review, document remaining limits and commit explicit paths. 358 tests, lint/shell/build passed. Historical raw failures retained, delivery source comparison saved, single-agent self-review. FOLLOW stage d2af310; final capture stage committed separately.

## Evidence-driven extensions

- First original 4 m/s figure-eight reached Y after actual MINCO, then missed at ~1 m: transverse relative velocity 0.5–0.7 m/s, IBVS saturation and visual recovery. Constant-turn counterexample (4 m/s, 1 m/s², 40 ms measurement age, independent actuator lag) also fails; capture-only measured KF velocity-change feedforward passes the offline counterexample under the same limits. Two left/right tests failed before implementation. This is a separate capture-guidance repair, after FOLLOW validation.
- Variable S passed 24.9572 s actual MINCO, 100%, no fallback. First Z passed acquisition and entered MINCO, then an invalid prediction arrived after the preceding MINCO diagnostic. Raw data proves a fresh prediction and lawful FOLLOW fallback on the next cycle; the observer stopped on a mixed asynchronous snapshot. Observer repair waits at most one existing 50 ms control cycle only when the invalid prediction is newer; a newer MINCO diagnostic, overdue response or physical fault still fails. 126 affected observer/FOLLOW tests pass. No online runtime, freshness or lock threshold changed by this observer repair.
- Final-runtime F2 02 stopped correctly on lost navigation before MINCO/Y. Gazebo clock and DDS attitude continued while localizer navigation/observations stopped; exact root cause was not reproduced. Passive stack registration was added to diagnostic F2 03 and capture 02 without changing online code or gates. Both passed; a stack dump after capture-world pause is expected and distinguished in the delivery. This unrelated negative run remains preserved and is not counted as a success.
