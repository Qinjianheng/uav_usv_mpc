# P7 radius capture implementation plan

**Goal:** X → MINCO FOLLOW → Y → IBVS/PNG capture within the existing 0.50 m three-dimensional radius, followed by Gazebo pause; validate the three existing variable-speed modes.

**Design:** Add an explicit simulation capture option to P7. Keep the original safe standoff profile available. During capture, descend at at most 0.60 m/s toward 0.33 m sea clearance. Use a horizontal reference `min(4.5, 0.30 + 2 * remaining_descent)` so closure waits for descent; the final 0.30 m horizontal / 0.33 m vertical reference fits inside the existing radius. At ranges below 1 m, replace singular PN arithmetic with bounded position/target-velocity matching. Only fresh vision and KF inputs permit guidance. The existing final sea barrier and recovery retain authority. Truth remains evaluation-only; the evaluator already detects sphere entry and pauses Gazebo without a collision trigger.

**Execution:** One agent, sequential work, per the user's authorization. No push, no changes to motion profiles, KF, capture radius, historical evidence or physical limits.

- [x] Add failing tests for coordinated descent, near-zero LOS, invalid inputs and opt-in/default separation; implement pure capture reference and Tracker integration.
- [x] Extend the owned SITL runner with capture outcome/freeze recording and separate capture acceptance. Test observer handling of success before paused-navigation staleness.
- [x] Run affected regression, lint, shell checks and package build. Confirm package prefix and source snapshot.
- [x] Run straight complete X/Y capture and diagnose failures before bounded retests. Final 08 captured at 12.3056 s / 0.498403 m; world clock/target freeze verified, optional repeated native pose audit unavailable.
- [x] Run variable_speed_eight, variable_speed_s and variable_speed_z sequentially. All failed before capture, with original qualification gates intact. Preserve these negative results; no claim of three-mode capture success.
- [x] Produce actual distance/height/trajectory curves, check pause/frozen world and target, document exact results and remaining limits, and self-review. Code committed with explicit paths; documentation commit records the negative variable-mode results and unavailable repeated native-pose proof.

**Review focus:** Stale vision must cancel descent; zero horizontal range must remain finite; descent cannot bypass the sea barrier; a pause result cannot be confused with physical capture evidence; successful standoff runs do not qualify capture or the three new motion modes.
