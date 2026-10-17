# P7 FOLLOW and visual guidance

2026-10-17, starting HEAD `e1a7666`. Work proceeds sequentially in the specified checkout; no parallel agents, remote push or historical evidence edits.

P7.0 software preparation restores the BCTRA predictor and ROS wrapper from the **local R11 source snapshot**, not an assumed Git equivalent. KF, shared MINCO mathematics and current safety fixes are retained. The launch graph selects exactly one main predictor (`bctra` or `imm`); ordinary launch keeps its IMM default, while P7 explicitly selects BCTRA.

`p7_guidance.yaml` uses R11 P47/C qva, adaptive scheduling, 1.6 s horizon, 5 Hz and P44/D fallback, with R11 admission and horizontal acceleration feedforward. P6 uncertainty/viewpoint features and experimental time profiles are disabled explicitly. The existing five USV motion types are unchanged. BCTRA retains the original acquisition/source transport stamps and 125 ms acquisition deadline.

Software checks: **145 passed** in the affected prediction, launch, strict visual boundary, IMM compatibility, cadence, feedforward, admission and P7 runner tests; four ROS packages built; project flake8, shell syntax and `git diff --check` passed. Package prefix: `/home/qin/data/uav_usv_mpc/install/uav_control`.

Actual F2 `20261017_p7_f2_01` passed: 100% MINCO, continuous 24.948 s, zero fallback, 1250 native samples, full-window horizontal viewpoint RMSE 0.0534 m and height RMSE 0.0111 m. Actual PX4 ULog confirms finite horizontal feedforward; cleanup had no errors or survivors. Curves/metrics: `data/experiments/20261017_p7_f2_analysis_01/`.

First F3 `20261017_p7_f3_01` failed after continuous 41.316 s: a 29 ms receiver validation blocked callbacks, and the correctly retained 125 ms gate revoked plan 181 as `TARGET_STATE_STALE`. New KF input was already published but queued; the next control cycle was fresh. The target remained visible and the controller used `INPUT_WAIT`. The inherited P53 observer stopped this as a 0.2 s lock recovery timeout, although the controller already has a 0.45 s input-wait window. Failed evidence is retained.

The P7 **read-only observer** now distinguishes visible `INPUT_WAIT` from actual visual loss, using that existing 0.45 s bound and retaining all physical/Offboard stops. Online control, 125 ms freshness and lease rules are unchanged. The 50 targeted observer/input-role regressions passed. A single F3 diagnostic retest is justified; F2 remains applicable only after comparing every online source/launch/config byte to its frozen snapshot.

Diagnostic F3 retest `20261017_p7_f3_02` passed: 100% MINCO, continuous 59.950 s, zero fallback, 3000 native samples. Full-window horizontal viewpoint RMSE 0.7524 m, height RMSE 0.0141 m, actual-to-reference 3D RMSE 0.2154 m; actual peak horizontal speed/acceleration 4.521 m/s and 2.415 m/s². Visibility and lock were 100%; execution audit and cleanup passed. The revised input-wait classification was not needed during this successful retest; the first run's receiver callback scheduling vulnerability remains a known source of lawful fallback. Curves/metrics: `data/experiments/20261017_p7_f3_analysis_01/`.

**P7.0 is complete for these straight/eight simulation scenarios.** F3 turn error remains comparable to historical R11, rather than centimeter-level tracking. IBVS/PNG and unified P7 Y development can now proceed. This intermediate configuration is still for X/FOLLOW only until that integration is complete.

Bounded FOLLOW entry (after build and ROS/workspace setup):

```bash
python3 scripts/p7_follow_sitl.py data/experiments/20261017_p7_f2_01 --scene f2
python3 scripts/p7_follow_sitl.py data/experiments/20261017_p7_f3_01 --scene f3 \
  --prior-f2 data/experiments/20261017_p7_f2_01/summary.json
```

Each entry performs one run into an exclusive directory, preserves full source/config snapshots and hashes, and cleans only its owned processes. F3 rejects failed or different-source F2 evidence. The existing Tracker is the sole Offboard publisher; truth remains evaluation-only.

## P7 guidance integration

The opt-in P7 controller now uses the validated camera FLU ray: `e_u=-y/x`, `e_v=-z/x`, equivalent to calibrated pinhole normalized image coordinates. IBVS owns yaw with a continuous 0.03 deadband, 0.6 rad/s limit and 1.2 rad/s² command slew. PNG uses N=3; horizontal velocity is integrated using actual dt, bounded by 6.2 m/s and 2 m/s². Longitudinal closure is at most 0.75 m/s, ending at 4.5 m horizontal standoff. Height remains -5 m NED through existing vertical shaping and sea protection; no descent/contact path is enabled.

Y requires fresh navigation and currently locked FOLLOW. It revokes active/pending FOLLOW immediately, clears legacy trajectories, and starts guidance from the last emitted command. The legacy intercept planner neither solves nor publishes in P7, and mission transitions cannot activate its plans. Loss returns to existing braking/search/recovery. Seven image/math tests plus a crossing-target kinematic convergence check and actual timer/state tests are included; **258 affected regressions passed**, with integration build and project static checks passed. Real C/D guidance flights are pending.

Manual isolated SITL (after build and sourcing ROS/workspace):

```bash
python3 scripts/p7_follow_sitl.py data/experiments/20261017_p7_manual_01 --manual
```

Use a fresh directory each time. This creates a task-owned SITL grant and runs the existing native console: 1–5 choose the USV mode before X; X takes off and follows; Y switches to safe guidance; Q sends mission abort and cleans up; R retains the existing clean-session restart. Each manual session is bounded to 280 s. A direct ROS launch still requires the existing isolated-SITL grant; do not disable that guard.
