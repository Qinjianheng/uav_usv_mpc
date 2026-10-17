# P7 FOLLOW and visual guidance

2026-10-17, starting HEAD `e1a7666`. Work proceeds sequentially in the specified checkout; no parallel agents, remote push or historical evidence edits.

P7.0 software preparation restores the BCTRA predictor and ROS wrapper from the **local R11 source snapshot**, not an assumed Git equivalent. KF, shared MINCO mathematics and current safety fixes are retained. The launch graph selects exactly one main predictor (`bctra` or `imm`); ordinary launch keeps its IMM default, while P7 explicitly selects BCTRA.

`p7_guidance.yaml` uses R11 P47/C qva, adaptive scheduling, 1.6 s horizon, 5 Hz and P44/D fallback, with R11 admission and horizontal acceleration feedforward. P6 uncertainty/viewpoint features and experimental time profiles are disabled explicitly. The existing five USV motion types are unchanged. BCTRA retains the original acquisition/source transport stamps and 125 ms acquisition deadline.

Software checks: **145 passed** in the affected prediction, launch, strict visual boundary, IMM compatibility, cadence, feedforward, admission and P7 runner tests; four ROS packages built; project flake8, shell syntax and `git diff --check` passed. Package prefix: `/home/qin/data/uav_usv_mpc/install/uav_control`.

Actual F2 `20261017_p7_f2_01` passed: 100% MINCO, continuous 24.948 s, zero fallback, 1250 native samples, full-window horizontal viewpoint RMSE 0.0534 m and height RMSE 0.0111 m. Actual PX4 ULog confirms finite horizontal feedforward; cleanup had no errors or survivors. Curves/metrics: `data/experiments/20261017_p7_f2_analysis_01/`.

First F3 `20261017_p7_f3_01` failed after continuous 41.316 s: a 29 ms receiver validation blocked callbacks, and the correctly retained 125 ms gate revoked plan 181 as `TARGET_STATE_STALE`. New KF input was already published but queued; the next control cycle was fresh. The target remained visible and the controller used `INPUT_WAIT`. The inherited P53 observer stopped this as a 0.2 s lock recovery timeout, although the controller already has a 0.45 s input-wait window. Failed evidence is retained.

The P7 **read-only observer** now distinguishes visible `INPUT_WAIT` from actual visual loss, using that existing 0.45 s bound and retaining all physical/Offboard stops. Online control, 125 ms freshness and lease rules are unchanged. The 50 targeted observer/input-role regressions passed. A single F3 diagnostic retest is justified; F2 remains applicable only after comparing every online source/launch/config byte to its frozen snapshot.

IBVS/PNG and unified P7 Y behavior remain gated on healthy P7.0 flights. This intermediate configuration is for X/FOLLOW validation only; it is not a completed P7 guidance release.

Bounded FOLLOW entry (after build and ROS/workspace setup):

```bash
python3 scripts/p7_follow_sitl.py data/experiments/20261017_p7_f2_01 --scene f2
python3 scripts/p7_follow_sitl.py data/experiments/20261017_p7_f3_01 --scene f3 \
  --prior-f2 data/experiments/20261017_p7_f2_01/summary.json
```

Each entry performs one run into an exclusive directory, preserves full source/config snapshots and hashes, and cleans only its owned processes. F3 rejects failed or different-source F2 evidence. The existing Tracker is the sole Offboard publisher; truth remains evaluation-only.
