# P6.1 successor supply implementation plan

> **For agentic workers:** Use superpowers:executing-plans. The user explicitly authorizes autonomous implementation, testing, one gated F2 and committing/pushing the independent branch.

**Goal:** Reduce real MINCO successor latency while retaining125ms acquisition freshness,450ms leases, full dynamics/FOV and actual ACK/ACTIVE ownership.
**Architecture:** Keep one existing worker and current interfaces. Analyze immutable recorded cycles; batch repeated whole-sphere geometry with scalar boundary/error fallback, then quantify identical-input effects. Add precise clock-domain audit and worker CPU timing. Change scheduling only if a separate reproduced defect demands it.
**Spec:** User P6.1 task,2026-10-15, sections1-10; this plan records its executable scope.
**Workspace:** /home/qin/data/uav_usv_mpc, branch p6/p61-successor-supply-fix from master fe96e9cdc35754fb09fde4726be316bd23f15fd9. Reuse the explicit current overlay/R11 data, no worktree cleanup.

## Global constraints

- No truth online, retiming, stale input substitution, relaxed geometry/dynamics/ownership, baseline modifications or unbounded queue.
- Preserve PR4 cooldown and post-wire revalidation audit. Keep acquisition/native epochs and existing50ms activation margin.
- At most one new same-config60s F2 after all offline tests/build and independent review. No matrix and no extra SITL retries.
- Preserve raw evidence and source/version manifests. Stage explicit paths only; commit/push user-authorized branch.

## Review focus

1. Scalar/batch equality around front/range/FOV boundaries, invalid numeric input and arbitrary rotations.
2. Revalidation still covers the full1.6s curve and current prediction; late candidates remain rejected.
3. Busy worker, two missed cycles, ACK latency and renewed input opportunities preserve existing one-worker/retry contracts.
4. Performance distributions distinguish measured wall/CPU from synthetic or replay clocks; missing historical fields remain missing.
5. Qualification binds exact current bytes/config and cannot reuse old successes.

### Task1: Independent analysis and baseline

- [x] Inspect current master/PR4, Git status and original remote manifest.
- [x] Run clean baseline suite (2130 passed,1 skipped).
- [x] Profile actual saved requests with unchanged parameters and real processing time (320/320 valid;P95~8ms vs recorded51.6ms).
- [ ] Build cycle analysis tool/tests; rank the three loss chains and preserve unknown dequeue/GIL scheduling details explicitly.
- [ ] Record original/replayed timings and immutable inputs/source fingerprints.

### Task2: Equivalent whole-target batch geometry

Files: guidance/camera_visibility_batch.py; test/test_p61_visibility_batch.py.
- [ ] Write and observe a failing test for the new vectorized geometry primitive, using independent known geometry and the unchanged scalar kernel as supplemental oracle.
- [ ] Implement batch transforms/projections with the original radius/calibration/range/FOV constraints and exact scalar fallback for invalid or boundary cases.
- [ ] Test random full poses, axial/radial ranges, camera-plane crossings, overflow/nonfinite/rotation failures and full validation consistency.
- [ ] Repeat same saved-request benchmark; require actual P95/P99 improvement, retain every sample/failure.

### Task3: Clock-domain observability and safety replay

Files: controllers/p4_follow_planner_node.py; scripts/p61_supply_analysis.py; test/test_p61_supply_analysis.py; test/test_p61_successor_replay.py.
- [ ] Record worker thread CPU plus ROS/monotonic start/end and parent completion/build/wire timing; no clock mixing or unbounded diagnostics queue.
- [ ] Replay cycles26/27,78/79/80,139/140/141 with real solver, stale acquisition, prediction updates, busy/completion/retry, delayed ACK and consecutive deadline failure tests.
- [ ] Verify original complete physical constraints and rejection behavior, not mock optimizer success.
- [ ] Commit meaningful offline implementation/analysis stage after full verification.

### Task4: Qualification and one F2

- [ ] Run full pytest from src/uav_control, four-package build/current prefix, flake8, shell syntax and diff checks.
- [ ] Independent scoped review; fix actionable safety/correctness blockers with tests.
- [ ] Compare full original pilot configs, freeze current source inventory, generate fresh qualification/readiness and inspect host sessions/devices.
- [ ] Execute one bounded60s linear/BCTRA-primary/IMM-shadow/nominal F2. Stop on unsafe/unstable gates; retain real failure if any.
- [ ] Audit owner/native/ACK/ACTIVE, FOV/dynamics, all latency tails, lease failures, input freshness and compare failed/healthy F2.

### Task5: Delivery

- [ ] Document implemented/static/offline/actual/unresolved statuses in docs/current/p61_successor_supply_review.md.
- [ ] Preserve SHA256 manifests and stage only explicit code/test/report/compact analysis paths.
- [ ] Commit and push p6/p61-successor-supply-fix. Attach any PR only if one is created; no auto-merge.
