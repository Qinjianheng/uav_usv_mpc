# P5.3 Continuous MINCO Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans inline, then one independent final qualification review.

**Goal:** Explain the actual F2 lease losses, repair the demonstrated remaining supply blocker, and prepare one source-bound F2 revalidation.
**Architecture:** Preserve the sole Tracker, the accepted-reference protocol and fixed P47/C H1.6/P44/D. Restore actual recorded events and measured delays separately from explicitly injected counterfactuals; missing events remain unknown.
**Tech Stack:** Python, ROS2 Humble, PX4/Gazebo evidence, pytest, colcon.
**Spec:** `/home/qin/.codex/attachments/e1788edf-6de3-4f16-a599-a22f6f9ec0c3/已粘贴的文本.txt`

## Global Constraints

- Current clean master dfb0daa83ffc1d0c7e65486307f6c50f1a57db73. No agent commit/push, branch changes, history cleanup or old-checkout writes.
- p47_follow_guided, H1.6, qva, refinement=false, adaptive, P44/D; retain 125ms raw TTL, 450ms lease, 50ms activation window and full dynamics/attitude/sphere-FOV/sea checks.
- Three authorized P52 F2 rounds are consumed, including the in-progress round that finished when P53 arrived. All failed sustained ownership. No further F2 without a concrete new authorization; no F3 before full F2 pass.
- New raw evidence: data/experiments/20261013_p53_offline_01. Sealed P52/older outputs stay byte-for-byte.

## Review Focus

- Actual replay cannot invent missing local callback events or turn recorded prefix checks into new geometric proof.
- Waiting for fresh inputs must stop at real parent/start limits, with no extra worker retries.
- Wakeup clock reversal, expired parent and callback delay must still revoke/reject using actual completion time.
- Successful solver output may expire during synchronous serialization; publication records must reflect the actual wire population.
- Compare full predefined observation windows, retaining all cooldown and lawful FOLLOW recovery time.

### Task 1: Actual causal timeline and measured populations

Files: scripts/p53_f2_analysis.py, test/test_p53_f2_analysis.py, docs/current/p53_f2_root_cause.md.

- [x] Test missing/duplicate events, full-window ownership and first-blocker ordering.
- [x] Analyze immutable F2 rounds2/3: all proposals/ACKs/owners, per-loss upstream events, schedule/solve/receive/start margins and actual PX4 evidence.
- [x] Produce source-hashed analysis and report; distinguish actual observation from inference.

### Task 2: Recorded-event protocol replay and bounded faults

Files: scripts/p53_event_replay.py, test/test_p53_event_replay.py.

- [x] Restore recorded curves, contexts and measured control/validation/ACK times; compare observed lifecycle, including >20 actual successors and START_MISSED.
- [x] Add separately labelled delay/failure counterfactuals for the 17 requested faults; retain strict TTL/lease/parent/context and cooldown invariants.
- [x] Extend accepted-start tests for clock reversal, parent loss and injected callback delay; do not assert unrecorded executor internals.

### Task 3: Minimal demonstrated supply repair

Files: determined by Task1 profiling; tests in test/test_p53_supply.py.

- [x] Reproduce the dominant remaining blocker using the exact saved requests and measured budgets.
- [x] Make the smallest scheduling/performance change with RED-to-GREEN evidence; retain mathematical validators and all deadlines.
- [x] Repeat matched-input replay and affected protocol tests; record residual failures rather than hiding them.

### Task 4: Qualified delivery

Files: scripts/p53_minco_sitl.py, current P53 reports, docs/current/audit/p53_follow_minco/.

- [x] Full pytest from src/uav_control, four-package build, flake8, shell/launch/messages/prefix and Git checks.
- [x] Independent final source review; resolve substantive findings and verify protected SHA manifests/current source hashes/live preflight.
- [x] Generate readiness/qualification and unique one-shot next-F2 command. Answer all fifteen questions with actual or unverified scope.
- [x] Prepare the request for one new bounded F2 after all authorized offline work is reviewable. F3 remains conditional.

Ruling: the user's autonomous execution instruction overrides extra plan/design approval checkpoints. Execute inline, preserve the ledger, and never infer unlimited flight authorization.

## 2026-10-13 autonomous continuation

The latest human instruction explicitly authorizes inspection, repairs, moderate metric adjustments and experiments without another approval checkpoint. Preserve every consumed attempt; use at most three new, separately claimed P53 diagnostic rounds, each justified by a concrete change or diagnostic question. F3 remains conditional on actual F2 evidence.

- [ ] Reproduce P53 round1 pre-planner stop: one 50ms lock dropout, visible target, healthy native navigation/Offboard, then immediate lock recovery. Separate observer stop policy from unchanged Tracker fail-closed admission.
- [ ] Add RED-to-GREEN bounded observer recovery (maximum 200ms), preserving immediate physical/Offboard/input/owner faults and full F2 denominator. Persist per-round namespaces without overwriting round1.
- [ ] Build and qualify updated sources, then run the publication-priority candidate in F2. Investigate real supply failures before changing planner dynamics or TTL/lease.
- [ ] Trace the late original-FOLLOW disturbance to the earliest measured perception/KF event; avoid blind Q/R or gain tuning.
- [ ] Complete physical F2 and conditional F3 validation, independent review, necessary regressions and updated delivery reports. Keep every unpassed criterion explicit.
