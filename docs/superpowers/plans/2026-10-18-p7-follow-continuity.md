# P7 FOLLOW continuity repair

> Execute inline in the existing `codex/p7-visual-guidance` checkout. User authorized diagnosis, repair and bounded simulation validation; no additional handoff or agent delegation.

**Goal:** Diagnose and repair the two fallbacks in `20261018_p7_variable_capture_f2_01`, then measure actual ownership in the same straight 2 m/s, 25 s regression.

**Architecture:** Keep receiver safety decisions authoritative. Validate a completed successor against its original request and handover, rather than constructing a discarded later request. Preserve evidence of any remaining input-delivery loss before changing another component.

**Spec:** User request plus `AGENTS.md` and `docs/current/p7_variable_capture_20261018.md`.

**Constraints:** 125 ms original input age; 50 ms native bracket; 450 ms nonrenewable lease; no truth feedback, fixed delay, wider capture radius, blind filter tuning, F3 or hardware. Preserve old experiments and frozen baseline. Source changes and builds finish before X. Reuse the user's named workspace and current branch; no push.

## Task 1: Original successor completion

- [x] Save the original two fallback timelines and source hashes in a new analysis directory.
- [x] Extend the real-runner completion regression to periodic scheduling; observe failure before the production change.
- [x] Exercise original request expiry, latest invalid/stale input, mission/generation and parent changes.
- [x] Reuse the existing completion-input validation in periodic execution; preserve independent publication/receiver checks.
- [x] Run affected tests, inspect every failure and run the full package suite.

## Task 2: Receiver input delivery and actual regression

- [x] Record the exact input context when execution is revoked, if existing logs cannot distinguish delayed callbacks from missing publication.
- [x] Build, check package prefix and run one isolated straight F2 with the original observer and gates.
- [ ] If it fails, retain the run, diagnose the first failure, add a failing regression and make one evidence-based fix before another run.
- [ ] Measure actual MINCO/FOLLOW ownership, uninterrupted windows, fallback reasons, native input freshness and cleanup; never infer ownership from solver feasibility.

## Task 3: Delivery and review

- [ ] Run full tests, flake8, required shell syntax, build and diff checks on final sources.
- [ ] Review the diff separately, especially completion validation, explicit invalid inputs, receiver identity and lease boundaries.
- [ ] Save source-bound report and selected lightweight baseline with source paths and SHA256; link current documentation.
- [ ] Commit only explicit code/test/doc/baseline paths; leave ignored raw data local and do not push.

## Review focus / progress

Periodic versus delivery scheduling; stale original request even with a newer prediction; invalid latest prediction; changed mission/generation/active parent; no new live permission from mathematical feasibility.

Initial evidence: plan 98 was feasible with original start 1792300165.2127955 and parent deadline 1792300165.2907522. At completion 1792300165.1008198, constructing a discarded request at now+0.15 crossed deadline-0.05 and produced `NO_SUCCESSOR_ACTIVATION_MARGIN`. First fallback preceded by cached source 1792300151.706685, expiring at 1792300151.831685; new KF source 1792300151.7547023 existed at the independent observer before revocation. Its precise Tracker consumption time is absent from the old log.

Progress: periodic completion RED (1 failed / delivery control passed); completion and context GREEN (75 passed). Full first run: 2154 passed, 2 skipped, 1 new-test indentation lint failure; corrected and rerun. Two packages built; prefix verified. No simulation has started while sources/builds changed.

F2_01: healthy/completed; MINCO 100%, longest 24.9435317516 s, no fallback ACKs, f3_eligible true, owned cleanup clean. Full suite 2155 passed / 2 skipped / 3 warnings (37.49 s); flake8 and build passed. F2_02 is the final identical-runtime repeat; no F3 is planned.
