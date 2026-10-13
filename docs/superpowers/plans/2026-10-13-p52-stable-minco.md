# P5.2 Stable MINCO Execution Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans inline; one fresh independent final qualification reviewer.

**Goal:** Repair evidenced blockers in the existing FOLLOW→MINCO chain and prepare a bounded actual F2/F3 validation.
**Architecture:** Keep the Tracker as the only executor and admission authority. Preserve original physical epochs, strict validation, fixed P47/C H1.6 and P44/D. Diagnose saved inputs before changing performance code.
**Tech Stack:** Python, ROS2 Humble, NumPy/SciPy, pytest, colcon.
**Spec:** `/home/qin/.codex/attachments/9f9afc58-1c1a-4ec6-9cf6-18ba23c90763/已粘贴的文本.txt`

## Global Constraints

- Base master 413b54fefa35c01f8d235aa553249d2071de23f5; initially clean. No branch switch, commit, push, historical rewrite or old checkout writes.
- Fixed P47/C qva, refinement=false, adaptive, H1.6, P44/D; 125ms raw TTL, 450ms lease and 50ms activation window unchanged.
- Full dynamics, attitude, whole-target FOV and sea validation remain independent at Receiver.
- A-E autonomous; F2 and conditional F3 require explicit new attempt authorization. No flight is currently authorized.
- Preserve P4/P5/P5.1 evidence byte-for-byte. New output: data/experiments/20261013_p52_offline_01/.

## Review Focus

- Configured entry thresholds must govern both Tracker and read-only warmup geometry.
- Invalid or physically impossible latest KF data must not silently reuse an older healthy row to accumulate stability.
- Old boot/mission/generation events must not restore a retired execution identity, including after a rejected heartbeat.
- Recovery, input interruption and two solve failures must never leave an expired ACTIVE; valid successors must bypass entry cooldown.
- Current qualification and the new attempt root must bind actual tested sources without inheriting old flight attempts.

### Task 1: Receiver eligibility and identity consistency (A)

Files: follow_admission.py, trajectory_tracker_node.py, p4_follow_planner_node.py, p5_live_health.py; new test_p52_execution_chain.py.

- [x] Reproduce hardcoded settling thresholds, invalid latest KF fallback, identity change before tick and retired heartbeat restoration.
- [x] Pass AdmissionConfig explicitly to shared settling, reset on explicit invalid latest inputs, consume identity loss immediately, retain heartbeat identity high-water/retired boots.
- [x] Test configured tight/loose entry, stationary/reversing target, large initial error, pending/active identity changes and event ordering; run affected P5.1/Tracker regressions.

### Task 2: Current saved-input timing and supply diagnosis (B/C)

Files: new scripts/p52_offline_analysis.py; performance code only if a profile proves a blocker.

- [x] Fingerprint protected evidence; replay the three exact historical requests with current guarded solver, recording raw ages, actual wall costs and outcomes, without percentile extrapolation.
- [x] Attribute old 131 lease losses and distinguish producer/receiver/worker waits from missing data. Preserve unknown CPU/executor causality.
- [x] For any evidenced code bottleneck, RED→GREEN minimal equivalent/conservative optimization, original strict validators and regressions.

### Task 3: Extended protocol faults and fresh verification (D)

Files: reusable p51_receiver_replay.py plus new p52_receiver_replay.py/test_p52_receiver_replay.py.

- [x] Demonstrate >20 actual Receiver successor activations in a labelled no-flight fixture.
- [x] Cover two solve failures, interrupted input, changed prediction, sustained performance loss and lawful recovery/reentry, besides original faults.
- [x] Run fresh focused/full pytest, four-package build, flake8, shell, installed message, launch and Git checks.

### Task 4: Independent qualification and bounded run preparation (E/H)

Files: new p52_minco_sitl.py and three current P52 reports plus compact audit package.

- [x] Preserve old P51 interfaces; give P52 its own attempt root and explicit readiness/source-bound admission.
- [x] Fresh read-only reviewer audits complete diff/spec/ledger; fix substantive findings with failing tests and final whole-suite verification.
- [x] Verify environment conflicts without launching simulation. Report limitations and all fifteen requested answers; qualify only offline readiness.
- [x] Prepare concrete F2/conditional F3 commands. Ask once for new attempt authorization only after all authorized offline work is complete.

Ruling: the user's explicit autonomous execution overrides extra skill plan/design approvals. Keep current master and retain the ledger because auto-commit is forbidden. Flight permission remains a separate user requirement.
