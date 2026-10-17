# MINCO delivery implementation plan

> Execute inline with superpowers:executing-plans; one implementation agent. An independent frozen-source safety review is required before any new F2.

**Goal:** Make the actual worker path and complete original-epoch delivery budget agree, retaining125ms freshness and independent Tracker authority.

**Architecture:** Immutable per-request delivery plan locks one solver path and its deadlines. Real-time completion uses typed request/result before full audit snapshots. Successful and failed timing populations remain distinct but both inform delivery risk. Receiver uses nominal model demand and refreshes local legality after expensive validation.

**Tech stack:** Python/NumPy/SciPy, ROS2 Humble, existing MINCO/IMM/Tracker.

**Spec:** `data/experiments/20261017_p61r3_supply_budget_07/ARCHITECTURE.md`, current user task.

## Constraints and review focus

- Preserve original image/navigation epochs,125ms,450ms lease,150ms lead,50ms activation gap, physical constraints, all independent validation and failure fallback.
- No evaluator truth enters planning. No old evidence is edited. No automatic F3. The user separately authorizes a 200ms experimental branch and one newly qualified F2 only if the repaired125ms F2 remains unsatisfactory; no unchanged-case retry.
- Treat late failure costs as censored timing evidence, not geometric infeasibility or permanent execution permission.
- Test wrong-path fallback, stale/reversed clocks, identity mismatch, deadline during receiver validation, and audit failure without duplicated wire publication.
- Keep delivery requests/log replay restorable; no hidden inherited lease extension.

## Tasks

- [x] 1. Finite profiling: extract all36 original successor inputs and request113; record thread/process CPU, wall, scheduler runqueue and context switches under sequential and representative callback contention. Decide any worker-isolation change from measured evidence.
- [x] 2. `guidance/follow_delivery_budget.py`: immutable DeliveryPlan plus separate timing history including failures. Tests for native deadline, selected-path price, worker stop and aging. Wrap/restore only integrated requests; keep legacy inputs compatible.
- [x] 3. P4 runner and P47: select one affordable path before submit, bind plan, execute only that path, enforce its deadline, retain complete timing/outcome evidence and per-new-image budget rejection. Tests must prove no implicit route upgrade/fallback.
- [x] 4. Shadow runner completion and P4 publication: typed critical completion precedes recursive snapshots. All original audit records remain; no double proposal; current prediction revalidation and wire-time checks stay independent. Tests inject slow snapshots.
- [x] 5. Tracker and profiling: policy-specific model demand on proposal, final local receiver context refresh after curve checks, wall/CPU stage and final ACK timestamps. Tests for nominal/robust model demand and expired-during-validation rejection.
- [x] 6. Replay all36 with real MINCO and independent receiver math, including113 and consecutive lease identity/continuity. Compare recorded baseline costs/deadlines without claiming ROS/PX4 execution. If the bounded path still lacks125ms margin, stop flying and report the measured remaining limit.
- [ ] 7. Run relevant regressions, stable full regression once if necessary for qualification, build, lint/shell/diff/prefix checks; commit exact source/test/docs paths only. Freeze new case/config/source hashes and obtain real independent review; no self-review substituted.
- [ ] 8. Only if offline support and all gates pass: new qualification/Readiness and one isolated60s F2. Preserve failure and stop; no F3.

Execution ledger and artifacts: `data/experiments/20261017_p61r3_delivery_architecture_08/`.

## Latest conditional experiment authorization

Complete and qualify the125ms architecture first and run one bounded F2. If actual continuous control is still unsatisfactory, design an explicit case-bound200ms target observation/prediction contract while retaining125ms baseline and separate navigation/attitude freshness. Compare all36 original successors at125/150/200ms, including113; obtain fresh tests/build/independent review/qualification/Readiness before one bounded experimental F2. Preserve failure and stop; no F3.
