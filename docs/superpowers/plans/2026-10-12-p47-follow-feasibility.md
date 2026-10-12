# P4.7 MINCO diagnosis and FOLLOW feasibility implementation plan

> **For agentic workers:** Use superpowers:executing-plans for the algorithm work;
> independent Stage A physical and lifecycle audits are delegated under
> superpowers:dispatching-parallel-agents. Follow the user's ordered stages.

**Goal:** Diagnose actual P4.5 execution before improving FOLLOW-derived MINCO
terminal conditions, derivative feasibility and budget scheduling on frozen inputs.

**Architecture:** Keep the existing forecast, MINCO map, strict validator, Accepted
parent boundary and single Tracker publisher. Preserve P4.6 as an unchanged
comparison engine; add a distinct P4.7 research engine with fixed total time and
bounded terminal A / V+A / Q+V+A corrections. Scheduling uses original expiry,
publication reserve and bounded fallback; it never renews input or lease deadlines.

**Tech Stack:** Existing ROS2/Python/NumPy/SciPy/MINCO, pytest, matplotlib, pyulog.

**Spec:** User P4.7 attachment at
`/home/qin/.codex/attachments/c27bec95-fb58-4671-befc-5881676f98d7/已粘贴的文本.txt`.

## Global constraints

- Work only in `/home/qin/data/uav_usv_mpc`; start HEAD353c3e1, clean, remotes verified.
- No commit/push, old-checkout writes, historical evidence rewrites, broad staging.
- No BCTRA/camera/USV/PX4 parameter or hard constraint changes; no extra setpoint publisher.
- Preserve native epochs, 125ms freshness, Accepted PVA/yaw/rate boundaries, nominal
  lease and all isolated-session gates. Truth is offline evaluation only.
- Default no SITL. At most one bounded F2 only after all five user gates pass.
- User authorizes autonomous ordinary stages; no extra staged approval flow.

## Review focus

- Invalid/expired/new-prediction or forged Accepted parent cannot produce a valid output.
- Endpoint corrections never alter initial PVA or create vertical reference drift.
- Exact derivative peaks, not sampled jerk proxies, decide hard feasibility.
- Budget exhaustion preserves a previously validated curve only until original deadline.
- Partial rolling windows cannot masquerade as all-arm after20-second improvement.

## Task 1: Stage A, actual execution diagnosis

Files: new `scripts/p47_physical_diagnosis.py`, `scripts/p47_execution_timeline.py`,
their focused tests; output new `data/experiments/20261010_p47_follow_guided/`.

- [x] Inventory P4.5 complete raw logs/configs/ULogs and source SHA256.
- [x] RED/GREEN same-native-epoch PVA and XY/Z error decomposition, no extrapolation.
- [x] RED/GREEN direct ownership transitions, lifecycle matching, causal rejection history.
- [x] Produce new CSV/JSON/PNG; report confirmed vs missing signals separately.
- [x] Integrate `docs/current/p47_execution_root_cause.md`; decide whether a major
  execution design issue prevents later SITL. No control strategy modification here.

## Task 2: Stage B, reproduce and explain derivative failures

Files: new offline derivative diagnosis and test; use immutable P4.6/F3 sources.

- [x] Record every rejected initial PVA/Q/T, FOLLOW interval jerk and exact MINCO
  maxima with piece/time; classify simultaneous speed/acceleration/yaw failures.
- [x] Single-variable counterfactuals for terminal A, V+A, Q, T; retain endpoint
  position and initial boundary, record bounded changes and strict outcomes.
- [x] Derive the smallest useful correction interface from evidence before code.

## Task 3: Stage B/C, separate P4.7 engine

Files: new `guidance/follow_terminal_correction.py`, new
`controllers/follow_guided_feasible.py`, minimal registration, optional pure rollout
performance path; dedicated tests. Exact names may change with recorded evidence.

- [x] RED/GREEN bounded terminal A/V and optional Q, initial PVA immutable, finite
  coefficients, exact extrema, correction failure and full FOV/sea checks.
- [x] FOLLOW P/V + desired USV viewpoint P/V + jerk + terminal adjustment penalty;
  reuse existing coefficient map and full geometry proxy. No yaw/T joint optimization.
- [x] RED/GREEN real remaining deadline scheduling, explicit pre/post budget ablation,
  strict final validation, separately marked P44/D fallback and expiry invalidation.
- [x] Compare 5/10/20ms numerical baseline; optimize allocation/repeated work only
  after equivalence evidence. Keep original P4.6 comparison behavior selectable.

## Task 4: Stage D, frozen comparison and decision

Files: extend/reuse P4.6 study helpers in new P4.7 script/tests, immutable new outputs.

- [x] Four arms: P44/D, frozen P46 fixed, improved fixed boundary, low-dimensional
  correction; separate endpoint-variable and pre/post scheduling ablations.
- [x] Full F2/F3, H1.2/H1.6, same inputs/model/horizon/expiry/common initialization;
  independent and single-episode rolling, no invented F2 5Hz samples.
- [x] Report independent/overall/fallback and rejection classes, timing quantiles,
  matched horizontal/height/relative-velocity errors, extrema/FOV and boundary offsets.
- [x] Decide default from evidence. Failed targets remain explicit; if offline or
  Stage A gates fail, skip SITL and deliver independent research mode.

## Task 5: Stage F/G, review and delivery

- [x] Requested one independent review, fixed Important metadata loss with RED/GREEN;
  reviewer quota ended before complete final read-through (reported limitation).
- [x] Actual full pytest count, four packages, scoped flake8, all shell syntax,
  messages/launch/config, source/baseline hashes, diff and ROS prefix checks.
- [x] Reports `p47_minco_closed_loop_diagnosis.md` and
  `p47_follow_guided_feasibility.md`; tables, source index and standalone figures.
- [x] No automatic integration or expanded experiments.

## Ledger / rulings

- 2026-10-12: Stage A raw offline_evidence and ULogs exist for all four main MINCO F2 runs.
- Ruling: work in user-specified clean research checkout, no new worktree or commits —
  preserves the requested workspace and source/data paths; Git index remains untouched.
- Ruling: detailed task already supplies purpose, alternatives, order and stop conditions;
  autonomous execution overrides additional skill approval/commit stages.
- Task 1 complete: physical 8 and lifecycle 9 focused tests pass; immutable-source
  checks pass. H1.6 reference drift dominates ACTIVE height error; full-owner
  excursions are worse and reported separately. 131 direct lease expiries, upstream
  116 no replacement / 9 rejected / 6 missed activation. Stage B may proceed;
  actual plant counterfactual remains unverified, so SITL gate is not yet satisfied.
- Task 2 complete: historical H1.2 162 hJ failures exactly reproduced, all piece0,
  155 at start. A/VA remove 1/4; Q/QVA remove 151/155, only 38 full admissions.
  H1.6 historical112 similarly QVA removes110 but24 full admissions. Main C/D
  preregistered QVA (fixed/refined) from these facts, before new comparison.
- Task 3 implementation: 23 new correction/scheduler/selector tests RED→GREEN;
  related P46 tests combined53 pass; root-config flake8 clean. New selector is
  opt-in and current execution YAML unchanged. C2 1214 same-input-H comparisons:
  scalar10 matches original10 exactly, 5/20ms not equivalent, retain10ms.
- Ruling: preserve legacy control_effort implementation for frozen algorithms;
  its three cross coefficients are wrong. New objective/analysis uses independent
  exact integral and retains legacy-reported field. Cost if wrong: legacy users
  still see incorrect effort reports until a separately validated correction.
- Final review finding resolved: preserve boundary, returned repair and final selected
  candidates independently, with validation and retained flag. RED/GREEN verified.

## Final experiment decision, 2026-10-12

- First full matrix `formal_20261012_1` retained as failed design: pursuit-weighted
  initialization raised frozen line436 hJ from 7.07 to 54.09; independent BVLS/KKT
  confirmed the quadratic solver was solving its intended (unsuitable) objective.
- Second full matrix `formal_20261012_2`: bounded exact jerk-energy initialization,
  104 configurations, both H, no input changes. F3 single final 450/489, historical
  hJ fixed 155/162; matched after20 H1.2 XY .492m vs P44 1.351m, H1.6 .432 vs .505m.
- Do not replace default: F2/H1.2 matched after20 XY .089→.188m and relative velocity
  .067→.219m/s; full-target minimum FOV margin also decreases. Optional refinement
  adds cost with little additional quality. Gate 2 fails, therefore SITL skipped.
- A concrete bounded F2 preparation wrapper has five-gate check, exclusive single
  attempt claim, 25 observed FOLLOW seconds and 1m height-error stop. Unit tested;
  not invoked and no actual attempt claim or runtime grant created.
- Fresh regression: 1568 passed, 1 skipped; four-package build, scoped flake8 and
  12 static/message/launch/shell/diff checks passed. Original 63 source input files,
  frozen baseline 46 hashes and all six formal analysis package manifests verified.
- Current HEAD changed externally to 741000c during execution. Preserve that commit;
  no automatic staging/commit/push. Formal2 source before/after 173 hashes match,
  later docstring/format changes have identical executable AST and saved snapshots.
