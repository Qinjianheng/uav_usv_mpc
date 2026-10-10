# P4.6 implementation plan

User specification: attachment `已粘贴的文本.txt`, P4.6, 2026-10-10.
The user authorizes autonomous implementation, offline validation and documentation;
no commit/push, original-checkout writes, or repeated SITL. Work in the requested
clean checkout at a00aebb0. Preserve all execution and freshness gates.

## Design

Reuse a private FlightGuidanceCore instance at its 50 ms command period. It consumes
only the request's target forecast. Integrate a jerk/acceleration bounded velocity
response with an explicit time constant and 10 ms numerical step. Keep measured
acceleration distinct from the command's placeholder acceleration. A missing actual
command is explicitly initialized from execution-start velocity, never claimed measured.
Sample the complete virtual P/V/A sequence at H/3, 2H/3 and H; map through the existing
MINCO implementation. Keep execution-start and accepted parent boundaries immutable.

Extend TrackingMincoObjective to accept an explicit frozen P/V/A sequence, weights,
and Q-only versus Q/T/yaw dimensions. The new mode uses FOLLOW P/V tracking, jerk,
visibility and jerk-switch continuity; all final candidates use validate_seed.
Optimize even infeasible initial seeds. Bound iterations/time, retain only fully
validated output, and label P44/D fallback separately. Select a default from evidence.

## Tasks / verification

1. RED/GREEN: pure original-FOLLOW rollout, altitude, shaping, response, forecast-only,
   straight/turn/slow/wrapped-yaw inputs, seed P/V/A and fit errors.
2. RED/GREEN: explicit-reference objective and gradients; fixed/Q/QT-yaw solve,
   invalid-initial repair attempt, strict failure, fallback, deadline and provenance.
3. Register engine, expose finite settings, retain p44_adaptive selection and gates.
   Verify runtime/config and existing tracker integration tests.
4. Four-arm F2/F3 independent and common-initial ideal rolling comparison at H=1.2
   and 1.6; write raw attempts, timing, fitting, altitude, constraints, continuity,
   fallback and source checksums to a new experiments directory. No truth input.
5. Recheck historical full_initial failures; focused tests, full pytest, four-package
   build, flake8, shell syntax, diff checks and ros2 package-prefix verification.
6. Review actual diff and evidence, write docs/current/p46_follow_guided_minco.md
   with negative results and limits, source manifest, exact reproduction commands.

## Progress

- Initial audit: clean requested HEAD and remotes verified; current virtual_follow
  duplicates a simplified law and direct refinement requires an already feasible seed.
- Two independent audits delegated: old regression failures; frozen-data availability.
- Implemented private original-FOLLOW rollout, explicit bounded virtual response,
  three-piece PVA mapping, full-reference objective, fixed/Q/QT engine and marked
  P44/D fallback. Added internal integration deadline and allocation step bounds
  after code review; preserved Accepted boundary and all execution gates.
- Dedicated algorithm and study tests pass. Historical behind-camera flake was
  isolated to the semantic test clock; added independent 30/31 ms deadline tests.
  Tracker E501 was already fixed in the starting HEAD; production revalidation
  and the historical log remain unchanged.
- Formal 32-configuration frozen F2/F3 matrix completed without --limit at H1.2/1.6:
  4856 independent solves and 4608 rolling rows including stopped/unattempted rows.
  Reviewed fitting, derivative extrema, fallbacks, matched windows and timing.
- Negative performance result: fixed initialization fits FOLLOW well, but jerk
  overshoot and conservative budget reserves reduce feasibility. Q/QT do not
  establish an overall advantage; no four-arm common after-20-second population.
  Keep new refinement none; retain P44 selection. No SITL was run.
- Final full regression: 1484 passed, 1 skipped; final four-package build passed.
  Report, phase/fit analyses, source hashes and verification logs are delivered in
  docs/current/p46_follow_guided_minco.md and the new P4.6 experiment directory.
  Implementation/offline work is complete; performance acceptance is explicitly
  not achieved. No staging, commit, push or original-checkout writes.
