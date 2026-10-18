# P7 variable-speed capture validation

Goal: Sequential full X → actual MINCO FOLLOW → Y → IBVS/PNG captures for variable-speed eight, S and Z. Preserve 2–4 m/s profiles, 0.50 m 3-D radius, 125 ms input validity, original lock/dynamics/sea guards, native epochs and single Tracker ownership. Truth remains evaluation/offline only.

Execution: Single agent, autonomous per user's explicit instruction. Reuse the current task branch. No phase confirmation, automatic parameter search or blind retries. Each first attempt is bounded; each failed run is preserved, diagnosed and summarized before one-cause repair and bounded retest. Any required production change starts with a failing regression, followed by appropriate tests/build and current-source qualification. Keep data/videos and frozen evidence intact; explicit stage commits, no push.

- [x] Confirm clean source, package prefix, previous current-source qualification and isolated preflight. Copy passive stack diagnosis, excluding expected post-capture pause.
- [x] Run variable-speed eight capture once; analyze capture, physical states, ownership, image validity and world pause. First attempt healthy: 13.1649 s, 0.481009 m, radius 0.50 m, source unchanged.
- [ ] Run variable-speed S capture once; apply the same independent checks.
- [ ] Run variable-speed Z capture once; apply the same independent checks.
- [ ] For any failure: trace raw timing/input/control, preserve findings, add a failing test, repair one cause, verify and restart the affected bounded experiment. Report any unreproduced condition without hiding it.
- [ ] Save curves, full configuration/source fingerprints, negative evidence and SHA256; self-review, update current documentation, verify cleanup and commit explicit paths.

Review focus: Transient input loss must not acquire stale authority; terminal pause supersedes live freshness only after an authoritative result; capture feedforward resets on invalid input; target detection and full-frame containment are distinct; world pause and supplemental repeated-native-pose proof remain separate.

## Diagnosed S qualification failure

S 01 reached the original radius (10.8699 s / 0.479015 m) and paused, but `healthy=false`: one recovery setpoint contained finite position/acceleration after near-field visual loss. Native trace proves the guarded brake/climb was already active; the next SAFE_RECOVERY branch mapped it through the legacy position interface. A rejected offline feedback candidate is preserved and not applied. The scoped fix keeps P7 airborne search/recovery in velocity-only mode while retaining the identical guarded command and legacy non-P7 behavior. Three P7 recovery tests failed before implementation; the three non-P7 cases passed. All six pass afterward; 118 focused regressions pass. Full-suite lint found a transient indentation issue and two existing P7 docstring issues; those are corrected before final full-suite rerun. Raw S failure remains unchanged.
