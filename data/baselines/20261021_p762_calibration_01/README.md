# P762 actual A1_03 calibration failure

Start HEAD: 2609bbf0096f531f84a691c0d128e25b4d842dd9.
Implementation/source identity: a4779cd28e7dce53ade41f0dc9492aa70ad1547c.
Report: docs/current/p762_bodyrate_calibration_validation.md.

Exactly one authorized new A1 was run. The old pose-pairing crash did not recur; hover passed and seven thrust levels were recorded. RESET after the maximum pulse failed the unchanged 7m/s bound (actual ascent 7.407m/s); executed collective stayed at 1 for about 240ms after the Velocity request. Fixed-window TWR was 2.818 below 2.85. Rate/maneuver stages were not reached. LAND was requested, but actual landing/disarm was not achieved within 20s. The bounded SITL was then stopped and cleaned up.

Stage assessment is completed=false, qualified=false, success=false, with exit code 2. No A2/B/C/D, no retraining/render, no A qualification. No standards were relaxed. New primary sessions used: 1; task-global count: 3/5. The unconsumed conditional A2 slot was not used after A1 failure.

- session_A1_03/: original raw calibration/wire/PVA/pose/reference/commands/collective/mode/motor records, actual identity, unchanged unsuccessful analysis/result, independent stage assessment, logs, cleanup and native PX4 ULog (original 04_49_41.ulg copied losslessly).
- audit/: frozen identities, directed red/green tests, lint/build/config/prefix, actual failure metrics and two-second context, offline reproduction/plot scripts, charts, historical-failure integrity and final process preflight.
- config/: complete unchanged contract/profile; actual unchanged SDF and collision envelope are pinned by identity and P761's config/models evidence. Do not confuse model-derived TWR3 with the measured incomplete flight.

The pairing replay/flight counts use different sampling events; the report explains this distinction. A2 braking/response data are absent; the 240ms A1 collective withdrawal measurement is not A2 braking qualification.

file_manifest.json records selected source SHA256, saved SHA256 and encoding. gzip restores source bytes exactly. Verify this package with `sha256sum -c SHA256SUMS`. Old baselines and A1_01/02 were not rewritten. The prior 2380-test full regression belongs to P761; this task used 20 relevant tests, not another full run.
