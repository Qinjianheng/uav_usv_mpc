# P762 bounded calibration

Start 2609bbf0096f531f84a691c0d128e25b4d842dd9, clean. User supplied the design and authorized one A1_03 and conditional one A2_01, no B/C/D or retries.

1. Verify native pairing and repair only deterministic epoch defects; separate completed/qualified/success and require actual landing, immutable logs/current identity.
2. Directed red/green tests, lint/shell/build, audit unchanged model/contract/PX4/weight, verify isolation and freeze all source changes before A1.
3. Run A1_03 once, preserve raw logs and first-failure two-second context. Stop on failure; static patch allowed, no extra flight.
4. Only if A1 qualifies and frozen identities remain unchanged run A2_01 once, then independently generate/validate A if all requirements pass.
5. Report actual dynamics, pairing ages, stage assessments, qualification and process cleanup; directed final checks and local commit, no push/merge/full-regression repetition.

Outcome: one A1_03 executed, no retry/A2/B/C/D. Pairing fault absent; hover passed; maximum-pulse RESET caused actual 7.407m/s ascent bound violation after 240ms collective withdrawal latency. Thrust-window TWR2.818 below2.85; rates/maneuver not reached; landing/disarm not attained within20s. Stage assessment false, no A. Twenty directed tests and build/lint checks pass. Implementation a4779cd28e7dce53ade41f0dc9492aa70ad1547c; preserve raw logs/ULog and report.
