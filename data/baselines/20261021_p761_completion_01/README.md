# P761 mechanical repair and failed A1 evidence

Date: 2026-10-21. Branch: codex/p76-paper-faithful-yolo-dkf.
Start: 8ad7fa604122cf029164f1aca4d2af28289f2ba2.
Final candidate implementation: 6f44dfc4ff9f30e943356579501824348f479661.

Report: docs/current/p761_paper_architecture_completion.md.
Full local sources: data/experiments/20261021_p761_01/.

**No A/B/detector qualification or actual capture is claimed.** A1_01 failed after the native float32 JSON relay crashed; A1_02 failed at native pose pairing after arming and landed/disarmed. The final causal pairing candidate passed offline/unit checks, not another flight. A2/B/C/D were not run. Two of five primary session slots and one actual 33-pose Gazebo-only render were used; the same-stage one-retry limit stopped further flights.

- config/: complete final contract/profile, selected session SDFs and manifest. Full unchanged meshes remain at their pinned source paths. Default P7 and external DDS/binary are unchanged.
- geometry/: A/B/C analytical screening; one actual A 33-pose render, all 33 positive and 7 negative original images/labels; sampled geometry is not fast flight qualification.
- detector/: actual pinned YOLO replay plus live ROS inference during static rendering. No retraining and no formal detector qualification.
- session_A1_01/ and session_A1_02/: immutable raw calibration/wire records and logs, complete measured identities, unsuccessful analyses/results, cleanup and each original native PX4 ULog. gzip restores the exact source bytes. A1_01's large console EOF loop is retained losslessly.
- audit/: current 2380-passed available regression, 102-passed directed checks, build/lint/prefix, independent phase wiring probe with synthetic observations, failure/repair proofs, fingerprint revocations and final unqualified candidate identity. R11 historical-input modules remain explicitly excluded. Each flight's identity remains distinct from the later candidate.

file_manifest.json records original source paths/hashes and saved encoding/hashes. Verify package bytes with `sha256sum -c SHA256SUMS` here. Restoring gzip members yields their source_sha256. This package is new evidence; previous baselines were not rewritten. Do not copy failed-run booleans or model TWR into qualified A/B manifests.
