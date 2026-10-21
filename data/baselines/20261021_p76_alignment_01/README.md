# P76 32° / physical clearance / PX4 interface evidence

Source checkout: codex/p76-paper-faithful-yolo-dkf, starting HEAD 0bb5f4909967c7c4fb285bed1b269a96fac551e5.
Date: 2026-10-21. This is a newly curated, independent evidence package; older baselines are unchanged.

The report is docs/current/p76_32deg_physical_clearance_px4.md. No armed flight or capture qualification is claimed.

- config/: full final ROS/contract configuration, actual session SDFs and manifest, DDS before/after. The three SDFs are selected model sources; full meshes stay in the recorded PX4/source model locations and are pinned by the model manifest.
- audit/: fresh source fingerprints, passing current available regression (2363 passed, 2 skipped), initial red evidence, package/build identity and formal flight refusal. Two R11-dependent test modules were excluded explicitly in the report.
- probe_03/: real disarmed mode/feedback proof, exact wire packets and original native PX4 ULog compressed losslessly. This is neither measured physical force nor descent/braking calibration.
- failed_probe_01/02/: actual unsuccessful diagnostics retained before the verified third probe. Complete local runs remain in data/experiments/20261021_p76_px4_probe_01 through _03.
- visibility/: finite static 32° Gazebo geometry, with source contract/manifest/frame index and selected original images. Level FOLLOW images: 00000,00011,00022; fully occluded pitch=-0.25 FOLLOW images: 00004,00015,00026. All 33 frames remain in the source experiment. Render and final runtime contracts differ only in later reference-datum settings; the geometric camera/follow layout is pinned independently in summary.json.

file_manifest.json records each selected source path, source SHA256, saved path/encoding and saved SHA256. For gzip entries, decompression restores the original source bytes exactly. SHA256SUMS covers every package member except itself. Verify with `sha256sum -c SHA256SUMS` from this directory.

Do not promote this package to A/B/C/D qualification: measured thrust/rates/braking are missing and 3/15 static FOLLOW cases are completely occluded. The formal launch gate remains fail-closed.
