# P764 selected frozen evidence

Source commit `c8e90ccfe2d6c828abeb25654f6b5f606df54e7c`; base `f0f6c4a33b0ae56f1868ee47d72247dd363f9315`.

**A1 NO-GO.** Historical unsafe-pulse refusal passes, the independently analyzed protocol is blocked. Real E1 has zero valid 3D observations and missed executor cycles; E2 fails its fixed 0.5 s stress case. No A/B/detector qualification, A1/A2/B/C/D flight or native 0.25 m capture was produced. Original flight budget is unchanged at 3/5. This package contains one separately authorized unarmed diagnostic, not an old-budget flight session.

- `e0_historical_replay.json` and `e0_new_protocol.json`: distinct historical and prospective proof, exact source/raw/model identities. No executable A1 schedule.
- `nonflight_01/`: same-session full typed detector RLE packets, invalid/valid observations, original native epochs, exact callback inputs, phase labels, state outputs and cleanup. RGB-D headers are recorded; pixel arrays and full per-sample graph history are absent.
- `e1_*.json`: actual measurements, matched old/new callback replay and explicit postmortem. The original `one_yolo=false` includes startup discovery; final graph has one detector. All E1 qualification remains false.
- `e2_control_replay.json`, `e2_handover_replay.json`, `e2_initialization_review.json`: frozen-source software math/dispatch/native-epoch tests. `e2_initialization_pre_review.json` preserves its previous candidate identity, not current qualification.
- `qualification_audit.json`, `offline_gates.json`, `source_identity.json`: missing A/B/C/D, unchanged budget, current source/config/physical identity and fail-closed NO-GO.
- `checks/`: lossless logs, including the first broad style failure, review red tests, final 2493-passed regression, build and lint. The two unavailable R11 historical-input tests are explicitly excluded. Repeated broad confirmation followed concrete new review/gate fixes; no repeat simulation.

The unarmed diagnostic exposed a temporary-stdin EOF busy loop in PX4. Its original console bytes are retained compressed, with original byte SHA/length in `nonflight_01/raw_log_identity.json`; `gzip -dc PX4_SITL.txt.gz` restores them exactly. The wrapper fix has a subprocess regression; no second diagnostic measured its runtime improvement. `executed_sources.tar.gz` records the collection-time collector/launch/config and the harness snapshot restored by reversing the single stdin fix; `executed_terminal_wrapper.py.gz` contains the actual executed wrapper bytes. These are distinct from the later corrected source commit. A matching trace cannot qualify a changed runtime automatically.

`historical_E0_models.tar.gz` preserves the complete historical model tree. All actual diagnostic model files, including the manifest, are byte-identical to this tree. `model_variants.json` lists every original file hash and restoration instructions, avoiding a second copy of identical mesh assets. Current complete contract/profile are in `config/`. Original P761/P762 logs remain at their original paths and their frozen packages; this package does not rewrite them.

Verify this package from its directory with `sha256sum -c SHA256SUMS`. To replay, source ROS and this workspace, then use the committed `p763_exit_replay.py`, `p764_real_replay.py`, `p764_handover_replay.py` and `p764_qualification_audit.py` with new output paths. Historical evidence records its original absolute input paths and hashes. Reconstruct model copies at a new location and pass `--historical-model-root` for E0; do not alter historical identity files. Experiments/videos outside this explicit selection are not committed.

Full Chinese interpretation: `docs/current/p764_architecture_validation.md`.
