# P5.1 FOLLOW convergence and continuous nominal MINCO

Binding spec: user attachment `5f32f1a3-bb64-4543-91cb-58d5dc640963/已粘贴的文本.txt`.
Base: 51007415a87d839f8b18ebe6e78c5b4418856ee6, master, initially clean.

1. A/C diagnosis: fingerprint prior evidence; replay the three P5 requests with stage profiles;
   reconstruct historical proposal/ACK/lease populations without another flight.
2. A/B: one receiver-owned eligibility policy, common-epoch FOLLOW error, 5s entry,
   performance hysteresis, 2s recovery then new 5s stability and three validated probes;
   identity-scoped bounded attempts. Planner consumes the receiver heartbeat.
3. C: preserve valid old/pending curves on solve failure; reject stale parents at activation;
   retain every ACK; schedule only latest inputs with measured handover margin and no TTL,
   lease, activation-window or safety-limit extension. Profile remaining-budget fallback.
4. D/E: genuine receiver/runner integration and fault replay; reference-height and PVA checks;
   full package pytest, flake8, four-package build, launch/interface/shell/diff checks.
5. G: switching, timing and validation reports, compact source/evidence SHA index, curves and
   event timeline; distinguish test/replay evidence from actual flight evidence.
6. F: only after A–E pass AND a new explicit human flight authorization, at most one F2
   (25s actual MINCO), then one F3 (60s) if F2 qualifies. Failure stops further flights.

Rulings: the detailed user design and autonomous A–E execution supersede additional skill
design approval gates. Keep the explicitly restored master; do not create/switch branches,
commit, push, modify the old checkout or frozen evidence. No new control publisher.

Review focus: loss of pending or active authority between callbacks; stale/replayed heartbeat
and ACK; parent expiry across computation; probes cannot execute; cooldown cannot block
valid successors; observer failure remains fatal; startup freshness is an inhibition only
while original Tracker still reports a legal FOLLOW path.
