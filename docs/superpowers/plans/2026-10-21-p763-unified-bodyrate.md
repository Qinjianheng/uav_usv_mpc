# P763 implementation and offline gates

User's written specification: /home/qin/.codex/attachments/fad5fd87-1896-41ba-9f0c-e2c59c50c584/已粘贴的文本.txt.
Start 718295a920a7eddbd3ffc5224ec0e38662baf6cc, clean, same branch. The supplied specification authorizes the architectural change and E0–E2; E3+ require explicit per-stage flight authorization. No new SITL/render/training in this implementation stage. Existing three sessions remain counted.

## Design and interfaces

- Keep paper PNG/LOS/IBVS/attitude/thrust formulas and gains byte-identical. Existing Minco planning/admission unchanged. Only P76 opt-in execution and estimation semantics change; default P7 unchanged.
- `common.p76_phase.estimator_active(node)` is true in FOLLOW_ACTIVE/Y_HANDOVER/PAPER_INTERCEPT_ACTIVE. `follow_active` remains a separate consumer-authorization gate for predictors/planners. ACK schema2 exposes active/ready/source plus `control_authorized`; phase/epoch labels never reset filter state at Y. Clock/frame reset does reset.
- `control.p76_minco_bodyrate_tracker.BodyrateTracker(contract, mass, maximum_thrust, thrust_map)` converts `step(reference11, position3, velocity3, rotation3x3, dt, angular_rates3)` into rates/thrust/force_newtons/acceleration; reference11 is same-epoch P/V/A/yaw/yaw_rate. It also offers `initialize(rates3, thrust)` for execution continuity. Bounded position/velocity feedback, jerk/tilt/force/rate/slew guards. Gains are explicit offline candidates, not qualified physical control.
- `P76TrackerRuntime` has sole 200Hz PX4 output. Legacy 20Hz callback remains supervisor/admission only; its private reference sink cannot publish PX4. Active admitted MINCO is sampled at actual control epoch with lease/source bounds, never replayed 20Hz velocity. Non-MINCO positioning uses an explicit smooth waypoint reference; no Y target3D feedback.
- ModeCoordinator schema2 separates `guidance_owner` from estimator activity and `px4_interface=BODY_RATE`. Y revokes MINCO and seeds the paper controller/sole output limiter with current native pose/rates/executed thrust. No normal mode change. Recovery is independent Body-rate braking or one latched native LAND/failsafe.
- `p763_calibration_control` contains conservative pulse admission (7m/s, .24s measured worst delay, model force bounds, altitude/clearance) and recorded-failure replay. Calibration can bootstrap a separately labelled unqualified physical map from A1_03 only after exact model/PX4 verification; formal Tracker still requires qualification.
- Calibration initial takeoff may use existing velocity setup; after first body preparation THRUST/BODY_BRAKE/BODY_HOLD/rates/maneuver/A2 all stay Body-rate. No change to seven levels/durations or standards. New A schema2 requires actual body braking, native LAND takeover and real landed/disarmed evidence, not renamed old velocity recovery. Stable A/B/detector paths unchanged.

## Tasks and review focus

1. Add red tests then implement continuously running filters, Y posterior continuity, ACK schema2 and authorization; update observer. Test duplicate/stale, fast attitude, clock/frame reset and no Y target3D consumption. E1 benchmark distinguishes synchronized real-data coverage from fixture results; never retime unrelated recordings and call them native.
2. Add red tests then implement body-rate reference sampler/outer loop/output limiter/recovery and unique runtime outlet. Test sign, same-epoch PVA, expired leases, clock/native absence, rate/thrust continuity and late MINCO reappearance. Small finite offline gain/lag stability grid, not online tuning.
3. Add conservative calibration admission/brake/hold and schema2 assessment/manifest. E0 reconstructs actual A1_03 velocity and .24s delay; refusal of an unsafe pulse is a valid safety result, not a successful dynamics measurement.
4. Run E0/E1/E2 offline tools, directed tests then at most one justified available full regression, lint/shell/build/prefix and immutability checks. Review meaningful physics/runtime/false-qualification risks once.
5. Freeze final identities; report docs/current/p763_unified_bodyrate_parallel_dkf.md, preserve new independent evidence/SHA and all prior failures. Local commit explicit paths only, no merge/push. Any offline input/safety gap remains a real flight blocker; no fake A/B or capture success.

Execution uses the supplied user design, inline implementation plus independent scoped parallel work (dispatching-parallel-agents). No repeated design approval is sought for already authorized work. Final E3 authorization is a separate user requirement and is not inferred from the broad task.
