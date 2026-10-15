"""Shadow-only model payloads must not occupy the BCTRA consumer's critical callbacks."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np

from test_p6_imm_node import real_sequence_780
from test_progress_follow import model
from uav_control.controllers.follow_mpc_shadow_node import prediction_from_message
from uav_control.tracking.imm_target_predictor_node import prediction_to_message
from uav_control.guidance import p6_follow


def test_bctra_keeps_bounded_raw_evidence_without_decoding_until_imm_needs_it(monkeypatch):
    configured = model()
    configured.p6_policy = p6_follow.P6Policy()
    runtime = p6_follow.P6Runtime(configured)
    engine, _, _, result = real_sequence_780()
    encoded = p6_follow.json.dumps(engine.diagnostics(result))
    calls = []
    decode = p6_follow.json.loads

    def recorded_decode(value):
        calls.append(value)
        return decode(value)

    monkeypatch.setattr(p6_follow.json, 'loads', recorded_decode)
    runtime.diagnostic(SimpleNamespace(data=encoded))
    parsed = prediction_from_message(prediction_to_message(result, .001))
    bctra = replace(parsed, model='BCTRA_BOUNDED_Z')
    assert runtime.prediction(bctra) is bctra
    assert not calls
    attached = runtime.prediction(parsed)
    assert len(calls) == 1
    assert np.asarray(attached.model_positions).shape == (3, 41, 3)
    assert attached.uncertainty_identity == (1, 780, result.source_stamp,
                                             result.observation_stamp)


def test_deferred_bad_or_mismatched_diagnostics_cannot_grant_a_model_identity():
    configured = model()
    configured.p6_policy = p6_follow.P6Policy()
    runtime = p6_follow.P6Runtime(configured)
    _, _, _, result = real_sequence_780()
    parsed = prediction_from_message(prediction_to_message(result, .001))
    for _ in range(100):
        runtime.diagnostic(SimpleNamespace(data='not JSON'))
    assert len(runtime.pending_diagnostics) <= 16
    runtime.diagnostic(SimpleNamespace(data='{"mission_id":1,"sequence_id":780}'))
    assert runtime.prediction(parsed) is parsed
    assert not parsed.uncertainty_identity
    assert not runtime.pending_diagnostics
