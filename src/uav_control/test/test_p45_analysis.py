"""Independent time weighting and unknown-gap cases for actual control ownership."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))
from p45_execution_analysis import ownership  # noqa: E402


def test_ownership_counts_time_not_number_of_messages():
    rows = [dict(stamp=t, status=s) for t, s in (
        (0., 'FOLLOW'), (.05, 'MINCO_FOLLOW'), (.15, 'FOLLOW'), (.2, 'FOLLOW'))]
    result = ownership(rows)
    assert result['minco_fraction'] == pytest.approx(.5)
    assert result['fallback_count'] == 1


def test_missing_control_evidence_is_not_attributed_to_minco():
    rows = [dict(stamp=t, status='MINCO_FOLLOW') for t in (0., .05, 1.)]
    result = ownership(rows)
    assert result['seconds']['UNKNOWN_GAP'] == pytest.approx(.95)
    assert result['minco_fraction'] == pytest.approx(.05)
