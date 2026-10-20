#!/usr/bin/env python3
"""Read-only A/B/C/D eligibility audit; never pretend a stopped gate is a flight test."""
import argparse
import json
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/uav_control'))
from uav_control.common.p76_contract import load_contract  # noqa: E402
from uav_control.common.p76_readiness import (  # noqa: E402
    sha256, source_fingerprint, validate_calibration,
)


def assess(contract_path, model_root, geometry_summaries, output):
    """Freeze exact current inputs and explain why dependent SITL stages must stop."""
    output.mkdir(parents=True, exist_ok=False)
    contract = load_contract(contract_path)
    px_root = Path(contract['dynamics']['px4_model_root']).parents[3]
    dds_path = px_root/'src/modules/uxrce_dds_client/dds_topics.yaml'
    dds = yaml.safe_load(dds_path.read_text())
    published = {item['topic'] for item in dds['publications']}
    a_reasons = []
    if '/fmu/out/vehicle_thrust_setpoint' not in published:
        a_reasons.append('EXECUTED_THRUST_FEEDBACK_NOT_EXPORTED_BY_CURRENT_PX4_DDS')
    try:
        validate_calibration(contract, model_root, ROOT)
    except (ValueError, OSError) as error:
        a_reasons.append(str(error))
    geometries = [json.loads(path.read_text()) for path in geometry_summaries]
    geometry_failed = any(g['completely_occluded_follow'] > 0 for g in geometries)
    result = dict(
        scope='Preflight eligibility only. No PX4/Gazebo flight was launched by this audit.',
        contract_sha256=sha256(contract_path), sources=source_fingerprint(ROOT),
        px4_dds=dict(path=str(dds_path), sha256=sha256(dds_path), publications=sorted(published)),
        geometry_sources={str(path): sha256(path) for path in geometry_summaries},
        stages=dict(A=dict(state='NOT_RUN', eligible=False, reasons=a_reasons),
                    B=dict(state='NOT_RUN', eligible=False, reasons=[
                        'A_NOT_QUALIFIED']+(['BOTH_ALLOWED_LAYOUTS_HAVE_AIRFRAME_OCCLUSION']
                                            if geometry_failed else [])),
                    C=dict(state='NOT_RUN', eligible=False, reasons=[
                        'CURRENT_A_AND_25S_B_REQUIRED', 'NO_NATIVE_025_CAPTURE_EVIDENCE']),
                    D=dict(state='NOT_RUN', eligible=False, reasons=['CURRENT_C_REQUIRED'])),
        paper_closed_loop_qualified=False)
    (output/'eligibility.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result['stages'], indent=2))
    return 2


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--contract', type=Path,
                        default=ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')
    parser.add_argument('--model-root', type=Path, required=True)
    parser.add_argument('--geometry-summary', type=Path, action='append', required=True)
    options = parser.parse_args()
    raise SystemExit(assess(options.contract, options.model_root,
                            options.geometry_summary, options.output))
