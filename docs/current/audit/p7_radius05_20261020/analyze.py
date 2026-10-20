"""Read-only native-geometry comparison of 0.25 m and 0.50 m P7 runs."""
import hashlib
import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

ROOT = Path('/home/qin/data/uav_usv_mpc')
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'src/uav_control'))
from uav_control.control.p7_spatial_guidance import capture_entry_reference


def read(path):
    return [json.loads(line) for line in path.open() if line.strip()]


def threshold_entry(pairs, radius):
    """Sample or legal held-target segment only; never interpolate a target teleport."""
    entries = []
    for row in pairs:
        if row['distance'] <= radius:
            entries.append(dict(stamp=row['stamp'], source='NATIVE_SAMPLE',
                                distance=row['distance']))
    for a, b in zip(pairs, pairs[1:]):
        if not b['segment_valid']:
            continue
        assert math.dist(a['target_position'], b['target_position']) <= 1e-9
        r = np.subtract(a['uav_position'], a['target_position'])
        delta = np.subtract(b['uav_position'], b['target_position'])-r
        aa, bb, cc = float(delta@delta), float(2*r@delta), float(r@r-radius**2)
        disc = bb*bb-4*aa*cc
        if aa <= 1e-15 or disc < 0:
            continue
        fraction = (-bb-math.sqrt(disc))/(2*aa)
        if 0 <= fraction <= 1:
            entries.append(dict(stamp=a['stamp']+fraction*(b['stamp']-a['stamp']),
                                source='HELD_TARGET_SEGMENT', distance=radius,
                                left_native=a['native_stamp'], right_native=b['native_stamp'],
                                fraction=fraction))
    return min(entries, key=lambda r:r['stamp']) if entries else None


def main():
    definitions = {
        '28deg_025m_original': ROOT/'data/experiments/p7_manual_20261019_112340_740393470_984611',
        '28deg_050m_current': ROOT/'data/experiments/20261020_p7_radius05_capture_01',
    }
    report = dict(scope='Two individual, phase-unmatched runs. Threshold change is not evidence of improved control accuracy. Current run is censored by capture/pause.', runs={})
    manifest = {}
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    for label, run in definitions.items():
        summary_file = next((run/'evaluator').glob('*_summary.json'))
        evaluator = json.loads(summary_file.read_text())
        start = evaluator['intercept_started_at']
        capture_file = next((run/'evaluator').glob('*_capture.jsonl'))
        pairs = [r for r in read(capture_file) if r['kind']=='EVALUATED' and r['stamp']>=start]
        assert all(r['maximum_bracket']<=.05+1e-9 and 0<=r['decision_stamp']-r['stamp']<=.125+1e-9 for r in pairs)
        assert all(abs(math.dist(r['uav_position'],r['target_position'])-r['distance'])<1e-9 for r in pairs)
        closest = min(pairs, key=lambda r:r['distance'])
        evidence_file = run/'offline_evidence.jsonl'
        evidence = read(evidence_file)
        observations = [r for r in evidence if r['kind']=='observation' and start<=r['stamp']<=pairs[-1]['stamp']]
        invalid = next((r for r in observations if not r['valid']), None)
        diagnostics = [r for r in evidence if r['kind']=='diagnostic' and start<=r['stamp']<=pairs[-1]['stamp']]
        status = {}
        for a,b in zip(diagnostics,diagnostics[1:]):
            status[a['status']] = status.get(a['status'],0)+max(0.,b['stamp']-a['stamp'])
        entries = {}
        for radius in (.25,.5):
            entry = threshold_entry(pairs,radius)
            if entry:entry['seconds_after_y'] = entry['stamp']-start
            entries[str(radius)] = entry
        config_file = run/'config/flight.yaml'
        config = yaml.safe_load(config_file.read_text())
        summary_path = run/'summary.json'
        summary = json.loads(summary_path.read_text()) if summary_path.exists() else None
        result = dict(run=str(run), y=start, configured_radius=config['intercept_evaluator_node']['ros__parameters']['evaluation_capture_radius'],
                      outcome=evaluator['outcome'], native_samples=len(pairs),
                      native_minimum=dict(closest, seconds_after_y=closest['stamp']-start,
                          horizontal=math.dist(closest['uav_position'][:2],closest['target_position'][:2]),
                          vertical=abs(closest['uav_position'][2]-closest['target_position'][2])),
                      threshold_replay=entries, first_invalid=invalid,
                      first_invalid_seconds=None if invalid is None else invalid['stamp']-start,
                      observation_valid_fraction=sum(r['valid'] for r in observations)/len(observations),
                      diagnostic_status_seconds=status,
                      minimum_native_height=min(-r['uav_position'][2] for r in pairs),
                      summary=summary)
        near_changes = []
        previous_near = False
        for row in pairs:
            near = math.dist(row['uav_position'][:2], row['target_position'][:2]) < 1.
            if near != previous_near:
                near_changes.append(dict(seconds_after_y=row['stamp']-start, inside=near))
            previous_near = near
        result['native_horizontal_near_changes'] = near_changes
        report['runs'][label]=result
        t=[r['stamp']-start for r in pairs]
        axes[0].plot(t,[r['distance'] for r in pairs],label=label)
        near=[r for r in pairs if r['distance']<1.]
        axes[1].plot([math.dist(r['uav_position'][:2],r['target_position'][:2]) for r in near],
                     [abs(r['uav_position'][2]-r['target_position'][2]) for r in near],label=label)
        axes[1].scatter(result['native_minimum']['horizontal'],result['native_minimum']['vertical'],s=40)
        for f in (summary_file,capture_file,evidence_file,config_file,summary_path,run/'source_snapshot.json',run/'working_tree.patch'):
            if f.exists():manifest[str(f.relative_to(ROOT))]=hashlib.sha256(f.read_bytes()).hexdigest()
    report['nominal_geometry']=[dict(target_height=h,uav_height=.33,vertical=.33-h,
        default_reference_distance=math.hypot(.3,.33-h),
        allowed_horizontal_025=math.sqrt(.25**2-(.33-h)**2) if abs(.33-h)<=.25 else None,
        allowed_horizontal_050=math.sqrt(.5**2-(.33-h)**2) if abs(.33-h)<=.5 else None)
        for h in (-.15,0.,.08,.10,.15)]
    report['level_camera_corridor']=[dict(angle=28.,minimum_center_depth=d,
        entry=capture_entry_reference(np.eye(3),camera_pitch_down=math.radians(28.),minimum_center_depth=d).tolist(),
        entry_distance=float(np.linalg.norm(capture_entry_reference(np.eye(3),camera_pitch_down=math.radians(28.),minimum_center_depth=d)))) for d in (.01,.02)]
    for radius in (.25,.5):
        axes[0].axhline(radius,ls='--',color='gray',label=f'{radius:g} m criterion')
        theta=np.linspace(0,np.pi/2,180)
        axes[1].plot(radius*np.cos(theta),radius*np.sin(theta),'--',label=f'{radius:g} m boundary')
    axes[0].set(xlabel='Seconds after accepted Y',ylabel='Native 3-D distance (m)',title='Capture criterion and actual distance',yscale='log')
    axes[1].axvline(.3,ls=':',color='black',label='0.30 m horizontal reference')
    axes[1].set(xlabel='Horizontal separation (m)',ylabel='Absolute vertical separation (m)',title='Near-field native geometry',xlim=(0,1),ylim=(0,.65))
    axes[1].set_aspect('equal',adjustable='box')
    for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(OUT/'radius_comparison.png',dpi=170);plt.close(fig)
    (OUT/'comparison.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    (OUT/'input_SHA256SUMS.json').write_text(json.dumps(manifest,indent=2)+'\n')
    for label,result in report['runs'].items():
        print(label,json.dumps({k:result[k] for k in ('outcome','native_minimum','threshold_replay','first_invalid_seconds','observation_valid_fraction','minimum_native_height')},indent=2))


if __name__=='__main__':
    main()
