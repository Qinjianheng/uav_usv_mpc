"""Offline causal first-failure context; does not qualify or modify the recorded flight."""
from pathlib import Path
import gzip
import json
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path('/home/qin/data/uav_usv_mpc')
sys.path.insert(0, str(ROOT/'src/uav_control'))
from uav_control.common.p76_body_geometry import navigation_vertices, lowest_world_z
from uav_control.common.p76_native import NativePoseHistory, NativeReferenceHistory

RUN = ROOT/'data/experiments/20261021_p761_01/session_A1_03'
OUT = ROOT/'data/experiments/20261021_p762_01'
rows = [json.loads(l) for l in (RUN/'calibration.jsonl').read_text().splitlines()]
phase_start = {}
annotated = []
for r in rows:
    if r['kind'] == 'phase':
        phase_start[r['phase']] = r['stamp']
    t = r.get('stamp', phase_start.get(r['phase'], 0)+r['phase_elapsed'])
    annotated.append((t, r))
first_index = next(i for i, (_, r) in enumerate(annotated) if r['kind'] == 'failure')
failure_time = annotated[first_index][0]
prefix = rows[:first_index]
paired = [r for r in prefix if r['kind'] == 'paired_state'][-1]
state = next(r for r in reversed(prefix) if r['kind'] == 'state' and r['stamp']==paired['stamp'])
pose_row = min((r for r in rows if r['kind']=='pose'), key=lambda r:abs(r['stamp']-state['stamp']))
rotation = Rotation.from_quat(np.array(pose_row['q'])[[1,2,3,0]]).as_matrix()
home = next(r for r in rows if r['kind']=='state')
last_status = [r for r in rows if r['kind']=='wire' and r['topic']=='vehicle_status'][-1]
last_landed = [r for r in rows if r['kind']=='wire' and r['topic']=='vehicle_land_detected'][-1]
context = [(t,r) for t,r in annotated if failure_time-2<=t<=failure_time+.15]
with gzip.open(OUT/'first_failure_context_2s.jsonl.gz','wt') as stream:
    for t,r in context:stream.write(json.dumps(dict(analysis_epoch=t, recorded=r))+'\n')
ages = [r['age'] for r in rows if r['kind']=='paired_state']
identity = json.loads((RUN/'identity.json').read_text())
model_root = Path(json.loads((RUN/'p76_session_model.json').read_text())['model_root'])
points = navigation_vertices(identity['model']['collision_envelope'], model_root)
poses, refs = NativePoseHistory(), NativeReferenceHistory()
for r in rows:
    if r['kind']=='pose':poses.add(r['stamp'],r['q'],r['omega'],[0.,0.,0.])
    elif r['kind']=='reference':refs.add(r['stamp'],r['ref_alt'])
# Preserve complete histories offline, as the live bounded history intentionally retains only seconds.
qrows = [r for r in rows if r['kind']=='pose']
refrows = [r for r in rows if r['kind']=='reference']
clearances = []
for r in rows:
    if r['kind']!='state' or r['phase'] in ('WAIT_NATIVE','PRESTREAM','TAKEOFF','LAND'):continue
    near = [p for p in qrows if abs(p['stamp']-r['stamp'])<.026]
    refnear = [p for p in refrows if abs(p['stamp']-r['stamp'])<.026]
    ph,rh = NativePoseHistory(),NativeReferenceHistory()
    for p in near:ph.add(p['stamp'],p['q'],p['omega'],[0.,0.,0.])
    for p in refnear:rh.add(p['stamp'],p['ref_alt'])
    try:
        rot=ph.at(r['stamp']).rotation;sea=rh.at(r['stamp'])
        clearances.append(sea-lowest_world_z(points,r['p'],rot))
    except ValueError:pass
result = dict(first_failure=annotated[first_index][1],failure_epoch=failure_time,
              control_source=paired, paired_state=state,
              speed_norm=float(np.linalg.norm(state['v'])),
              tilt_projection=float(np.max(np.abs(rotation[:2,2]))),
              horizontal_from_first_state=float(np.linalg.norm(np.array(state['p'][:2])-home['p'][:2])),
              paired_age=dict(count=len(ages),p50=float(np.median(ages)),p95=float(np.percentile(ages,95)),max=max(ages)),
              native_pose_bracket_failures=sum(r.get('reason')=='P76_NATIVE_POSE_BRACKET' for r in rows),
              minimum_active_calibration_clearance=min(clearances),
              final_status=last_status['values'],final_landed=last_landed['values'],
              context_rows=len(context), scope='Offline analysis of immutable actual A1_03, no qualification')
(OUT/'first_failure_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k not in ('final_status','final_landed')},indent=2))
# Compact actual motor/collective/mode timeline around the command change.
for t,r in context:
    if r['kind'] in ('phase','mode','failure'):
        print(t-failure_time,r)
