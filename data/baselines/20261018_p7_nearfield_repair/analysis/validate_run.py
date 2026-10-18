"""Strict physical result window, ownership, independent point and integrity checks."""
import bisect
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

ws = Path('/home/qin/data/uav_usv_mpc')
run = ws/'data/experiments/20261018_p7_nearfield_repair_s_01'
out = Path(__file__).parent
summary = json.loads((run/'summary.json').read_text())
rows = [json.loads(x) for x in (run/'offline_evidence.jsonl').open()]
result = summary['capture_result']
end = result['stamp']
start = end-result['elapsed']
nav = sorted([r for r in rows if r['kind']=='uav' and r['valid']], key=lambda r:r['stamp'])
boat = sorted([r for r in rows if r['kind']=='ideal' and r['valid']], key=lambda r:r['stamp'])


def point(records, epoch):
    stamps = [r['stamp'] for r in records]
    i = bisect.bisect_left(stamps, epoch)
    if i < len(stamps) and abs(stamps[i]-epoch) < 1e-9:
        return np.array(records[i]['p']), [stamps[i],stamps[i]]
    assert 0 < i < len(stamps)
    a,b = records[i-1:i+1]
    assert 0 < b['stamp']-a['stamp'] <= .05
    f = (epoch-a['stamp'])/(b['stamp']-a['stamp'])
    return (1-f)*np.array(a['p'])+f*np.array(b['p']), [a['stamp'],b['stamp']]


p,pb = point(nav,end)
q,qb = point(boat,end)
selected = [r for r in rows if start <= r.get('stamp',r['receipt']) <= end]
guidance = [r for r in selected if r['kind']=='p7_guidance']
diag = [r for r in selected if r['kind']=='diagnostic']
refs = [r for r in selected if r['kind']=='reference']
vision = [r for r in csv.DictReader(next((run/'evaluator').glob('*_vision.csv')).open())
          if start <= float(r['measurement_stamp']) <= end]
invalid = [r for r in vision if r['observation_valid'].lower()!='true']
minco = [r for r in rows if r['kind']=='diagnostic' and r['status']=='MINCO_FOLLOW'
         and r['stamp'] < start]
pre = [r for r in rows if r['kind']=='diagnostic' and minco[0]['stamp']<=r['stamp']<start]
acks = [r for r in rows if r['kind']=='follow_ack' and r['state'] in ('REVOKED','EXPIRED')]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
manifest = json.loads((run/'SHA256SUMS.json').read_text())
bad = [name for name,h in manifest.items() if sha(run/name)!=h]
snapshot = json.loads((run/'source_snapshot.json').read_text())
source_bad = [name for name,h in snapshot['files'].items()
              if name.startswith(('src/','scripts/')) and sha(ws/name)!=h]
r = dict(source_run=str(run),strict_window=[start,end],
         evaluator_distance_m=result['minimum_distance'], evaluator_elapsed_s=result['elapsed'],
         independent_point_distance_at_result_m=float(np.linalg.norm(q-p)),
         result_brackets=dict(navigation=pb,target=qb),
         guidance_samples=len(guidance), diagnostic_samples=len(diag),
         diagnostic_status=dict(Counter(r['status'] for r in diag)),
         visible_all=all(r['visible'] for r in diag), locked_all=all(r['locked'] for r in diag),
         image_samples=len(vision), invalid_observations=len(invalid),
         minimum_red_pixels=min(int(r['red_pixel_count']) for r in vision),
         image_error_u_max=max(abs(r['error_u']) for r in guidance),
         image_error_v_max=max(abs(r['error_v']) for r in guidance),
         post_result_guidance_samples_excluded=sum(r['kind']=='p7_guidance' and r['stamp']>end
                                                  for r in rows),
         velocity_only=all(all(math.isfinite(v) for v in r['v']) and
                           not any(math.isfinite(v) for v in (*r['p'],*r['a'])) for r in refs),
         pre_y_first_minco=minco[0]['stamp'],
         pre_y_observed_minco_span_s=minco[-1]['stamp']-minco[0]['stamp'],
         pre_y_owner_status=dict(Counter(r['status'] for r in pre)),
         pre_y_minco_sample_fraction=sum(r['status']=='MINCO_FOLLOW' for r in pre)/len(pre),
         revocations=[{k:r[k] for k in ('stamp','state','reasons')} for r in acks],
         run_manifest_mismatches=bad, current_source_mismatches=source_bad,
         raw_sha256=sha(run/'offline_evidence.jsonl'), source_fingerprint=summary['source_fingerprint'],
         scope='Physical result epoch, no extrapolation or time fit. Valid detection differs from center/whole-object containment. Raw summary retained; unbounded post-result guidance extrema are excluded here.')
assert not bad and not source_bad
assert not invalid and r['visible_all'] and r['locked_all'] and r['velocity_only']
assert r['independent_point_distance_at_result_m'] <= .5
assert r['pre_y_minco_sample_fraction']==1.
assert all(a['state']=='REVOKED' and a['reasons']==['P7_GUIDANCE_ENTERED'] for a in acks)
(out/'scoped_validation.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps(r,indent=2))
