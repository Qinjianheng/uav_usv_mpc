"""Recompute live admission, phase timing and system-load summaries from raw artifacts."""
from pathlib import Path
import collections
import json
import sys
import numpy as np

root=Path(sys.argv[1]).resolve()

def stats(values):
 a=np.asarray(values,dtype=float)
 a=a[np.isfinite(a)]
 return dict(count=len(a),p50=float(np.percentile(a,50)),p95=float(np.percentile(a,95)),
             p99=float(np.percentile(a,99))) if len(a) else dict(count=0)

def summarize(rows):
 complete=[r for r in rows if r['event']=='completion']
 out=dict(events=len(rows),completions=len(complete),
   core_statuses=dict(collections.Counter(r.get('core_result',{}).get('solver_status') for r in complete)),
   admission_statuses=dict(collections.Counter(r['admission_status'] for r in complete)),
   reject_statuses=dict(collections.Counter(r['admission_status'] for r in rows if r['event']=='reject')),
   published_valid=sum(r.get('published_valid',False) for r in complete),
   marker_valid=sum(r.get('final_marker_valid',False) for r in complete),
   whole_cycle_s=stats([r['whole_cycle_time'] for r in complete]))
 out['phase_seconds']={key:stats([r['core_result']['timing'][key] for r in complete
                   if key in r.get('core_result',{}).get('timing',{})]) for key in (
       'initialization','preparation','minco_optimization','validation',
       'yaw_construction_within_optimization','total')}
 out['metrics']={key:stats([r['core_result']['metrics'][key] for r in complete
                   if key in r.get('core_result',{}).get('metrics',{})]) for key in (
       'visible_fraction','minimum_horizontal_margin','minimum_vertical_margin','follow_rmse',
       'jerk_integral','maximum_horizontal_speed','maximum_horizontal_acceleration',
       'maximum_horizontal_jerk','maximum_yaw_rate','maximum_tilt_rad')}
 out['violated_constraints']=dict(collections.Counter(key for r in complete
       for key,val in r.get('core_result',{}).get('constraint_violations',{}).items() if val>1e-6))
 out['input_age_seconds']={key:stats([r['request']['now_stamp']-r['request']['context'][key]
       for r in complete]) for key in ('navigation_stamp','observation_stamp','prediction_source_stamp')}
 out['execution_lead_s']=stats([r['request']['context']['execution_start_stamp']-
     r['request']['now_stamp'] for r in complete])
 out['prediction_expires_before_execution']=sum(r['request']['context']['prediction_valid_until']
     <=r['request']['context']['execution_start_stamp'] for r in complete)
 out['scheduling_and_serialization_over_solver_s']=stats([r['whole_cycle_time']-
       r['core_result'].get('solve_time',r['whole_cycle_time']) for r in complete])
 return out

rows=[json.loads(s) for p in (root/'shadow').glob('*.jsonl') for s in p.read_text().splitlines()]
result=dict(all=summarize(rows),research_only=True,accepted_by_tracker=False)
if (root/'diagnostic_shutdown.json').exists():
 split=json.loads((root/'diagnostic_shutdown.json').read_text())['stamp']
 result['full_phase']=summarize([r for r in rows if r['event_stamp']<split])
 result['light_phase']=summarize([r for r in rows if r['event_stamp']>=split])
for p in root.glob('*_load.json'):
 load=json.loads(p.read_text())
 result[p.stem]={key:dict(mean=float(np.mean([r[key] for r in load['samples']])),
                         **stats([r[key] for r in load['samples']])) for key in (
     'cpu_percent','rss_mib','research_cpu_percent','research_rss_mib')}
(root/'analysis.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
