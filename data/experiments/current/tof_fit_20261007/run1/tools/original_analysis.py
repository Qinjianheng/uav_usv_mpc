import csv,collections,json,math
from pathlib import Path
import numpy as np
root=Path('/home/qin/data/uav_usv_mpc/data/experiments/current/tof_fit_20261007/run1')
result=json.loads((root/'probe_summary.json').read_text())['result']
end=result['stamp']['sec']+result['stamp']['nanosec']*1e-9
csvroot=Path('/home/qin/data/uav_usv_mpc/data/experiments/current')
rows=list(csv.DictReader(next(csvroot.glob('*20261007_104013*vision.csv')).open()))
# Select at the mission result by receipt for validity accounting; accuracy
# always also requires the preserved acquisition time and causal truth match.
pre=[x for x in rows if x['receipt_stamp'] and float(x['receipt_stamp'])<=end]
valid=[x for x in pre if x['valid']=='True' and x['truth_available']=='True' and 0<float(x['measurement_stamp'])<=end and math.isfinite(float(x['position_3d_error']))]
analysis={'result_cutoff_ros_s':end,'selection':'receipt <= result; valid accuracy also requires 0 < acquisition <= result, aligned truth and finite error; no post-timeout metrics','raw_rows':len(rows),'rows_before_result':len(pre),'valid_accuracy_samples':len(valid),'rejections_before_result':dict(collections.Counter(x['rejection_reason'] for x in pre)),'by_phase':{}}
for phase in sorted({x['approach_phase'] for x in pre}):
 v=[x for x in valid if x['approach_phase']==phase];stats={}
 for k in ('position_3d_error','error_x','error_y','error_z','camera_center_error_3d'):
  e=np.array([float(x[k]) for x in v if x[k] and math.isfinite(float(x[k]))]);stats[k]={'n':len(e),'rmse_m':None if not len(e) else float(np.sqrt(np.mean(e**2))),'p95_abs_m':None if not len(e) else float(np.quantile(abs(e),.95)),'max_abs_m':None if not len(e) else float(max(abs(e)))}
 analysis['by_phase'][phase]=stats
# Stable FOLLOW window: exclude 0.5s after entry; stop at FAR_GUIDANCE.
probe=[json.loads(x) for x in (root/'probe.jsonl').open()]
phases=[x['data'] for x in probe if x['kind']=='phase']
def seconds(stamp):return stamp['sec']+stamp['nanosec']*1e-9
follow=next(seconds(x['stamp']) for x in phases if x['state_name']=='FOLLOW')+.5
far=next(seconds(x['stamp']) for x in phases if x['state_name']=='FAR_GUIDANCE')
v=[x for x in valid if follow<=float(x['measurement_stamp'])<far];e=np.array([float(x['position_3d_error']) for x in v]);analysis['stable_follow_window']={'begin_ros_s':follow,'end_ros_s':far,'n':len(v),'rmse_m':None if not len(e) else float(np.sqrt(np.mean(e**2))),'p95_m':None if not len(e) else float(np.quantile(e,.95)),'max_m':None if not len(e) else float(max(e))}
main=list(csv.DictReader(next(csvroot.glob('*20261007_104013*mission_1.csv')).open()))
active=[x for x in main if x['intercept_started']=='True' and x['intercept_elapsed_time'] and float(x['intercept_elapsed_time'])<=result['elapsed_time'] and x['distance'] and math.isfinite(float(x['distance']))]
nearest=min(active,key=lambda x:float(x['distance']));analysis['nearest_during_task']={k:nearest[k] for k in ('time','intercept_elapsed_time','phase','distance','horizontal_distance','vertical_error','target_locked','kf_state_age','planner_failure_reason','sea_safety_state','body_clearance')}
(root/'analysis.json').write_text(json.dumps(analysis,indent=2));print(json.dumps(analysis,indent=2))
