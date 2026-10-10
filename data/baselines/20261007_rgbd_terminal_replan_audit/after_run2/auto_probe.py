import json, sys, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import Bool, String
from uav_usv_interfaces.msg import MissionState, InterceptResult, TargetState, ControllerDiagnostic, InterceptTrajectory, TargetPrediction
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node as GzNode

root=Path(sys.argv[1]);root.mkdir(parents=True,exist_ok=True)
stream=(root/'probe.jsonl').open('x',buffering=1)
def record(kind,obj):
 stream.write(json.dumps({'receipt_monotonic':time.monotonic(),'kind':kind,'data':obj})+'\n')
rclpy.init();node=Node('tof_validation_probe')
q=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
state={'ready':False,'ready_since':None,'phase':'','follow_since':None,'x':None,'y':None,'result':None,'result_time':None}
def ready(m):
 state['ready']=m.data
 if m.data and state['ready_since'] is None:state['ready_since']=time.monotonic()
def phase(m):
 if m.state_name!=state['phase']:
  print('PHASE',m.state_name,flush=True);record('phase',message_to_ordereddict(m))
  state['phase']=m.state_name
  state['follow_since']=time.monotonic() if m.state_name=='FOLLOW' else None
def result(m):
 if state['result'] is None:
  state['result']=message_to_ordereddict(m);state['result_time']=time.monotonic()
  record('result',state['result']);print('RESULT',state['result'],flush=True)
def target(m):record('truth_evaluation_only',message_to_ordereddict(m))
node.create_subscription(Bool,'/simulation/impact/flight_ready',ready,latched)
node.create_subscription(MissionState,'/mission/state',phase,latched)
node.create_subscription(InterceptResult,'/simulation/impact/result',result,q)
node.create_subscription(TargetState,'/target/state',target,q)
node.create_subscription(TargetState,'/tracking/target_state',lambda m:record('kf',message_to_ordereddict(m)),QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))

def controller(m):record('controller',message_to_ordereddict(m))
def trajectory(m):record('trajectory',message_to_ordereddict(m))
def prediction(m):record('prediction',message_to_ordereddict(m))
node.create_subscription(ControllerDiagnostic,'/control/diagnostic',controller,q)
node.create_subscription(InterceptTrajectory,'/planning/intercept_trajectory',trajectory,q)
node.create_subscription(TargetPrediction,'/planning/target_prediction',prediction,QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))

pub=node.create_publisher(String,'/simulation/impact/command',q)
gz=GzNode()
def stats(m):record('world_stats',{'paused':m.paused,'sim_time':m.sim_time.sec+m.sim_time.nsec*1e-9,'iterations':m.iterations})
gz.subscribe(WorldStatistics,'/world/default/stats',stats)
start=time.monotonic();repeats=[]
try:
 while rclpy.ok() and time.monotonic()-start<220:
  now=time.monotonic();rclpy.spin_once(node,timeout_sec=.03)
  if state['ready'] and state['ready_since'] and now-state['ready_since']>=10 and state['x'] is None and pub.get_subscription_count()>=2:
   state['x']=now;repeats.extend([(now+i*.15,'X') for i in range(3)])
   print('SEND X',flush=True)
  if state['follow_since'] and now-state['follow_since']>=3 and state['y'] is None:
   state['y']=now;repeats.extend([(now+i*.15,'Y') for i in range(3)])
   print('SEND Y',flush=True)
  for entry in repeats[:]:
   if now>=entry[0]:
    pub.publish(String(data=entry[1]));record('command',entry[1]);repeats.remove(entry)
  if state['result_time'] and now-state['result_time']>=7:break
  if state['y'] and now-state['y']>100:break
 with (root/'probe_summary.json').open('w') as f:json.dump(state,f,indent=2)
 if not state['result']:print('NO RESULT',state,flush=True)
finally:
 gz.unsubscribe('/world/default/stats');node.destroy_node();rclpy.shutdown();stream.close()
