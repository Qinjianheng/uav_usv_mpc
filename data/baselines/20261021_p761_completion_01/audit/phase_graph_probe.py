"""Actual ROS estimator phase wiring only; synthetic observation, no PX4 or flight."""
import json
from pathlib import Path
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from uav_usv_interfaces.msg import TargetObservation, TargetState, TargetImageState
from uav_control.common.p76_phase import phase_qos
from uav_control.tracking.spatial_dkf_node import SpatialDkfNode
from uav_control.tracking.image_dkf_node import ImageDkfNode
root=Path('/home/qin/data/uav_usv_mpc')
rclpy.init(args=['--ros-args','-p','contract_file:='+str(root/'src/uav_usv_bringup/config/p76_contract.yaml'),'-p','p76_mode_topic:=/control/p76_mode'])
spatial,image,observer=SpatialDkfNode(),ImageDkfNode(),Node('p761_phase_probe')
executor=SingleThreadedExecutor()
for n in (spatial,image,observer):executor.add_node(n)
acks={};counts={'spatial':0,'image':0};result=[]
def ack(m):
 p=json.loads(m.data);acks[p['estimator']]=p
subs=[observer.create_subscription(String,'/tracking/p76_filter_ack',ack,100)]
for key,cls,topic in [('spatial',TargetState,'/tracking/target_state'),('image',TargetImageState,'/tracking/image_state')]:
 subs.append(observer.create_subscription(cls,topic,lambda m,k=key:counts.__setitem__(k,counts[k]+1),100))
pub=observer.create_publisher(String,'/control/p76_mode',phase_qos())
obs=observer.create_publisher(TargetObservation,'/perception/front/target_observation',10)
try:
 for phase,epoch in [('FOLLOW_ACTIVE',0),('Y_HANDOVER',1),('PAPER_INTERCEPT_ACTIVE',1),('SAFE_RECOVERY',1)]:
  start=time.monotonic()
  while time.monotonic()-start<.7:
   now=observer.get_clock().now().to_msg()
   pub.publish(String(data=json.dumps(dict(phase=phase,epoch=epoch,y_stamp=observer.get_clock().now().nanoseconds*1e-9))))
   m=TargetObservation(stamp=now,frame_id='local_ned',valid=True,covariance=[.01,0.,0.,0.,.01,0.,0.,0.,.01]);m.position.x=8.;obs.publish(m)
   executor.spin_once(timeout_sec=.01)
  before=dict(counts);start=time.monotonic()
  while time.monotonic()-start<.2:executor.spin_once(timeout_sec=.01)
  expected={'spatial':phase=='FOLLOW_ACTIVE','image':phase in ('Y_HANDOVER','PAPER_INTERCEPT_ACTIVE')}
  assert all(acks[k]['active']==v for k,v in expected.items()),(phase,acks)
  assert all((counts[k]>before[k])==v for k,v in expected.items()),(phase,before,counts)
  result.append(dict(phase=phase,active=expected,output_counts=dict(counts)))
 (root/'data/experiments/20261021_p761_01/phase_graph.json').write_text(json.dumps(dict(scope='Actual ROS wiring with synthetic observations; not estimator accuracy or flight qualification',passed=True,phases=result),indent=2)+'\n')
finally:
 for n in (spatial,image,observer):executor.remove_node(n);n.destroy_node()
 executor.shutdown();rclpy.shutdown()
