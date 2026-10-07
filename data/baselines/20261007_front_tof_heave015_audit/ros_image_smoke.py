import json,time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from uav_control.perception.tof_depth_node import TofDepthNode
from uav_control.perception.rgbd_target_localizer import aligned_camera_qos
rclpy.init()
model=TofDepthNode()
probe=Node('tof_image_smoke_probe')
received=[]
source=probe.create_publisher(Image,'/camera/front/depth/ideal',aligned_camera_qos())
sink=probe.create_subscription(Image,'/camera/front/depth/image_raw',received.append,aligned_camera_qos())
executor=SingleThreadedExecutor()
executor.add_node(model);executor.add_node(probe)
message=Image(height=480,width=640,encoding='32FC1',step=2560)
message.header.stamp.sec=101;message.header.stamp.nanosec=123456789
message.header.frame_id='front_camera_optical_frame'
message.data=np.full((480,640),2.,dtype='<f4').tobytes()
start=time.monotonic();first_publish=None
try:
 while time.monotonic()-start<5 and not received:
  source.publish(message)
  if first_publish is None:first_publish=time.monotonic()
  executor.spin_once(timeout_sec=.02)
 assert received,'RELIABLE downstream did not receive any modeled depth'
 result=received[-1]
 assert result.header==message.header
 values=np.frombuffer(result.data,dtype='<f4')
 assert .95<np.isfinite(values).mean()<1.
 print(json.dumps({'received':len(received),'stamp_preserved':True,'size':[result.width,result.height],'valid_fraction':float(np.isfinite(values).mean())}))
finally:
 executor.shutdown();model.destroy_node();probe.destroy_node();rclpy.shutdown()
