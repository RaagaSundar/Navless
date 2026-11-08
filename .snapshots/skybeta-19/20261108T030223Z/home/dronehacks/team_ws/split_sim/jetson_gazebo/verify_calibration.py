#!/usr/bin/env python3
"""Read-only verification of source calibration and actual published static TF."""
import json
from pathlib import Path
import time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
rclpy.init()
node=rclpy.create_node('passive_sim_calibration_check')
seen={}
def receive(msg):
    for item in msg.transforms:
        t=item.transform.translation;q=item.transform.rotation
        seen[item.child_frame_id]={'parent':item.header.frame_id,'translation':[t.x,t.y,t.z],'quaternion':[q.x,q.y,q.z,q.w]}
sub=node.create_subscription(TFMessage,'/tf_static',receive,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
start=time.monotonic()
while time.monotonic()-start<3:
    rclpy.spin_once(node,timeout_sec=.05)
cal=json.loads(Path('/root/gazebo_state/adapter-calibration.json').read_text())
l=seen[cal['left']['frame_id']];r=seen[cal['right']['frame_id']]
baseline_tf=l['translation'][1]-r['translation'][1]
baseline_projection=-cal['right']['p'][3]/cal['right']['p'][0]
result={'source_calibration':cal,'actual_static_tf':seen,'baseline_tf_m':baseline_tf,'baseline_projection_m':baseline_projection,
        'passed':l['parent']==r['parent']=='base_link' and abs(baseline_tf-baseline_projection)<1e-8 and abs(baseline_tf-.075)<1e-8}
Path('/root/gazebo_state/tf-and-calibration-check.json').write_text(json.dumps(result,indent=2))
print(json.dumps({'baseline_tf_m':baseline_tf,'baseline_projection_m':baseline_projection,'passed':result['passed']}))
node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if result['passed'] else 1)
