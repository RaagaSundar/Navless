#!/usr/bin/env python3
"""Stationary simulation-only camera-loss test. No publishers or flight commands."""
import json
from pathlib import Path
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String

rclpy.init()
node=rclpy.create_node('passive_sim_camera_loss_test')
pause=Path('/root/gazebo_state/pause_images')
seen={'raw':[],'validated':[],'gate':[]}
started=time.monotonic()
subs=[]
for key,topic in [('raw','/visual_slam/tracking/odometry'),('validated','/mapping/validated_odometry')]:
    subs.append(node.create_subscription(Odometry,topic,lambda msg,k=key:seen[k].append(time.monotonic()-started),qos_profile_sensor_data))
subs.append(node.create_subscription(String,'/mapping/gate_status',lambda msg:seen['gate'].append([time.monotonic()-started,json.loads(msg.data)]),10))
paused=resumed=False
try:
    while time.monotonic()-started<14:
        elapsed=time.monotonic()-started
        if elapsed>=2 and not paused:
            pause.touch(); paused=True
        if elapsed>=8 and not resumed:
            pause.unlink(missing_ok=True); resumed=True
        rclpy.spin_once(node,timeout_sec=.02)
finally:
    pause.unlink(missing_ok=True)
    node.destroy_node()
    rclpy.try_shutdown()
result={'pause_wall_s':[2,8],'settled_loss_window_s':[3,8],
        'validated_during_settled_loss':sum(3<=t<8 for t in seen['validated']),
        'raw_during_settled_loss':sum(3<=t<8 for t in seen['raw']),
        'validated_after_resume':sum(9<=t<14 for t in seen['validated']),
        'events':seen}
result['passed']=result['validated_during_settled_loss']==0 and result['raw_during_settled_loss']==0 and result['validated_after_resume']>0
Path('/root/gazebo_state/loss-test.json').write_text(json.dumps(result,indent=2))
print(json.dumps({k:v for k,v in result.items() if k!='events'},indent=2))
raise SystemExit(0 if result['passed'] else 1)
