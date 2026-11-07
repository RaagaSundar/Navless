"""Record map observer outputs only; no publishers, truth, flight or EKF inputs."""
import argparse,json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
rclpy.init();n=Node('local_course_obj_observer_recorder',parameter_overrides=[])
f=Path(a.output).open('a',buffering=1)
def write(kind,m):
 d={'wall_time':time.time(),'kind':kind}
 if kind in ('status','sync_status'):
  try:d['data']=json.loads(m.data)
  except ValueError:d['data']=m.data
 else:
  q=m.pose.orientation;x=m.pose.position;d.update(t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,frame=m.header.frame_id,xyz=[x.x,x.y,x.z],qxyzw=[q.x,q.y,q.z,q.w])
 f.write(json.dumps(d)+'\n')
n.create_subscription(String,'/team/localize/status',lambda m:write('status',m),10)
n.create_subscription(PoseStamped,'/team/localize/base_pose',lambda m:write('base_pose',m),10)
n.create_subscription(String,'/local_course/map_observer/sync_status',lambda m:write('sync_status',m),10)
try:rclpy.spin(n)
finally:f.close();n.destroy_node();rclpy.try_shutdown()
