"""Observer-only bounded exact-acquisition stamp join; never extrapolates poses."""
import json,time
from collections import OrderedDict
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
rclpy.init();n=Node('obj_observer_exact_stamp_sync')
clouds=OrderedDict(); odoms=OrderedDict();counts={'released':0,'expired':0,'bounded_drop':0}
p=n.create_publisher(PointCloud2,'/local_course/map_observer/paired_points',2)
d=n.create_publisher(String,'/local_course/map_observer/sync_status',2)
def stamp(m):return m.header.stamp.sec*1000000000+m.header.stamp.nanosec
def cloud(m):clouds[stamp(m)]=(time.monotonic(),m)
def odom(m):odoms[stamp(m)]=time.monotonic()
n.create_subscription(PointCloud2,'/team/stereo/points',cloud,qos_profile_sensor_data)
n.create_subscription(Odometry,'/local_course/validated_odometry',odom,qos_profile_sensor_data)
def tick():
 now=time.monotonic()
 for k,(received,m) in list(clouds.items()):
  if k in odoms and now-odoms[k]>=.1:
   p.publish(m);del clouds[k];counts['released']+=1
  elif now-received>3:
   del clouds[k];counts['expired']+=1
 for table in [clouds,odoms]:
  while len(table)>100:table.popitem(last=False);counts['bounded_drop']+=1
 d.publish(String(data=json.dumps(dict(counts,pending=len(clouds),matched_policy='exact capture nanoseconds; 100ms observer delivery settle; 3wall-second expiry; bounded100'))))
n.create_timer(.05,tick)
try:rclpy.spin(n)
finally:n.destroy_node();rclpy.try_shutdown()
