"""Bench check, props off: does PX4 switch its heading to the magnetometer when vision drops out?

With the fleet's EKF2_MAG_TYPE = 0, EKF2 starts magnetometer fusion about 0.5 s into a vision gap
and resets its yaw to magnetic north. The vision frame's "north" is the drone's left at power-up,
so the heading jumps by about 90 degrees until vision returns (76 in Gazebo), and every blind
correction leans the wrong way. This watches PX4's estimator flags and yaw while you cover both
lenses for 6 s, then prints a verdict. Read-only: it only subscribes to two /fmu/out topics.

On the drone (props off, stack running, cuVSLAM tracking, check_topics.sh passing):

  docker exec -it skybeta-stack bash -lc \
    'source skybeta_env.sh; python3 /root/team_ws/src/Navless/tools/mag_check.py'

Follow the prompts: 10 s uncovered, cover both lenses fully at COVER, uncover at UNCOVER; 30 s
in all. After setting EKF2_MAG_TYPE = 5 and rebooting the flight controller, run it again.

Test off the drone (fake rclpy / px4_msgs, 50x speed): python -m pytest -q tools/test_mag_check.py
"""
import math
import os
import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import EstimatorStatusFlags, VehicleAttitude

FLAGS = ('cs_ev_pos', 'cs_ev_yaw', 'cs_inertial_dead_reckoning', 'cs_yaw_align', 'cs_mag_hdg',
         'cs_mag_3d', 'cs_mag_aligned_in_flight', 'cs_mag_field_disturbed')
MAG = ('cs_mag_hdg', 'cs_mag_3d', 'cs_mag_aligned_in_flight')
# PX4's /fmu/out publishers are best effort; a volatile subscriber matches volatile or transient local.
QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=10)
K = float(os.environ.get('MAG_CHECK_TIME_SCALE', '1'))     # test hook; leave unset on the drone
STEPS = ((0, 'Cameras uncovered: baseline.'), (10, '>>> COVER BOTH LENSES NOW <<<'),
         (16, '>>> UNCOVER <<<'), (30, None))

rclpy.init()
node = rclpy.create_node('mag_check')
s = {'flags': None, 'yaw': None, 'yaw_ev': None, 'mag': False, 'ev_off': False, 'jump': 0.0}
t0 = time.monotonic()


def el():
    return (time.monotonic() - t0) / K


def on_flags(m):
    now = {f: bool(getattr(m, f, False)) for f in FLAGS}
    if now != s['flags']:
        print(f"{el():5.1f} s  EKF: {' '.join(f[3:] for f in FLAGS if now[f]) or '-'}", flush=True)
        s['flags'] = now
    s['mag'] |= any(now[f] for f in MAG)
    s['ev_off'] |= not now['cs_ev_yaw']


def on_att(m):
    w, x, y, z = (float(v) for v in m.q)
    s['yaw'] = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    if s['flags'] and s['flags']['cs_ev_yaw']:
        s['yaw_ev'] = s['yaw']
    elif s['yaw_ev'] is not None:
        s['jump'] = max(s['jump'], abs((s['yaw'] - s['yaw_ev'] + 180) % 360 - 180))


node.create_subscription(EstimatorStatusFlags, '/fmu/out/estimator_status_flags', on_flags, QOS)
node.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', on_att, QOS)
step, shown = 0, -1
while rclpy.ok() and step < len(STEPS):
    if el() >= STEPS[step][0]:
        if STEPS[step][1]:
            print(f"{el():5.1f} s  {STEPS[step][1]}", flush=True)
        step += 1
    if s['yaw'] is not None and int(el()) > shown:
        shown = int(el())
        print(f"{el():5.1f} s  yaw {s['yaw']:7.2f} deg", flush=True)
    rclpy.spin_once(node, timeout_sec=0.05)

if s['flags'] is None:
    print('RESULT: nothing from PX4 on /fmu/out/estimator_status_flags. The PX4 link is down: '
          'run check_topics.sh and fix that first.')
elif not s['ev_off']:
    print('RESULT: INCONCLUSIVE. PX4 kept fusing vision yaw the whole time, so the magnetometer was '
          'never tested. Either tracking never dropped, or odometry kept arriving while lost (frozen '
          'poses through the stock bridge).')
elif s['mag']:
    print(f"RESULT: the MAGNETOMETER TOOK OVER the heading while vision was off (yaw moved up to "
          f"{s['jump']:.1f} deg). Same as the sim: set EKF2_MAG_TYPE = 5, reboot the flight "
          "controller, then run this again.")
else:
    print(f"RESULT: vision dropped and the magnetometer did NOT take over (yaw moved up to "
          f"{s['jump']:.1f} deg on the gyros).")
node.destroy_node()
rclpy.shutdown()
