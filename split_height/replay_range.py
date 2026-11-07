#!/usr/bin/env python3
"""Offline measured-sensor replay only; accepts an operator-retained PX4 ULog."""
import argparse
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'split_sim/course_runtime'))
from conditional_range import WindowRangeHealth
from frames import ned_to_launch_odom


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ulog',type=Path,required=True)
    parser.add_argument('--start',type=float,default=133.)
    parser.add_argument('--end',type=float,default=136.30)
    parser.add_argument('--control-bearing',type=float,required=True,help='independently measured control frame, degrees')
    parser.add_argument('--start-bearing',type=float,default=0.)
    parser.add_argument('--datum-ned',type=float,nargs=3,default=(0.,0.,0.))
    args=parser.parse_args()
    import numpy as np
    from pyulog import ULog
    course=json.loads((ROOT/'split_sim/course_runtime/navless/gazebo_sim/course/course_v3.json').read_text())
    route=json.loads((ROOT/'split_sim/course_runtime/astar-route.json').read_text())
    guard=WindowRangeHealth(next(w for w in course['walls'] if 'window' in w))
    ulog=ULog(str(args.ulog),message_name_filter_list=['distance_sensor','estimator_status_flags','vehicle_local_position'])
    ranges=ulog.get_dataset('distance_sensor').data
    flags=ulog.get_dataset('estimator_status_flags').data
    positions=ulog.get_dataset('vehicle_local_position').data
    fields=('cs_ev_hgt','cs_baro_hgt','cs_fake_hgt','cs_baro_fault','fs_bad_acc_vertical','cs_rng_fault',
            'cs_rng_stuck','cs_rng_hgt','cs_rng_terrain','cs_rng_kin_consistent')
    events=[];index=0
    for now in np.arange(args.start,args.end,.02):
        while index<len(ranges['timestamp']) and ranges['timestamp'][index]/1e6<=now:
            guard.receive(ranges['timestamp'][index]/1e6,float(ranges['current_distance'][index]),'downward_range_link',.02,4.)
            index+=1
        fi=np.searchsorted(flags['timestamp']/1e6,now,side='right')-1
        pi=np.searchsorted(positions['timestamp']/1e6,now,side='right')-1
        if min(fi,pi)<0:raise ValueError('replay interval precedes required measurements')
        status=SimpleNamespace(timestamp=int(flags['timestamp'][fi]),**{k:bool(flags[k][fi]) for k in fields})
        local=ned_to_launch_odom([float(positions[k][pi])-d for k,d in zip(('x','y','z'),args.datum_ned)],args.control_bearing)
        b=math.radians(args.start_bearing)
        east=route['launch_origin_enu'][0]+local[0]*math.sin(b)-local[1]*math.cos(b)
        north=route['launch_origin_enu'][1]+local[0]*math.cos(b)+local[1]*math.sin(b)
        result=guard.check(float(now),status,east,north,True,True)
        if not events or events[-1]['result']!=list(result):events.append({'t':round(float(now),3),'result':list(result)})
    print(json.dumps({'scope':'offline local-position/range/flags replay; airborne authority assumed for retained interval, not independently validated here',
                      'events':events,'qualification':'No simulator truth or commands; not flight success.'},indent=2))


if __name__=='__main__':main()
