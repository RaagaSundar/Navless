"""Separate OpenCV measured stereo profile; existing RTAB launch is unchanged.

Default TF comes from reviewed calibration publishers or recorded /tf_static.
VO_SIM_FIXTURE_TF=1 opts into the documented static local simulator mount.
"""
import json
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    root=Path(__file__).parent
    actions=[]
    if os.environ.get('VO_SIM_FIXTURE_TF')=='1':
        config=json.loads((root/'sim-frames.json').read_text())
        for side in ('left','right'):
            x,y,z=config['stereo_body_flu_mount_m'][side]
            actions.append(Node(package='tf2_ros',executable='static_transform_publisher',
                name=f'opencv_{side}_optical_tf',
                arguments=['--x',str(x),'--y',str(y),'--z',str(z),
                    '--qx','-0.5','--qy','0.5','--qz','-0.5','--qw','0.5',
                    '--frame-id','base_link','--child-frame-id',f'oak_{side}_camera_optical_frame'],
                parameters=[{'use_sim_time':True}]))
    actions.append(ExecuteProcess(cmd=['python3',str(root/'vo_node.py'),
                   '--ros-args','-p','use_sim_time:=true',
                   '-p','recovery_horizon_s:='+os.environ.get('VO_RECOVERY_HORIZON_S','0.0')],
                   additional_env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'},output='screen'))
    return LaunchDescription(actions)
