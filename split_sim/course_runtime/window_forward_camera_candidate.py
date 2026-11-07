"""Staged fixed-forward stereo window yaw overlay. No flight commands/truth.

Body forward +X FLU and +X FRD coincide. Camera optical +Z is body forward.
Geographic compass/NED yaw = atan2(east,north); ENU yaw = pi/2-compass.
The caller must separately register this geographic yaw into PX4's control
frame exactly as it does for the validated first-exterior policy.
"""
from hybrid_camera_candidate import HybridCameraLook


class WindowForwardLook(HybridCameraLook):
    def __init__(self,route_data,config,launch_height_m=.07):
        config=dict(config)
        config['hybrid_trigger_arc_m']=config['activation_arc_m']
        super().__init__(route_data,config,launch_height_m)
