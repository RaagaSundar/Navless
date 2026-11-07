"""Simulation assets only. Reuse the course generator without editing team code."""
import importlib.util
import os
import sys
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    'master_assets', '/workspace/Navless/master_sim/make_assets.py')
assets = importlib.util.module_from_spec(spec)
spec.loader.exec_module(assets)
assets.CAM_X, assets.CAM_Z = 0.1053, 0.0304
assets.WIDTH, assets.HEIGHT = 640, 400
assets.RATE_HZ = 10.0
out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)
assets.make_course(str(out))
quad = Path(assets.make_quad(str(out)))
sdf = quad.read_text()
# Gazebo mono8 images are passed directly to cuVSLAM. No resize/colour ambiguity.
sdf = sdf.replace('<format>R8G8B8</format>', '<format>L8</format>')
# Truth odometry is NEVER a visual-estimation input in this split setup.
sdf = sdf.replace('<dimensions>3</dimensions>',
                  '<dimensions>3</dimensions><odom_covariance_topic>/split/unused_truth</odom_covariance_topic>')
quad.write_text(sdf)
