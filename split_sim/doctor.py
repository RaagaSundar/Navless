"""Read-only source/install doctor. --source-only never invokes Docker or SSH."""
import argparse,hashlib,json,shutil,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];HERE=Path(__file__).resolve().parent
REQUIRED=['local_sim/Dockerfile','local_sim/bootstrap-px4.sh','master_sim/make_assets.py','master_sim/bridge.yaml','gazebo_sim/worlds/dronehacks.sdf','gazebo_sim/models/dh_quad/model.sdf','gazebo_sim/models/dh_course/model.sdf','appearance_profile/generate.py','appearance_profile/verify.py','depthcloud/stereo_cloud.py','depthcloud/cloud_localize.py','gazebo_sim/models/dh_course/meshes/course_visual.obj']
REQUIRED += ['split_sim/'+x for x in ['station.py','bootstrap_mac.sh','setup_jetson.sh','compose.yaml','run_laptop.sh','run_jetson.sh','sensor_link.py','imu_link.py','sim_slam.launch.py','make_assets.py','range_sensor.py','start_px4.sh','split_network.sh','run_split_course.sh','prepare_split.sh','viewer/map_server.py','viewer/odom_gate.py','viewer/source_admission.py','preview/server.py','preview/index.html','preview/app.js','preview/render_chase.py','export_run_videos.py','export_drone_video.py','course_runtime/mission.py','course_runtime/sources.sha256','capture_flight_sensors.py','seal_sensor_capture.py','split_record.py','check_startup_recording.py','wait_settled.py']]
def check_source(root=ROOT):
 missing=[p for p in REQUIRED if not(root/p).is_file()]
 if missing:raise ValueError('missing tracked prerequisites: '+', '.join(missing))
 count=0
 for line in (root/'split_sim/course_runtime/sources.sha256').read_text().splitlines():
  digest,name=line.split('  ',1);relative=Path(name)
  if relative.is_absolute() or '..' in relative.parts:raise ValueError('unsafe frozen manifest member')
  p=root/'split_sim/course_runtime'/relative
  if hashlib.sha256(p.read_bytes()).hexdigest()!=digest:raise ValueError('frozen runtime hash mismatch: '+name)
  count+=1
 return {'source_files':'present','frozen_runtime_members':count,'baseline_run':'213553 failed/experimental, not course-qualified','lifecycle':'no simulators or network calls in source-only check'}
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source-only',action='store_true');p.add_argument('--local',action='store_true');a=p.parse_args();r=check_source()
 if a.local:
  r['tools']={k:shutil.which(k) for k in ['docker','ssh','scp','python3','git']}
  r['px4_binary']=(HERE/'state/PX4-Autopilot/build/px4_sitl_default/bin/px4').is_file()
  r['vendor_image']='checked on authenticated Jetson by setup_jetson --install; organiser prerequisite'
 print(json.dumps(r,indent=2));return 0
if __name__=='__main__':
 try:raise SystemExit(main())
 except (OSError,ValueError) as e:print(str(e),file=sys.stderr);raise SystemExit(1)
