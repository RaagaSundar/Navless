"""Exercise cold-discovery retry and wrong-mode/graph rejection without ROS or control."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class PreparedSensorTests(unittest.TestCase):
    def qualify(self, case):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            (root/'state').mkdir()
            (root/'state/px4-current').symlink_to('/world')
            run=root/'run'
            run.mkdir()
            (run/'prepared-world').write_text('/world\n')
            (run/'platform.json').write_text('{}')
            (run/'simulated-vehicle.sdf').write_text(
                '<model><sensor name="oak_left"><update_rate>15</update_rate></sensor>'
                '<sensor name="oak_right"><update_rate>15</update_rate></sensor></model>')
            for name in ('check_stereo_rate.py','check_imu_stereo.py'):
                (run/name).write_text('')
            script=r"""
source "$LIB"
run="$PWD/run"; profile=stereo-imu; expected=True; SSH=(fake_ssh)
check_px4_sensors() { return 0; }
fake_ssh() {
  case "$*" in
    *'ros2 param get'*)
      if [[ "$CASE" == wrong ]]; then echo 'Boolean value is: False'; return; fi
      if [[ ! -f "$run/called" ]]; then touch "$run/called"; echo 'Node not found'; return 1; fi
      echo 'Boolean value is: True' ;;
    *'ros2 topic info'*)
      printf 'Node name: visual_slam_node\nEndpoint type: SUBSCRIPTION\n'
      [[ "$CASE" == missing ]] || printf 'Node name: split_ordered_body_imu\nEndpoint type: PUBLISHER\n' ;;
  esac
}
qualify_prepared
"""
            result=subprocess.run(['bash','-c',script],cwd=root,
                env=dict(os.environ,LIB=str(Path(__file__).resolve().with_name('prepare_split.sh')),CASE=case),
                capture_output=True,text=True)
            return result.returncode,(run/'prepared-sources-qualified').exists()

    def test_cold_retry_succeeds(self):
        self.assertEqual(self.qualify('cold'),(0,True))

    def test_wrong_live_mode_fails(self):
        self.assertEqual(self.qualify('wrong'),(1,False))

    def test_missing_endpoint_fails(self):
        self.assertEqual(self.qualify('missing'),(1,False))


if __name__=='__main__':
    unittest.main()
