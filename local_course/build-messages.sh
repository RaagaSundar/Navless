#!/usr/bin/env bash
# Run on the host; build only the course's required full PX4 interfaces.
# Source must match the PX4 firmware. No flight commands are issued.
set -euo pipefail
container=${COURSE_CONTAINER:-local-course-sim}
source_dir=${PX4_MSGS_SOURCE:-/root/ros2_ws/src/px4_msgs}
docker exec -i "$container" bash -s -- "$source_dir" <<'BUILD'
set -eo pipefail
src=$1
out=/course-state/fast_msgs
source /opt/ros/jazzy/setup.bash
if [[ -f "$out/local_setup.bash" ]]; then
  echo "Existing overlay: $out (remove explicitly only when rebuilding is intended)"
  exit 0
fi
[[ $(uname -m) == aarch64 ]] || { echo 'Expected native ARM64 container' >&2; exit 1; }
[[ -f "$src/package.xml" && -d "$src/msg" ]]
work=$(mktemp -d /tmp/course-fast-msgs.XXXXXX)
python3 - "$src" "$work" <<'PY'
from pathlib import Path
import shutil, sys
src=Path(sys.argv[1]); dst=Path(sys.argv[2])/'src'/'px4_msgs'
dst.mkdir(parents=True); (dst/'msg').mkdir()
for name in ('CMakeLists.txt','package.xml'):
    shutil.copy2(src/name, dst/name)
names=set('OffboardControlMode TrajectorySetpoint VehicleCommand VehicleCommandAck VehicleStatus VehicleLocalPosition VehicleLandDetected EstimatorStatusFlags VehicleOdometry'.split())
queue=list(names)
while queue:
    name=queue.pop()
    for line in (src/'msg'/f'{name}.msg').read_text().splitlines():
        line=line.split('#')[0].strip()
        if not line: continue
        typ=line.split()[0].split('[')[0].split('/')[-1]
        if (src/'msg'/f'{typ}.msg').exists() and typ not in names:
            names.add(typ); queue.append(typ)
for name in sorted(names):
    shutil.copy2(src/'msg'/f'{name}.msg', dst/'msg'/f'{name}.msg')
print('Copied full unmodified definitions:', ', '.join(sorted(names)))
PY
cd "$work"
CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-4} colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF
source "$work/install/local_setup.bash"
python3 - "$src" "$work/install" <<'PY'
from pathlib import Path
import sys
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import serialize_message, deserialize_message
src=Path(sys.argv[1]); install=Path(sys.argv[2])
for msg in sorted((install/'share/px4_msgs/msg').glob('*.msg')):
    assert msg.read_bytes() == (src/'msg'/msg.name).read_bytes(), msg
    cls=get_message('px4_msgs/msg/'+msg.stem)
    instance=cls()
    assert deserialize_message(serialize_message(instance), cls) == instance
    print('Verified definition and native CDR typesupport:',msg.stem)
PY
# Keep build artifacts on Linux-native /tmp; copy only the completed install.
cp -a "$work/install" "$out"
echo "Ready: source /opt/ros/jazzy/setup.bash; source $out/local_setup.bash"
echo "Native build and logs retained: $work"
BUILD
