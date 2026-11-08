#!/usr/bin/env bash
# Build navless_flight_tests in a scratch workspace and fly each test against fake_vehicle.py on an
# ISOLATED, localhost-only ROS domain (nothing reaches the drone's PX4 or ROS graph).
# Run inside the stack container:  bash /path/to/flight_tests/test/run_fake_tests.sh /path/to/flight_tests
set +u
SRC="$(cd "${1:-$(dirname "$0")/..}" && pwd)"
WS=/tmp/ft_ws
rm -rf "$WS" && mkdir -p "$WS/src" && cp -r "$SRC" "$WS/src/navless_flight_tests"
source /opt/ros/humble/setup.bash
source /root/skybeta_ws/install/setup.bash 2>/dev/null || true
cd "$WS" && colcon build --packages-select navless_flight_tests 2>&1 | tail -3 || exit 1
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
echo "== launch files load:"
for t in hover yaw forward; do
  ros2 launch navless_flight_tests "$t.launch.py" --show-args >/dev/null 2>&1 && echo "  $t.launch.py ok" || echo "  $t.launch.py FAILED"
done
run() {   # test, start yaw, time limit, fail mode
  echo "== test $1 (start heading $2 deg${4:+, $4})"
  python3 -u "$SRC/test/fake_vehicle.py" "$2" "$3" "${4:-}" > /tmp/ft_fake.log 2>&1 &
  F=$!
  ros2 run navless_flight_tests basic_flight --ros-args -p test:="$1" > /tmp/ft_node.log 2>&1 &
  N=$!
  wait $F
  kill $N 2>/dev/null; wait $N 2>/dev/null
  grep -E "test \"|Offboard|done ->|-> |touched down|not finished|ended|stale|disarmed|CLAMP|Traceback|Error" /tmp/ft_node.log | sed 's/^/   node: /' | cut -c1-170
  grep FAKE_RESULT /tmp/ft_fake.log | sed 's/^/   /'
}
run hover 90 60
run yaw 90 60
run forward 30 70
run hover 90 30 pilot
