#!/usr/bin/env bash
# Live side-ToF bench view on the drone: the checkout's tube/tof_driver.py in --view mode (no ROS topics),
# every frame logged to ~/team_ws/tof_bench/view.csv by tube/log_view.py. Run it on the drone host, not
# inside the container, after checking `hostname` is your own drone:
#
#   ssh -t dronehacks@<drone> bash team_ws/src/Navless/tube/bench_view.sh
#   python3 ~/team_ws/src/Navless/tube/view_log_summary.py ~/team_ws/tof_bench/view.csv    # afterwards
#
# Ctrl+C stops it; it also stops by itself after 25 minutes. It refuses while another tof_driver (e.g. the
# competitor launch's) is reading the sensors: two readers garble each other's readings.
C="${SKYBETA_CONTAINER:-skybeta-stack}"
if docker exec "$C" pgrep -f '[t]of_driver.py' >/dev/null; then
  echo "Another tof_driver is already reading the sensors - not starting a second one."; exit 1
fi
mkdir -p ~/team_ws/tof_bench
exec docker exec -it "$C" bash -c 'cd /root/team_ws/src/Navless/tube && PYTHONPATH=/root/team_ws/pylibs timeout --foreground -s INT 1500 python3 -u tof_driver.py --view | python3 -u log_view.py /root/team_ws/tof_bench/view.csv'
