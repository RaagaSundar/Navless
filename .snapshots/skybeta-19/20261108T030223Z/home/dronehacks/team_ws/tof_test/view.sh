#!/usr/bin/env bash
# Live side-ToF view on skybeta-19: the checkout's own tube/tof_driver.py (what the competitor launch runs) in
# --view mode, so no ROS topics, logged to ~/team_ws/tof_test/view.csv. Ctrl+C stops it; it also stops by itself
# after 25 minutes.
h=$(hostname); [ "$h" = skybeta-19 ] || { echo "NOT skybeta-19 ($h) - stopping"; exit 1; }
if docker exec skybeta-stack pgrep -f '[t]of_driver.py' >/dev/null; then
  echo "Another tof_driver is already reading the sensors - not starting a second one."; exit 1
fi
exec docker exec -it skybeta-stack bash -c 'cd /root/team_ws/tof_test && PYTHONPATH=/root/team_ws/pylibs timeout --foreground -s INT 1500 python3 -u /root/team_ws/src/Navless/tube/tof_driver.py --view | python3 -u log_view.py view.csv'
