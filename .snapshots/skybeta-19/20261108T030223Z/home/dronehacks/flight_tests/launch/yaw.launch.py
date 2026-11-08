"""First-flight test "yaw" (see navless_flight_tests/basic_flight.py): flight_interface + our node.

    COMPETITOR_PKG=navless_flight_tests  COMPETITOR_LAUNCH=yaw.launch.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import description  # noqa: E402


def generate_launch_description():
    return description('yaw')
