"""Off-drone tests for navless_nav/mirror_relay.py (no ROS needed)."""
import json
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from navless_nav.mirror_relay import FrameServer, make_frame, make_image_frame  # noqa: E402


def test_frame_carries_tof_and_node_with_no_return_as_null():
    f = json.loads(make_frame(1.0, 0.0, ((0, 0, 0), (0, 0, 0, 1)),
                              tof=(0.4123, float('inf'), 0.03), node='ROUTE'))
    assert f['tof'] == {'left': 0.412, 'right': None, 'age': 0.03} and f['node'] == 'ROUTE'
    assert json.loads(make_frame(1.0, 0.0, ((0, 0, 0), (0, 0, 0, 1))))['tof'] is None


def test_frame_forwards_px4_view_for_the_height_check():
    f = json.loads(make_frame(1.0, 0.0, ((0, 0, 0.01), (0, 0, 0, 1)),
                              state={'px4_link': True, 'landed': True,
                                     'local_enu': [0.02, -0.01, 0.61]}))
    assert f['px4_link'] is True and f['landed'] is True and f['local_enu'] == [0.02, -0.01, 0.61]
    f = json.loads(make_frame(1.0, 0.0, ((0, 0, 0), (0, 0, 0, 1))))
    assert f['local_enu'] is None and f['px4_link'] is None


def test_image_frame_round_trips_the_jpeg_bytes():
    import base64
    f = json.loads(make_image_frame(2.5, b'\xff\xd8fakejpeg\xff\xd9', 320, 200))
    assert base64.b64decode(f['img']) == b'\xff\xd8fakejpeg\xff\xd9'
    assert (f['w'], f['h'], f['t']) == (320, 200, 2.5) and 'p' not in f


def test_frame_carries_pose_state_setpoint_and_camera():
    line = make_frame(12.3456, 0.0213, ((1.0, -2.0, 0.85), (0.0, 0.0, 0.7071068, 0.7071068)),
                      vo_state=1, state={'armed': True, 'offboard_active': False,
                                         'nav_state_name': 'POSCTL'},
                      setpoint=(1.2, -2.0, 0.85), cam=((0.1, 0.0, -0.02), (0, 0.17, 0, 0.985)))
    assert line.endswith('\n') and '\n' not in line[:-1]
    f = json.loads(line)
    assert f['t'] == 12.346 and f['age'] == 0.021
    assert f['p'] == [1.0, -2.0, 0.85] and f['q'][3] == 0.70711
    assert f['vo_state'] == 1 and f['armed'] and not f['offboard'] and f['nav'] == 'POSCTL'
    assert f['setpoint'] == [1.2, -2.0, 0.85] and f['cam']['p'] == [0.1, 0.0, -0.02]


def test_frame_without_optional_fields():
    f = json.loads(make_frame(1.0, None, ((0, 0, 0), (0, 0, 0, 1))))
    assert f['setpoint'] is None and f['cam'] is None and f['vo_state'] is None
    assert f['armed'] is False and f['age'] is None


def test_server_streams_lines_and_drops_a_closed_client():
    server = FrameServer(0, host='127.0.0.1')
    try:
        c = socket.create_connection(('127.0.0.1', server.port), timeout=2)
        for _ in range(50):
            if server.n_clients:
                break
            time.sleep(0.02)
        assert server.n_clients == 1
        for i in range(3):
            server.send(make_frame(i, 0.0, ((i, 0, 0), (0, 0, 0, 1))))
        buf = b''
        while buf.count(b'\n') < 3:
            buf += c.recv(4096)
        frames = [json.loads(x) for x in buf.decode().splitlines()[:3]]
        assert [f['p'][0] for f in frames] == [0, 1, 2]
        c.close()
        for _ in range(20):                      # the next sends notice the closed peer
            server.send(make_frame(9, 0.0, ((0, 0, 0), (0, 0, 0, 1))))
            if not server.n_clients:
                break
            time.sleep(0.05)
        assert server.n_clients == 0
    finally:
        server.close()
