"""Validation tests run in an isolated ROS domain, never on the aircraft topics."""
import copy
import math
import time
import pytest
from nav_msgs.msg import Odometry
from map_server import valid_odometry


@pytest.fixture
def sample():
    m = Odometry()
    m.header.frame_id = 'odom'
    m.child_frame_id = 'base_link'
    m.header.stamp.sec = 100
    m.pose.pose.orientation.w = 1.0
    for i in (0,7,14,21,28,35):
        m.pose.covariance[i] = 0.01
        m.twist.covariance[i] = 0.02
    return m


def test_fresh_tracked_estimate(sample):
    assert valid_odometry(sample,100.06,True,0.03)


@pytest.mark.parametrize('now,tracking,age',[(100.5,True,0.01),(99.8,True,0.01),(100.06,False,0.01),(100.06,True,0.5)])
def test_stale_or_failed_tracking_stops_relay(sample,now,tracking,age):
    assert not valid_odometry(sample,now,tracking,age)


def test_wrong_frame_is_rejected(sample):
    sample.header.frame_id = 'map'
    assert not valid_odometry(sample,100.06,True,0.03)


def test_invalid_pose_rejected(sample):
    sample.pose.pose.position.x = math.nan
    assert not valid_odometry(sample,100.06,True,0.03)


def test_nonunit_quaternion_rejected(sample):
    sample.pose.pose.orientation.w = 0.0
    assert not valid_odometry(sample,100.06,True,0.03)


def test_invalid_covariance_rejected(sample):
    sample.pose.covariance[0] = -1.0
    assert not valid_odometry(sample,100.06,True,0.03)


def test_unknown_covariance_rejected(sample):
    for i in (0,7,14,21,28,35):
        sample.pose.covariance[i] = 1.0
    assert not valid_odometry(sample,100.06,True,0.03)


def test_repeated_or_old_sample_is_not_republished(sample):
    import rclpy
    from odom_gate import OdomGate
    rclpy.init()
    node=OdomGate()
    try:
        now=node.get_clock().now().nanoseconds
        sample.header.stamp.sec=now//1000000000
        sample.header.stamp.nanosec=now%1000000000
        node.tracking=True
        node.status_time=time.monotonic()
        node.odom(sample)
        assert node.accepted==1
        node.odom(sample)
        assert node.accepted==1 and node.rejected==1
        node.tracking=False
        sample.header.stamp.nanosec+=1
        node.odom(sample)
        assert node.accepted==1 and node.rejected==2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
