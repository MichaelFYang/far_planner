#!/usr/bin/env python3
# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT
"""Keep cleared obstacles excluded while accumulated terrain still retains them."""

import struct
import threading
import time
import unittest

import rospy
import rostest
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def wait_for(predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError('Timed out waiting for terrain retention callback')


class ClockDriver:
    def __init__(self):
        self.publisher = rospy.Publisher('/clock', Clock, queue_size=10)
        self.lock = threading.Lock()
        self.base = 1.0
        self.started = time.monotonic()
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def set_time(self, seconds):
        with self.lock:
            self.base = seconds
            self.started = time.monotonic()

    def run(self):
        while not self.stopped.is_set() and not rospy.is_shutdown():
            with self.lock:
                seconds = self.base + time.monotonic() - self.started
            self.publisher.publish(Clock(clock=rospy.Time.from_sec(seconds)))
            self.stopped.wait(0.02)

    def close(self):
        self.stopped.set()
        self.thread.join(timeout=1.0)


def cloud(x, intensity, header):
    message = PointCloud2()
    message.header = header
    message.height = message.width = 1
    message.fields = [PointField(name=name, offset=4*i,
                                 datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
    message.point_step = message.row_step = 16
    message.is_dense = True
    message.data = struct.pack('<ffff', x, 0.0, 0.0, intensity)
    return message


class DynamicTerrainRetentionTest(unittest.TestCase):
    def test_stale_terrain_does_not_restore_cleared_obstacle(self):
        clock = ClockDriver()
        odometry = rospy.Publisher('/odom_world', Odometry, queue_size=10)
        scans = rospy.Publisher('/scan_cloud', PointCloud2, queue_size=10)
        terrain = rospy.Publisher('/terrain_cloud', PointCloud2, queue_size=10)
        observations = []
        observed = rospy.Subscriber('/FAR_obs_debug', PointCloud2,
                                    observations.append, queue_size=10)

        def observe(seconds, scan_endpoint=None):
            clock.set_time(seconds)
            wait_for(lambda: rospy.Time.now().to_sec() >= seconds)
            time.sleep(0.1)
            pose = Odometry()
            pose.header = Header(stamp=rospy.Time.from_sec(seconds), frame_id='map')
            pose.pose.pose.orientation.w = 1.0
            odometry.publish(pose)
            if scan_endpoint is not None:
                scans.publish(cloud(scan_endpoint, 0.0, pose.header))
            before = len(observations)
            terrain.publish(cloud(2.0, 1.0, pose.header))
            wait_for(lambda: len(observations) > before)
            return observations[-1].width

        try:
            wait_for(lambda: all(pub.get_num_connections() > 0 for pub in
                                 (odometry, scans, terrain)) and observed.get_num_connections() > 0)
            retention = rospy.get_param('/far_planner/Util/dynamic_obs_dacay_time')
            self.assertGreater(observe(10.0, 2.0), 0)
            self.assertEqual(observe(11.0, 4.0), 0)
            self.assertEqual(observe(23.0), 0)
            self.assertEqual(observe(24.0), 0, 'Stale terrain restored the cleared obstacle')
            expiry = 11.0 + retention + 2.0
            observe(expiry, 2.0)
            self.assertGreater(observe(expiry+1.0, 2.0), 0)
        finally:
            clock.close()


if __name__ == '__main__':
    rospy.init_node('test_dynamic_terrain_retention_ros1', anonymous=True)
    rostest.rosrun('far_planner', 'test_dynamic_terrain_retention_ros1', DynamicTerrainRetentionTest)
