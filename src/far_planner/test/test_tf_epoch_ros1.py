#!/usr/bin/env python3
# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT
"""Ensure a clock rewind invalidates TF before the next TF message arrives."""

import struct
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage


def wait_for(predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError('Timed out waiting for planner TF epoch callback')


class ClockDriver:
    def __init__(self):
        self.publisher = rospy.Publisher('/clock', Clock, queue_size=10)
        self.lock = threading.Lock()
        self.base = 20.0
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


def cloud(stamp):
    message = PointCloud2()
    message.header = Header(stamp=stamp, frame_id='sensor')
    message.height = message.width = 1
    message.fields = [PointField(name=name, offset=4 * index,
                                 datatype=PointField.FLOAT32, count=1)
                      for index, name in enumerate(('x', 'y', 'z', 'intensity'))]
    message.point_step = message.row_step = 16
    message.is_dense = True
    message.data = struct.pack('<ffff', 2.0, 0.0, 0.0, 1.0)
    return message


class TfEpochTest(unittest.TestCase):
    def test_old_epoch_tf_is_not_used_before_new_tf_arrives(self):
        clock = ClockDriver()
        odometry = rospy.Publisher('/odom_world', Odometry, queue_size=10)
        terrain = rospy.Publisher('/terrain_cloud', PointCloud2, queue_size=10)
        transforms = rospy.Publisher('/tf', TFMessage, queue_size=10)
        received = []
        rospy.Subscriber('/FAR_obs_debug', PointCloud2, received.append, queue_size=10)
        stamp = rospy.Time(5, 0)

        def publish_pose():
            pose = Odometry()
            pose.header = Header(stamp=stamp, frame_id='map')
            pose.pose.pose.orientation.w = 1.0
            odometry.publish(pose)

        def publish_tf(offset):
            transform = TransformStamped()
            transform.header = Header(stamp=stamp, frame_id='map')
            transform.child_frame_id = 'sensor'
            transform.transform.translation.x = offset
            transform.transform.rotation.w = 1.0
            transforms.publish(TFMessage(transforms=[transform]))

        def x_values(message):
            offset = next(field.offset for field in message.fields if field.name == 'x')
            return [struct.unpack_from('<f', message.data, index + offset)[0]
                    for index in range(0, len(message.data), message.point_step)]

        try:
            wait_for(lambda: rospy.Time.now().to_sec() >= 20.0 and
                     all(p.get_num_connections() > 0 for p in
                         (odometry, terrain, transforms)))
            publish_tf(1.0)
            time.sleep(0.1)
            publish_pose()
            time.sleep(0.1)
            terrain.publish(cloud(stamp))
            wait_for(lambda: received and x_values(received[-1]))
            self.assertTrue(any(abs(x - 3.0) < 1e-4 for x in x_values(received[-1])))

            clock.set_time(4.9)
            time.sleep(0.2)  # Let the planner observe the rewind before new data.
            before = len(received)
            publish_pose()
            time.sleep(0.1)
            terrain.publish(cloud(stamp))
            time.sleep(0.2)
            self.assertEqual(len(received), before,
                             'New-epoch terrain used a TF cached in the old epoch')

            publish_tf(2.0)
            wait_for(lambda: len(received) > before)
            self.assertTrue(any(abs(x - 4.0) < 1e-4 for x in x_values(received[-1])))
        finally:
            clock.close()


if __name__ == '__main__':
    rospy.init_node('test_tf_epoch_ros1', anonymous=True)
    rostest.rosrun('far_planner', 'test_tf_epoch_ros1', TfEpochTest)
