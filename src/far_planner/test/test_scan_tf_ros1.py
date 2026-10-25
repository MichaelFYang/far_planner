#!/usr/bin/env python3
"""Exercise delayed scan TF delivery through the running ROS 1 planner."""

import struct
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage


def wait_for(predicate, timeout=12.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError('Timed out waiting for planner scan callback')


def cloud(frame, stamp):
    message = PointCloud2()
    message.header = Header(stamp=stamp, frame_id=frame)
    message.height = message.width = 1
    message.fields = [PointField(name=name, offset=4 * index,
                                 datatype=PointField.FLOAT32, count=1)
                      for index, name in enumerate(('x', 'y', 'z', 'intensity'))]
    message.point_step = message.row_step = 16
    message.is_dense = True
    message.data = struct.pack('<ffff', 2.0, 0.0, 0.0, 1.0)
    return message


class ScanTfRetryTest(unittest.TestCase):
    def test_scan_is_processed_after_tf_arrives(self):
        odometry = rospy.Publisher('/odom_world', Odometry, queue_size=10)
        scans = rospy.Publisher('/scan_cloud', PointCloud2, queue_size=10)
        terrain = rospy.Publisher('/terrain_cloud', PointCloud2, queue_size=10)
        transforms = rospy.Publisher('/tf', TFMessage, queue_size=10)
        rays = []
        rospy.Subscriber('/FAR_scanGrid_debug', PointCloud2, rays.append, queue_size=10)
        wait_for(lambda: all(p.get_num_connections() > 0 for p in
                             (odometry, scans, terrain, transforms)))

        stamp = rospy.Time.now()
        pose = Odometry()
        pose.header = Header(stamp=stamp, frame_id='map')
        pose.pose.pose.orientation.w = 1.0
        odometry.publish(pose)
        time.sleep(0.15)
        scans.publish(cloud('late_sensor', stamp))
        time.sleep(0.15)

        transform = TransformStamped()
        transform.header = Header(stamp=stamp, frame_id='map')
        transform.child_frame_id = 'late_sensor'
        transform.transform.translation.x = 1.0
        transform.transform.rotation.w = 1.0
        transforms.publish(TFMessage(transforms=[transform]))
        time.sleep(0.25)
        terrain.publish(cloud('map', stamp + rospy.Duration.from_sec(0.1)))
        wait_for(lambda: len(rays) > 0)
        self.assertGreater(rays[-1].width, 0,
                           'Scan was not ray traced after its delayed TF arrived')


if __name__ == '__main__':
    rospy.init_node('test_scan_tf_ros1', anonymous=True)
    rostest.rosrun('far_planner', 'test_scan_tf_ros1', ScanTfRetryTest)
