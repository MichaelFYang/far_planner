#!/usr/bin/env python3
"""Exercise delayed scan and terrain inputs through the running ROS 1 planner."""

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
from visibility_graph_msg.msg import Graph


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
        terrain_updates = []
        graphs = []
        rospy.Subscriber('/robot_vgraph', Graph, graphs.append, queue_size=10)
        rospy.Subscriber('/FAR_scanGrid_debug', PointCloud2, rays.append, queue_size=10)
        rospy.Subscriber('/FAR_obs_debug', PointCloud2, terrain_updates.append,
                         queue_size=10)
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
        terrain_stamp = stamp + rospy.Duration.from_sec(0.1)
        later_pose = Odometry()
        later_pose.header = Header(stamp=terrain_stamp, frame_id='map')
        later_pose.pose.pose.orientation.w = 1.0
        odometry.publish(later_pose)
        time.sleep(0.15)
        terrain.publish(cloud('map', terrain_stamp))
        wait_for(lambda: len(rays) > 0)
        self.assertGreater(rays[-1].width, 0,
                           'Scan was not ray traced after its delayed TF arrived')
        wait_for(lambda: len(terrain_updates) > 0)
        # The configured image threshold suppresses every obstacle vertex.
        # Trajectory initialization must still run on the graph update.
        wait_for(lambda: any(node.is_navpoint for graph in graphs for node in graph.nodes))

        # A terrain cloud newer than odometry waits for the matching sample.
        deferred_stamp = rospy.Time.now()
        before = len(terrain_updates)
        terrain.publish(cloud('map', deferred_stamp))
        time.sleep(0.2)
        self.assertEqual(len(terrain_updates), before)
        deferred_pose = Odometry()
        deferred_pose.header = Header(stamp=deferred_stamp, frame_id='map')
        deferred_pose.pose.pose.orientation.w = 1.0
        odometry.publish(deferred_pose)
        wait_for(lambda: len(terrain_updates) > before)

        # Missing stamped TF is retried by the wall timer without another input.
        missing_tf_stamp = rospy.Time.now()
        missing_tf_pose = Odometry()
        missing_tf_pose.header = Header(stamp=missing_tf_stamp, frame_id='map')
        missing_tf_pose.pose.pose.orientation.w = 1.0
        odometry.publish(missing_tf_pose)
        time.sleep(0.15)
        before = len(terrain_updates)
        terrain.publish(cloud('late_terrain', missing_tf_stamp))
        time.sleep(0.2)
        self.assertEqual(len(terrain_updates), before)
        terrain_tf = TransformStamped()
        terrain_tf.header = Header(stamp=missing_tf_stamp, frame_id='map')
        terrain_tf.child_frame_id = 'late_terrain'
        terrain_tf.transform.rotation.w = 1.0
        transforms.publish(TFMessage(transforms=[terrain_tf]))
        wait_for(lambda: len(terrain_updates) > before)

        # Late odometry TF must release both the odometry and terrain queues.
        late_odom_stamp = rospy.Time.now()
        late_pose = Odometry()
        late_pose.header = Header(stamp=late_odom_stamp, frame_id='late_odom')
        late_pose.pose.pose.orientation.w = 1.0
        before = len(terrain_updates)
        odometry.publish(late_pose)
        terrain.publish(cloud('late_odom', late_odom_stamp))
        time.sleep(0.2)
        self.assertEqual(len(terrain_updates), before)
        odom_tf = TransformStamped()
        odom_tf.header = Header(stamp=late_odom_stamp, frame_id='map')
        odom_tf.child_frame_id = 'late_odom'
        odom_tf.transform.translation.x = 1.0
        odom_tf.transform.rotation.w = 1.0
        transforms.publish(TFMessage(transforms=[odom_tf]))
        wait_for(lambda: len(terrain_updates) > before)
        self.assertGreater(terrain_updates[-1].width, 0)

        # A scan already waiting for stamped TF gets a short chance to pair
        # with terrain at the same measurement time.
        paired_stamp = rospy.Time.now()
        paired_pose = Odometry()
        paired_pose.header = Header(stamp=paired_stamp, frame_id='map')
        paired_pose.pose.pose.orientation.w = 1.0
        odometry.publish(paired_pose)
        time.sleep(0.15)
        scans.publish(cloud('paired_sensor', paired_stamp))
        time.sleep(0.15)
        before = len(terrain_updates)
        rays_before = len(rays)
        terrain.publish(cloud('map', paired_stamp))
        time.sleep(0.2)
        self.assertEqual(len(terrain_updates), before,
                         'Terrain ran while its queued scan awaited TF')
        paired_tf = TransformStamped()
        paired_tf.header = Header(stamp=paired_stamp, frame_id='map')
        paired_tf.child_frame_id = 'paired_sensor'
        paired_tf.transform.rotation.w = 1.0
        transforms.publish(TFMessage(transforms=[paired_tf]))
        wait_for(lambda: len(terrain_updates) > before and len(rays) > rays_before)
        self.assertGreater(rays[-1].width, 0,
                           'Late scan TF was not used by the terrain update')

        # Terrain may arrive before its matching scan. Its short wait must
        # allow that later scan callback to affect the same map update.
        time.sleep(0.25)
        late_scan_stamp = rospy.Time.now()
        late_scan_pose = Odometry()
        late_scan_pose.header = Header(stamp=late_scan_stamp, frame_id='map')
        late_scan_pose.pose.pose.orientation.w = 1.0
        odometry.publish(late_scan_pose)
        time.sleep(0.15)
        before = len(terrain_updates)
        rays_before = len(rays)
        terrain.publish(cloud('map', late_scan_stamp))
        time.sleep(0.1)
        self.assertEqual(len(terrain_updates), before,
                         'Terrain ran before its scan arrived')
        scans.publish(cloud('map', late_scan_stamp))
        wait_for(lambda: len(terrain_updates) > before and len(rays) > rays_before)
        self.assertGreater(rays[-1].width, 0,
                           'Scan arriving after terrain was not ray traced')


if __name__ == '__main__':
    rospy.init_node('test_scan_tf_ros1', anonymous=True)
    rostest.rosrun('far_planner', 'test_scan_tf_ros1', ScanTfRetryTest)
