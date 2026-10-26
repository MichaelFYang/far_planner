#!/usr/bin/env python3
# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT
"""Keep measurement-time poses available at the default graph update rate."""

import struct
import time
import unittest

import rospy
import rostest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visibility_graph_msg.msg import Graph


class CallbackCadenceTest(unittest.TestCase):
    def test_delayed_terrain_matches_high_rate_odometry(self):
        odometry = rospy.Publisher('/odom_world', Odometry, queue_size=100)
        terrain = rospy.Publisher('/terrain_cloud', PointCloud2, queue_size=10)
        free_updates, graphs = [], []
        free_sub = rospy.Subscriber('/FAR_free_debug', PointCloud2, free_updates.append,
                                    queue_size=20)
        graph_sub = rospy.Subscriber('/robot_vgraph', Graph, graphs.append, queue_size=20)
        deadline = time.monotonic() + 10.0
        while not all(p.get_num_connections() for p in (odometry, terrain, free_sub, graph_sub)):
            self.assertLess(time.monotonic(), deadline, 'Planner did not connect')
            time.sleep(0.02)

        ground = PointCloud2()
        ground.height = 1
        ground.fields = [PointField(name=name, offset=4*i,
                                    datatype=PointField.FLOAT32, count=1)
                         for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
        ground.data = b''.join(struct.pack('<ffff', x*0.5, y*0.5, 0.0, 0.0)
                               for x in range(-4, 5) for y in range(-4, 5))
        ground.width = 81
        ground.point_step = 16
        ground.row_step = ground.width * ground.point_step
        ground.is_dense = True
        began = time.monotonic()
        next_terrain = began + 0.2
        while time.monotonic() - began < 3.0:
            stamp = rospy.Time.now()
            pose = Odometry()
            pose.header = Header(stamp=stamp, frame_id='map')
            pose.pose.pose.position.z = 0.75
            pose.pose.pose.orientation.w = 1.0
            odometry.publish(pose)
            if time.monotonic() >= next_terrain:
                # Transport/processing delay remains within the 0.2 s policy.
                ground.header = Header(stamp=stamp-rospy.Duration(0.12), frame_id='map')
                terrain.publish(ground)
                next_terrain += 0.2
            time.sleep(0.005)
        time.sleep(0.5)
        # Preserve the 5 Hz sensor stream independently of 2.5 Hz graph ticks.
        self.assertGreaterEqual(sum(m.width > 0 for m in free_updates), 10,
                                'Low-rate graph updates starved measurement-time odometry')
        self.assertTrue(any(g.nodes for g in graphs), 'Free terrain did not initialize a graph')


if __name__ == '__main__':
    rospy.init_node('test_callback_cadence_ros1')
    rostest.rosrun('far_planner', 'test_callback_cadence_ros1', CallbackCadenceTest)
