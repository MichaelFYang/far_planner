#!/usr/bin/env python3
# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT
"""Check free-terrain startup, trajectory sampling, and goal output in ROS 1."""

import struct
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visibility_graph_msg.msg import Graph


def wait_for(predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError('Timed out waiting for free-terrain planning')


class TrajectorySamplingTest(unittest.TestCase):
    def test_free_terrain_initializes_graph_and_goal(self):
        graphs, free_updates, waypoints = [], [], []
        graph_sub = rospy.Subscriber('/robot_vgraph', Graph, graphs.append, queue_size=10)
        free_sub = rospy.Subscriber('/FAR_free_debug', PointCloud2, free_updates.append, queue_size=10)
        waypoint_sub = rospy.Subscriber('/way_point', PointStamped, waypoints.append, queue_size=10)
        odometry = rospy.Publisher('/odom_world', Odometry, queue_size=10)
        terrain = rospy.Publisher('/terrain_cloud', PointCloud2, queue_size=10)
        goals = rospy.Publisher('/goal_point', PointStamped, queue_size=10)
        wait_for(lambda: all(endpoint.get_num_connections() > 0 for endpoint in
                             (odometry, terrain, goals, graph_sub, free_sub, waypoint_sub)))
        pose = Odometry()
        pose.header = Header(stamp=rospy.Time.now(), frame_id='map')
        pose.pose.pose.position.z = 0.75
        pose.pose.pose.orientation.w = 1.0
        cloud = PointCloud2()
        cloud.header = pose.header
        points = [(0.5*x, 0.5*y, 0.0, 0.0)
                  for x in range(-4, 5) for y in range(-4, 5)]
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [PointField(name=name, offset=4*i,
                                   datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
        cloud.point_step = 16
        cloud.row_step = cloud.width * cloud.point_step
        cloud.is_dense = True
        cloud.data = b''.join(struct.pack('<ffff', *point) for point in points)
        odometry.publish(pose)
        terrain.publish(cloud)
        wait_for(lambda: len(free_updates) > 0)
        self.assertGreater(free_updates[-1].width, 0)
        wait_for(lambda: len(graphs) > 0)
        self.assertEqual(graphs[0].size, 1)
        self.assertEqual(len(graphs[0].nodes), 1)
        trajectory = graphs[0].nodes[0]
        self.assertTrue(trajectory.is_navpoint)
        self.assertAlmostEqual(trajectory.position.z, 0.75)
        wait_for(lambda: len(graphs) >= 4)
        for graph in list(graphs):
            self.assertEqual([node.id for node in graph.nodes], [trajectory.id])
        goal = PointStamped()
        goal.header = Header(stamp=rospy.Time.now(), frame_id='map')
        goal.point.x = 1.5
        goal.point.z = 0.75
        goals.publish(goal)
        wait_for(lambda: any(abs(msg.point.x - 1.5) < 1e-5
                             and abs(msg.point.y) < 1e-5
                             and abs(msg.point.z - 0.75) < 1e-5
                             and msg.header.frame_id == 'map'
                             for msg in waypoints))


if __name__ == '__main__':
    rospy.init_node('test_trajectory_sampling_ros1', anonymous=True)
    rostest.rosrun('far_planner', 'test_trajectory_sampling_ros1', TrajectorySamplingTest)
