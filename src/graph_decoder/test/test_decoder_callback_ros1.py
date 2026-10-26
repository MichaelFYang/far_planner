#!/usr/bin/env python3
# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT
"""Check graph rejection and replacement in the running ROS 1 decoder."""

import time
import unittest

import rospy
import rostest
from std_srvs.srv import Trigger
from visibility_graph_msg.msg import Graph, Node
from visualization_msgs.msg import MarkerArray


def wait_for(predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError('Timed out waiting for decoder callback')


class DecoderCallbackTest(unittest.TestCase):
    def test_keeps_last_valid_graph(self):
        published = []
        markers = []
        rospy.Subscriber('/decoded_vgraph', Graph, published.append, queue_size=10)
        rospy.Subscriber('/graph_decoder_viz', MarkerArray, markers.append, queue_size=10)
        graphs = rospy.Publisher('/robot_vgraph', Graph, queue_size=10)
        rospy.wait_for_service('/request_graph_service', timeout=10)
        request = rospy.ServiceProxy('/request_graph_service', Trigger)
        wait_for(lambda: graphs.get_num_connections() > 0)

        def request_graph(expected_size, expected_robot):
            previous = len(published)
            result = request()
            self.assertTrue(result.success)
            wait_for(lambda: len(published) > previous)
            graph = published[-1]
            self.assertEqual(graph.size, expected_size)
            self.assertEqual(graph.robot_id, expected_robot)
            self.assertEqual(len(graph.nodes), expected_size)
            return graph

        valid = Graph()
        valid.header.frame_id = 'map'
        valid.robot_id = 7
        for identifier in (1, 2):
            node = Node()
            node.header.frame_id = 'map'
            node.id = identifier
            node.surface_dirs = [node.position, node.position]
            valid.nodes.append(node)
        valid.nodes[0].connect_nodes = [2]
        valid.size = 2
        graphs.publish(valid)
        wait_for(lambda: len(markers) > 0)
        self.assertEqual([n.id for n in request_graph(2, 7).nodes], [1, 2])

        malformed = Graph()
        malformed.header.frame_id = 'map'
        malformed.robot_id = 9
        malformed.nodes = valid.nodes[:1]
        malformed.nodes[0].connect_nodes = [99]
        malformed.size = 1
        graphs.publish(malformed)
        time.sleep(0.35)
        self.assertEqual([n.id for n in request_graph(2, 7).nodes], [1, 2])

        empty = Graph()
        empty.header.frame_id = 'map'
        empty.robot_id = 11
        previous_markers = len(markers)
        graphs.publish(empty)
        wait_for(lambda: len(markers) > previous_markers)
        request_graph(0, 11)


if __name__ == '__main__':
    rospy.init_node('test_decoder_callback_ros1', anonymous=True)
    rostest.rosrun('graph_decoder', 'test_decoder_callback_ros1', DecoderCallbackTest)
