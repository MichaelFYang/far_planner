#!/usr/bin/env python3
# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT
"""Check graph rejection and replacement in the running ROS 1 decoder."""

import os
import struct
import tempfile
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Point
from std_msgs.msg import String
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
        marker_sub = rospy.Subscriber('/graph_decoder_viz', MarkerArray, markers.append, queue_size=10)
        graphs = rospy.Publisher('/robot_vgraph', Graph, queue_size=10)
        save_requests = rospy.Publisher('/save_file_dir', String, queue_size=10)
        read_requests = rospy.Publisher('/read_file_dir', String, queue_size=10)
        rospy.wait_for_service('/request_graph_service', timeout=10)
        request = rospy.ServiceProxy('/request_graph_service', Trigger)
        wait_for(lambda: graphs.get_num_connections() > 0
                 and save_requests.get_num_connections() > 0
                 and read_requests.get_num_connections() > 0
                 and marker_sub.get_num_connections() > 0)

        def request_graph(expected_size, expected_robot):
            previous = len(published)
            result = request()
            self.assertTrue(result.success)
            wait_for(lambda: len(published) > previous)
            graph = published[-1]
            self.assertEqual(graph.size, expected_size)
            self.assertEqual(graph.robot_id, expected_robot)
            self.assertEqual(graph.header.frame_id, 'map')
            self.assertEqual(len(graph.nodes), expected_size)
            return graph

        valid = Graph()
        valid.header.frame_id = 'map'
        valid.robot_id = 7
        for identifier, z_values in ((1, (-1.0, -1.0)),
                                     (2, (0.25, 0.75))):
            node = Node()
            node.header.frame_id = 'map'
            node.id = identifier
            node.position.x = 1.23456789 if identifier == 1 else 0.0
            node.surface_dirs = [
                Point(x=0.5, y=0.0, z=z_values[0]),
                Point(x=0.0, y=0.5, z=z_values[1]),
            ]
            node.FreeType = 3 if identifier == 1 else 1
            valid.nodes.append(node)
        valid.nodes[0].connect_nodes = [2]
        valid.nodes[0].poly_connects = [2]
        valid.nodes[0].contour_connects = [2]
        valid.nodes[0].trajectory_connects = [2]
        valid.size = 2
        graphs.publish(valid)
        wait_for(lambda: len(markers) > 0)
        marker_by_name = {marker.ns: marker for marker in markers[-1].markers}
        self.assertEqual(len(marker_by_name), 12)
        for name in ('global_vgraph', 'visibility_edge', 'polygon_edge'):
            self.assertEqual(len(marker_by_name[name].points), 2)
        round_trip = request_graph(2, 7)
        self.assertEqual([n.id for n in round_trip.nodes], [1, 2])
        for original, decoded in zip(valid.nodes, round_trip.nodes):
            self.assertEqual(decoded.FreeType, original.FreeType)
            self.assertEqual(decoded.surface_dirs, original.surface_dirs)
            for field in ('connect_nodes', 'poly_connects',
                          'contour_connects', 'trajectory_connects'):
                self.assertEqual(list(getattr(decoded, field)),
                                 list(getattr(original, field)))

        with tempfile.TemporaryDirectory(prefix='far-graph-save-') as directory:
            save_path = os.path.join(directory, 'saved_graph.vgh')
            save_requests.publish(String(data=save_path))

            def save_complete():
                if not os.path.exists(save_path):
                    return False
                with open(save_path, encoding='utf-8') as graph_file:
                    return graph_file.read().endswith('\n')

            wait_for(save_complete)
            with open(save_path, encoding='utf-8') as graph_file:
                saved_x = float(graph_file.readline().split()[2])
            self.assertEqual(struct.pack('f', saved_x),
                             struct.pack('f', valid.nodes[0].position.x))
            previous = len(published)
            read_requests.publish(String(data=save_path))
            wait_for(lambda: len(published) > previous)
            loaded = published[-1]
            self.assertEqual([node.id for node in loaded.nodes], [1, 2])
            self.assertEqual(struct.pack('f', loaded.nodes[0].position.x),
                             struct.pack('f', valid.nodes[0].position.x))
            for field in ('connect_nodes', 'poly_connects',
                          'contour_connects', 'trajectory_connects'):
                self.assertEqual(list(getattr(loaded.nodes[0], field)), [2])

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
