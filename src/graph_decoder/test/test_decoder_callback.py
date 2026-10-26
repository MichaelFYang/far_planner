# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT
"""Exercise graph replacement and rejection in the running decoder."""

import os
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visibility_graph_msg.msg import Graph, Node
from visualization_msgs.msg import MarkerArray


def test_decoder_keeps_last_valid_graph():
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    with tempfile.TemporaryDirectory(prefix='far-decoder-test-') as directory:
        os.environ['ROS_LOG_DIR'] = directory
        rclpy.init()
        node = rclpy.create_node('decoder_callback_test')
        published = []
        markers = []
        node.create_subscription(Graph, '/decoded_vgraph', published.append, 10)
        node.create_subscription(MarkerArray, '/graph_decoder_viz', markers.append, 10)
        graphs = node.create_publisher(Graph, '/robot_vgraph', 10)
        save_requests = node.create_publisher(String, '/save_file_dir', 10)
        client = node.create_client(Trigger, '/request_graph_service')

        with open(os.path.join(directory, 'decoder.log'), 'w+') as log:
            process = subprocess.Popen(
                [os.environ['GRAPH_DECODER_EXECUTABLE']],
                stdout=log, stderr=subprocess.STDOUT)

            def spin_until(predicate, timeout=10.0):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    assert process.poll() is None, 'Decoder exited during callback test'
                    rclpy.spin_once(node, timeout_sec=0.02)
                    if predicate():
                        return
                raise AssertionError('Timed out waiting for decoder callback')

            def request_graph(expected_size, expected_robot):
                previous = len(published)
                future = client.call_async(Trigger.Request())
                spin_until(lambda: future.done() and len(published) > previous)
                assert future.result().success
                graph = published[-1]
                assert graph.size == expected_size
                assert graph.robot_id == expected_robot
                assert len(graph.nodes) == expected_size
                return graph

            try:
                spin_until(lambda: client.service_is_ready()
                           and graphs.get_subscription_count() > 0
                           and save_requests.get_subscription_count() > 0)
                valid = Graph()
                valid.header.frame_id = 'map'
                valid.robot_id = 7
                for identifier, z_values in ((1, (-1.0, -1.0)),
                                             (2, (0.25, 0.75))):
                    graph_node = Node()
                    graph_node.header.frame_id = 'map'
                    graph_node.id = identifier
                    graph_node.position.x = 1.23456789 if identifier == 1 else 0.0
                    graph_node.surface_dirs = [
                        Point(x=0.5, y=0.0, z=z_values[0]),
                        Point(x=0.0, y=0.5, z=z_values[1]),
                    ]
                    graph_node.freetype = 3 if identifier == 1 else 1
                    valid.nodes.append(graph_node)
                valid.nodes[0].connect_nodes = [2]
                valid.size = 2
                graphs.publish(valid)
                spin_until(lambda: len(markers) > 0)
                round_trip = request_graph(2, 7)
                assert [n.id for n in round_trip.nodes] == [1, 2]
                for original, decoded in zip(valid.nodes, round_trip.nodes):
                    assert decoded.freetype == original.freetype
                    assert decoded.surface_dirs == original.surface_dirs

                save_path = os.path.join(directory, 'saved_graph.vgh')
                save_requests.publish(String(data=save_path))

                def save_complete():
                    if not os.path.exists(save_path):
                        return False
                    with open(save_path, encoding='utf-8') as graph_file:
                        return graph_file.read().endswith('\n')

                spin_until(save_complete)
                with open(save_path, encoding='utf-8') as graph_file:
                    saved_x = float(graph_file.readline().split()[2])
                assert struct.pack('f', saved_x) == struct.pack('f', valid.nodes[0].position.x), \
                    'Graph save lost float coordinate precision'

                malformed = Graph()
                malformed.header.frame_id = 'map'
                malformed.robot_id = 9
                malformed.nodes = valid.nodes[:1]
                malformed.nodes[0].connect_nodes = [99]
                malformed.size = 1
                graphs.publish(malformed)
                # The decoder emits no marker acknowledgement for a rejected graph.
                deadline = time.monotonic() + 0.35
                spin_until(lambda: time.monotonic() >= deadline, 1.0)
                assert [n.id for n in request_graph(2, 7).nodes] == [1, 2]

                empty = Graph()
                empty.header.frame_id = 'map'
                empty.robot_id = 11
                previous_markers = len(markers)
                graphs.publish(empty)
                spin_until(lambda: len(markers) > previous_markers)
                request_graph(0, 11)
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
                node.destroy_node()
                rclpy.shutdown()
