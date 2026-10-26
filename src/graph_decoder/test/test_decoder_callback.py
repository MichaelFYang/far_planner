# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT
"""Exercise graph replacement and rejection in the running decoder."""

import os
import signal
import subprocess
import tempfile
import time

import rclpy
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
                           and graphs.get_subscription_count() > 0)
                valid = Graph()
                valid.header.frame_id = 'map'
                valid.robot_id = 7
                for identifier in (1, 2):
                    graph_node = Node()
                    graph_node.header.frame_id = 'map'
                    graph_node.id = identifier
                    graph_node.surface_dirs = [graph_node.position, graph_node.position]
                    valid.nodes.append(graph_node)
                valid.nodes[0].connect_nodes = [2]
                valid.size = 2
                graphs.publish(valid)
                spin_until(lambda: len(markers) > 0)
                assert [n.id for n in request_graph(2, 7).nodes] == [1, 2]

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
