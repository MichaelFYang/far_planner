# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT
"""Check trajectory sampling when the image filter emits no obstacle vertices."""

import os
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import PointCloud2, PointField
from visibility_graph_msg.msg import Graph


def test_trajectory_initializes_without_new_contour_vertices():
    os.environ['ROS_DOMAIN_ID'] = str(100 + (os.getpid() + 1) % 100)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    with tempfile.TemporaryDirectory(prefix='far-trajectory-test-') as directory:
        os.environ['ROS_LOG_DIR'] = directory
        rclpy.init()
        node = rclpy.create_node('trajectory_sampling_test')
        graphs = []
        node.create_subscription(Graph, '/robot_vgraph', graphs.append, 10)
        odometry = node.create_publisher(Odometry, '/odom_world', 10)
        terrain = node.create_publisher(PointCloud2, '/terrain_cloud', 10)
        with open(os.path.join(directory, 'planner.log'), 'w+') as log:
            process = subprocess.Popen([
                os.environ['FAR_PLANNER_EXECUTABLE'], '--ros-args',
                '-p', 'is_opencv_visual:=false', '-p', 'is_pub_boundary:=false',
                '-p', 'is_static_env:=false', '-p', 'main_run_freq:=10.0',
                '-p', 'sensor_range:=10.0', '-p', 'terrain_range:=5.0',
                '-p', 'map_handler/map_grid_max_length:=100.0',
                '-p', 'map_handler/map_grad_max_height:=10.0',
                '-p', 'c_detector/filter_count_value:=100',
            ], stdout=log, stderr=subprocess.STDOUT)

            def spin_until(predicate, timeout=10.0):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    assert process.poll() is None, 'Planner exited during trajectory test'
                    rclpy.spin_once(node, timeout_sec=0.02)
                    if predicate():
                        return
                raise AssertionError('Timed out waiting for trajectory graph')

            try:
                client = node.create_client(GetParameters, '/far_planner_node/get_parameters')
                spin_until(lambda: client.service_is_ready()
                           and odometry.get_subscription_count() > 0
                           and terrain.get_subscription_count() > 0
                           and node.count_publishers('/robot_vgraph') > 0)
                request = GetParameters.Request()
                request.names = ['world_frame']
                ready = client.call_async(request)
                spin_until(ready.done)
                assert ready.result().values[0].string_value == 'map'

                pose = Odometry()
                pose.header.frame_id = 'map'
                pose.header.stamp = node.get_clock().now().to_msg()
                pose.pose.pose.position.z = 0.75
                pose.pose.pose.orientation.w = 1.0
                cloud = PointCloud2()
                cloud.header = pose.header
                points = [(0.5*x, 0.5*y, 0.0, 0.0)
                          for x in range(-4, 5) for y in range(-4, 5)]
                # This initializes the obstacle map, but its single pixel vote
                # stays below the configured image threshold: no new vertices.
                points.append((4.0, 4.0, 0.0, 1.0))
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
                spin_until(lambda: len(graphs) > 0)
                assert graphs[0].size == 1
                assert len(graphs[0].nodes) == 1
                trajectory = graphs[0].nodes[0]
                assert trajectory.is_navpoint
                assert abs(trajectory.position.z - 0.75) < 1e-5
                spin_until(lambda: len(graphs) >= 4)
                assert all([graph_node.id for graph_node in graph.nodes] == [trajectory.id]
                           for graph in graphs), 'Stationary graph ticks duplicated trajectory nodes'
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read())
                raise
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
                node.destroy_node()
                rclpy.shutdown()
