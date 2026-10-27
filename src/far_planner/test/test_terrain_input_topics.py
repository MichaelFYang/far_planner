# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT
"""Check scan/terrain field semantics through the installed launch file."""

import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import PointCloud2, PointField


def test_local_occupancy_uses_terrain_cost_instead_of_scan_reflectivity():
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    with tempfile.TemporaryDirectory(prefix='far-input-topics-') as directory:
        os.environ['ROS_LOG_DIR'] = directory
        config = Path(directory) / 'settings.yaml'
        config.write_text(json.dumps({'far_planner': {'ros__parameters': {
            'is_static_env': False, 'is_opencv_visual': False,
            'is_pub_boundary': False, 'main_run_freq': 10.0,
            'sensor_range': 10.0, 'terrain_range': 5.0,
            'map_handler/map_grid_max_length': 100.0,
            'map_handler/map_grad_max_height': 10.0,
            'c_detector/filter_count_value': 100,
        }}}))
        rclpy.init()
        node = rclpy.create_node('terrain_input_topics_test')
        maps = []
        node.create_subscription(PointCloud2, '/local_terrain_map_debug', maps.append, 10)
        odometry = node.create_publisher(Odometry, '/state_estimation', 10)
        global_terrain = node.create_publisher(PointCloud2, '/terrain_map_ext', 10)
        local_terrain = node.create_publisher(PointCloud2, '/terrain_map', 10)
        scans = node.create_publisher(PointCloud2, '/registered_scan', 10)

        def cloud(points, header):
            message = PointCloud2()
            message.header = header
            message.height, message.width = 1, len(points)
            message.fields = [PointField(name=name, offset=4*i,
                                         datatype=PointField.FLOAT32, count=1)
                              for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
            message.point_step = 16
            message.row_step = 16 * len(points)
            message.is_dense = True
            message.data = b''.join(struct.pack('<ffff', *point) for point in points)
            return message

        def has_free_obstacle_cell(message):
            offsets = {field.name: field.offset for field in message.fields}
            return any(abs(struct.unpack_from('<f', message.data, i+offsets['x'])[0]-1.0) < 0.12
                       and abs(struct.unpack_from('<f', message.data, i+offsets['y'])[0]) < 0.12
                       for i in range(0, len(message.data), message.point_step))

        with open(Path(directory) / 'launch.log', 'w+') as log:
            launch_file = Path(get_package_share_directory('far_planner')) / 'launch' / 'far_planner.launch'
            process = subprocess.Popen([
                'ros2', 'launch', str(launch_file),
                'use_rviz:=false', 'config:=' + str(config.with_suffix('')),
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

            def spin_until(predicate, description, timeout=8.0):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    assert process.poll() is None, 'Launch exited during input-topic test'
                    rclpy.spin_once(node, timeout_sec=0.02)
                    if predicate():
                        return
                raise AssertionError('Timed out waiting for ' + description)

            def pose():
                message = Odometry()
                message.header.frame_id = 'map'
                message.header.stamp = node.get_clock().now().to_msg()
                message.pose.pose.position.z = 0.75
                message.pose.pose.orientation.w = 1.0
                odometry.publish(message)
                return message

            try:
                client = node.create_client(GetParameters, '/far_planner/get_parameters')
                spin_until(lambda: client.service_is_ready()
                           and all(pub.get_subscription_count() > 0 for pub in
                                   (odometry, global_terrain, local_terrain, scans)),
                           'launch subscriptions')
                request = GetParameters.Request()
                request.names = ['is_static_env']
                ready = client.call_async(request)
                spin_until(ready.done, 'planner executor')
                assert not ready.result().values[0].bool_value
                first = pose()
                global_terrain.publish(cloud([(0.5*x, 0.5*y, 0.0, 0.0)
                                              for x in range(-4, 5) for y in range(-4, 5)],
                                             first.header))
                spin_until(lambda: maps and has_free_obstacle_cell(maps[-1]),
                           'initial free local grid')
                observed = pose()
                # The raw scan and terrain describe the same wall. Only the
                # terrain intensity encodes its height cost above the ground.
                scans.publish(cloud([(1.0, 0.0, 1.0, 0.0)], observed.header))
                local_terrain.publish(cloud([(1.0, 0.0, 1.0, 1.0)], observed.header))
                before = len(maps)
                spin_until(lambda: len(maps) > before and not has_free_obstacle_cell(maps[-1]),
                           'terrain obstacle occupancy', timeout=4.0)
                assert maps[-1].header.frame_id == 'map'
                cleared = pose()
                scans.publish(cloud([(1.0, 0.0, 1.0, 0.0)], cleared.header))
                local_terrain.publish(cloud([(1.0, 0.0, 0.0, 0.0)], cleared.header))
                before = len(maps)
                spin_until(lambda: len(maps) > before and has_free_obstacle_cell(maps[-1]),
                           'replacement free terrain')
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read()[-3000:])
                raise
            finally:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
