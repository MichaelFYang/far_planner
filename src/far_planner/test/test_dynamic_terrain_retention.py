# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT
"""Keep cleared obstacles excluded while upstream terrain still retains them."""

import os
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField


def test_stale_terrain_cannot_restore_a_recently_cleared_obstacle():
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    with tempfile.TemporaryDirectory(prefix='far-retention-test-') as directory:
        os.environ['ROS_LOG_DIR'] = directory
        rclpy.init()
        node = rclpy.create_node('dynamic_terrain_retention_test')
        observations = []
        node.create_subscription(PointCloud2, '/FAR_obs_debug', observations.append, 10)
        clocks = node.create_publisher(Clock, '/clock', 10)
        odometry = node.create_publisher(Odometry, '/odom_world', 10)
        scans = node.create_publisher(PointCloud2, '/scan_cloud', 10)
        terrain = node.create_publisher(PointCloud2, '/terrain_cloud', 10)

        def cloud(x, intensity, header):
            message = PointCloud2()
            message.header = header
            message.height = message.width = 1
            message.fields = [PointField(name=name, offset=4*i,
                                         datatype=PointField.FLOAT32, count=1)
                              for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
            message.point_step = message.row_step = 16
            message.is_dense = True
            message.data = struct.pack('<ffff', x, 0.0, 0.0, intensity)
            return message

        with open(os.path.join(directory, 'planner.log'), 'w+') as log:
            process = subprocess.Popen([
                os.environ['FAR_PLANNER_EXECUTABLE'], '--ros-args',
                '-p', 'use_sim_time:=true', '-p', 'is_static_env:=false',
                '-p', 'is_opencv_visual:=false', '-p', 'is_pub_boundary:=false',
                '-p', 'main_run_freq:=0.01', '-p', 'util/dyosb_update_thred:=0',
                '-p', 'sensor_range:=10.0', '-p', 'terrain_range:=5.0',
                '-p', 'map_handler/map_grid_max_length:=100.0',
                '-p', 'map_handler/map_grad_max_height:=10.0',
            ], stdout=log, stderr=subprocess.STDOUT)

            def spin_until(predicate, timeout=8.0):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    assert process.poll() is None, 'Planner exited during retention test'
                    rclpy.spin_once(node, timeout_sec=0.02)
                    if predicate():
                        return
                raise AssertionError('Timed out waiting for terrain retention callback')

            def observe(seconds, scan_endpoint=None):
                tick = Clock()
                tick.clock.sec = int(seconds)
                tick.clock.nanosec = round((seconds-int(seconds))*1e9)
                clocks.publish(tick)
                settled = time.monotonic() + 0.2
                spin_until(lambda: time.monotonic() >= settled)
                pose = Odometry()
                pose.header.frame_id = 'map'
                pose.header.stamp = tick.clock
                pose.pose.pose.orientation.w = 1.0
                odometry.publish(pose)
                if scan_endpoint is not None:
                    scans.publish(cloud(scan_endpoint, 0.0, pose.header))
                before = len(observations)
                terrain.publish(cloud(2.0, 1.0, pose.header))
                spin_until(lambda: len(observations) > before)
                return observations[-1].width

            try:
                client = node.create_client(GetParameters, '/far_planner_node/get_parameters')
                spin_until(lambda: client.service_is_ready()
                           and all(pub.get_subscription_count() > 0 for pub in
                                   (clocks, odometry, scans, terrain)))
                request = GetParameters.Request()
                request.names = ['util/dynamic_obs_dacay_time']
                ready = client.call_async(request)
                spin_until(ready.done)
                retention = ready.result().values[0].double_value
                assert observe(10.0, scan_endpoint=2.0) > 0
                assert observe(11.0, scan_endpoint=4.0) == 0
                # A 10-second terrain decay can retain points longer because
                # its voxel pruning runs periodically. Its new header stamp
                # does not make the retained points fresh raw measurements.
                assert observe(23.0) == 0
                assert observe(24.0) == 0, 'Stale terrain restored the cleared obstacle'
                # The exclusion remains bounded: a later real obstacle can
                # enter the map again after the configured interval expires.
                expiry = 11.0 + retention + 2.0
                observe(expiry, scan_endpoint=2.0)
                assert observe(expiry+1.0, scan_endpoint=2.0) > 0
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read()[-3000:])
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
                if process.returncode != 0:
                    log.flush()
                    log.seek(0)
                    print(log.read()[-3000:])
                assert process.returncode == 0, 'Planner did not shut down cleanly'
