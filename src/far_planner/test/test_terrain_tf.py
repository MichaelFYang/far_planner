# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT
"""Exercise the real terrain callback through ROS, including failed TF delivery."""

import os
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
from rclpy.parameter_client import AsyncParameterClient
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage


def test_terrain_transform_is_timestamped_and_atomic():
    # Keep this process and its child away from the user's running robot graph.
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    with tempfile.TemporaryDirectory(prefix='far-tf-test-') as directory:
        os.environ['ROS_LOG_DIR'] = directory
        rclpy.init()
        node = rclpy.create_node('terrain_tf_test')
        received = []
        rays = []
        node.create_subscription(PointCloud2, '/FAR_obs_debug', received.append, 10)
        node.create_subscription(PointCloud2, '/FAR_scanGrid_debug', rays.append, 10)
        scans = node.create_publisher(PointCloud2, '/scan_cloud', 10)
        clock = node.create_publisher(Clock, '/clock', 10)
        clouds = node.create_publisher(PointCloud2, '/terrain_cloud', 10)
        odom = node.create_publisher(Odometry, '/odom_world', 10)
        transforms = node.create_publisher(TFMessage, '/tf', 10)
        with open(os.path.join(directory, 'planner.log'), 'w+') as log:
            process = subprocess.Popen([
                os.environ['FAR_PLANNER_EXECUTABLE'], '--ros-args',
                '-p', 'is_opencv_visual:=false', '-p', 'main_run_freq:=0.01',
                '-p', 'world_frame:=map', '-p', 'use_sim_time:=true',
                '-p', 'is_static_env:=false',
            ], stdout=log, stderr=subprocess.STDOUT)

            def spin_until(predicate, timeout=10.0):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    assert process.poll() is None, 'Planner exited during callback test'
                    rclpy.spin_once(node, timeout_sec=0.02)
                    if predicate():
                        return
                raise AssertionError('Timed out waiting for planner callback')

            def settle(duration=0.3):
                deadline = time.monotonic() + duration
                spin_until(lambda: time.monotonic() >= deadline, duration + 2.0)

            def cloud(frame, seconds, nanoseconds=0):
                msg = PointCloud2()
                msg.header.frame_id = frame
                msg.header.stamp.sec = seconds
                msg.header.stamp.nanosec = nanoseconds
                msg.height = msg.width = 1
                msg.fields = [PointField(name=name, offset=4*i,
                                         datatype=PointField.FLOAT32, count=1)
                              for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
                msg.point_step = msg.row_step = 16
                msg.is_dense = True
                msg.data = struct.pack('<ffff', 2.0, 0.0, 0.0, 1.0)
                return msg

            try:
                spin_until(lambda: clouds.get_subscription_count() > 0
                           and odom.get_subscription_count() > 0
                           and transforms.get_subscription_count() > 0
                           and node.count_publishers('/FAR_obs_debug') > 0)
                # Publishers are created before the planner finishes initializing.
                # A service response proves its executor is now processing callbacks.
                client = AsyncParameterClient(node, '/far_planner_node')
                spin_until(client.services_are_ready)
                ready = client.get_parameters(['world_frame'])
                spin_until(ready.done)
                assert ready.result().values[0].string_value == 'map'
                spin_until(lambda: clock.get_subscription_count() > 0
                           and scans.get_subscription_count() > 0)
                tick = Clock()
                tick.clock.sec = 20
                clock.publish(tick)
                settle()
                pose = Odometry()
                pose.header.frame_id = 'map'
                pose.header.stamp.sec = 10
                pose.pose.pose.orientation.w = 1.0
                odom.publish(pose)
                samples = []
                for seconds, x in ((10, 1.0), (12, 3.0)):
                    tf = TransformStamped()
                    tf.header.frame_id = 'map'
                    tf.child_frame_id = 'sensor'
                    tf.header.stamp.sec = seconds
                    tf.transform.translation.x = x
                    tf.transform.rotation.w = 1.0
                    samples.append(tf)
                transforms.publish(TFMessage(transforms=samples))
                settle()
                clouds.publish(cloud('sensor', 10))
                spin_until(lambda: len(received) == 1)
                result = received[-1]
                assert result.header.frame_id == 'map'
                x_offset = next(field.offset for field in result.fields if field.name == 'x')
                xs = [struct.unpack_from('<f', result.data, offset + x_offset)[0]
                      for offset in range(0, len(result.data), result.point_step)]
                assert xs and all(abs(x - 3.0) < 1e-5 for x in xs), xs

                # Each rejected cloud must return before map updates/publication.
                for rejected in (cloud('missing', 10), cloud('sensor', 9),
                                 cloud('sensor', 12, 1), cloud('map', 0), cloud('', 10), cloud('map', 21)):
                    clouds.publish(rejected)
                    settle()
                    assert len(received) == 1, 'Rejected cloud reached the mapping pipeline'
                # A delayed scan must raycast from its own odometry pose (x=0),
                # even after odometry has advanced to x=5. Older odometry is rejected.
                pose.header.stamp.sec = 13
                pose.pose.pose.position.x = 50.0
                pose.pose.pose.orientation.w = 0.0
                odom.publish(pose)
                settle()
                pose.header.stamp.sec = 12
                pose.pose.pose.position.x = 5.0
                pose.pose.pose.orientation.w = 1.0
                odom.publish(pose)
                settle()
                pose.header.stamp.sec = 11
                pose.pose.pose.position.x = 50.0
                odom.publish(pose)
                scans.publish(cloud('map', 10))
                settle()
                clouds.publish(cloud('map', 10, 100000000))
                spin_until(lambda: len(received) == 2 and len(rays) == 2)

                def x_values(msg):
                    offset = next(field.offset for field in msg.fields if field.name == 'x')
                    return [struct.unpack_from('<f', msg.data, i + offset)[0]
                            for i in range(0, len(msg.data), msg.point_step)]

                assert x_values(rays[-1]) and max(x_values(rays[-1])) < 2.0
                assert x_values(received[-1]), 'Older odometry moved the map crop center'
                clouds.publish(cloud('map', 10, 100000000))
                settle()
                assert len(received) == 2, 'Duplicate terrain was processed again'

                # A newer terrain update must not clear obstacles with an old scan.
                clouds.publish(cloud('/map', 11))
                spin_until(lambda: len(received) == 3 and len(rays) == 3)
                assert not x_values(rays[-1]), 'Stale scan was reused for ray clearing'

                # Rewinding simulated time must clear old observations and ordering
                # guards, allowing a fresh odometry/cloud pair in the new epoch.
                tick.clock.sec = 5
                clock.publish(tick)
                settle()
                # A queued message from the previous epoch must not prevent
                # fresh (lower-stamped) odometry from being accepted.
                pose.header.stamp.sec = 12
                odom.publish(pose)
                settle()
                pose.header.stamp.sec = 5
                pose.pose.pose.position.x = 0.0
                odom.publish(pose)
                settle()
                clouds.publish(cloud('map', 5))
                spin_until(lambda: len(received) == 4)
                assert x_values(received[-1])
                assert all(abs(x - 2.0) < 1e-5 for x in x_values(received[-1]))

                # The scan arrives between odometry stamps, before the later
                # odometry callback. It must be retained for interpolation.
                scans.publish(cloud('map', 5, 50000000))
                settle()
                pose.header.stamp.nanosec = 100000000
                pose.pose.pose.position.x = 0.5
                odom.publish(pose)
                settle()
                clouds.publish(cloud('map', 5, 60000000))
                spin_until(lambda: len(received) == 5 and len(rays) == 5)
                assert x_values(rays[-1]), 'Deferred scan was not ray traced'

                # A scan can outlive a temporarily missing TF. No new scan or
                # odometry callback should be needed after that TF arrives.
                tick.clock.sec = 6
                clock.publish(tick)
                settle(0.1)
                scans.publish(cloud('late_sensor', 5, 100000000))
                settle(0.15)
                clouds.publish(cloud('map', 5, 260000000))
                spin_until(lambda: len(received) == 6 and len(rays) == 6)
                assert not x_values(rays[-1]), 'Scan was used before its TF arrived'
                late_tf = TransformStamped()
                late_tf.header.frame_id = 'map'
                late_tf.child_frame_id = 'late_sensor'
                late_tf.header.stamp.sec = 5
                late_tf.header.stamp.nanosec = 100000000
                late_tf.transform.translation.x = 1.0
                late_tf.transform.rotation.w = 1.0
                transforms.publish(TFMessage(transforms=[late_tf]))
                settle(0.25)
                clouds.publish(cloud('map', 5, 270000000))
                spin_until(lambda: len(received) == 7 and len(rays) == 7)
                assert x_values(rays[-1]), 'Scan was not retried after its TF arrived'
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read())
                raise
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                node.destroy_node()
                rclpy.shutdown()
