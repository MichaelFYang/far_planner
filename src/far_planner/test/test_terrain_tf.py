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
from rcl_interfaces.srv import GetParameters
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
                client = node.create_client(GetParameters, '/far_planner_node/get_parameters')
                spin_until(client.service_is_ready)
                request = GetParameters.Request()
                request.names = ['world_frame']
                ready = client.call_async(request)
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
                malformed = cloud('sensor', 10)
                malformed.data = bytes()
                clouds.publish(malformed)
                settle()
                assert len(received) == 0, 'Truncated terrain cloud reached the map'
                malformed = cloud('sensor', 10)
                malformed.fields = malformed.fields[1:]
                clouds.publish(malformed)
                settle()
                assert len(received) == 0, 'Terrain cloud without X reached the map'
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
                # A delayed scan uses its own odometry pose (x=0). Terrain
                # also requires a nearby pose at its measurement time.
                pose.header.stamp.sec = 13
                pose.pose.pose.position.x = 50.0
                pose.pose.pose.orientation.w = 0.0
                odom.publish(pose)
                settle()
                pose.header.stamp.sec = 10
                pose.header.stamp.nanosec = 100000000
                pose.pose.pose.position.x = 0.0
                pose.pose.pose.orientation.w = 1.0
                odom.publish(pose)
                settle()
                scans.publish(cloud('map', 10))
                settle()
                clouds.publish(cloud('map', 10, 100000000))
                spin_until(lambda: len(received) == 2 and len(rays) == 2)

                def x_values(msg):
                    offset = next(field.offset for field in msg.fields if field.name == 'x')
                    return [struct.unpack_from('<f', msg.data, i + offset)[0]
                            for i in range(0, len(msg.data), msg.point_step)]

                assert x_values(rays[-1]) and max(x_values(rays[-1])) < 2.0
                assert x_values(received[-1])
                pose.header.stamp.nanosec = 150000000
                pose.pose.pose.position.x = 0.1
                odom.publish(pose)
                settle()
                pose.header.stamp.sec = 11
                pose.header.stamp.nanosec = 0
                pose.pose.pose.position.x = 2.5
                odom.publish(pose)
                settle()
                pose.header.stamp.sec = 12
                pose.pose.pose.position.x = 5.0
                odom.publish(pose)
                settle()
                pose.header.stamp.sec = 11
                pose.pose.pose.position.x = 50.0
                odom.publish(pose)
                clouds.publish(cloud('map', 10, 100000000))
                settle()
                assert len(received) == 2, 'Duplicate terrain was processed again'
                clouds.publish(cloud('map', 10, 150000000))
                settle()
                assert len(received) == 2, 'Old terrain replaced the current local map'

                # A newer terrain update must not clear obstacles with an old scan.
                clouds.publish(cloud('/map', 12))
                spin_until(lambda: len(received) == 3 and len(rays) == 3)
                assert not x_values(rays[-1]), 'Stale scan was reused for ray clearing'
                assert x_values(received[-1]), 'Older odometry moved the map crop center'

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
                pose.header.stamp.nanosec = 260000000
                odom.publish(pose)
                settle()
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
                pose.header.stamp.nanosec = 270000000
                odom.publish(pose)
                settle()
                clouds.publish(cloud('map', 5, 270000000))
                spin_until(lambda: len(received) == 7 and len(rays) == 7)
                assert x_values(rays[-1]), 'Scan was not retried after its TF arrived'

                # A delayed terrain cloud is cropped at its interpolated
                # measurement pose, even though the latest odometry moved far.
                pose.header.stamp.nanosec = 310000000
                pose.pose.pose.position.x = 0.0
                odom.publish(pose)
                settle()
                pose.header.stamp.nanosec = 410000000
                pose.pose.pose.position.x = 30.0
                odom.publish(pose)
                settle()
                clouds.publish(cloud('map', 5, 310000000))
                spin_until(lambda: len(received) == 8)
                assert x_values(received[-1]), 'Terrain used the latest pose for cropping'

                # Terrain waits for later odometry and for a delayed stamped TF.
                clouds.publish(cloud('map', 5, 450000000))
                settle(0.15)
                assert len(received) == 8, 'Terrain ran before its later odometry'
                pose.header.stamp.nanosec = 500000000
                odom.publish(pose)
                spin_until(lambda: len(received) == 9)
                clouds.publish(cloud('late_terrain', 5, 500000000))
                settle(0.15)
                assert len(received) == 9, 'Terrain ran before its TF arrived'
                terrain_tf = TransformStamped()
                terrain_tf.header.frame_id = 'map'
                terrain_tf.child_frame_id = 'late_terrain'
                terrain_tf.header.stamp.sec = 5
                terrain_tf.header.stamp.nanosec = 500000000
                terrain_tf.transform.translation.x = 30.0
                terrain_tf.transform.rotation.w = 1.0
                transforms.publish(TFMessage(transforms=[terrain_tf]))
                spin_until(lambda: len(received) == 10)
                assert x_values(received[-1]), 'Terrain was not retried after TF arrived'

                # Odometry itself can wait for stamped TF. Queued terrain at
                # the same time must resume without a new sensor callback.
                pose.header.frame_id = 'late_odom'
                pose.header.stamp.nanosec = 600000000
                pose.pose.pose.position.x = 0.0
                odom.publish(pose)
                clouds.publish(cloud('late_odom', 5, 600000000))
                settle(0.2)
                assert len(received) == 10, 'Terrain ran before odometry TF arrived'
                odom_tf = TransformStamped()
                odom_tf.header.frame_id = 'map'
                odom_tf.child_frame_id = 'late_odom'
                odom_tf.header.stamp.sec = 5
                odom_tf.header.stamp.nanosec = 600000000
                odom_tf.transform.translation.x = 35.0
                odom_tf.transform.rotation.w = 1.0
                transforms.publish(TFMessage(transforms=[odom_tf]))
                spin_until(lambda: len(received) == 11)
                assert any(abs(x - 37.0) < 1e-4 for x in x_values(received[-1])), (
                    'Odometry or terrain did not resume at the stamped TF')

                # TF can arrive after newer odometry. The terrain still uses
                # its exact measurement-time pose from history.
                pose.header.frame_id = 'map'
                pose.header.stamp.nanosec = 700000000
                pose.pose.pose.position.x = 30.0
                odom.publish(pose)
                settle()
                clouds.publish(cloud('late_history_terrain', 5, 700000000))
                settle()
                assert len(received) == 11, 'Terrain ran before its TF arrived'
                pose.header.stamp.nanosec = 950000000
                odom.publish(pose)
                settle()
                assert len(received) == 11, 'Terrain ran without its stamped TF'
                history_tf = TransformStamped()
                history_tf.header.frame_id = 'map'
                history_tf.child_frame_id = 'late_history_terrain'
                history_tf.header.stamp.sec = 5
                history_tf.header.stamp.nanosec = 700000000
                history_tf.transform.translation.x = 30.0
                history_tf.transform.rotation.w = 1.0
                transforms.publish(TFMessage(transforms=[history_tf]))
                spin_until(lambda: len(received) == 12)
                assert x_values(received[-1]), 'Delayed TF discarded terrain with a valid historic pose'

                # A terrain update at the older scan stamp must use that scan
                # and its origin, even after a newer scan has been processed.
                tick.clock.sec = 7
                clock.publish(tick)
                settle()
                pose.header.stamp.sec = 6
                pose.header.stamp.nanosec = 0
                pose.pose.pose.position.x = 0.0
                odom.publish(pose)
                settle()
                scans.publish(cloud('map', 6))
                settle()
                pose.header.stamp.nanosec = 100000000
                pose.pose.pose.position.x = 5.0
                odom.publish(pose)
                settle()
                newer_scan = cloud('map', 6, 100000000)
                newer_scan.data = struct.pack('<ffff', 8.0, 0.0, 0.0, 1.0)
                scans.publish(newer_scan)
                settle()
                before = len(received)
                rays_before = len(rays)
                clouds.publish(cloud('map', 6))
                spin_until(lambda: len(received) == before + 1 and len(rays) == rays_before + 1)
                assert x_values(rays[-1]) and max(x_values(rays[-1])) < 2.0, (
                    'Terrain used the newer scan or scanner origin')

                # Terrain should briefly wait when a compatible scan has
                # arrived but is still waiting for its measurement-time TF.
                pose.header.stamp.sec = 6
                pose.header.stamp.nanosec = 200000000
                pose.pose.pose.position.x = 0.0
                odom.publish(pose)
                settle()
                scans.publish(cloud('paired_sensor', 6, 200000000))
                settle(0.05)
                before = len(received)
                rays_before = len(rays)
                clouds.publish(cloud('map', 6, 200000000))
                settle(0.1)
                assert len(received) == before, 'Terrain ran while its queued scan awaited TF'
                paired_tf = TransformStamped()
                paired_tf.header.frame_id = 'map'
                paired_tf.child_frame_id = 'paired_sensor'
                paired_tf.header.stamp.sec = 6
                paired_tf.header.stamp.nanosec = 200000000
                paired_tf.transform.rotation.w = 1.0
                transforms.publish(TFMessage(transforms=[paired_tf]))
                spin_until(lambda: len(received) == before + 1 and len(rays) == rays_before + 1)
                assert x_values(rays[-1]), 'Late scan TF was not used by the terrain update'
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
