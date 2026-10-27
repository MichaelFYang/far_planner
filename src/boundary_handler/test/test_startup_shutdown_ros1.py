#!/usr/bin/env python
# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT
"""Cancellation while reading initialization input must not generate a graph."""

import errno
import io
import os
import shutil
import signal
import subprocess
import tempfile
import time
import unittest

import rosgraph
import roslib.packages
import rostest


steady_time = getattr(time, 'monotonic', lambda: os.times()[4])


class StartupShutdownTest(unittest.TestCase):
    def exercise(self, cancel):
        directory = tempfile.mkdtemp(prefix='boundary-startup-shutdown-')
        self.addCleanup(shutil.rmtree, directory)
        samples = os.path.join(roslib.packages.get_pkg_dir('far_planner'), 'data')
        shutil.copyfile(os.path.join(samples, 'boundary.ply'),
                        os.path.join(directory, 'boundary.ply'))
        trajectory = os.path.join(directory, 'trajectory.txt')
        os.mkfifo(trajectory)
        graph = os.path.join(directory, 'result.vgh')
        previous_graph = b'previous graph\n'
        if cancel:
            with open(graph, 'wb') as previous:
                previous.write(previous_graph)
        log_path = os.path.join(directory, 'boundary.log')
        executable = roslib.packages.find_node('boundary_handler', 'boundary_handler')[0]
        node = '/boundary_startup/probe_' + str(os.getpid()) + '_' + str(int(cancel))
        master = rosgraph.Master('/boundary_startup_probe')

        def registered():
            return any(node in nodes for entries in master.getSystemState()
                       for _, nodes in entries)

        def read_log():
            with io.open(log_path, encoding='utf-8') as stream:
                return stream.read()

        with open(log_path, 'w') as log:
            process = subprocess.Popen([
                executable, '__name:=' + node.rsplit('/', 1)[1],
                '__ns:=/boundary_startup', '_folder_path:=' + directory,
                '_graph_file:=result.vgh',
            ], stdout=log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
            writer = None
            try:
                deadline = steady_time() + 10.0
                while steady_time() < deadline:
                    self.assertIsNone(process.poll(), read_log())
                    try:
                        writer = os.open(trajectory, os.O_WRONLY | os.O_NONBLOCK)
                        break
                    except OSError as error:
                        if error.errno != errno.ENXIO:
                            raise
                    time.sleep(0.01)
                self.assertIsNotNone(writer, 'Trajectory reader did not start: ' + read_log())
                if cancel:
                    self.assertTrue(registered(), read_log())
                    process.send_signal(signal.SIGINT)
                    deadline = steady_time() + 5.0
                    while registered() and process.poll() is None:
                        self.assertLess(steady_time(), deadline, read_log())
                        time.sleep(0.01)
                    if process.poll() is None:
                        with open(os.path.join(samples, 'trajectory.txt'), 'rb') as sample:
                            first_pose = sample.readline()
                        self.assertEqual(os.write(writer, first_pose), len(first_pose))
                os.close(writer)
                writer = None
                deadline = steady_time() + 5.0
                while process.poll() is None:
                    self.assertLess(steady_time(), deadline, read_log())
                    time.sleep(0.01)
                output = read_log()
                self.assertEqual(process.returncode, 0 if cancel else 1, output)
                if cancel:
                    with open(graph, 'rb') as saved:
                        self.assertEqual(saved.read(), previous_graph, output)
                    self.assertNotIn('Boundary setup error', output)
                else:
                    self.assertFalse(os.path.exists(graph), output)
                    self.assertIn('Boundary setup error', output)
            finally:
                if writer is not None:
                    os.close(writer)
                if process.poll() is None:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except OSError as error:
                        if error.errno != errno.ESRCH:
                            raise
                    process.wait()

    def test_cancellation_does_not_generate_a_graph(self):
        self.exercise(True)

    def test_invalid_trajectory_still_fails(self):
        self.exercise(False)


if __name__ == '__main__':
    rostest.rosrun('boundary_handler', 'test_startup_shutdown_ros1', StartupShutdownTest)
