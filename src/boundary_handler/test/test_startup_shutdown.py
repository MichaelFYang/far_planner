# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT
"""Cancel blocked initialization before graph generation; retain input errors."""

import errno
import os
import shutil
import signal
import subprocess
import tempfile
import time

import pytest


@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGTERM, None])
def test_shutdown_during_trajectory_read(stop_signal):
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    environment['ROS_LOCALHOST_ONLY'] = '1'
    samples = os.environ['BOUNDARY_SAMPLE_DATA']
    with tempfile.TemporaryDirectory(prefix='boundary-startup-shutdown-') as directory:
        environment['ROS_LOG_DIR'] = directory
        shutil.copyfile(os.path.join(samples, 'boundary.ply'),
                        os.path.join(directory, 'boundary.ply'))
        trajectory = os.path.join(directory, 'trajectory.txt')
        os.mkfifo(trajectory)
        graph = os.path.join(directory, 'result.vgh')
        previous_graph = b'previous graph\n'
        if stop_signal is not None:
            with open(graph, 'wb') as previous:
                previous.write(previous_graph)
        log_path = os.path.join(directory, 'boundary.log')

        def read_log():
            with open(log_path) as stream:
                return stream.read()

        with open(log_path, 'w') as log:
            process = subprocess.Popen([
                os.environ['BOUNDARY_HANDLER_EXECUTABLE'], '--ros-args',
                '-p', 'folder_path:=' + directory, '-p', 'graph_file:=result.vgh',
            ], env=environment, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            writer = None
            try:
                deadline = time.monotonic() + 10.0
                while time.monotonic() < deadline:
                    assert process.poll() is None, read_log()
                    try:
                        writer = os.open(trajectory, os.O_WRONLY | os.O_NONBLOCK)
                        break
                    except OSError as error:
                        if error.errno != errno.ENXIO:
                            raise
                    time.sleep(0.01)
                assert writer is not None, 'Trajectory reader did not start: ' + read_log()
                if stop_signal is not None:
                    process.send_signal(stop_signal)
                    deadline = time.monotonic() + 3.0
                    while 'signal_handler(' not in read_log():
                        assert process.poll() is None, read_log()
                        assert time.monotonic() < deadline, read_log()
                        time.sleep(0.01)
                    # The handler wakes the SDK's asynchronous shutdown thread.
                    time.sleep(0.05)
                    with open(os.path.join(samples, 'trajectory.txt'), 'rb') as sample:
                        first_pose = sample.readline()
                    assert os.write(writer, first_pose) == len(first_pose)
                os.close(writer)
                writer = None
                return_code = process.wait(timeout=5.0)
                output = read_log()
                assert return_code == (0 if stop_signal is not None else 1), output
                if stop_signal is not None:
                    with open(graph, 'rb') as saved:
                        assert saved.read() == previous_graph, output
                    assert 'Boundary setup error' not in output, output
                    assert 'terminate called' not in output, output
                else:
                    assert not os.path.exists(graph), output
                    assert 'Boundary setup error' in output, output
            finally:
                if writer is not None:
                    os.close(writer)
                if process.poll() is None:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=5.0)


@pytest.mark.parametrize('valid_output', [True, False])
def test_generated_graph_lifecycle(valid_output):
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    environment['ROS_LOCALHOST_ONLY'] = '1'
    samples = os.environ['BOUNDARY_SAMPLE_DATA']
    with tempfile.TemporaryDirectory(prefix='boundary-graph-lifecycle-') as directory:
        environment['ROS_LOG_DIR'] = directory
        for name in ('boundary.ply', 'trajectory.txt'):
            shutil.copyfile(os.path.join(samples, name), os.path.join(directory, name))
        graph_file = 'result.vgh' if valid_output else 'missing/result.vgh'
        graph = os.path.join(directory, graph_file)
        log_path = os.path.join(directory, 'boundary.log')

        def read_log():
            with open(log_path) as stream:
                return stream.read()

        with open(log_path, 'w') as log:
            process = subprocess.Popen([
                os.environ['BOUNDARY_HANDLER_EXECUTABLE'], '--ros-args',
                '-p', 'folder_path:=' + directory, '-p', 'graph_file:=' + graph_file,
            ], env=environment, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            try:
                if valid_output:
                    deadline = time.monotonic() + 10.0
                    while not os.path.exists(graph):
                        assert process.poll() is None, read_log()
                        assert time.monotonic() < deadline, read_log()
                        time.sleep(0.01)
                    with open(graph) as saved:
                        assert len(saved.readlines()) == 25
                    process.send_signal(signal.SIGINT)
                return_code = process.wait(timeout=10.0)
                output = read_log()
                assert return_code == (0 if valid_output else 1), output
                assert 'LeakSanitizer' not in output, output
                assert 'ERROR: AddressSanitizer' not in output, output
                assert 'runtime error:' not in output, output
                if not valid_output:
                    assert not os.path.exists(graph), output
                    assert 'Boundary setup error' in output, output
            finally:
                if process.poll() is None:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=5.0)
