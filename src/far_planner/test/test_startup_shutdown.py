# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT
"""Cancellation during initialization must exit normally without an executor error."""

import os
import signal
import subprocess
import tempfile
import time

import pytest


@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGTERM])
def test_shutdown_during_initialization(stop_signal):
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    environment['ROS_LOCALHOST_ONLY'] = '1'
    with tempfile.TemporaryDirectory(prefix='far-startup-shutdown-') as directory:
        environment['ROS_LOG_DIR'] = directory
        log_path = os.path.join(directory, 'planner.log')

        def read_log():
            with open(log_path) as log:
                return log.read()

        with open(log_path, 'w') as log:
            process = subprocess.Popen([
                os.environ['FAR_PLANNER_EXECUTABLE'], '--ros-args',
                '-p', 'is_opencv_visual:=false',
                '-p', 'map_handler/map_grid_max_length:=100.0',
                '-p', 'map_handler/map_grad_max_height:=10.0',
            ], env=environment, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            try:
                deadline = time.monotonic() + 10.0
                while time.monotonic() < deadline:
                    output = read_log()
                    assert process.poll() is None, output
                    if 'FAR Planner Processing Objects Initiated' in output:
                        assert 'FAR Planner Initiated Complete' not in output, output
                        break
                    time.sleep(0.01)
                else:
                    pytest.fail('Planner did not reach initialization: ' + output)
                process.send_signal(stop_signal)
                return_code = process.wait(timeout=5.0)
                output = read_log()
                assert return_code == 0, output
                assert 'Startup failed' not in output, output
                assert '[FATAL]' not in output, output
            finally:
                if process.poll() is None:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=5.0)


def test_invalid_startup_parameter_still_fails():
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    environment['ROS_LOCALHOST_ONLY'] = '1'
    with tempfile.TemporaryDirectory(prefix='far-invalid-startup-') as directory:
        environment['ROS_LOG_DIR'] = directory
        result = subprocess.run([
            os.environ['FAR_PLANNER_EXECUTABLE'], '--ros-args',
            '-p', 'main_run_freq:=0.0',
        ], env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            universal_newlines=True, timeout=10.0)
        assert result.returncode == 1, result.stdout
        assert 'Startup failed' in result.stdout, result.stdout
        assert 'main_run_freq' in result.stdout, result.stdout
