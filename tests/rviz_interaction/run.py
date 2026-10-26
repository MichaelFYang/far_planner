#!/usr/bin/env python3
# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT

"""Run the rendered RViz regression with an isolated display and ROS graph."""

import argparse
import json
import math
import os
from pathlib import Path
import re
import select
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client


def stop(process):
    """Stop the process group, including roscore's children."""
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        process.wait()
        return
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        pass
    # The leader may have exited while a child ignored SIGTERM.
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait()


def failure(output, message):
    path = output / "results.json"
    try:
        report = json.loads(path.read_text())
    except (OSError, ValueError):
        report = {}
    if not isinstance(report, dict):
        report = {}
    report.update(success=False, status="failed", error=message)
    path.write_text(json.dumps(report, indent=2) + "\n")
    print("FAIL: {}\nArtifacts: {}".format(message, output), file=sys.stderr)


def interrupt(_signal, _frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path, help="compiled rviz_interaction executable")
    parser.add_argument("--output", type=Path, help="artifact directory (default: new /tmp directory)")
    parser.add_argument("--display", help="an already running, dedicated test display, e.g. :146")
    parser.add_argument("--ros-domain-id", type=int, default=227,
                        help="private ROS 2 domain, without other nodes (default: 227)")
    parser.add_argument("--timeout", type=float, default=90, help="executable timeout in seconds")
    args = parser.parse_args()
    binary = args.binary.resolve()
    if not binary.is_file() or not os.access(str(binary), os.X_OK):
        parser.error("binary must be an executable file")
    if args.display and not re.fullmatch(r":\d+(?:\.\d+)?", args.display):
        parser.error("--display must name a local, dedicated test display (:N)")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be positive and finite")
    if not 0 <= args.ros_domain_id <= 232:
        parser.error("--ros-domain-id must be between 0 and 232")
    if os.environ.get("ROS_VERSION") not in ("1", "2"):
        parser.error("source ROS and the built FAR workspace first")

    output = (args.output or Path(tempfile.mkdtemp(prefix="far-rviz-interaction-"))).resolve()
    output.mkdir(parents=True, exist_ok=True)
    # Keep Python 3.6 compatibility for ROS 1 Melodic.
    if (output / "results.json").exists():
        (output / "results.json").unlink()
    env = os.environ.copy()
    for variable, directory in (("ROS_HOME", "ros_home"), ("ROS_LOG_DIR", "ros_logs"),
                                ("XDG_CACHE_HOME", "cache"), ("XDG_CONFIG_HOME", "config"),
                                ("XDG_RUNTIME_DIR", "runtime")):
        path = output / directory
        path.mkdir(exist_ok=True)
        env[variable] = str(path)
    (output / "runtime").chmod(0o700)
    env.update(QT_QPA_PLATFORM="xcb", QT_X11_NO_MITSHM="1", LIBGL_ALWAYS_SOFTWARE="1",
               GALLIUM_DRIVER="llvmpipe", LP_NUM_THREADS="2")
    processes, logs = [], []

    def launch(command, log_name, pass_fds=()):
        log = (output / log_name).open("w")
        logs.append(log)
        process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, pass_fds=pass_fds)
        processes.append(process)
        return process

    signal.signal(signal.SIGTERM, interrupt)
    error_message = None
    try:
        if args.display:
            env["DISPLAY"] = args.display
        else:
            xvfb = shutil.which("Xvfb")
            if not xvfb:
                raise RuntimeError("Xvfb is required; install xvfb or pass a dedicated --display")
            read_fd, write_fd = os.pipe()
            try:
                server = launch([xvfb, "-displayfd", str(write_fd), "-screen", "0", "1440x1000x24",
                                 "-nolisten", "tcp", "-ac"], "display.log", (write_fd,))
                os.close(write_fd)
                write_fd = None
                if not select.select([read_fd], [], [], 10)[0]:
                    raise RuntimeError("Xvfb did not report a display within 10 seconds")
                number = os.read(read_fd, 64).decode().strip()
                if not number.isdigit() or server.poll() is not None:
                    raise RuntimeError("Xvfb failed to start; see display.log")
                env["DISPLAY"] = ":" + number
            finally:
                os.close(read_fd)
                if write_fd is not None:
                    os.close(write_fd)

        if env["ROS_VERSION"] == "1":
            if not shutil.which("roscore"):
                raise RuntimeError("roscore is unavailable in the sourced ROS environment")
            with socket.socket() as port:
                port.bind(("127.0.0.1", 0))
                master_port = port.getsockname()[1]
            env["ROS_MASTER_URI"] = "http://127.0.0.1:{}".format(master_port)
            env["ROS_IP"] = "127.0.0.1"
            env.pop("ROS_HOSTNAME", None)
            master = launch(["roscore", "-p", str(master_port)], "roscore.log")
            rpc = xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"])
            deadline = time.monotonic() + 15
            while True:
                if master.poll() is not None:
                    raise RuntimeError("owned roscore exited; see roscore.log")
                try:
                    if rpc.getPid("/far_rviz_interaction_wait")[0] == 1:
                        break
                except OSError:
                    pass
                if time.monotonic() >= deadline:
                    raise RuntimeError("owned roscore did not become ready within 15 seconds")
                time.sleep(0.1)
        else:
            env.update(ROS_DOMAIN_ID=str(args.ros_domain_id), ROS_LOCALHOST_ONLY="1",
                       ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")

        process = launch([str(binary), str(output)], "rviz.log")
        try:
            code = process.wait(timeout=args.timeout)
        except subprocess.TimeoutExpired:
            raise RuntimeError("RViz test exceeded {} seconds".format(args.timeout))
        if code:
            raise RuntimeError("RViz test exited with code {}; see rviz.log".format(code))
        report = json.loads((output / "results.json").read_text())
        if (not isinstance(report, dict) or not report.get("success") or
                report.get("status") != "complete"):
            raise RuntimeError("RViz test did not write a successful final report")
    except (OSError, ValueError, RuntimeError) as error:
        error_message = str(error)
        return_code = 1
    except KeyboardInterrupt:
        error_message = "interrupted"
        return_code = 130
    else:
        return_code = 0
    finally:
        for process in reversed(processes):
            stop(process)
        for log in logs:
            log.close()
    # A child may write its report during shutdown; record runner failures last.
    if error_message is not None:
        failure(output, error_message)
    else:
        print("PASS: rendered RViz interactions\nArtifacts: {}".format(output))
    return return_code


if __name__ == "__main__":
    sys.exit(main())
