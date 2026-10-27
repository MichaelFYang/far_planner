# Rendered RViz regression

This optional test loads the installed Goalpoint and TeleopPanel plugins into a
real RViz render panel and drives Qt mouse events. `COLCON_IGNORE` keeps it out of
normal workspace builds. Build and source FAR first, using the matching branch
and ROS distribution.

Prerequisites are CMake, Python 3, Xvfb (`xvfb` on Ubuntu), software OpenGL, and the
ROS desktop development packages, including Qt Test. Tested SDKs are Humble,
Jazzy, Lyrical (Qt6), Noetic, and Melodic (Qt5).

```sh
# After sourcing ROS and install/setup.bash (devel/setup.bash on ROS 1):
far_ui_source="$(pwd)/tests/rviz_interaction"
mkdir -p /tmp/far-rviz-build
(cd /tmp/far-rviz-build && cmake "$far_ui_source")
cmake --build /tmp/far-rviz-build -- -j2
(cd /tmp/far-rviz-build && ctest --output-on-failure)
```

Alternatively, choose a results directory or a dedicated test display:

```sh
python3 tests/rviz_interaction/run.py /tmp/far-rviz-build/rviz_interaction \
  --output /tmp/far-rviz-results
```

These commands also work with Melodic's CMake 3.10. The standalone runner uses
Python 3.6 or newer and does not import ROS Python bindings.

The runner starts and stops its own Xvfb and, on ROS 1, its own local ROS master.
ROS 2 uses localhost domain 227; keep that domain free of other nodes or choose
another with `--ros-domain-id`. It redirects ROS and Qt state into the artifact
directory without changing `HOME`. `--display :N` reuses an already running,
dedicated test display; do not use your desktop display. This is useful when
binding a test X server's Unix socket into an SDK container.

Checks cover:

- One goal/resume pair per viewport gesture, fixed-frame changes, initial height,
  transformed height, missing-TF rejection/recovery, and invalid-topic recovery.
- A moving transform whose height is 1 m at odometry time and 5 m at the latest
  sample. Odometry at 1.25 m must produce a 2.25 m goal, not 6.25 m.
- On ROS 2, a switch to paused RViz simulation time at 200 seconds updates goal,
  resume, and manual joystick command timestamps.
- Planning/update toggles, graph reset, navigation resume, and read/save dialog
  paths, cancellation, and `.vgh` suffix handling.
- Joystick motion, release-to-stop, idle publication, undersized-widget stopping,
  and finite axes.

`results.json`, `rviz.log`, and screenshots are written under the selected output
directory (CTest uses `/tmp/far-rviz-build/results`). A failed, timed-out, or
incomplete run returns nonzero. File dialogs verify path publication only; the
test does not start the graph decoder, planner, or vehicle simulator. It uses
RViz's rendering and plugin managers in a test window, rather than loading the
stock RViz application layout. Keep planner replay and simulator validation for
navigation changes.
