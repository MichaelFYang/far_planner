# Repository Guidelines

## Project Structure & Module Organization

This branch supports ROS 1 Melodic (Ubuntu 18.04) and Noetic (Ubuntu 20.04). Keep ROS 1 APIs and catkin compatibility when porting changes from the ROS 2 branches.

- `src/far_planner/`: visibility-graph planning, mapping, terrain processing, and visualization; implementation in `src/`, headers in `include/far_planner/`.
- `src/graph_decoder/` and `src/boundary_handler/`: graph serialization and custom-boundary processing.
- `src/visibility_graph_msg/msg/`: shared graph and node message definitions.
- `src/goalpoint_rviz_plugin/` and `src/teleop_rviz_plugin/`: Qt5/RViz goal selection and teleoperation, with headers in package-named `include/` subdirectories.
- Package `launch/`, `config/`, and `rviz/` directories contain XML launch files, YAML parameters, and display settings. Planner `data/` contains sample inputs; `img/` contains README media.

## Build, Test, and Development Commands

Source the matching ROS distribution before working, for example `source /opt/ros/noetic/setup.bash`. From the repository root:

```sh
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
catkin_make run_tests
catkin_test_results
roslaunch far_planner far_planner.launch
```

Use `-DCMAKE_BUILD_TYPE=RelWithDebInfo` for debugging. `catkin_make` generates `src/CMakeLists.txt`; keep this machine-specific file untracked. Launch the separate simulator with `roslaunch vehicle_simulator system_indoor.launch` from its sourced workspace.

## Coding Style & Naming Conventions

Use C++14 and preserve surrounding style: core code generally uses four-space indentation; RViz plugins use two spaces. Use snake_case filenames, PascalCase class names, and established member-variable naming. Keep plugin headers under `include/<package>/`. Preserve copyright notices and the MIT license. Avoid unrelated formatting changes.

## Testing Guidelines

No dedicated behavioral tests or coverage threshold are currently configured. Build every affected package and exercise changed planning or control behavior in simulation. Record ROS version, simulator environment, commands, and results. Put new automated tests under the owning package's `test/` directory and register them with catkin.

## Commit & Pull Request Guidelines

Use focused, imperative commit subjects naming the affected component. Describe behavior changes, link relevant issues, and include validation results. Add RViz screenshots for visual changes. Exclude generated `build/`, `devel/`, `install/`, and logs. Develop incremental changes on `refactor/melodic-noetic`; preserve branch-specific compatibility when sharing fixes.
