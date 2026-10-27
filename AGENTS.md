# Repository Guidelines

## Project Structure & Module Organization

This ROS 2 workspace targets Humble (Ubuntu 22.04) and Jazzy (Ubuntu 24.04) on the `humble-jazzy` branch.

- `src/far_planner/`: visibility-graph planning, terrain processing, mapping, and visualization; implementation in `src/`, headers in `include/far_planner/`.
- `src/graph_decoder/` and `src/boundary_handler/`: graph serialization and custom-boundary processing.
- `src/visibility_graph_msg/msg/`: shared `Graph.msg` and `Node.msg` interfaces.
- `src/goalpoint_rviz_plugin/` and `src/teleop_rviz_plugin/`: RViz goal selection and teleoperation.
- Package `launch/` and `config/` directories hold launch scripts and YAML parameters. Planner `data/` contains sample graphs/boundaries; `rviz/` contains display configurations. `img/` holds README media.

## Build, Test, and Development Commands

Source the installed ROS distribution first, for example `source /opt/ros/jazzy/setup.zsh`. Run from the repository root:

```sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.zsh
colcon test
colcon test-result --verbose
ros2 launch far_planner far_planner.launch
```

These commands build all packages, load the workspace, run registered checks, report results, and start the planner with RViz. For Bash, use `setup.bash` instead.

The simulator is supplied separately by the Autonomous Exploration Development Environment referenced in `README.md`. In its sourced workspace, run `ros2 launch vehicle_simulator system_indoor.launch`. Use `config:=matterport` on the planner launch for Matterport settings.

## Coding Style & Naming Conventions

Preserve surrounding style: core planner C++ generally uses four-space indentation; RViz plugins use two spaces. Core packages use C++14; RViz plugins use C++17. Use snake_case filenames and variables, PascalCase class names, and trailing underscores for member fields where established. Match local method naming and brace placement. No repository-wide formatter configuration exists; RViz packages register `ament_lint_auto` checks.

## Testing Guidelines

No dedicated unit-test suite or coverage threshold is configured. Run the colcon checks above and validate planning changes in simulation: set goals, inspect graph/path updates, and exercise affected controls. Report the environment and observed results. For new automated tests, prefer package-local `test/test_<feature>.cpp` files and register them with ament/CMake.

## Commit & Pull Request Guidelines

History uses short subjects such as `update launch` without enforced prefixes. Prefer descriptive, imperative subjects identifying the affected component. Keep changes focused. PRs should explain behavior changes, link relevant issues, list build/test results and ROS distribution, and include RViz screenshots for visual changes. Exclude generated `build/`, `install/`, and `log/` output.
