# Click-to-Go Robot

A small two-wheel-drive robot that builds a map of a room with a lidar and drives to a point you click on that map. An Arduino Nano drives the motors through an L298N, and a laptop runs the mapping and navigation (ROS2 Jazzy, SLAM Toolbox, Nav2). The two sides talk over a custom binary serial protocol.

This was an initial prototype. The end-to-end demo worked on 22 October 2026, and development of this version has stopped.

## Documentation

- [Technical reference](docs/technical-reference.md): repository map, setup from a fresh clone, the tooling (`just`, `uv`, PlatformIO, Docker), hardware and pin tables, the firmware and serial protocol, the ROS2 workspace, calibration, tests and troubleshooting.
- [Prototype report](docs/prototype-report-abridged.md): what was built, what it showed, and the lessons for the next build.

## Repository layout

| Path | Contents |
|---|---|
| `motor_control_system/` | Arduino Nano firmware (PlatformIO) and the Python host package, scripts and hardware tests |
| `ros2_ws/` | ROS2 workspace and Dockerfile: bridge, bringup, safety monitor, lidar driver (submodule) |
| `cad/` | OpenSCAD model of the replacement encoder disk |
| `docs/` | Documentation |
| `Justfile` | Task runner; run `just` to list the recipes |

Clone with `--recurse-submodules` (the lidar driver is a submodule). See the technical reference for the rest.

## License

See [LICENSE](LICENSE). The lidar driver in `ros2_ws/src/ldlidar_sl_ros2` is a separate project under its own license.
