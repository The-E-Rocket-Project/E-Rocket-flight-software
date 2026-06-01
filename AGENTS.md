# E-Rocket Flight Software

ROS 2 (Jazzy) flight stack for an electric TVC rocket interfacing with PX4 Autopilot via MicroXRCEAgent.

## Quick start

```bash
cd workspace_erocket
source /opt/ros/jazzy/setup.bash
colcon build
source install/local_setup.bash  # or setup.zsh
```

## Launch files (all in `src/erocket/launch/`)

| Mode | Command | Nodes |
|------|---------|-------|
| HW baseline PID | `ros2 launch erocket offboard_computer_baseline.launch.py` | baseline_pid_controller, flight_mode, mission |
| HW generic controller | `ros2 launch erocket offboard_computer_generic.launch.py` | controller_generic, flight_mode, mission |
| SITL baseline | `ros2 launch erocket sitl_simulink_baseline.launch.py` | baseline_pid_controller, mission, mock_flight_mode |
| SITL generic | `ros2 launch erocket sitl_simulink_generic.launch.py` | controller_generic, mission, mock_flight_mode |
| MoCap forwarder | `ros2 launch erocket mocap_forwarder.launch.py` | vrpn_client_node, mocap_forwarder |

**MicroXRCEAgent** runs externally (serial `/dev/ttyACM0` @ 921600 baud), not inside launch files. Start it first.

**SITL overrides** `servo_active=false, motor_active=false` to lock actuators.

Use `mprocs` (configured in `workspace_erocket/mprocs.yaml`) to run everything together.

## Flight state machine

Transition via parameter set on the `/mission` node:
```bash
ros2 param set /mission offboard.flight_mode 3  # LIFT_OFF
ros2 param set /mission offboard.flight_mode 4  # START_MISSION
ros2 param set /mission offboard.flight_mode 5  # LAND
```

## Important architecture

- **Mission node** (`mission.cpp`): 1 Hz slow loop (state machine) + 100 Hz fast loop (trajectory generation). Outputs `SetpointC5` messages.
- **Controller nodes** (`baseline_pid_controller.cpp`, `controller_generic.cpp`): Run at 50 Hz (baseline) / 100 Hz (generic). Decoupled IO/math via `StateAggregator` + `SetpointAggregator`. Outer position PID loop → inner attitude PID loop → `Allocator`.
- **Allocator** (`allocator.hpp`): Maps forces/torques → servo angles + motor PWM. Header-only.
- **Flight Mode node** (`flight_mode.cpp`): PX4 state machine bridge. Publishes MAVLink `VehicleCommand` for mode changes.
- **Frame transforms lib** (`src/lib/frame_transforms.cpp`): ENU↔NED conversion for mocap data.
  - **Note**: `quaternion_to_euler` uses a custom Wikipedia formula (the library version didn't work).
- **System Monitor** (`system_monitor.cpp`): Background health checker for topic frequency/drops.

## Config

All parameters live in `src/erocket/config/offboard.yaml`. PID gains, actuator limits, vehicle mass, thrust curve — everything is there.

Notable: attitude PID I-term is zero for roll & pitch, only active on yaw (`k_i: [0.0, 0.0, 0.5]`).

## Trajectory generation

```bash
cd workspace_erocket/trajectory
python3 csv_to_header_with_transform_to_ned.py <input.csv>
```

Input CSV has unusual axis order (x=up, y=front, z=right). Script reorders to x=front, y=right, z=up. Output is a `.h` file with `Setpoints[][]` array, included at compile time by `mission.cpp`.

## Testing

- Linting via `ament_lint_auto` (set in CMakeLists.txt, `BUILD_TESTING`).
- Test executables: `mock_flight_mode`, `flight_mode_test`, `mocap_forwarder_test`.
- Launch tests: `flight_mode_test.launch.py`, `mocap_forwarder_test.launch.py`.

## Packages structure

| Package | Lang standard | Purpose |
|---------|--------------|---------|
| `src/erocket` | C++14 | Custom flight stack (main package) |
| `src/mocap_interface` | C++17 | VRPN client node (from Pegasus project) |
| `src/px4_msgs` | - | PX4 message definitions (upstream) |
| `src/px4_ros_demos` | - | PX4 ROS 2 demo nodes (upstream) |
| `src/vrpn_vendor` | - | VRPN vendor library |

## Gotchas

- `build/`, `install/`, `log/` are gitignored in workspace, `mprocs.log` and `mav.*` at root.
- `.vscode/settings.json` has a stale `cmake.sourceDirectory` path.
