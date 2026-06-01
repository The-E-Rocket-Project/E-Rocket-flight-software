# `erocket` package

ROS 2 flight stack for an electric TVC rocket interfacing with PX4 Autopilot via MicroXRCEAgent.

---
> ## AI USE disclaimer:
> AI chats were used to explain ROS 2 and PX4 concepts, features, and conventions in parallel, adding to the documentation available to read online.
>
>Earlier iterations of this documentation were written by hand, but that approach proved time-consuming and produced inconsistent, unevenly structured and unclear documentation  across the codebase. From this point forward, documentation is drafted with the assistance of Agentic AI (OpenCode). The goal is not to substitute human judgement but to aid in exploring, structuring, and describing the code systematically. 
>
>All AI-generated documentation has been reviewed against the actual source code by a human developer to verify technical accuracy.
---

## Directory layout

| Path | Purpose |
|------|---------|
| `config/` | Single parameter file `offboard.yaml` (PID gains, actuator limits, mass, thrust curve) |
| `include/erocket/` | Shared headers: `StateAggregator`, `SetpointAggregator`, `Allocator`, PID controllers, frame transforms |
| `launch/` | 7 launch files: offboard HW, SITL, mocap, test |
| `msg/` | 6 custom message definitions |
| `src/` | Node implementations (`.cpp` files) |
| `src/lib/` | `frame_transforms.cpp` — ENU↔NED conversion |
| `test/` | Test executables: SITL mock, flight_mode_test, mocap_forwarder_test |
| `CMakeLists.txt` | Build targets and dependencies |
| `package.xml` | Package manifest |

## Build targets

All built with `colcon build` from the workspace root.

### System executables

| Executable | Source | Notes |
|------------|--------|-------|
| `baseline_pid_controller` | `src/baseline_pid_controller.cpp` | 50 Hz outer/inner PID loop |
| `controller_generic` | `src/controller_generic.cpp` | 100 Hz user-defined controller |
| `mission` | `src/mission.cpp` | 1 Hz + 100 Hz dual loop |
| `flight_mode` | `src/flight_mode.cpp` | PX4 state machine bridge |
| `mocap_forwarder` | `src/mocap_forwarder.cpp` | ENU→NED pose transform |

### Test executables

| Executable | Source |
|------------|--------|
| `mock_flight_mode` | `test/sitl/mock_flight_mode.cpp` |
| `flight_mode_test` | `test/px4_ros2_communication/flight_mode_test.cpp` |
| `mocap_forwarder_test` | `test/px4_ros2_communication/mocap_forwarder_test.cpp` |

### Shared library

| Target | Source | Consumed by |
|--------|--------|-------------|
| `frame_transforms` | `src/lib/frame_transforms.cpp` | All controllers + mocap_forwarder + test nodes |

## Custom messages (`msg/`)

| Message | Topic | Key fields |
|---------|-------|------------|
| `FlightMode.msg` | `offboard/flight_mode/get`, `offboard/flight_mode/set` | `uint8 flight_mode` (INIT..ABORT) |
| `SetpointC5.msg` | `offboard/setpoint_c5_meters` | pos/vel/acc/jerk/snap + yaw |
| `AttitudeControllerDebug.msg` | `/offboard/attitude_controller/debug` | Roll/pitch/yaw, setpoints, tau_bar |
| `PositionControllerDebug.msg` | `/offboard/position_controller/debug` | Pos/vel/acc, desired attitude, u3 |
| `AllocatorDebug.msg` | `/offboard/allocator/debug` | M_bar, M_delta, gamma angles, PWMs |
| `GenericControllerDebug.msg` | `generic_controller/debug` | Timestamp stub |

All compiled via `rosidl_generate_interfaces()` in CMakeLists.txt.

## Dependencies

`rclcpp`, `px4_msgs`, `geometry_msgs`, `sensor_msgs`, `Eigen3`, `builtin_interfaces`, `rosidl_default_generators`.

## Tests

test/
├── sitl/mock_flight_mode.cpp           # PX4 state mock for SITL
└── px4_ros2_communication/
    ├── flight_mode_test.cpp            # Flight mode PX4 comm test
    └── mocap_forwarder_test.cpp        # Mocap pipeline test

Linting via `ament_lint_auto` when `BUILD_TESTING` is set.

## See also

- [`src/README.md`](src/README.md) — architecture, data flow, FSM
- [`src/lib/README.md`](src/lib/README.md) — frame transforms
- [`include/erocket/README.md`](include/erocket/README.md) — shared headers
- [`../../Documentation/architecture.md`](../../Documentation/architecture.md)
- [`../../Documentation/launch_guide.md`](../../Documentation/launch_guide.md)

