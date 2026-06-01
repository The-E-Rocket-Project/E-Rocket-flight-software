# `src/erocket/src/` — Node Implementations

Each `.cpp` file compiles into a ROS 2 executable node. The nodes share a common infrastructure defined in the `include/` headers — `StateAggregator`, `SetpointAggregator`, `Allocator`, PID controllers, frame transform library, and custom messages.

---

## System Architecture

```
Mocap System (VRPN)
  │
  ▼ PoseStamped (ENU)
MocapForwarder ──── transforms to NED ────► VehicleOdometry ──► PX4 EKF
                                                    │
                                                    │  /fmu/out/*
                                                    ▼
                                            StateAggregator
                                                    │
Mission ── SetpointC5 ──► SetpointAggregator ──► Controllers
                                                     │
                                                     ▼
                                              Allocator
                                                     │
                            ┌────────────────────────┤
                            ▼                        ▼
                    ActuatorServos            ActuatorMotors
                            │                        │
                            └────────► PX4 ◄─────────┘
                                        │
                              FlightMode (VehicleCommand)
                                        │
                                        ▼
                                       PX4

SystemMonitor ── subscribes to all critical topics ──► health checks
```

### External interfaces

| Direction | Transport | Topics |
|-----------|-----------|--------|
| PX4 → ROS 2 | MicroXRCEAgent (serial `/dev/ttyACM0`, 921600 baud) | `/fmu/out/vehicle_attitude`, `/fmu/out/vehicle_angular_velocity`, `/fmu/out/vehicle_local_position`, `/fmu/out/vehicle_odometry`, `/fmu/out/actuator_armed` |
| ROS 2 → PX4 | MicroXRCEAgent | `/fmu/in/actuator_motors`, `/fmu/in/actuator_servos`, `/fmu/in/offboard_control_mode`, `/fmu/in/vehicle_control_mode`, `/fmu/in/vehicle_visual_odometry` |
| ROS 2 → PX4 (service) | MicroXRCEAgent | `/fmu/vehicle_command` |
| Mocap → ROS 2 | VRPN (via `mocap_interface` package) | `/mocap/pose_enu/erocket` |
| QGC → PX4 | MAVLink (via `mavproxy.py` on `/dev/ttyAMA10`, 57600 baud) | UDP broadcast `192.168.1.255:14550` |

### Internal data flow

1. **State ingestion**: `StateAggregator` subscribes to four PX2 `/fmu/out/*` topics and derives quaternion → rotation matrix → Euler angles on each attitude callback.
2. **Setpoint ingestion**: `SetpointAggregator` subscribes to three setpoint topics (`SetpointC5`, `Vector3Stamped` attitude, `Vector3Stamped` translation) and presents unified `PositionSetpoint` / `AttitudeSetpoint` structs.
3. **Control loop** (timer-driven, decoupled from network callbacks):
   - Position PID (outer loop): computes desired acceleration from position/velocity error → desired attitude + thrust magnitude `u3`.
   - Attitude PID (inner loop): computes torque vector `tau_bar` from attitude error.
   - Attitude output is converted to thrust vector components and `tau_delta_bar`.
4. **Allocation**: `Allocator` maps thrust vector + differential torque → servo angles + motor PWMs using:
   - Motor thrust curve (`newtons = (pwm·m + b) / 1000 · g`)
   - Delta torque curve (`tau_delta_bar = a·ΔM + b·M_bar + c`)
   - `arcsin`-based servo angle from thrust vector components
5. **PWM output**: Published as `ActuatorMotors` and `ActuatorServos` to PX4.

---

## Nodes

### `baseline_pid_controller.cpp` — `baseline_pid_controller` node

The main flight controller. Runs at **50 Hz** via a wall timer.

**Flight mode behaviour:**

| Mode | Action |
|------|--------|
| `< ARM` | `compute_allocation_neutral()` — no output |
| `ARM` | Sinusoidal servo wiggle test (3 s), then motor ramp (4 s) via `indirect_actuation()` |
| `TAKE_OFF` / `IN_MISSION` / `LANDING` | Position PID → Attitude PID → `compute_allocation()` |
| `ABORT` | Zero allocation, `rclcpp::shutdown()` |

**Subscribes to:** `/fmu/out/*` (via `StateAggregator`), `/offboard/setpoint_c5_meters` (via `SetpointAggregator`), `offboard/flight_mode/get`

**Publishes to:** `/fmu/in/actuator_motors`, `/fmu/in/actuator_servos`, `/offboard/attitude_controller/debug`, `/offboard/position_controller/debug`, `/offboard/allocator/debug`

**Key config** (`offboard.yaml`): attitude gains (`k_p`, `k_d`, `k_i`), position gains (`k_p`, `k_d`, `k_i`, `k_ff`), controller active flags.

### `controller_generic.cpp` — `generic_controller` node

Template node for custom control algorithms. Same architecture as `baseline_pid_controller` but uses `GenericController` (user-implemented) instead of the PID chain.

Runs at configurable frequency (default **100 Hz**, parameter `offboard.controller.generic.frequency_hertz`).

The `GenericController::compute()` stub reads state + setpoint from aggregators and returns an empty `AllocatorInput`. Implement custom logic inside that method.

### `flight_mode.cpp` — `flight_mode` node

Bridges ROS 2 flight mode requests to PX4 via MAVLink `VehicleCommand` service (`/fmu/vehicle_command`). Also maintains the offboard control mode heartbeat at 10 Hz.

**Valid state transitions:**

```
INIT ──► PRE_ARM ──► ARM ──► TAKE_OFF ──► IN_MISSION ──► LANDING ──► MISSION_COMPLETE
  │                                                                                │
  └──── ABORT ◄────────────────────────────────────────────────────────────────────┘
```

| Transition | PX4 action |
|------------|-----------|
| `INIT → PRE_ARM` | Send `VEHICLE_CMD_DO_SET_MODE` (manual → offboard) |
| `PRE_ARM → ARM` | Send `VEHICLE_CMD_COMPONENT_ARM_DISARM` (arm) |
| `ARM → TAKE_OFF` | Local state change only, publish `VehicleControlMode` (armed, offboard) |
| `TAKE_OFF → IN_MISSION` | Local state change only |
| `→ LANDING` | Local state change only |
| `LANDING → MISSION_COMPLETE` | Send disarm command |
| `→ ABORT` | Send disarm + local ABORT, then `rclcpp::shutdown()` on next ABORT message |

State is shared via a custom `FlightMode` message on two topics:
- `offboard/flight_mode/set` — request input (published by `Mission` or by `ros2 param set`)
- `offboard/flight_mode/get` — current state output (consumed by `baseline_pid_controller`, `controller_generic`, `mission`)

**Subscribes to:** `offboard/flight_mode/set`

**Publishes to:** `offboard/flight_mode/get`, `/fmu/in/offboard_control_mode` (heartbeat, 10 Hz), `/fmu/in/vehicle_control_mode`

### `mission.cpp` — `mission` node

The mission orchestrator. Runs two concurrent loops:

**Slow loop (1 Hz)** — state machine transitions:
- `INIT` → requests `PRE_ARM`
- `PRE_ARM` → requests `ARM`
- `ARM` → initialises position setpoint to current position, records ground state
- `IN_MISSION` → log mission elapsed time
- `ABORT` → shutdown

**Fast loop (100 Hz)** — trajectory generation + emergency monitoring:
- Checks `EmergencySwitch` (RC kill switch via PX4 `actuator_armed.manual_lockdown` flag)
- `TAKE_OFF`: cosine smooth climb from ground to `takeoff_climb_height_meters` over `takeoff_climb_duration_seconds`
- `IN_MISSION`: iterates through compile-time `Setpoints[][]` array (generated by `trajectory/csv_to_header_with_transform_to_ned.py`) or publishes static setpoints
- `LANDING`: cosine smooth descent from hover position to ground over `landing_descent_duration_seconds`

**Dynamic parameter control** (via `ros2 param set`):

| Parameter | Effect |
|-----------|--------|
| `offboard.flight_mode 3` | Request `TAKE_OFF` |
| `offboard.flight_mode 4` | Request `IN_MISSION` |
| `offboard.flight_mode 5` | Request `LANDING` |

**Subscribes to:** `/fmu/out/*` (via `StateAggregator`), `offboard/flight_mode/get`, `/fmu/out/actuator_armed`

**Publishes to:** `offboard/flight_mode/set`, `offboard/setpoint_c5_meters`, `offboard/attitude_setpoint_degrees`, `offboard/translation_position_setpoint_meters`

### `mocap_forwarder.cpp` — `mocap_forwarder` node

Converts VRPN motion capture pose data from ENU (ROS 2 standard) to NED (PX4 standard) and publishes as `VehicleOdometry` for the PX4 EKF.

**Transform pipeline:**
```
ENU PoseStamped
  → transform_static_frame(position, ENU_TO_NED)        [X↔Y swap, Z negate]
  → transform_orientation(q, BASELINK_TO_AIRCRAFT)      [180° about X]
  → transform_orientation(q, ENU_TO_NED)                [NED↔ENU quaternion]
  → VehicleOdometry (NED, POSE_FRAME_NED)
```

**Subscribes to:** `/mocap/pose_enu/erocket` (`geometry_msgs/PoseStamped`)

**Publishes to:** `/fmu/in/vehicle_visual_odometry` (PX4), `/offboard/mocap_orientation_enu_debug`, `/offboard/mocap_orientation_ned_debug`

### `system_monitor.cpp` — `system_monitor` node

Background health checker. Subscribes to all critical topics and tracks:

| Metric | Implementation |
|--------|---------------|
| Message frequency | Ring buffer of 20 inter-message times |
| Message latency | Ring buffer of 20 age measurements |
| Dropped messages | Detected via timestamp going backwards |
| Stale topics | Timeout-based (configurable, default 2000 ms) |

**Default frequency expectations:**

| Topic | Expected Hz |
|-------|-------------|
| `/fmu/out/vehicle_attitude` | 100 |
| `/fmu/out/vehicle_angular_velocity` | 100 |
| `/fmu/out/vehicle_local_position` | 50 |
| `/fmu/out/vehicle_odometry` | 50 |
| `/fmu/in/actuator_motors` | 50 |
| `/fmu/in/actuator_servos` | 50 |
| `offboard/flight_mode/get` | 1 |

Reports every 500 ms via `RCLCPP_WARN` / `RCLCPP_DEBUG`.

**Parameters:** `max_message_age_ms`, `min_expected_frequency_hz`, `check_interval_ms`, `stale_topic_timeout_ms`

---

## Flight Mode State Machine

```
            ros2 param set /mission offboard.flight_mode N
            ┌──────────────────────────────────────────┐
            ▼                                          │
   ┌───────┴────────┐                                  │
   │     INIT (0)   │                                  │
   └───────┬────────┘                                  │
           │ (auto at 1 Hz)                             │
           ▼                                            │
   ┌───────┴────────┐                                  │
   │   PRE_ARM (1)  │ ◄── send VEHICLE_CMD_DO_SET_MODE │
   └───────┬────────┘       (offboard mode)            │
           │ (auto at 1 Hz)                             │
           ▼                                            │
   ┌───────┴────────┐                                  │
   │    ARM (2)     │ ◄── send ARM command +            │
   └───────┬────────┘       servo wiggle test (3 s)     │
           │                + motor ramp (4 s)          │
           ▼                                            │
   ┌───────┴────────┐                                  │
   │  TAKE_OFF (3)  │ ◄── cosine climb trajectory      │
   └───────┬────────┘                                   │
           │                                            │
           ▼                                            │
   ┌───────┴────────┐                                  │
   │ IN_MISSION (4) │ ◄── compile-time Setpoints[][]   │
   └───────┬────────┘       or static setpoints         │
           │                                            │
           ▼                                            │
   ┌───────┴────────┐                                  │
   │  LANDING (5)   │ ◄── cosine descent trajectory    │
   └───────┬────────┘                                   │
           │                                            │
           ▼                                            │
   ┌───────┴────────┐                                  │
   │MISSION_COMPLETE│ ◄── send disarm command           │
   │    (6)         │                                   │
   └────────────────┘                                   │
                                                        │
   ┌────────────────────────────────────────────────────┘
   │
   ▼ (at any time via RC kill switch or param set)
   ┌───────┴────────┐
   │   ABORT (255)  │ ◄── disarm + shutdown
   └────────────────┘
```

The state machine is split across two nodes:
- **`mission.cpp`**: 1 Hz loop drives `INIT → PRE_ARM → ARM`. Parameter callback handles `TAKE_OFF`, `IN_MISSION`, `LANDING` transitions.
- **`flight_mode.cpp`**: Handles PX4-side actions (arming, mode changes, disarming) and validates that the transition is legal.
- **`baseline_pid_controller.cpp` / `controller_generic.cpp`**: React to flight mode via `offboard/flight_mode/get` subscription.

---

## See also

- [`src/lib/README.md`](lib/README.md) — frame transforms library
- `include/erocket/` — headers for `StateAggregator`, `SetpointAggregator`, `Allocator`, PID controllers, `VehicleConstants`, `EmergencySwitch`, custom messages
- `msg/` — custom ROS 2 message definitions (`FlightMode`, `SetpointC5`, debug types)
- `config/offboard.yaml` — centralised parameter file
