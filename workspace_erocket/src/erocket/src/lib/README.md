# `src/lib/` — Frame Transforms Library

A single-file library providing reference frame conversions between ENU (ROS 2 standard) and NED (PX4 standard), plus quaternion/Euler utilities.

## Files

### `frame_transforms.cpp` / `include/erocket/frame_transforms.h`

The header declares the public API; the `.cpp` implements non-trivial functions. Inline helpers (unit conversions, template wrappers) live in the header.

**Key types:**

| Type | Description |
|------|-------------|
| `EulerAngle` | `{roll, pitch, yaw}` — doubles for each axis |
| `StaticTF` enum | `NED_TO_ENU`, `ENU_TO_NED`, `AIRCRAFT_TO_BASELINK`, `BASELINK_TO_AIRCRAFT`, `ECEF_TO_ENU`, `ENU_TO_ECEF` |
| `Covariance3d/6d/9d` | Covariance matrix types matching ROS message layouts |

**Unit conversions** (header inlines):

| Function | |
|----------|--|
| `radians_to_degrees()` / `degrees_to_radians()` | Scalar, `Eigen::Vector3d`, and `EulerAngle` overloads |
| `quaternion_to_euler_radians()` / `quaternion_to_euler_degrees()` | Quaternion `→` Euler (custom Wikipedia formula) |
| `euler_radians_to_quaternion()` / `euler_degrees_to_quaternion()` | Euler `→` quaternion |

> `quaternion_to_euler` uses a custom Wikipedia formula rather than the Eigen `.eulerAngles()` method, which did not work correctly.

**Static frame transforms** (`transform_static_frame`):

| Transform | Operation |
|-----------|-----------|
| `NED_TO_ENU` / `ENU_TO_NED` | Reflection-based: swap X↔Y, negate Z |
| `AIRCRAFT_TO_BASELINK` / `BASELINK_TO_AIRCRAFT` | 180° rotation about X axis |
| `ECEF_TO_ENU` / `ENU_TO_ECEF` | Rotation via geodetic origin lat/lon |

Overloads exist for `Eigen::Vector3d`, `Covariance3d`, `Covariance6d`, `Covariance9d`.

**Dynamic frame transforms** (`transform_frame` — rotate a vector/covariance by a quaternion):

- `transform_frame(vec, q)` — rotates a 3D vector
- `transform_frame(cov, q)` — rotates 3×3, 6×6, or 9×9 covariance

**Convenience templates** (header-only):

| Wrapper | Equivalent to |
|---------|---------------|
| `ned_to_enu_orientation(q)` | `transform_orientation(q, ENU_TO_NED)` |
| `enu_to_ned_local_frame(v)` | `transform_static_frame(v, ENU_TO_NED)` |
| `ros_to_px4_orientation(q)` | `aircraft_to_baselink(enu_to_ned(q))` |
| `px4_to_ros_orientation(q)` | `baselink_to_aircraft(ned_to_enu(q))` |

And matching pairs for `ned_to_enu`, `aircraft_to_baselink`, `baselink_to_aircraft`, `ecef_to_enu`, `enu_to_ecef`.

**`namespace utils::quaternion`** (header implementations):

| Function | Purpose |
|----------|---------|
| `quaternion_from_euler(Vector3d)` / `quaternion_from_euler(r,p,y)` | YPR (ZYX) → quaternion |
| `quaternion_to_euler(q)` / `quaternion_to_euler(q, r, p, y)` | Quaternion → Vector3d or by-reference |
| `eigen_quat_to_array(q, arr)` | Eigen → `std::array<float,4>` (w,x,y,z) |
| `array_to_eigen_quat(arr)` | `std::array<float,4>` (w,x,y,z) → Eigen |
| `quaternion_get_yaw(q)` | Extract yaw from quaternion |

**`namespace utils::types`** (header templates):

| Function | Purpose |
|----------|---------|
| `covariance_to_array(cov, arr)` | Eigen covariance → flat array |
| `covariance_urt_to_array(cov, arr)` | Upper-right triangle → flat array |
| `array_urt_to_covariance_matrix(arr, cov)` | Flat array → full symmetric covariance |

## Usage notes

- The `MocapForwarder` node uses this library to convert VRPN pose data (ENU) to PX4's expected NED frame via `transform_static_frame(pos, ENU_TO_NED)` and `transform_orientation(q, ENU_TO_NED)` chained with `BASELINK_TO_AIRCRAFT`.
- Static constants (`NED_ENU_Q`, `AIRCRAFT_BASELINK_Q`, `NED_ENU_REFLECTION_XY`, etc.) are module-level and initialized once.
