# VTOL (QuadPlane) Support for ArduPilot Gazebo Simulation

This document describes the VTOL (Vertical Take-Off and Landing) vehicle extension added
to the ArduPilot + Gazebo Harmonic simulation stack. It covers all new files, the launch
interface, architecture differences versus the Iris quadrotor, and known issues.

---

## Table of Contents

1. [Overview](#overview)
2. [Vehicle Comparison](#vehicle-comparison)
3. [Quick Start](#quick-start)
4. [Architecture](#architecture)
5. [New Files Added](#new-files-added)
6. [Modified Files](#modified-files)
7. [Motor and Servo Mapping](#motor-and-servo-mapping)
8. [ROS-Gazebo Bridge Topics](#ros-gazebo-bridge-topics)
9. [Build Instructions](#build-instructions)
10. [Known Issues and Fixes](#known-issues-and-fixes)
11. [Troubleshooting](#troubleshooting)

---

## Overview

The simulation previously supported only the **Iris quadrotor** (ArduCopter). This
extension adds the **Alti Transition QuadPlane VTOL** (ArduPlane + QuadPlane mode), a
hybrid fixed-wing aircraft with four vertical lift rotors and one forward pusher propeller.

The vehicle is selected with a single `vehicle:=` argument at launch time — no code
changes are required to switch between them.

---

## Vehicle Comparison

| Property | Quadrotor (`quadrotor`) | QuadPlane VTOL (`quadplane`) |
|----------|------------------------|------------------------------|
| Model | Iris with front camera | Alti Transition with front camera |
| ArduPilot binary | `arducopter` | `arduplane` |
| Flight controller mode | Copter (GUIDED) | QuadPlane (QLOITER / FBWA) |
| SITL param file | `gazebo-iris.parm` | `gazebo-quadplane.parm` |
| Lift motors | 4 × vertical (SERVO1–4) | 4 × vertical (SERVO5–8) |
| Fixed-wing surfaces | — | Ailerons (SERVO1–2), elevator (SERVO2), rudder (SERVO4) |
| Forward motor | — | 1 × pusher prop (SERVO3, function 70) |
| Gazebo world | `iris_tracking` | `quadplane_tracking` |
| Bridge config | `iris_tracking_bridge.yaml` | `quadplane_tracking_bridge.yaml` |
| ROS TF source | SDF (sdformat_urdf) | Hand-crafted URDF (`quadplane.urdf`) |

---

## Quick Start

### Prerequisites

All packages must be built (see [Build Instructions](#build-instructions)) and the
workspace sourced before launching.

```bash
source ~/ardu_ws/install/setup.bash
```

### Launch the Iris quadrotor (unchanged default)

```bash
ros2 launch drone_nl_control nl_drone.launch.py \
    vehicle:=quadrotor \
    mission:="find the walking person"
```

### Launch the QuadPlane VTOL

```bash
ros2 launch drone_nl_control nl_drone.launch.py \
    vehicle:=quadplane \
    mission:="find the walking person"
```

### Launch without RViz (saves resources)

```bash
ros2 launch drone_nl_control nl_drone.launch.py \
    vehicle:=quadplane \
    rviz:=false \
    mission:="find the military truck"
```

### Available launch arguments

| Argument | Default | Choices | Description |
|----------|---------|---------|-------------|
| `vehicle` | `quadrotor` | `quadrotor`, `quadplane` | Vehicle type to simulate |
| `mission` | _(empty)_ | any string | Natural language mission description |
| `rviz` | `true` | `true`, `false` | Launch RViz alongside the simulation |

---

## Architecture

### Quadrotor stack (unchanged)

```
nl_drone.launch.py  [vehicle:=quadrotor]
        │
        └── iris_tracking.launch.py
                ├── iris_cam.launch.py
                │       ├── arducopter SITL  (port 2019, JSON FDM → Gazebo port 9002)
                │       ├── robot_state_publisher  (SDF → URDF via sdformat_urdf)
                │       └── parameter_bridge  (iris_tracking_bridge.yaml)
                ├── gz_sim_server  (world: iris_tracking.sdf)
                └── gz_sim_gui
```

### QuadPlane VTOL stack (new)

```
nl_drone.launch.py  [vehicle:=quadplane]
        │
        └── quadplane_tracking.launch.py
                ├── robots/quadplane.launch.py
                │       ├── arduplane SITL   (port 2019, JSON FDM → Gazebo port 9002)
                │       ├── robot_state_publisher  (quadplane.urdf — hand-crafted)
                │       └── parameter_bridge  (quadplane_tracking_bridge.yaml)
                ├── gz_sim_server  (world: quadplane_tracking.sdf)
                └── gz_sim_gui
```

Both stacks expose the same `/front_camera/image` and `/imu` topics, so the
`mission_controller` node works identically with either vehicle.

---

## New Files Added

### 1. Gazebo model wrapper

**`src/ardupilot_gazebo/models/alti_transition_quad_with_front_cam/`**

| File | Description |
|------|-------------|
| `model.config` | Model metadata; declares SDF 1.9 and dependency on `alti_transition_quad` |
| `model.sdf` | SDF 1.9 wrapper — merges `model://alti_transition_quad` with `<include merge="true">` and adds a 640×480 front camera at the nose (`0.65 0 −0.05`) |

The base `alti_transition_quad` model is provided by the `ardupilot_sitl_models` package
and is resolved automatically via `GZ_SIM_RESOURCE_PATH` (set by the workspace DSV hooks).

---

### 2. Gazebo world

**`src/ardupilot_gz/ardupilot_gz_gazebo/worlds/quadplane_tracking.sdf`**

A copy of `iris_tracking.sdf` with:
- World name changed to `quadplane_tracking`
- Iris model replaced with `alti_transition_quad_with_front_cam` named `quadplane`
- Spawn pose `0 0 0.3` (no yaw rotation — VTOL faces forward along +X)
- Camera PIP topic updated to reference the `quadplane` model name

---

### 3. SITL parameter file

**`src/ardupilot_gazebo/config/gazebo-quadplane.parm`**

Minimal ArduPlane + QuadPlane parameter set loaded by SITL on startup.
Key parameters:

```
Q_ENABLE         1    # Enable QuadPlane mode
Q_FRAME_CLASS    1    # Quadrotor frame class
Q_FRAME_TYPE     3    # X frame type
SERVO1_FUNCTION  4    # Aileron
SERVO2_FUNCTION  19   # Elevator
SERVO3_FUNCTION  70   # Forward throttle (pusher motor)
SERVO4_FUNCTION  21   # Rudder
SERVO5_FUNCTION  33   # Motor 1 — Front-Right CW
SERVO6_FUNCTION  34   # Motor 2 — Rear-Left  CW
SERVO7_FUNCTION  35   # Motor 3 — Front-Left CCW
SERVO8_FUNCTION  36   # Motor 4 — Rear-Right CCW
```

---

### 4. ROS-Gazebo bridge configuration

**`src/ardupilot_gz/ardupilot_gz_bringup/config/quadplane_tracking_bridge.yaml`**

Maps 9 Gazebo topics to their ROS 2 equivalents for the `quadplane` model in the
`quadplane_tracking` world. See [ROS-Gazebo Bridge Topics](#ros-gazebo-bridge-topics)
for the full list.

---

### 5. Robot launch file

**`src/ardupilot_gz/ardupilot_gz_bringup/launch/robots/quadplane.launch.py`**

Starts the three per-robot components:

- **ArduPlane SITL** — uses `"command": "arduplane"` (not `arducopter`) and loads
  `gazebo-quadplane.parm` + `dds_udp.parm`
- **robot_state_publisher** — loads `urdf/quadplane.urdf` (see
  [Known Issues](#known-issues-and-fixes))
- **parameter_bridge** — reads `quadplane_tracking_bridge.yaml`

---

### 6. Scenario launch file

**`src/ardupilot_gz/ardupilot_gz_bringup/launch/quadplane_tracking.launch.py`**

Top-level launch file for the QuadPlane scenario. Includes:

- `robots/quadplane.launch.py`
- `gz_sim_server` with `quadplane_tracking.sdf`
- `gz_sim_gui`
- Optional RViz (controlled by `rviz:=` argument)

---

### 7. Minimal hand-crafted URDF

**`src/ardupilot_gazebo/urdf/quadplane.urdf`**

Used by `robot_state_publisher` for TF publishing. Describes the same kinematic
structure as the SDF (all motors, ailerons, elevator, IMU, front camera) but replaces
the capsule fuselage collision with a box so that `sdformat_urdf` does not crash.
See [Known Issues — Capsule geometry](#1-sdformat_urdf-crashes-on-capsule-geometry).

---

## Modified Files

### `src/drone_nl_control/launch/nl_drone.launch.py`

Added `vehicle` launch argument with `LaunchConfigurationEquals` conditions:

```python
from launch.conditions import LaunchConfigurationEquals

vehicle_arg = DeclareLaunchArgument(
    'vehicle',
    default_value='quadrotor',
    choices=['quadrotor', 'quadplane'],
    description='Vehicle type to simulate.',
)

iris_tracking_launch = IncludeLaunchDescription(
    ...,
    condition=LaunchConfigurationEquals('vehicle', 'quadrotor'),
)

quadplane_tracking_launch = IncludeLaunchDescription(
    ...,
    condition=LaunchConfigurationEquals('vehicle', 'quadplane'),
)
```

Only the selected simulation is started; the other is skipped entirely.

---

### `src/ardupilot_gazebo/CMakeLists.txt`

Added install rule for the new `urdf/` directory:

```cmake
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)
```

---

## Motor and Servo Mapping

The Alti Transition QuadPlane uses the following servo/motor layout:

```
                  FRONT
          Motor 3 (CCW)  Motor 1 (CW)
               [FR-Left]  [FR-Right]
                   \          /
                    \        /
               ------[body]------
                    /        \
                   /          \
          Motor 4 (CCW)  Motor 2 (CW)
              [RR-Right]  [RR-Left]

          Pusher motor (forward): Motor F (rear center)
```

| SERVO | Function ID | Role |
|-------|------------|------|
| SERVO1 | 4 | Aileron |
| SERVO2 | 19 | Elevator |
| SERVO3 | 70 | Forward throttle (pusher prop) |
| SERVO4 | 21 | Rudder |
| SERVO5 | 33 | Lift Motor 1 — Front-Right CW |
| SERVO6 | 34 | Lift Motor 2 — Rear-Left CW |
| SERVO7 | 35 | Lift Motor 3 — Front-Left CCW |
| SERVO8 | 36 | Lift Motor 4 — Rear-Right CCW |

---

## ROS-Gazebo Bridge Topics

The `quadplane_tracking_bridge.yaml` bridges the following topics:

| ROS 2 Topic | Gazebo Topic | Message Type | Direction |
|-------------|-------------|--------------|-----------|
| `/clock` | `/clock` | `rosgraph_msgs/Clock` | GZ → ROS |
| `/joint_states` | `/world/quadplane_tracking/model/quadplane/joint_state` | `sensor_msgs/JointState` | GZ → ROS |
| `/odom` | `/model/quadplane/odometry` | `nav_msgs/Odometry` | GZ → ROS |
| `/gz/tf` | `/world/quadplane_tracking/pose/info` | `tf2_msgs/TFMessage` | GZ → ROS |
| `/gz/tf_static` | `/world/quadplane_tracking/pose/info` | `tf2_msgs/TFMessage` | GZ → ROS |
| `/front_camera/image` | `/world/quadplane_tracking/model/quadplane/link/front_camera_link/sensor/front_camera/image` | `sensor_msgs/Image` | GZ → ROS |
| `/front_camera/camera_info` | `/world/quadplane_tracking/model/quadplane/link/front_camera_link/sensor/front_camera/camera_info` | `sensor_msgs/CameraInfo` | GZ → ROS |
| `/imu` | `/world/quadplane_tracking/model/quadplane/link/imu_link/sensor/imu_sensor/imu` | `sensor_msgs/Imu` | GZ → ROS |

---

## Build Instructions

After adding the new packages or modifying existing files, rebuild the affected packages:

```bash
cd ~/ardu_ws
source /opt/ros/humble/setup.bash

# Build packages that changed
colcon build --packages-select \
    ardupilot_gazebo \
    ardupilot_gz_bringup \
    ardupilot_gz_gazebo \
    drone_nl_control

source install/setup.bash
```

> **Note:** Do **not** use `--symlink-install` with `drone_nl_control`. It uses a
> `setup.py`-based `ament_python` build that does not support editable installs.
> If you see `option --editable not recognized`, downgrade setuptools:
> ```bash
> pip install "setuptools<70"
> ```

---

## Known Issues and Fixes

### 1. `sdformat_urdf` crashes on capsule geometry

**Symptom:**

```
[robot_state_publisher] Unknown geometry shape: capsule
[robot_state_publisher] Failed to convert geometry on collision [fuselage_collision]
```

**Root cause:** The `alti_transition_quad` SDF model uses a `<capsule>` collision on the
fuselage. The `sdformat_urdf` plugin only supports the basic URDF geometry types (box,
cylinder, sphere, mesh) and exits with an error when it encounters `capsule`.

**Fix (already applied):** `quadplane.launch.py` loads
`ardupilot_gazebo/urdf/quadplane.urdf` — a hand-crafted URDF that describes the same
kinematic tree but substitutes a `<box>` for the capsule. The SDF is still used by
Gazebo for physics; the URDF is only used by `robot_state_publisher` for TF publishing.

---

### 2. ArduPlane SITL crashes once on first cold boot

**Symptom:** MAVProxy reports `[Errno 111] Connection refused` immediately after launch.
ArduPlane exits with code 1 and then restarts automatically.

**Root cause:** ArduPlane starts before Gazebo's JSON FDM socket (port 9002) is ready,
crashes, then restarts. On the second attempt Gazebo is ready and everything proceeds
normally.

**Workaround:** Wait ~30–60 s after launching. The process restarts automatically.

---

### 3. `GZ_SIM_RESOURCE_PATH` must be set before launching

**Symptom:** Gazebo fails to spawn the model with `Unable to find file with URI [model://alti_transition_quad]`.

**Root cause:** `model://` URIs are resolved via `GZ_SIM_RESOURCE_PATH`. Both
`ardupilot_gazebo` and `ardupilot_sitl_models` register their model directories via
ament environment hooks (`.dsv.in` / `.sh.in` files). These hooks run automatically when
the workspace is sourced with `source install/setup.bash`.

**Fix:** Always source the workspace before launching:

```bash
source ~/ardu_ws/install/setup.bash
```

Do **not** source only `/opt/ros/humble/setup.bash` — that will not set `GZ_SIM_RESOURCE_PATH`.

---

### 4. Wrong SITL binary selected

**Symptom:** After switching to `vehicle:=quadplane`, the terminal shows
`Starting sketch 'ArduCopter'` instead of `'ArduPlane'`.

**Root cause:** A stale build or stale `install/` directory from a previous build that
included incorrect defaults.

**Fix:** Rebuild and re-source:

```bash
colcon build --packages-select ardupilot_gz_bringup
source install/setup.bash
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `Unable to find file with URI [model://alti_transition_quad]` | `GZ_SIM_RESOURCE_PATH` not set | `source ~/ardu_ws/install/setup.bash` (not just ROS setup) |
| `Unknown geometry shape: capsule` crash in robot_state_publisher | sdformat_urdf capsule limitation | Rebuild `ardupilot_gazebo`; ensure `quadplane.launch.py` loads the URDF, not the SDF |
| `Starting sketch 'ArduCopter'` when using `vehicle:=quadplane` | Stale build cache | `colcon build --packages-select ardupilot_gz_bringup && source install/setup.bash` |
| Gazebo world loads but no model appears | SITL crashed before Gazebo spawned it | Wait ~60 s for SITL to restart automatically |
| `/front_camera/image` topic missing | Bridge not started or world name mismatch | Check bridge yaml uses world name `quadplane_tracking` and model name `quadplane` |
| `PackageNotFoundError: ardupilot_gz_bringup` | Package not built | `colcon build --packages-select ardupilot_gz_bringup` |
| ArduPlane keeps crashing (not the one-time cold-start crash) | Wrong parameter file or Q_ENABLE not set | Verify `gazebo-quadplane.parm` contains `Q_ENABLE 1` and is being passed to SITL |
| Mission controller reports `waiting for /front_camera/image` indefinitely | Bridge topic path wrong | Confirm world and model names in `quadplane_tracking_bridge.yaml` |
