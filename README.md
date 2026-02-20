# ArduPilot Iris Tracking Simulation

A Gazebo Harmonic simulation environment for an ArduPilot Iris quadcopter with front camera, designed for vehicle/person tracking scenarios. Features a realistic outdoor scene with grass terrain, roads, trees, vehicles, a walking person, and a picture-in-picture (PIP) camera feed in the Gazebo GUI.

## Prerequisites

| Component | Version |
|-----------|---------|
| ROS 2 | Humble |
| Gazebo | Harmonic (gz-sim 8.x) |
| ArduPilot SITL | With DDS support |
| OS | Ubuntu 22.04 |

### Install ROS 2 Humble

Follow the official guide: https://docs.ros.org/en/humble/Installation.html

### Install Gazebo Harmonic

```bash
export GZ_VERSION=harmonic
```

Follow: https://gazebosim.org/docs/harmonic/install_ubuntu

### Install ArduPilot SITL Build Dependencies

Follow the `Installing Build Dependencies` section:
https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_DDS#installing-build-dependencies

## Workspace Setup

### 1. Create the workspace

```bash
mkdir -p ~/ardu_ws/src
cd ~/ardu_ws
```

### 2. Clone the source repositories

```bash
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src
```

This pulls in the following packages:

| Package | Description |
|---------|-------------|
| `ardupilot_gz` | Main meta-package (bringup, gazebo worlds, description, application) |
| `ardupilot_gazebo` | Gazebo models and plugin configs (iris, gimbal, vehicles) |
| `ardupilot_sitl` | ArduPilot SITL launch integration |
| `ardupilot_msgs` | Custom ROS 2 message types |
| `ros_gz` | ROS-Gazebo bridge |
| `micro_ros_agent` | DDS agent for ArduPilot communication |
| `sdformat_urdf` | SDF to URDF conversion for robot_state_publisher |
| `gps_umd` | GPS message dependencies |

### 3. Install ROS dependencies

```bash
cd ~/ardu_ws
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y
```

### 4. Build

```bash
cd ~/ardu_ws
export GZ_VERSION=harmonic
colcon build
```

To rebuild only the world/model packages after editing SDF files:

```bash
colcon build --packages-select ardupilot_gazebo ardupilot_gz_gazebo
```

## Running the Simulation

### 1. Source the workspace

```bash
cd ~/ardu_ws
source install/setup.bash
```

### 2. Launch the iris tracking world

```bash
ros2 launch ardupilot_gz_bringup iris_tracking.launch.py rviz:=false
```

This single command starts:
- **Gazebo server** (`gz sim -s -r`) with the `iris_tracking` world
- **Gazebo GUI** (`gz sim -g`) with 3D viewport and PIP camera overlay
- **ArduPilot SITL** (Copter) with DDS over UDP
- **MAVProxy** GCS on `tcp:127.0.0.1:5760`
- **micro_ros_agent** for DDS-ROS 2 bridging
- **ros_gz_bridge** for Gazebo-to-ROS topic bridging
- **robot_state_publisher** for TF

To also open RViz:

```bash
ros2 launch ardupilot_gz_bringup iris_tracking.launch.py rviz:=true
```

### 3. Connect a GCS (optional)

```bash
mavproxy.py --master udp:127.0.0.1:14550 --console --map
```

Or connect QGroundControl / Mission Planner to `udp:127.0.0.1:14550`.

## World Description

### Scene Layout

The `iris_tracking.sdf` world contains a 100m x 100m outdoor environment:

```
         (-25,5)P1                    (35,10)P3
              |                           |
   (-20,-15)O1    Drone(0,0)    (25,20)O2
              |       |              |
   (-15,25)O3  ======Road1(40m)=====(15,10)Prius
              |       |              |
              |  (8,5)Person    (20,-5)Hatchback
              |       |              |
   (-10,15)Humvee  ===Road2(30m)=====
              |                      |
         (10,-25)P2             (30,-20)O4
```

### Models

| Model | Type | Position | Source |
|-------|------|----------|--------|
| `iris` | Quadcopter (drone) | `0, 0, 0.195` | Custom (`iris_with_front_cam`) |
| `prius` | Civilian sedan | `15, 10, 0` | Gazebo Fuel (OpenRobotics) |
| `hatchback` | Hatchback car | `20, -5, 0` | Gazebo Fuel (OpenRobotics) |
| `military_humvee` | Military vehicle | `-10, 15, 0` | Custom (box geometry) |
| `walking_person` | Pedestrian | `8, 5, 0` | Gazebo Fuel (OpenRobotics) |
| `oak_1` to `oak_4` | Oak trees | Perimeter | Gazebo Fuel (OpenRobotics) |
| `pine_1` to `pine_3` | Pine trees | Perimeter | Gazebo Fuel (OpenRobotics) |

### Environment Features

- **Ground plane**: Green grass-colored (`RGB 0.2, 0.45, 0.1`)
- **Road 1**: 40m x 3m dark asphalt along X-axis (main road past vehicles)
- **Road 2**: 3m x 30m dark asphalt along Y-axis at x=15 (T-intersection)
- **Sky**: Enabled with directional sun casting shadows
- **Coordinates**: Canberra, Australia (`-35.363, 149.165`) — ArduPilot default

### Drone: `iris_with_front_cam`

The Iris drone model extends `iris_with_gimbal` with an additional fixed front-facing camera:

| Property | Value |
|----------|-------|
| Camera resolution | 640 x 480 |
| FOV | 1.39 rad (~80 deg) |
| Update rate | 15 Hz |
| Mount | Fixed to `base_link` facing forward |
| Clip range | 0.1m to 1000m |

### GUI: Picture-in-Picture Camera

The Gazebo GUI includes a floating **Front Camera** PIP window (350x270 px) anchored to the top-right corner of the 3D viewport. It displays the live feed from the drone's front camera sensor.

The GUI also includes play/pause/step controls (bottom-left) and simulation stats (bottom-right).

## ROS 2 Topics

The `ros_gz_bridge` exposes the following topics:

### Camera Topics

| ROS 2 Topic | Type | Description |
|-------------|------|-------------|
| `/front_camera/image` | `sensor_msgs/msg/Image` | Front camera RGB image (640x480 @ 15Hz) |
| `/front_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Front camera intrinsics |
| `/gimbal_camera/image` | `sensor_msgs/msg/Image` | Gimbal camera RGB image |
| `/gimbal_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Gimbal camera intrinsics |

### Sensor Topics

| ROS 2 Topic | Type | Description |
|-------------|------|-------------|
| `/imu` | `sensor_msgs/msg/Imu` | IMU data |
| `/navsat` | `sensor_msgs/msg/NavSatFix` | GPS fix |
| `/air_pressure` | `sensor_msgs/msg/FluidPressure` | Barometric pressure |
| `/magnetometer` | `sensor_msgs/msg/MagneticField` | Magnetometer data |
| `/battery` | `sensor_msgs/msg/BatteryState` | Battery state |

### Navigation Topics

| ROS 2 Topic | Type | Description |
|-------------|------|-------------|
| `/odometry` | `nav_msgs/msg/Odometry` | Vehicle odometry |
| `/joint_states` | `sensor_msgs/msg/JointState` | Joint states |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation clock |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Transform frames |

### ArduPilot DDS Topics

| ROS 2 Topic | Description |
|-------------|-------------|
| `/ap/battery/battery0` | Battery via DDS |
| `/ap/clock` | ArduPilot clock |
| `/ap/navsat/navsat0` | GPS via DDS |
| `/ap/tf_static` | Static transforms via DDS |

### Verify topics are publishing

```bash
# Check front camera is streaming
ros2 topic hz /front_camera/image

# List all active topics
ros2 topic list

# View camera feed in rqt
ros2 run rqt_image_view rqt_image_view /front_camera/image
```

## ByteTrack + YOLO11m Target Tracker

An interactive object tracker that uses **YOLO11m** for detection and **ByteTrack** for multi-object tracking on the drone's front camera feed. The node automatically arms the drone, takes off to hover altitude, then opens an OpenCV window showing live detections.

### Python Dependencies

```bash
pip3 install ultralytics pymavlink opencv-python-headless
```

The YOLO11m model weights (`yolo11m.pt`, ~39 MB) are downloaded automatically on first run.

### Running the Tracker

1. **Launch the simulation** (if not already running):

```bash
cd ~/ardu_ws
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_tracking.launch.py rviz:=false
```

2. **Unpause the simulation** — click Play in Gazebo GUI or run:

```bash
gz service -s /world/iris_tracking/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'pause: false'
```

3. **Run the tracker** (in a new terminal):

```bash
cd ~/ardu_ws
source install/setup.bash
ros2 run ardupilot_gz_application byte_tracker
```

The node will:
- Connect to ArduPilot via MAVLink (`udpin:127.0.0.1:14550`)
- Switch to GUIDED mode, arm, and take off to 8m
- Open the **ByteTracker** OpenCV window with live detections

### Controls

| Key / Action | Effect |
|-------------|--------|
| **Left click** on a bounding box | Select that object as the tracking target (turns green) |
| **C** | Clear the selected target |
| **Q** | Quit the tracker |

### Repositioning the Drone

The front camera faces forward, so the drone needs to be positioned near and facing the objects. You can reposition via pymavlink:

```python
# Example: fly to (10, 3) at 5m altitude and yaw east (90 deg)
from pymavlink import mavutil
conn = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
conn.wait_heartbeat()

# Move to position (NED frame: north, east, down)
conn.mav.set_position_target_local_ned_send(
    0, conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    0b0000111111111000,
    10, 3, -5,  # 10m north, 3m east, 5m up
    0, 0, 0, 0, 0, 0, 0, 0,
)

# Yaw to face east
conn.mav.command_long_send(
    conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
    90, 25, 1, 0, 0, 0, 0,
)
```

### Detection Notes

- YOLO11m is trained on real-world COCO data; detection performance on Gazebo's synthetic models varies
- The Fuel models (Prius, Hatchback, Walking person) are detected more reliably than simple box-geometry models
- The confidence threshold is set to `0.10` to maximize detections in simulation
- The bottom of the OpenCV window shows the current detection count and frame number

## Active Tracker (Centering Mode)

Steers the drone to keep the selected target **always centered** in the camera frame. The drone adjusts yaw, altitude, and forward/backward distance in real time.

```bash
ros2 run ardupilot_gz_application active_tracker
```

### How It Works

The control loop runs at 10 Hz and computes three velocity components:

| Axis | Camera Error | Drone Action |
|------|-------------|--------------|
| **Yaw rate** | Target left/right of center | Rotate to center horizontally |
| **Vertical (vz)** | Target above/below center | Climb/descend to center vertically |
| **Forward (vx)** | Bounding box too small/large | Move forward/backward to maintain distance |

The target bounding box area is maintained at ~4% of the image. A dead zone of 5% prevents jitter when nearly centered.

### Controls

| Key / Action | Effect |
|-------------|--------|
| **Left click** on a bounding box | Select target — drone begins centering |
| **C** | Clear target — drone stops and hovers |
| **Q** | Quit (drone stops safely) |

### HUD Elements

- White crosshair at image center
- Green bounding box and line from crosshair to tracked target
- "TRACKING ID:N" status banner (shows "LOST — holding" if target disappears)
- Steering telemetry logged to terminal: `ex`, `ey`, `area`, `vx`, `vz`, `yaw_r`

### Tuning Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `KP_YAW` | 0.6 | Yaw rate gain (rad/s per normalised error) |
| `KP_ALT` | 1.2 | Vertical speed gain (m/s per normalised error) |
| `KP_FWD` | 0.8 | Forward speed gain (m/s per normalised area error) |
| `TARGET_AREA_FRAC` | 0.04 | Desired bbox area as fraction of image |
| `DEADZONE` | 0.05 | Normalised error below which no correction is sent |

## Pursuit Tracker (Chase-to-Impact Mode)

Aggressively pursues the selected target — centers it, then **charges forward at increasing speed** until the bounding box fills the frame (impact).

```bash
ros2 run ardupilot_gz_application pursuit_tracker
```

### Pursuit State Machine

```
IDLE ──click──> CENTERING ──centered──> CHARGING ──bbox≥35%──> IMPACT
  ^                                                              |
  └──────────────── 3 sec cooldown ─────────────────────────────┘
```

| State | Behavior | Forward Speed |
|-------|----------|---------------|
| **IDLE** | Hovering, no target selected | 0 |
| **CENTERING** | Adjusting yaw/altitude to center target | 0.5 m/s (slow) |
| **CHARGING** | Target centered — accelerating toward it | 0.5 – 4.0 m/s |
| **IMPACT** | Bounding box filled 35%+ of frame — target reached | 0 (stops) |

During **CHARGING**, speed scales with how tightly centered the target is — the more centered, the faster the drone flies. Speed also increases as the drone gets closer (larger bounding box).

### Controls

| Key / Action | Effect |
|-------------|--------|
| **Left click** on a bounding box | Begin pursuit |
| **C** | Abort pursuit — drone stops and hovers |
| **Q** | Quit (drone stops safely) |

### HUD Elements

- Black banner at top showing pursuit state with color coding:
  - Cyan: CENTERING
  - Green: CHARGING
  - Red: IMPACT
- Closing progress bar at bottom (fills as drone approaches target)
- Percentage readout showing how close to impact threshold

### Tuning Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `PURSUIT_MIN_VX` | 0.5 | Minimum forward speed during pursuit (m/s) |
| `PURSUIT_MAX_VX` | 4.0 | Maximum forward speed (m/s) |
| `CENTERING_THRESH` | 0.15 | Max normalised error to transition to CHARGING |
| `IMPACT_AREA_FRAC` | 0.35 | Bbox area fraction that triggers IMPACT |

## Tracker Summary

| Node | Command | Behavior |
|------|---------|----------|
| `byte_tracker` | `ros2 run ardupilot_gz_application byte_tracker` | Passive — detect and select (no steering) |
| `active_tracker` | `ros2 run ardupilot_gz_application active_tracker` | Centers target, maintains distance |
| `pursuit_tracker` | `ros2 run ardupilot_gz_application pursuit_tracker` | Centers then charges until impact |

All three nodes share the same launch prerequisites (simulation running + unpaused) and mouse/keyboard controls.

## File Structure

```
ardu_ws/src/
├── ardupilot_gz/
│   ├── ardupilot_gz_bringup/
│   │   ├── launch/
│   │   │   ├── iris_tracking.launch.py       # Main launch file
│   │   │   └── robots/
│   │   │       └── iris_cam.launch.py         # Robot-specific launch (SITL + bridge)
│   │   └── config/
│   │       └── iris_tracking_bridge.yaml      # Gazebo-ROS bridge topic config
│   ├── ardupilot_gz_gazebo/
│   │   └── worlds/
│   │       └── iris_tracking.sdf              # Primary world file (used by launch)
│   ├── ardupilot_gz_application/
│   │   ├── drone_tracker.py                   # Color-based gimbal tracker
│   │   ├── byte_tracker.py                    # YOLO11m + ByteTrack interactive tracker
│   │   ├── active_tracker.py                  # Centering tracker (keeps target centered)
│   │   └── pursuit_tracker.py                 # Pursuit tracker (chase-to-impact)
│   └── ardupilot_gz_description/
│
├── ardupilot_gazebo/
│   ├── worlds/
│   │   └── iris_tracking.sdf                  # Sync copy of world file
│   └── models/
│       ├── iris_with_front_cam/               # Drone model with front camera
│       │   └── model.sdf
│       ├── iris_with_gimbal/                  # Base iris gimbal model
│       ├── military_vehicle/                  # Custom military vehicle
│       │   └── model.sdf
│       └── ...                                # Other models (zephyr, runway, etc.)
│
├── ardupilot_sitl/                            # SITL launch integration
├── micro_ros_agent/                           # DDS agent
└── ros_gz/                                    # ROS-Gazebo bridge
```

## Troubleshooting

### Models not loading (Fuel download)

On first launch, Gazebo downloads models from Fuel (Oak tree, Pine Tree, Prius, Hatchback, Walking person). This requires internet access and may take a few minutes. Models are cached in `~/.gz/fuel/`.

### Simulation starts paused

The world starts paused by default. Click the **Play** button in the bottom-left of the Gazebo GUI or press `Space` to start the simulation.

### EKF not initializing

Wait for the MAVProxy output to show:
```
EKF3 IMU0 initialised
EKF3 IMU1 initialised
GPS 1: detected u-blox
```
This typically takes 5-10 seconds after simulation starts running.

### PIP window not showing image

The front camera only publishes when the simulation is running (unpaused). Press Play first, then check the PIP window updates.

### Rebuilding after world edits

If you modify the SDF world files, rebuild and relaunch:

```bash
cd ~/ardu_ws
colcon build --packages-select ardupilot_gazebo ardupilot_gz_gazebo
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_tracking.launch.py rviz:=false
```

**Important**: Keep both copies of `iris_tracking.sdf` in sync:
- `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_tracking.sdf` (primary, used by launch)
- `src/ardupilot_gazebo/worlds/iris_tracking.sdf` (sync copy)
