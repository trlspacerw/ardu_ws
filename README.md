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
