# Multi-Drone Distributed SLAM

A ROS 2 package for **multi-drone distributed SLAM** using **ROS 2 Humble**, **Gazebo Harmonic**, **PX4 SITL**, **MAVROS2**, and **RTAB-Map**. The system enables multiple drones to operate simultaneously in simulation, run SLAM independently, and merge their maps offline into a unified representation of the environment.

## Overview

Large-scale environment mapping is difficult with a single drone because of limited battery life, constrained field of view, and mission-time limitations. This project explores a distributed approach where multiple drones divide the environment and map it in parallel. Each drone runs its own sensing and SLAM pipeline, while the generated RTAB-Map databases can later be merged offline into a single global map.

This project was built as part of a robotics course/research effort focused on:
- collaborative exploration,
- distributed mapping,
- multi-agent simulation, and
- map fusion for larger environments.

## Key Features

- Multi-drone simulation in **Gazebo Harmonic**
- Flight stack integration with **PX4 SITL**
- ROS 2 communication using **MAVROS2** and **ROS-GZ Bridge**
- Independent RGB-D SLAM pipelines using **RTAB-Map**
- Offline database merging for a unified map
- Support for custom drone nodes and utility scripts inside the package

## Tech Stack

- **OS:** Ubuntu 22.04
- **Middleware:** ROS 2 Humble
- **Simulator:** Gazebo Harmonic
- **Autopilot:** PX4 SITL
- **Bridge / Flight Interface:** MAVROS2, ROS-GZ Bridge
- **SLAM:** RTAB-Map ROS 2
- **Language:** Python

## Repository Structure

```text
multi_drone_slam/
├── config/                  # Configuration files
├── launch/                  # ROS 2 launch files
├── models/                  # Gazebo models and terrain assets
├── multi_drone_slam/        # Python package source
├── resource/                # ROS package resources
├── scripts/                 # Utility scripts
├── Test/                    # Test assets / experiments
├── RAS_PROJECT_REPORT.pdf   # Project report
├── package.xml
├── setup.py
└── README.md
````

## Team

* Mainak Saha
* Sakshi Khade
* Mrunmayee Valunj
* Khushal Sharma

## My Contributions

* Configured and tuned **RTAB-Map** pipelines for distributed mapping
* Helped integrate the multi-drone ROS 2 simulation workflow
* Worked on namespaced SLAM execution and offline map-merging setup
* Supported testing and debugging across the mapping pipeline

## Requirements

Before running the project, make sure the following are installed:

* Ubuntu 22.04
* ROS 2 Humble Hawksbill
* Gazebo Harmonic
* PX4 v1.14+ (SITL)
* MAVROS2
* RTAB-Map ROS 2
* ROS-GZ Bridge
* QGroundControl

## Installation

### 1) Install ROS 2 Humble

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

Source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

### 2) Create a workspace

```bash
mkdir -p ~/drone_ws/src
cd ~/drone_ws/src
```

### 3) Clone this repository

```bash
git clone https://github.com/khushalrs/multi_drone_slam.git
```

### 4) Clone PX4 Autopilot

```bash
cd ~/drone_ws
git clone https://github.com/PX4/PX4-Autopilot.git --branch v1.14.0 --recursive
```

### 5) Install required ROS packages

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-mavros
sudo apt install ros-humble-rtabmap-ros
sudo apt install ros-humble-ros-gz-bridge
```

### 6) Build the workspace

```bash
cd ~/drone_ws
colcon build --packages-select multi_drone_slam rtabmap_launch
source install/setup.bash
```

## Running the Project

### Step 1: Launch the multi-drone simulation

```bash
ros2 launch multi_drone_slam multi_drone_spawn.launch.py
```

This launches the Gazebo environment and spawns multiple drones for simulation.

### Step 2: Start QGroundControl

```bash
./QGroundControl.AppImage
```

Use QGroundControl to monitor telemetry and vehicle status.

### Step 3: Bridge Gazebo sensor topics to ROS 2

#### Drone 1

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /rgb_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args \
  -r /rgb_camera:=/drone1/rgb_camera \
  -r /camera_info:=/drone1/camera_info \
  -r /depth_camera:=/drone1/depth_camera
```

#### Drone 2

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /rgb_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args \
  -r /rgb_camera:=/drone2/rgb_camera \
  -r /camera_info:=/drone2/camera_info \
  -r /depth_camera:=/drone2/depth_camera
```

### Step 4: Publish static transforms

#### Drone 1

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 base_link_1 OakD-Lite-Modify_1/base_link

ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 odom_1 base_link_1
```

#### Drone 2

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 base_link_2 OakD-Lite-Modify_2/base_link

ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 odom_2 base_link_2
```

### Step 5: Start MAVROS for each drone

#### Drone 1

```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://@127.0.0.1:14580 -r __ns:=/drone1
```

#### Drone 2

```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://@127.0.0.1:14581 -r __ns:=/drone2
```

### Step 6: Launch RTAB-Map for each drone

#### Drone 1

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  frame_id:=OakD-Lite-Modify_1/base_link \
  rgb_topic:=/drone1/rgb_camera \
  depth_topic:=/drone1/depth_camera \
  camera_info_topic:=/drone1/camera_info \
  odom_topic:=/drone1/odometry/in \
  subscribe_depth:=true \
  subscribe_scan:=false \
  approx_sync:=true \
  database_path:=~/drone_ws/rtabmap1.db
```

#### Drone 2

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  frame_id:=OakD-Lite-Modify_2/base_link \
  rgb_topic:=/drone2/rgb_camera \
  depth_topic:=/drone2/depth_camera \
  camera_info_topic:=/drone2/camera_info \
  odom_topic:=/drone2/odometry/in \
  subscribe_depth:=true \
  subscribe_scan:=false \
  approx_sync:=true \
  database_path:=~/drone_ws/rtabmap2.db
```

### Step 7: Run the package node

```bash
ros2 run multi_drone_slam dron3
```

### Step 8: Pause or resume mapping

#### Pause Drone 1

```bash
ros2 service call /drone1/pause_resume std_srvs/srv/SetBool "{data: true}"
```

#### Resume Drone 1

```bash
ros2 service call /drone1/pause_resume std_srvs/srv/SetBool "{data: false}"
```

You can repeat the same pattern for other drones if the corresponding services are available.

## Offline Map Merging

After each drone completes its mapping session, merge their RTAB-Map databases into a single map:

```bash
rtabmap-reprocess ~/drone_ws/rtabmap1.db ~/drone_ws/rtabmap2.db --output merged_map.db
```

This produces a merged database containing the fused map representation.

## Available ROS 2 Executables

The package currently registers the following ROS 2 executables:

```bash
dron1
dron2
dron3
imu_reframe
gz_link_pose_to_tf
circle_detection
```

## Use Cases

This project is useful for:

* collaborative mapping of large environments,
* infrastructure inspection scenarios,
* multi-agent robotics research,
* SLAM benchmarking in simulation,
* exploring map fusion workflows before real-world deployment.

## Known Limitations

* Map merging is currently performed **offline**, not in real time.
* The system depends on careful topic namespacing and transform consistency.
* Simulation assumptions may differ from real-world sensor and flight behavior.
* Setup is sensitive to version compatibility across ROS 2, Gazebo, PX4, and MAVROS.

## Future Improvements

* Real-time multi-agent map fusion
* Improved autonomous exploration strategies
* Better inter-drone coordination and task allocation
* Real-world deployment on physical drones
* Semantic annotations and defect detection integrated into the map

## Troubleshooting

### ROS 2 package not found

Make sure you have sourced both ROS 2 and your workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/drone_ws/install/setup.bash
```

### RTAB-Map not receiving camera data

Check whether:

* ROS-GZ topic bridges are running
* topic names match the drone namespace
* camera info is being published correctly

### MAVROS connection issues

Verify:

* PX4 SITL is running
* the `fcu_url` UDP port matches the correct drone
* QGroundControl is not conflicting with the setup

### TF tree issues

Inspect the transform tree using:

```bash
ros2 run tf2_tools view_frames
```

## License

This project is released under the MIT License. See the `LICENSE` file for details.

## Contact

If you have questions, suggestions, or want to collaborate, feel free to open an issue or submit a pull request.
