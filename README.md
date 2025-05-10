# Multi-Drone Distributed SLAM

This repository contains the implementation of a multi-drone distributed SLAM system using ROS 2 Humble, Gazebo Harmonic, PX4 SITL, MAVROS2, and RTAB-Map. The system enables simultaneous operation of multiple drones, each performing SLAM independently, and combines their maps offline for a unified environment representation.

## Motivation

Efficient mapping of large structures with limited flight time requires dividing the environment among multiple drones. By leveraging distributed SLAM, each drone explores a portion of the environment in parallel, reducing overall mission duration and allowing scalability to more drones and larger areas. Offline merging of individual maps ensures accurate global consistency without the complexity of real-time synchronization.

## Team Members

- Mainak Saha
- Sakshi Khade
- Mrunmayee Valunj
- Khushal Sharma (Me)

## Contributions

- **Mrunmayee Valunj & Sakshi Khade**: Developed and tested simultaneous drone spawning in Gazebo Harmonic.
- **Mainak Saha & Khushal Sharma**: Configured and tuned RTAB-Map SLAM pipelines for distributed mapping.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble Hawksbill
- Gazebo Harmonic
- PX4 v1.14+ (SITL)
- MAVROS2 (ros2 branch)
- RTAB-Map ROS2
- ROS‑GZ Bridge
- QGroundControl (for monitoring and control)

## Installation

1. Install ROS 2 Humble:
   ```bash
   sudo apt update && sudo apt install ros-humble-desktop
   ```
2. Install Gazebo Harmonic and PX4 dependencies:
   ```bash
   sudo apt install ros-humble-gazebo-ros-pkgs
   cd ~/drone_ws
   git clone https://github.com/PX4/PX4-Autopilot.git --branch v1.14.0 --recursive
   ```
3. Install MAVROS2 and RTAB-Map ROS2 packages:
   ```bash
   sudo apt install ros-humble-mavros ros-humble-rtabmap-ros ros-humble-ros-gz-bridge
   ```
4. Build your workspace:
   ```bash
   cd ~/drone_ws
   colcon build --packages-select multi_drone_slam rtabmap_launch
   source install/setup.bash
   ```

## Usage

1. **Launch multi-drone spawn in Gazebo**:
   ```bash
   ros2 launch multi_drone_slam multi_drone_spawn.launch.py
   ```
2. **Start QGroundControl** for drone telemetry:
   ```bash
   ./QGroundControl.AppImage
   ```
3. **Bridge Gazebo sensor topics to ROS 2** for each drone:

   **Drone 1:**
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

   **Drone 2:**
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
4. **Publish static transforms** for each drone’s sensor frames:
   ```bash
   # Drone 1
   ros2 run tf2_ros static_transform_publisher \
     0 0 0 0 0 0 base_link_1 OakD-Lite-Modify_1/base_link
   ros2 run tf2_ros static_transform_publisher \
     0 0 0 0 0 0 odom_1 base_link_1

   # Drone 2
   ros2 run tf2_ros static_transform_publisher \
     0 0 0 0 0 0 base_link_2 OakD-Lite-Modify_2/base_link
   ros2 run tf2_ros static_transform_publisher \
     0 0 0 0 0 0 odom_2 base_link_2
   ```
5. **Start MAVROS nodes** for each drone with namespace-specific remapping:
   ```bash
   # Drone 1
   ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://@127.0.0.1:14580 -r __ns:=/drone1


   # Drone 2
   ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://@127.0.0.1:14581 -r __ns:=/drone2

   ```
6. **Launch RTAB-Map SLAM** for each drone with namespaced topics:
   ```bash
   # Drone 1
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

   # Drone 2
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
7. **Run the custom multi-drone SLAM node**:
   ```bash
   ros2 run multi_drone_slam dron3
   ```
8. **Pause and resume mapping** for Drone 1 (and similarly for Drone 2 if desired):
   ```bash
   ros2 service call /drone1/pause_resume std_srvs/srv/SetBool "{data: true}"
   ros2 service call /drone1/pause_resume std_srvs/srv/SetBool "{data: false}"
   ```

## Offline Map Merging

After each drone completes its mapping session, combine individual RTAB-Map databases into a single global map:

```bash
rtabmap-reprocess ~/drone_ws/rtabmap1.db ~/drone_ws/rtabmap2.db --output merged_map.db
```

## License

This project is released under the MIT License. See [LICENSE](LICENSE) for details.

---

For questions or contributions, please open an issue or submit a pull request. Happy mapping!
