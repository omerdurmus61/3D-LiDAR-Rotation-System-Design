## Contents

1. [Introduction](#introduction)
2. [Mechanical Design and Manufacturing](#mechanical-design-and-manufacturing)
3. [System Architecture](#system-architecture)
4. [Installation](#installation)
5. [Usage](#usage)
6. [Visualization](#visualization)
7. [Results](#results)
8. [Contact](#contact)

   
# Introduction
The 3D LiDAR Rotation System enhances the capabilities of a limited-angle 3D LiDAR sensor by incorporating a servo motor-based electromechanical system. This setup enables data acquisition from previously unreachable angles, producing high-resolution point cloud data for environmental modeling and mapping.The scanning angle of the MRS 1000 3D LiDAR sensor from SICK is not sufficient to generate a high-resolution point cloud map. Therefore, I designed an electromechanical system that rotates the LiDAR around the y-axis using a servo motor. After that step, i developed a ROS package for that system. The ROS package includes software for LiDAR data transformation, mapping, modelling, visualization, motor control via rosserial, and integration with an IMU sensor. As a result, the system was able to obtain different point clouds from various areas of the environment. An external IMU was used to track the orientation of the platform. There is a real-time digital twin on RViz.

| ![LiDAR System](https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design/blob/master/images/physical_system_montaged.jpg) | ![LiDAR in Action](https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design/blob/master/images/gif2.gif) |
|------------------------------------|------------------------------------|
| 3D LiDAR Rotation System              | Visualization on Rviz               |

---

## Mechanical Design and Manufacturing
The electromechanical system  was designed using Autodesk Fusion 360 and features a robust frame constructed from aluminum profiles. The design incorporates several components manufactured with a 3D printer, ensuring the system’s mechanical integrity and adaptability.
| ![LiDAR System](https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design/blob/master/images/CAD1.png) | ![LiDAR in Action](https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design/blob/master/images/CAD2.png) | ![Visualization](https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design/blob/master/images/physical_system.gif) |
|:-----------------------------------:|:-------------------------------------:|:-------------------------------------:|
| CAD File                  | CAD File                       | Physical system and Rviz                |


## System Architecture
This ROS graph represents the integration of a 3D LiDAR rotation system, combining data from a SICK MRS 1000 LiDAR sensor, an IMU, and a servo motor. The /mcu_imu_gps_servo node handles servo motor positions and IMU data, while the /sick_mrs_1xxx node publishes raw LiDAR point cloud data on /cloud. The servo position data is processed through /nt16_to_servo_pos and broadcasted to /servo_pos_header. Both the IMU and LiDAR transformations are published to the /tf topic using /imu_tf_broadcaster and /LIDAR_tf_broadcaster, ensuring the alignment of frames in real-time. The raw point cloud is transformed into a global reference frame by /pointCloud_transformer, producing /transformed_cloud for mapping and visualization, with the system visualized in RViz.

| ![LiDAR System](https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design/blob/master/images/rosgraph.png) | 
|------------------------------------|
| RQT GRAPH                          |

| ![LiDAR System](https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design/blob/master/images/frame_tree.png) | ![LiDAR in Action](https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design/blob/master/images/frames.png) |
|------------------------------------|------------------------------------|
| TF Tree              | Frames on Rviz               |

## Installation
```bash
# Create a ROS workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone the repository
git clone https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design.git

# Navigate to the workspace and build the project
cd ~/catkin_ws
catkin_make

# Source the workspace
source devel/setup.bash
```

## Usage
This setup represents a complete pipeline for the 3D LiDAR rotation system. By running these launch files together, the system integrates servo control, IMU data fusion, LiDAR scanning, and visualization, creating a functional and real-time 3D mapping platform.
   
1. imu_gps_servo.launch handles servo control, IMU and GPS data via rosserial.
```bash
roslaunch lidar_rotation imu_gps_servo.launch
```
2. tf_imu_lidar.launch manages the frame transformations.
```bash
roslaunch lidar_rotation tf_imu_lidar.launch
```
3. visualization.launch provides custom Rviz setup for monitoring the system in real-time using markers and PointCloud.
```bash
roslaunch lidar_rotation visualization.launch
```
4. sick_mrs_1xxx.launch handles raw LiDAR data collection and streaming.
```bash
roslaunch sick_scan sick_mrs_1xxx.launch
```
For more details or to download the official SICK drivers, visit the [sic_scan](https://github.com/SICKAG/sick_scan) and [sic_scan_xd](https://github.com/SICKAG/sick_scan_xd) repositories.

| ![LiDAR System](https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design/blob/master/images/terminal_setup.png) | 
|------------------------------------|
| Terminal Setup                     |


## Visualization
| ![LiDAR System](https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design/blob/master/images/gif0.gif) | ![LiDAR in Action](https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design/blob/master/images/gif1.gif) |
|------------------------------------|------------------------------------|
| ![LiDAR System](https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design/blob/master/images/gif3.gif) | ![LiDAR in Action](https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design/blob/master/images/gif4.gif) |
## Results
| ![LiDAR System](https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design/blob/master/images/face.png) | ![LiDAR in Action](https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design/blob/master/images/indoor.png) |
|------------------------------------|------------------------------------|
| ![LiDAR System](https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design/blob/master/images/segmantation.png) | ![LiDAR in Action](https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design/blob/master/images/human.png) |

## Contact
For any questions or feedback, please contact:
- Email: [omercandurmuss@gmail.com]
