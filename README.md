# Tiar-01 Document.

## Table of Contents

- [Tiar-01 Document](#project-name)
  - [Table of Contents](#table-of-contents)
  - [Essential stacks and packages](#essential-stack-and-packages)
    - [ROS Navigation Stack](#ros-navigation-stack)
    - [Als ROS](#Als-ROS)
    - [Realsense ROS](#Realsense-ROS)
    - [Teb Local Planner](#Teb-local-planner)
    - [Gmapping](#gmapping)
    - [Aruco ROS](#aruco-ROS)
    - [Sick Scan](#sick-scan)
  - [Packages implemented](#packages-implemented)
    - [Control Motor](#control-motor)
    - [Move To Marker](#move-to-marker)
    - [ROS MQTT](#ros-mqtt)
    - [Depth Image to Laserscan](#depth-image-to-laserscan)
    - [Navigation](#navigation)
  - [Usage](#usage)

## Essential stacks and packages

### ROS Navigation Stack
The main stack responsible for the automotive movement of the robot.
#### Download
   ```sh
   git clone https://github.com/ros-planning/navigation.git
   ```
#### Dependence
   ```sh
   https://github.com/ros-planning/navigation_msgs.git
   ```

### Als ROS
Localization system for a robot with a 2D lidar.
#### Download
   ```sh
   git clone https://github.com/NaokiAkai/als_ros.git
   ```

### Realsense ROS
Intel Depth Camera with ROS support.
#### SDK for ROS1
   ```sh
   https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages
   ```
#### Realsense ROS1 Package
   ```sh
   git clone https://github.com/IntelRealSense/realsense-ros.git
   ```

### Teb Local Planner
Local Planner for Navigation.
#### Download
   ```sh
https://github.com/rst-tu-dortmund/teb_local_planner.git   
```

### Aruco ROS
Detect and measure the distance from the camera to the AR tag.
#### Download
   ```sh
https://github.com/pal-robotics/aruco_ros.git   
```

## Packages implemented

### Control Motor
This package communicates with the servo motor drive and calculates odometry for the robot.
   ```sh
https://github.com/TTGPDKIOT/robot_TIAR-01/tree/main/control_motor   
```
### Move To Marker
Describe:
   ```sh
https://github.com/SangPhamCV/Go-To-Marker   
```

### ROS MQTT:
Describe:
   ```sh
https://github.com/SangPhamCV/ROS-MQTT-Project
```
### Depth Image to Laserscan
Updating...
### Navigation
Updating...

## Usage
Please ensure that you have all the mentioned packages installed.
```bash
$ cd catkin_ws & catkin_make
```
Run the Node control_motor.py, odometr.py.
Run the launch file sick scan:
```bash
$ roslaunch sick_scan sick_tim_7xx.launch
```
All these operations can be executed within the launch file:
```bash
$ roslaunch tiar01 startup.launch
```
Navigation:
```bash
$ roslaunch als_ros mcl.launch
$ roslaunch tiar01 run.launch
```
Implement moving the robot to the AR tag: See Describe of Move To Marker Package.

# UPDATING...
