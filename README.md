# Grasshopper Manual Control and Odometry Estimation 

## Pre-requisites
Linux 22.04: https://releases.ubuntu.com/jammy/

ROS2 Humble: https://docs.ros.org/en/humble/index.html

VESC Connections in USB Hub: 
1. Left Front VESC – ttyACM0
2. Right Front VESC – ttyACM1
3. Left Rear VESC – ttyACM2
4. Right Rear VESC – ttyACM3
5. IMU VESC – ttyACM4 

# Robot Manual Control Launch Steps
## Clone the Repository into your Workspace
```bash
git clone https://github.com/HSM-Terra/GRASSHOPPER-Manual_Control_and_Odometry_Estimation
```

## Build the Workspace
1] Install all the dependencies

2] Navigate to each package folder and check the README for related dependencies specific to the concerned package

3] Build the Workspace

```bash
colcon build
```
## Launch the VESC Node
```bash
ros2 launch vesc_driver vesc_driver_node.launch.py 
```
Connect your Joystick via Bluetooth

## Launch the Joystick Control Node
```bash
ros2 launch ros2_vesc_drv vesc_with_joy.launch.py 
```
<img width="1049" height="600" alt="Joystick" src="https://github.com/user-attachments/assets/c5cb2728-5642-45a8-9d82-0844fe7eb676" />

# Grasshopper Odometry Estimation

## Pre-requisites 

Sick TiM 551 LiDAR: https://www.sick.com/at/en/products/lidar-and-radar-sensors/lidar-sensors/tim/tim551-2050001/p/p343045

## Connect the LiDAR via USB and Launch the ROS2 node
```bash
ros2 launch sick-tim sick_tim551_2050001_usb.launch.py
```

## Filtered Odometry Launch
```bash
ros2 launch ros2_odometry_estimation robot_odometry_filtered.launch.py
```

# FRE 2024 Task 1 and Task 2 Navigation Sequence (Crop Lane 1, Turning Sequence 1 & 2)

## Launch Navigation Node

```bash
ros2 launch local_controller node.launch.py
```

Individual Sequences and RANSAC Algorithm can be found in /local_controller/local_controller folder.

For Maize Crop Counting, please refer to the related repository: https://github.com/HSM-Terra/PERCEPTION-Maize-Crop-Detection-Tracking-and-Counting


# FRE Navigation Task 1 Implementation




https://github.com/user-attachments/assets/e3970a5e-56f4-4990-a12c-b500c49408ec



