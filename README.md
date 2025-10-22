# Grasshopper Manual Control and Odometry Estimation 

## Pre-requisites
Linux 22.04: https://releases.ubuntu.com/jammy/

ROS2 Humble: https://docs.ros.org/en/humble/index.html

Robot Connections: 

# Robot Manual Control Launch Steps
## Clone the Repository into your workspace
```bash
git clone https://github.com/HSM-Terra/GRASSHOPPER-Robot_Launch
```

## Build the workspace
```bash
colcon build
```
## Launch the VESC Node
```bash
ros2 launch vesc_driver vesc_driver_node.launch.py 
```
Connect your joystick via bluetooth

## Launch the Joystick Control Node
```bash
ros2 launch ros2_vesc_drv vesc_with_joy.launch.py 
```

# Grasshopper Odometry Estimation

## Pre-requisites 

Sick TiM 551 LiDAR

## Connect the LiDAR via USB and launch the ROS2 node
```bash
ros2 launch sick-tim sick_tim551_2050001_usb.launch.py
```

## Filtered Odometry Launch
```bash
ros2 launch ros2_odometry_estimation robot_odometry_filtered.launch.py
```

