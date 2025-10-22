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

# Grasshopper Odometry Estimation

## Pre-requisites 

Sick TiM 551 LiDAR

## Connect the LiDAR via USB and Launch the ROS2 node
```bash
ros2 launch sick-tim sick_tim551_2050001_usb.launch.py
```

## Filtered Odometry Launch
```bash
ros2 launch ros2_odometry_estimation robot_odometry_filtered.launch.py
```

