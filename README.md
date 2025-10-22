# Launching Grasshopper

## Pre-requisites
Linux 22.04

ROS2 Humble

Robot Connections

# Robot Manual Control Launch Steps
## Clone the Repositorz into your workspace

git clone 

## Build the workspace

colcon build

## Launch the VESC Node
ros2 launch vesc_driver vesc_driver_node.launch.py 

Connect your joystick via bluetooth

## Launch the Joystick Control Node

ros2 launch ros2_vesc_drv vesc_with_joy.launch.py 

