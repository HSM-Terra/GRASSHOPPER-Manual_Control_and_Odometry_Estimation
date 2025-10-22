import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        
        Node(
            package='odometry_estimator',
            executable='radian_to_rpm',
            name='radian_to_rpm_node_left',
            remappings=[("motor_rpm","motor_rpm_left_front"),
                        ('sensors/rotor_position','sensors/rotor_position_left_front')]
        ),
        Node(
            package='odometry_estimator',
            executable='radian_to_rpm',
            name='radian_to_rpm_node_right',
            remappings=[("motor_rpm","motor_rpm_right_front"),
                        ('sensors/rotor_position','sensors/rotor_position_right_front')]
        ),

    ])