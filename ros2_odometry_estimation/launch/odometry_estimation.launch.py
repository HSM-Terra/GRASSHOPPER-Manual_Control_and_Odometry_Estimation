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
            executable='odometry_estimator',
            name='odometry_estimator',
            remappings=[("sensors/rotor_position_left","motor_rpm_left_front"),
                        ('sensors/rotor_position_right','motor_rpm_right_front')]
        ),

    ])