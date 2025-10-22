# Copyright 2020 F1TENTH Foundation
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#
#   * Neither the name of the {copyright_holder} nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import  IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    vesc_config_left_front = os.path.join(
        get_package_share_directory('vesc_driver'),
        'params',
        'vesc_config_left_front.yaml'
        )
    
    vesc_config_right_front = os.path.join(
        get_package_share_directory('vesc_driver'),
        'params',
        'vesc_config_right_front.yaml'
        )
    
    vesc_config_left_rear = os.path.join(
        get_package_share_directory('vesc_driver'),
        'params',
        'vesc_config_left_rear.yaml'
        )
    
    vesc_config_right_rear = os.path.join(
        get_package_share_directory('vesc_driver'),
        'params',
        'vesc_config_right_rear.yaml'
        )
    
    vesc_config_imu = os.path.join(
        get_package_share_directory('vesc_driver'),
        'params',
        'vesc_config_imu.yaml'
        )
    
    return LaunchDescription([
        # Node(
        #         package="ros2_vesc_drv",
        #         executable="cmdv_mapper"
        #     ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('sick_tim'),
        #             'sick_tim551_2050001_usb.launch.py'
        #         ])
        #     ])
        # ),

        DeclareLaunchArgument(
            name="config_1",
            default_value=vesc_config_left_front,
            description="VESC yaml configuration file ttyACM0.",
            ),
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node_1',
            parameters=[LaunchConfiguration("config_1")],
            remappings=[("sensors/core","sensors/core_left_front"),
                        ("sensors/imu","sensors/imu_left_front"),
                        ("sensors/imu/raw","sensors/imu_left_front/raw"),
                        ("sensors/servo_position_command","sensors/left_front_servo_position_command"),
                        ("commands/dutycycle","commands/dutycycle/left_front"),
                        ('sensors/rotor_position','sensors/rotor_position_left_front')]
        ),

        DeclareLaunchArgument(
            name="config_2",
            default_value=vesc_config_right_front,
            description="VESC yaml configuration file ttyACM1.",
            ),
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node_2',
            parameters=[LaunchConfiguration("config_2")],
            remappings=[("sensors/core","sensors/core_right_front"),
                        ("sensors/imu","sensors/imu_right_front"),
                        ("sensors/imu/raw","sensors/imu_right_front/raw"),
                        ("sensors/servo_position_command","sensors/right_front_servo_position_command"),
                        ("commands/dutycycle","commands/dutycycle/right_front"),
                        ('sensors/rotor_position','sensors/rotor_position_right_front')]
        ),

        DeclareLaunchArgument(
            name="config_3",
            default_value=vesc_config_left_rear,
            description="VESC yaml configuration file ttyACM2.",
            ),
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node_3',
            parameters=[LaunchConfiguration("config_3")],
            remappings=[("sensors/core","sensors/core_left_rear"),
                        ("sensors/imu","sensors/imu_left_rear"),
                        ("sensors/imu/raw","sensors/imu_left_rear/raw"),
                        ("sensors/servo_position_command","sensors/left_rear_servo_position_command"),
                        ("commands/dutycycle","commands/dutycycle/left_rear"),
                        ('sensors/rotor_position','sensors/rotor_position_left_rear')]
        ),

        DeclareLaunchArgument(
            name="config_4",
            default_value=vesc_config_right_rear,
            description="VESC yaml configuration file ttyACM3.",
            ),
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node_4',
            parameters=[LaunchConfiguration("config_4")],
            remappings=[("sensors/core","sensors/core_right_rear"),
                        ("sensors/imu","sensors/imu_right_rear"),
                        ("sensors/imu/raw","sensors/imu_right_rear/raw"),
                        ("sensors/servo_position_command","sensors/right_rear_servo_position_command"),
                        ("commands/dutycycle","commands/dutycycle/right_rear"),
                        ('sensors/rotor_position','sensors/rotor_position_right_rear')]
        ),

        # DeclareLaunchArgument(
        #     name="config_5",
        #     default_value=vesc_config_imu,
        #     description="VESC yaml configuration file ttyACM4.",
        #     ),
        # Node(
        #     package='vesc_driver',
        #     executable='vesc_driver_node',
        #     name='vesc_driver_imu_node',
        #     parameters=[LaunchConfiguration("config_5")],
        #     remappings=[("sensors/core","sensors/core_imu_data"),
        #                 ("sensors/imu","sensors/imu_data"),
        #                 ("sensors/imu/raw","sensors/imu_data/raw"),
        #                 ("sensors/servo_position_command","sensors/imu_servo_position_command"),
        #                 ("commands/dutycycle","commands/dutycycle/imu"),
        #                 ('sensors/rotor_position','sensors/rotor_position_imu')]
        # )

    ])
