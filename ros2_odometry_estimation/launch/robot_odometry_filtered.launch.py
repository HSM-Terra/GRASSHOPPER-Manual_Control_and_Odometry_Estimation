# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import  IncludeLaunchDescription
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from launch.launch_description_sources import AnyLaunchDescriptionSource
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# def generate_launch_description():
#     return LaunchDescription([
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([
#                 PathJoinSubstitution([
#                     FindPackageShare('odometry_estimator'),
#                     'launch',
#                     'radian_to_rpm.launch.py'
#                 ])
#             ])
#         ),

#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([
#                 PathJoinSubstitution([
#                     FindPackageShare('odometry_estimator'),
#                     'launch',
#                     'odometry_estimation.launch.py'
#                 ])
#             ])
#         ),

#         IncludeLaunchDescription(
#             AnyLaunchDescriptionSource([
#                 PathJoinSubstitution([
#                     FindPackageShare('robot_assembly'),
#                     'launch',
#                     'tf.launch.xml'
#                 ])
#             ])
#         ),

#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([
#                 PathJoinSubstitution([
#                     FindPackageShare('ros2_laser_scan_matcher'),
#                     'launch',
#                     'lidar_based_odometry.launch.py'
#                 ])
#             ])
#         ),

#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([
#                 PathJoinSubstitution([
#                     FindPackageShare('robot_localization'),
#                     'launch',
#                     'ekf.launch.py'
#                 ])
#             ])
#         ),
    
#     ])


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource

def create_include_action(package_name, launch_file, source_type=PythonLaunchDescriptionSource):
    return IncludeLaunchDescription(
        source_type([
            PathJoinSubstitution([
                FindPackageShare(package_name),
                'launch',
                launch_file
            ])
        ])
    )

def generate_launch_description():
    return LaunchDescription([
        TimerAction(
            period=2.0,
            actions=[create_include_action('odometry_estimator', 'radian_to_rpm.launch.py')]
        ),
        TimerAction(
            period=5.0,  # 2 seconds after the first
            actions=[create_include_action('odometry_estimator', 'odometry_estimation.launch.py')]
        ),
        TimerAction(
            period=7.0,  # 2 seconds after the second
            actions=[create_include_action('robot_assembly', '4wd.launch.xml', AnyLaunchDescriptionSource)]
        ),
        TimerAction(
            period=9.0,  # 2 seconds after the third
            actions=[create_include_action('ros2_laser_scan_matcher', 'lidar_based_odometry.launch.py')]
        ),
        TimerAction(
            period=11.0,  # 2 seconds after the fourth
            actions=[create_include_action('robot_localization', 'ekf.launch.py')]
        ),
    ])

