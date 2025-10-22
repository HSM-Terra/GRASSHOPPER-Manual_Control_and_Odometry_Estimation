from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="joy", executable="joy_node", name="joy_node"),

        #logitech joy config.
        # Node(
        #     package="teleop_twist_joy",
        #     executable="teleop_node",
        #     parameters=[
        #         {"axis_angular": 2},               #right joy button
        #         {"axis_linear": {"x": 3}},
        #         {"require_enable_button": True},   #RB button in joy
        #         {"scale_linear": {"x": 0.4}},
        #         {"scale_angular": {"yaw": 0.4}},
        #     ],
        # ),

        # Ps4 controller config.
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            parameters=[
                {"axis_angular": {"yaw":3}},         #right joy button
                {"axis_linear": {"x": 4}},
                {"scale_linear": {"x": 0.25}},
                {"scale_angular": {"yaw": 0.25}},
                {"enable_button": 5},  #R1 Button in joy
            ],
        ),
        # Node(
        #     package="teleop_twist_joy",
        #     executable="teleop_node",
        #     parameters=[
        #         {"axis_angular": {"yaw":3}},         #right joy button
        #         {"axis_linear": {"x": 4}},
        #         {"scale_linear": {"low": 0.10}},
        #         {"scale_angular": {"low": 0.10}},
        #         {"scale_linear": {"high": 0.40}},
        #         {"scale_angular": {"high": 0.40}},
        #         {"enable_button": 5},  #R1 Button in joy
        #         {"enable_turbo_button": 7},
        #         {"require_enable_button": True},
        #     ],
        # ),
        Node(
            package="ros2_vesc_drv",
            executable="cmdv_mapper"
        ),
        Node(
            package="ros2_vesc_drv",
            executable="joy_control"
        )
    ])
