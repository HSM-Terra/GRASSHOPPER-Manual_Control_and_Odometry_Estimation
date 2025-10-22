import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import subprocess
import os
import signal

class JoystickLauncher(Node):

    def __init__(self):
        super().__init__('joystick_launcher')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.launch_process = None  # Store the process object
        self.button_0_was_pressed = False  # Track button 0 state
        self.button_1_was_pressed = False  # Track button 1 state

    def joy_callback(self, msg):
        # Check if button 0 is pressed and it was not pressed before
        if msg.buttons[1] == 1 and not self.button_0_was_pressed:
            self.button_0_was_pressed = True
            self.get_logger().info('Button 1 pressed. Launching the file.')
            self.launch_file()
        elif msg.buttons[0] == 0:
            self.button_0_was_pressed = False

        # Check if button 1 is pressed and it was not pressed before
        if msg.buttons[0] == 1 and not self.button_1_was_pressed:
            self.button_1_was_pressed = True
            self.get_logger().info('Button 0 pressed. Stopping the file.')
            self.stop_file()
        elif msg.buttons[1] == 0:
            self.button_1_was_pressed = False

    def launch_file(self):
        if self.launch_process is None or self.launch_process.poll() is not None:
            # Launch the file if not already running
            self.launch_process = subprocess.Popen(['ros2', 'launch', 'odometry_estimator', 'robot_odometry_filtered.launch.py'])
        else:
            self.get_logger().info('Launch file is already running.')

    def stop_file(self):
        if self.launch_process is not None:
            self.launch_process.send_signal(signal.SIGINT)
            try:
                self.launch_process.wait(timeout=5)
                self.get_logger().info('Launch file has been stopped.')
            except:
                self.get_logger().warn('Launch file has not been stopped. Forcefully killing it.')
                os.kill(self.launch_process.pid, signal.SIGKILL)
                self.launch_process.wait()
            self.launch_process = None
            # self.launch_process.wait()  # Wait for the process to terminate
            # self.launch_process = None
            # self.get_logger().info('Launch file has been stopped.')

def main(args=None):
    rclpy.init(args=args)
    joystick_launcher = JoystickLauncher()
    rclpy.spin(joystick_launcher)
    joystick_launcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

