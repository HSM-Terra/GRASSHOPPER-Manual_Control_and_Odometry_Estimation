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
        self.processes = {}  # Dictionary to store process objects
        self.button_states = {}  # Dictionary to store button states

    def joy_callback(self, msg):
        # Example configuration: mapping buttons to specific launch files and stop actions
        button_action_map = {
            2: ('launch', 'odometry_estimator', 'robot_odometry_filtered.launch.py'),
            1: ('launch', 'local_controller', 'node.launch.py'),
            3: ('stop', 2),
            0: ('stop', 1)
        }

        for button, action_info in button_action_map.items():
            if msg.buttons[button] == 1 and not self.button_states.get(button, False):
                self.button_states[button] = True
                self.get_logger().info(f'Button {button} pressed.')
                if action_info[0] == 'launch':
                    self.handle_launch(button, action_info[1], action_info[2])
                elif action_info[0] == 'stop':
                    self.handle_stop(action_info[1])
            elif msg.buttons[button] == 0:
                self.button_states[button] = False

    def handle_launch(self, button, package_name, launch_file):
        if button not in self.processes or self.processes[button] is None or self.processes[button].poll() is not None:
            self.processes[button] = subprocess.Popen(['ros2', 'launch', package_name, launch_file])
            self.get_logger().info(f'Launched {launch_file} from {package_name}.')
        else:
            self.get_logger().info(f'Launch file {launch_file} is already running.')

    def handle_stop(self, launch_button):
        if launch_button in self.processes and self.processes[launch_button] is not None:
            process = self.processes[launch_button]
            process.send_signal(signal.SIGINT)  # Send SIGINT (Ctrl+C)
            try:
                process.wait(timeout=5)  # Wait up to 5 seconds for graceful termination
                self.get_logger().info(f'Process for launch button {launch_button} has been stopped gracefully.')
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f'Process for launch button {launch_button} did not terminate gracefully. Forcefully killing it.')
                process.kill()  # Forcefully terminate if SIGINT did not work
                process.wait()  # Ensure it has terminated
            self.processes[launch_button] = None
        else:
            self.get_logger().info(f'No running process for launch button {launch_button} to stop.')

def main(args=None):
    rclpy.init(args=args)
    joystick_launcher = JoystickLauncher()
    rclpy.spin(joystick_launcher)
    joystick_launcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

