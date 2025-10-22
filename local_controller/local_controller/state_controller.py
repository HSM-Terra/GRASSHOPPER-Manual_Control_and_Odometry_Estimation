import rclpy
from rclpy.node import Node
from smach import State, StateMachine
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

class RansacState(State):
    def __init__(self, node):
        State.__init__(self, outcomes=['success', 'failure'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info('Calling RANSAC service')
        client = self.node.create_client(Empty, 'trigger_ransac')
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for RANSAC service...')
        
        req = Empty.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        
        if future.result() is not None:
            self.node.get_logger().info('RANSAC service call succeeded')
            return 'success'
        else:
            self.node.get_logger().error('RANSAC service call failed')
            return 'failure'

class Sequence1State(State):
    def __init__(self, node):
        State.__init__(self, outcomes=['success', 'failure'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info('Calling Sequence 1 service')
        client = self.node.create_client(Empty, 'trigger_sequence1')
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for Sequence 1 service...')
        
        req = Empty.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        
        if future.result() is not None:
            self.node.get_logger().info('Sequence 1 service call succeeded')
            return 'success'
        else:
            self.node.get_logger().error('Sequence 1 service call failed')
            return 'failure'

class Sequence2State(State):
    def __init__(self, node):
        State.__init__(self, outcomes=['success', 'failure'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info('Calling Sequence 2 service')
        client = self.node.create_client(Empty, 'trigger_sequence2')
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for Sequence 2 service...')
        
        req = Empty.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        
        if future.result() is not None:
            self.node.get_logger().info('Sequence 2 service call succeeded')
            return 'success'
        else:
            self.node.get_logger().error('Sequence 2 service call failed')
            return 'failure'

class CheckPositionState(State):
    def __init__(self, node):
        State.__init__(self, outcomes=['ransac','sequence1', 'sequence2'])
        self.node = node

    def execute(self, userdata):
        if 0.1 <= self.node.current_position_x <= 8.8:
            return 'ransac'
        elif self.node.current_position_x > 8.8:
            return 'sequence1'
        elif self.node.current_position_x < 0.1:
            return 'sequence2'

class FieldRobotController(Node):
    def __init__(self):
        super().__init__('field_robot_controller')
        self.odom_subscriber = self.create_subscription(Odometry, 'odometry/filtered', self.odom_callback, 10)
        self.current_position_x = 0.0

    def odom_callback(self, msg):
        self.current_position_x = msg.pose.pose.position.x

    def main(self):
        sm = StateMachine(outcomes=['finished'])

        with sm:
            StateMachine.add('RANSAC', RansacState(self), transitions={'success': 'CHECK_POSITION', 'failure': 'RANSAC'})
            StateMachine.add('CHECK_POSITION', CheckPositionState(self), transitions={'ransac':'RANSAC', 'sequence1': 'SEQUENCE1', 'sequence2': 'SEQUENCE2'})
            StateMachine.add('SEQUENCE1', Sequence1State(self), transitions={'success': 'SEQUENCE1', 'failure': 'RANSAC'})
            StateMachine.add('SEQUENCE2', Sequence2State(self), transitions={'success': 'SEQUENCE2', 'failure': 'RANSAC'})

        outcome = sm.execute()

def main(args=None):
    rclpy.init(args=args)
    node = FieldRobotController()
    node.main()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()