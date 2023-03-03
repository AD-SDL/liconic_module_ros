import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# from rclpy.clock import clock

from threading import Thread

from std_msgs.msg import String
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# from liconic_driver.liconic_driver import liconic_trobot # TODO:UNCOMMENT THIS WHEN IT IS READY

class liconicDescriptionClient(Node):

    def __init__(self, NODE_NAME = 'liconicDescriptionNode'):
        super().__init__(NODE_NAME)

        # self.liconic = liconic_trobot() #USE THIS LINE WHEN IT IS READY

        timer_period = 0.1  # seconds

        self.state = "UNKNOWN"
        joint_cb_group = ReentrantCallbackGroup()
        state_cb_group = ReentrantCallbackGroup()

        self.statePub = self.create_publisher(String, NODE_NAME + '/state',10)
        # self.stateTimer = self.create_timer(timer_period, callback = self.stateCallback, callback_group = state_cb_group)

        self.joint_publisher = self.create_publisher(JointState,'joint_states', 10, callback_group = joint_cb_group)
        self.joint_state_handler = self.create_timer(timer_period, callback = self.joint_state_publisher_callback, callback_group = joint_cb_group)
    
    
    def stateCallback(self):
        '''
        Publishes the liconic_description state to the 'state' topic. 
        '''
        msg = String()
        msg.data = 'State: %s' % self.state
        self.statePub.publish(msg)
        self.get_logger().info('Publishing State: "%s"' % msg.data)
        self.state = "READY"


    def joint_state_publisher_callback(self):
        
        # self.get_logger().info("BUGG")
        # joint_states = self.liconic.refresh_joint_state() #USE THIS FORMAT WHEN IT IS READY
        joint_states_liconic_1 = [1.2]
        liconic_joint_msg_liconic_1 = JointState()
        liconic_joint_msg_liconic_1.header = Header()
        liconic_joint_msg_liconic_1.header.stamp = self.get_clock().now().to_msg()
        liconic_joint_msg_liconic_1.name = ['liconic_Cap']
        liconic_joint_msg_liconic_1.position = joint_states_liconic_1
        liconic_joint_msg_liconic_1.velocity = []
        liconic_joint_msg_liconic_1.effort = []

        self.joint_publisher.publish(liconic_joint_msg_liconic_1)
        self.get_logger().info('Publishing joint states: "%s"' % str(joint_states_liconic_1))

        joint_states_liconic_2 = [0.0]
        liconic_joint_msg_liconic_2 = JointState()
        liconic_joint_msg_liconic_2.header = Header()
        liconic_joint_msg_liconic_2.header.stamp = self.get_clock().now().to_msg()
        liconic_joint_msg_liconic_2.name = ['liconic_Cap2']
        liconic_joint_msg_liconic_2.position = joint_states_liconic_2
        liconic_joint_msg_liconic_2.velocity = []
        liconic_joint_msg_liconic_2.effort = []

        self.joint_publisher.publish(liconic_joint_msg_liconic_2)
        self.get_logger().info('Publishing joint states: "%s"' % str(joint_states_liconic_2))


def main(args=None):
    rclpy.init(args=args)
    try:
        liconic_joint_state_publisher = liconicDescriptionClient()
        executor = MultiThreadedExecutor()
        executor.add_node(liconic_joint_state_publisher)

        try:
            liconic_joint_state_publisher.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            liconic_joint_state_publisher.get_logger().info('Keyboard interrupt, shutting down.\n')
        finally:
            executor.shutdown()
            liconic_joint_state_publisher.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()