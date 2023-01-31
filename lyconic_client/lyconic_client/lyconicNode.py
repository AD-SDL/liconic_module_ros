#! /usr/bin/env python3

import rclpy                 # import Rospy
from rclpy.node import Node  # import Rospy Node
from std_msgs.msg import String

from wei_services.srv import WeiDescription 
from wei_services.srv import WeiActions   

from time import sleep

from lyconic_driver.lyconic_driver import lyconic_trobot

class lyconicNode(Node):
    '''
    The lyconicNode inputs data from the 'action' topic, providing a set of commands for the driver to execute. It then receives feedback, 
    based on the executed command and publishes the state of the lyconic and a description of the lyconic to the respective topics.
    '''
    def __init__(self, NODE_NAME = "lyconicNode"):
        '''
        The init function is neccesary for the lyconicNode class to initialize all variables, parameters, and other functions.
        Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
        '''

        super().__init__(NODE_NAME)
        
        self.lyconic = lyconic()
        self.state = "READY"

        
        self.description = {
            'name': NODE_NAME,
            'type': 'lyconic_thermocicler',
            'actions':
            {
                'status':'',
                'open_lid':'',
                'close_lid':'',
                'run_program':'program_n'
            }
        }

        timer_period = 1  # seconds
        self.statePub = self.create_publisher(String, 'lyconic_state', 10)
        self.stateTimer = self.create_timer(timer_period, self.stateCallback)

        self.actionSrv = self.create_service(WeiActions, NODE_NAME + "/action_handler", self.actionCallback)
        self.descriptionSrv = self.create_service(WeiDescription, NODE_NAME + "/description_handler", self.descriptionCallback)

    def descriptionCallback(self, request, response):
        """The descriptionCallback function is a service that can be called to showcase the available actions a robot
        can preform as well as deliver essential information required by the master node.

        Parameters:
        -----------
        request: str
            Request to the robot to deliver actions
        response: str
            The actions a robot can do, will be populated during execution

        Returns
        -------
        str
            The robot steps it can do
        """
        response.description_response = str(self.description)

        return response

    def actionCallback(self, request, response):
        '''
        The actionCallback function is a service that can be called to execute the available actions the robot
        can preform.
        '''

        if request.action_handle=='status':
            self.lyconic.get_status()
            response.action_response = True
        if request.action_handle=='open_lid':            
            self.state = "BUSY"
            self.stateCallback()
            self.lyconic.open_lid()    
            response.action_response = True
        if request.action_handle=='close_lid':            
            self.state = "BUSY"
            self.stateCallback()
            self.lyconic.close_lid()    
            response.action_response = True
        if request.action_handle=='close_lid':
            self.state = "BUSY"
            self.stateCallback()
            vars = eval(request.vars)
            print(vars)
            prog = vars.get('program_n')
            self.lyconic.run_program(prog)
        self.state = "COMPLETED"

        return response

    def stateCallback(self):
        '''
        Publishes the lyconic state to the 'state' topic. 
        '''
        msg = String()
        msg.data = 'State: %s' % self.state
        self.statePub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.state = "READY"

def main(args = None):
    NAME = "lyconicNode"
    rclpy.init(args=args)  # initialize Ros2 communication
    node = lyconicNode(NODE_NAME=NAME)
    rclpy.spin(node)     # keep Ros2 communication open for action node
    rclpy.shutdown()     # kill Ros2 communication

if __name__ == '__main__':
    main()
