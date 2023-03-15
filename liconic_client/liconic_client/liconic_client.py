#! /usr/bin/env python3

import stx                  # import liconic driver
from stx import Stx

import rclpy                 # import Rospy
from rclpy.node import Node  # import Rospy Node

from wei_services.srv import WeiDescription  # import workcell execution interface (wei)
from wei_services.srv import WeiActions   

from std_msgs.msg import String         # other imports
from typing import List, Tuple
from time import sleep

class liconicNode(Node):
    '''
    The liconicNode inputs data from the 'action' topic, providing a set of commands for the driver to execute. It then receives feedback, 
    based on the executed command and publishes the state of the liconic and a description of the liconic to the respective topics.
    '''
    def __init__(self, NODE_NAME = "Liconic_Client_Node"):
        '''
        The init function is neccesary for the liconicNode class to initialize all variables, parameters, and other functions.
        Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
        '''

        super().__init__(NODE_NAME)
        
        self.liconic = Stx('/dev/ttyUSB3')  # TODO: this should be moved to a different file
        # self.state = "READY"

        self.description = {
            'name': NODE_NAME,
            'type': 'liconic_incubator',
            'actions':
            {
                'status':'',  # TODO: add the correct actions here
                'open_lid':'',
                'close_lid':'',
                'run_program':'program_n'
            }
        }

        timer_period = 1  # seconds
        self.statePub = self.create_publisher(String, 'liconic_state', 10)   # TODO: How does this actually connect back to the liconic state?
        self.stateTimer = self.create_timer(timer_period, self.stateCallback)

        self.actionSrv = self.create_service(WeiActions, NODE_NAME + "/action_handler", self.actionCallback)
        self.descriptionSrv = self.create_service(WeiDescription, NODE_NAME + "/description_handler", self.descriptionCallback)

    
    @property 
    def state(self) -> str: 
        """'READY' if device is ready, 'ERROR' if device reports error, 'BUSY' otherwise"""
        if self.liconic.ready == True: 
            return "READY"
        elif self.liconic.ready == False: 
            if self.liconic.has_error == True: 
                return "ERROR"
            else: 
                return "BUSY"
    
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

        # temperature actions
        if request.action_handle=="get_current_temp": 
            response.action_response = 0
            response.action_msg = str(self.liconic.climate_controller.current_temperature)
        elif request.action_handle=="get_target_temp":
            response.action_response = self.liconic.climate_controller.target_temperature
        elif request.action_handle=="set_target_temp":
            vars = eval(request.vars)
            temp = float(vars.get('temp'))   # must be a float 
            try: 
                self.liconic.climate_controller.target_temperature = temp
                response.action_response = 0
                response.action_msg = str(self.liconic.climate_controller.target_temperature)
            except Exception as error_msg:
                print("TODO: handle exception")
                response.action_response = -1
                response.action_msg = "shit'sfucked"
                # self.get_logger().error("------- Liconic Error message: " + str(error_msg) +  (" -------"))

                

        # if request.action_handle=='status':
        #     self.liconic.get_status()
        #     response.action_response = True
        # if request.action_handle=='open_lid':            
        #     self.state = "BUSY"
        #     self.stateCallback()
        #     self.liconic.open_lid()    
        #     response.action_response = True
        # if request.action_handle=='close_lid':            
        #     self.state = "BUSY"
        #     self.stateCallback()
        #     self.liconic.close_lid()    
        #     response.action_response = True
        # if request.action_handle=='close_lid':
        #     self.state = "BUSY"
        #     self.stateCallback()
        #     vars = eval(request.vars)
        #     print(vars)
        #     prog = vars.get('program_n')
        #     self.liconic.run_program(prog)
        # self.state = "COMPLETED"

        
        return response

    def stateCallback(self):
        '''
        Publishes the liconic state to the 'state' topic. 
        '''
        msg = String()
        msg.data = 'State: %s' % self.state
        self.statePub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.state = "READY"  # should not be setting this to ready


def main(args = None):
    NAME = "liconicNode"
    rclpy.init(args=args)  # initialize Ros2 communication
    node = liconicNode(NODE_NAME=NAME)
    rclpy.spin(node)     # keep Ros2 communication open for action node
    rclpy.shutdown()     # kill Ros2 communication

if __name__ == '__main__':
    main()
