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
    def machine_state(self) -> str: 
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

        # temperature control actions
        if request.action_handle == "get_current_temp": 
            response.action_response = 0
            response.action_msg = str(self.liconic.climate_controller.current_temperature)
        
        elif request.action_handle == "get_target_temp":
            response.action_response = 0
            response.action_msg = str(self.liconic.climate_controller.target_temperature)
        
        elif request.action_handle == "set_target_temp":
            vars = eval(request.vars)
            temp = float(vars.get('temp'))   # must be a float 
            # TODO: make sure this is a float and handle exceptions
            try: 
                self.liconic.climate_controller.target_temperature = temp
                response.action_response = 0
                response.action_msg = "Target temperature = " + str(self.liconic.climate_controller.target_temperature)
            except Exception as error_msg:
                print("TODO: handle exception")
                response.action_response = -1
                response.action_msg = "Error: Could not reset liconic temperature"
                self.get_logger().error("------- Liconic Error message: " + str(error_msg) +  (" -------"))

        # humidity control actions
        elif request.action_handle == "get_current_humidity": 
            response.action_response = 0
            response.action_msg = str(self.liconic.climate_controller.current_humidity)
        
        elif request.action_handle == "get_target_humidity":
            response.action_response = 0
            response.action_msg = str(self.liconic.climate_controller.target_humidity)

        elif request.action_handle == "set_target_humidity": 
            vars = eval(request.vars)
            humidity = float(vars.get('humidity'))  
            # TODO: make sure humidity is a float and handle exceptions
            try: 
                self.liconic.climate_controller.target_humidity = humidity
                response.action_response = 0
                response.action_msg = "Target humidity = " + str(self.liconic.climate_controller.target_humidity)
            except Exception as error_msg: 
                print("TODO: handle exception")
                response.action_response = -1
                response.action_msg = "Error: Could not reset liconic humidity"
                self.get_logger().error("------- Liconic Error message: " + str(error_msg) +  (" -------"))

        # CO2 control actions 
            # TODO - we don't use these actions now

        # Gas control actions - gas port 1
            # TODO - we don't use these actions now 

        # Gas control actions - gas port 2
            # TODO - we don't use these actions now 

        # Shaker actions
            # ACTIONS TO IMPLEMENT: 
            # Begin shake (or start shake?)
            # End shake  (or stop shake?)
            # Timed shake (but don't block the thread while waiting)
        elif request.action_handle == "begin_shake":
            vars = eval(request.vars)
            new_shaker_speed = int(vars.get('shaker_speed')) 
            # TODO: make sure shaker speed is a in 1 to 50 inclusive and handle exceptions
            # TODO: make shaker speed a client variable so it doesn't have to be specified each time 
            try: 
                if self.liconic.shaker_controller.shaker_is_active == True:
                    if not new_shaker_speed == self.liconic.shaker_controller.shaker_speed:
                        """already shaking but not at desired speed""" 
                        # stop shaking 
                        self.liconic.shaker_controller.stop_shaker()
                        # set shaking speed to new value 
                        self.liconic.shaker_controller.shaker_speed = new_shaker_speed
                        # restart shaking at new speed 
                        self.liconic.shaker_controller.activate_shaker()
                        # check that liconic is now shaking
                            # TODO: might need to wait a bit before checking, I'm not sure if will return True immediately
                        # format returns
                        response.action_response = 0
                        response.action_msg = "Liconic shaker activated, shaker speed: " + str(self.liconic.shaker_controller.shaker_speed)
                    else: 
                        """already shaking at desired speed"""
                        # format returns
                        response.action_response = 0
                        response.action_msg = "Liconic shaker activated, shaker speed: " + str(self.liconic.shaker_controller.shaker_speed)
                elif self.liconic.shaker_controller.shaker_is_active == False:
                    """not already shaking"""
                    # set shaking speed to new value (regardless of if already set to new value)
                    self.liconic.shaker_controller.shaker_speed = new_shaker_speed
                    # start shaking
                    self.liconic.shaker_controller.activate_shaker()
                    # TODO: check that liconic has started shaking 
                    # format returns 
                    response.action_response = 0
                    response.action_msg = "Liconic shaker activated, shaker speed: " + str(self.liconic.shaker_controller.shaker_speed)
                else: 
                    """catch for any other liconic shaking states"""  # maybe remove else statement
                    response.action_response = -1
                    response.action_msg = "Error: Liconic could not begin shake"
            
            except Exception as error_msg: 
                print("TODO: handle exception")
                response.action_response = -1
                response.action_msg = "Error: Liconic could not begin shake"
                self.get_logger().error("------- Liconic Error message: " + str(error_msg) +  (" -------"))


                # if shaker is shaking and new speed matches old speed - do nothing 
                # if shaker is shaking and new speed not = to old speed - stop shaking then start at new speed
                # is shaker not shaking, reset speed to new speed and start shaking 
            

                

        # Plate handling actions 
            # Load Plate 
            # Unload Plate 
            # Timed load, shake, unload? - how to do this?
            # Display Contents? 
        elif request.action_handle == "load_plate":
            pass
            # 1.) check that overall state is ready (or just state of the machine?)
            # 2.) check that plate is placed on l

        
        elif request.action_handle == "unload_plate": 
            pass 

        

        



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
        msg.data = 'State: %s' % self.machine_state
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
