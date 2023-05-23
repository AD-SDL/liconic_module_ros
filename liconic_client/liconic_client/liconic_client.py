#! /usr/bin/env python3

import os
import stx                  # import liconic driver
from stx import Stx
from pathlib import Path
import json

import rclpy                 # import Rospy
from rclpy.node import Node  # import Rospy Node

from wei_services.srv import WeiDescription  # import workcell execution interface (wei)
from wei_services.srv import WeiActions   

from std_msgs.msg import String         # other imports
from typing import List, Tuple
from time import sleep
from liconic_driver.resource_tracker import Resource


class liconicNode(Node):
    '''
    The liconicNode inputs data from the 'action' topic, providing a set of commands for the driver to execute. It then receives feedback, 
    based on the executed command and publishes the state of the liconic and a description of the liconic to the respective topics.
    '''
    def __init__(self, PORT='/dev/ttyUSB3', NODE_NAME = "Liconic_Client_Node"):
        '''
        The init function is neccesary for the liconicNode class to initialize all variables, parameters, and other functions.
        Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
        '''

        super().__init__(NODE_NAME)
        self.node_name = self.get_name()

        self.declare_parameter("port",PORT)

        # Receiving the real PORT from the launch parameters
        self.port =  self.get_parameter("port").get_parameter_value().string_value
        
        self.liconic = Stx(self.port)
        
        self.resources = Resource()

        self.resources_folder_path = '/home/rpl/liconic_temp/resources/'  # TODO: path to folder or path to direct file?
        # self.check_resources_folder()

        self.description = {
            'name': NODE_NAME,
            'type': 'liconic_incubator',
            'actions':
            {
                'status':'',  # TODO: should status be included here if it's a property?
                'get_current_temp':'',
                'get_target_temp':'',
                'set_target_temp':'temp',
                'get_current_humidity':'',
                'get_target_humidity':'',
                'set_target_humidity':'humidity',
                'begin_shake':'shaker_speed',
                'end_shake':'',
                'load_plate':'stacker, slot',
                'unload_plate':'stacker, slot',
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

        # check robot state, if ready proceede to action
        while self.machine_state != "READY":
            self.get_logger().warn("Waiting for LiCONiC Incubator to switch to READY state...") 
            sleep(0.2)
            # TODO: should there be a timeout here?

        # Temperature control actions
        if request.action_handle == "get_current_temp": 
            response.action_response = 0
            response.action_msg = str(self.liconic.climate_controller.current_temperature)
        
        elif request.action_handle == "get_target_temp":
            response.action_response = 0
            response.action_msg = str(self.liconic.climate_controller.target_temperature)
        
        elif request.action_handle == "set_target_temp":
            # collect variables 
            try:   
                vars = eval(request.vars)
                temp = float(vars.get('temp'))   # must be a float 
            except ValueError as error_msg:
                response.action_response = -1
                response.action_msg = "Error: temp variable must be a float"
                self.get_logger().error("------- Liconic Error message: " + str(error_msg) +  (" -------"))
            
            # complete action if no variable type exception thrown
            else: 
                try: 
                    self.liconic.climate_controller.target_temperature = temp
                    response.action_response = 0
                    response.action_msg = "Target temperature = " + str(self.liconic.climate_controller.target_temperature)
                except Exception as error_msg:
                    print("TODO: handle exception")
                    response.action_response = -1
                    response.action_msg = "Error: Liconic could not set target temperature"
                    self.get_logger().error("------- Liconic Error message: " + str(error_msg) +  (" -------"))
                    
        elif request.action_handle == "get_current_humidity": 
            response.action_response = 0
            response.action_msg = str(self.liconic.climate_controller.current_humidity)
        
        elif request.action_handle == "get_target_humidity":
            response.action_response = 0
            response.action_msg = str(self.liconic.climate_controller.target_humidity)

        elif request.action_handle == "set_target_humidity": 
            # collect variables 
            try:         
                vars = eval(request.vars)
                humidity = float(vars.get('humidity'))  
            except ValueError as error_msg: 
                response.action_response = -1
                response.action_msg = "Error: humidity variable must be a float"
                self.get_logger().error("------- Liconic Error message: " + str(error_msg) +  (" -------"))

            # complete action if no variable type exception thrown
            else: 
                try: 
                    self.liconic.climate_controller.target_humidity = humidity
                    response.action_response = 0
                    response.action_msg = "Target humidity = " + str(self.liconic.climate_controller.target_humidity)
                except Exception as error_msg: 
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
            # Timed shake (but don't block the thread while waiting) 
        elif request.action_handle == "begin_shake":
            # TODO: make shaker speed a client variable so it doesn't have to be specified each time 
            # collect variables 
            try: 
                vars = eval(request.vars)
                new_shaker_speed = int(vars.get('shaker_speed')) 
            except ValueError as error_msg: 
                response.action_response = -1
                response.action_msg = "Error: shaker speed variable is invalid. Must be an integer 1 to 50 inclusive."
                self.get_logger().error("------- Liconic Error message: " + str(error_msg) +  (" -------"))
            else: 
                # check that shaker speed int is 1 to 50 inclusive 
                if new_shaker_speed < 1 or new_shaker_speed > 50: 
                    response.action_response = -1
                    response.action_msg = "Error: shaker speed variable is invalid. Must be an integer 1 to 50 inclusive."
                    self.get_logger().error("------- Liconic Error message: " + str(error_msg) +  (" -------"))
                else: # shaker speed variable is valid 
                    try: 
                        if self.liconic.shaker_controller.shaker_is_active == True:
                            if not new_shaker_speed == self.liconic.shaker_controller.shaker_speed:
                                """already shaking but not at the desired speed""" 
                                # stop shaking 
                                self.liconic.shaker_controller.stop_shaker()
                                # set shaking speed to new value 
                                self.liconic.shaker_controller.shaker_speed = new_shaker_speed
                                # restart shaking at new speed 
                                self.liconic.shaker_controller.activate_shaker()
                                # TODO: 
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
                            # TODO: check that liconic is now shaking?
                            # format returns 
                            response.action_response = 0
                            response.action_msg = "Liconic shaker activated, shaker speed: " + str(self.liconic.shaker_controller.shaker_speed)
                        else: 
                            """catch for any other liconic shaking states"""  # maybe remove this else statement
                            response.action_response = -1
                            response.action_msg = "Error: Liconic could not begin shake"
                    
                    except Exception as error_msg: 
                        response.action_response = -1
                        response.action_msg = "Error: Liconic could not begin shake"
                        self.get_logger().error("------- Liconic Error message: " + str(error_msg) +  (" -------"))

        elif request.action_handle == "end_shake":
            self.liconic.shaker_controller.stop_shaker()
            # TODO: check that shaker did stop with shaker_is_active driver property
            response.action_response = 0
            response.action_msg = "Liconic shaker stopped"


        # Plate handling actions 
            # TODO: implement resouce tracking and way to visualize those resources
            # resource_file_flag = self.action_vars.get("use_existing_resources", "False")

        elif request.action_handle == "load_plate":
            try:
                vars = eval(request.vars)
                stacker = vars.get('stacker', None)
                slot = vars.get('slot', None)
                plate_id = vars.get('plate_id') # TODO: default plate id value?
            except ValueError as error_msg: 
                response.action_response = -1
                response.action_msg = "Error: stacker and slot variables must be integers"
                self.get_logger().error("------- Liconic Error message: " + str(error_msg) +  (" -------"))
            else:
                if stacker == None and slot == None: # TODO: What if user gives certain stack but not slot?
                    stacker, slot = self.resources.get_next_free_slot_int()
                # TODO: check that valid stack and slot numbers were chosen
                else:
                    stacker = int(stacker)
                    slot = int(slot)
                if self.resources.is_location_occupied(stacker, slot) == True:
                    self.get_logger().error("load_plate command cannot be completed, already plate in given position")
                else:
                    # complete action if no other exceptions raised
                    try: 
                        # check that transfer station is occupied
                        if not self.liconic.plate_handler.transfer_station_occupied: 
                            response.acttion_response = -1 
                            response.action_msg = "Error: cannot load liconic. No plate on transfer station"
                        else: 
                            self.liconic.plate_handler.move_plate_from_transfer_station_to_slot(stacker, slot)
                            sleep(20)  # wait for action to finish 
                            response.action_response = 0
                            response.action_msg = "Plate loaded into liconic stack " + str(stacker) + ", slot " + str(slot)
                            # edit resource file
                            self.get_logger().info("Updating liconic resource file")
                            self.resources.add_plate(plate_id, stacker, slot)
                    except Exception as error_msg: 
                        response.action_response = -1
                        response.action_msg = "Error: Liconic could not load plate"
                        self.get_logger().error("------- Liconic Error message: " + str(error_msg) +  (" -------"))

        elif request.action_handle == "unload_plate": 
            try: 
                vars = eval(request.vars)
                stacker = vars.get('stacker', None)
                slot = vars.get('slot', None)
                plate_id = vars.get('plate_id') # TODO: default plate id value?
            except ValueError as error_msg: 
                response.action_response = -1
                response.action_msg = "Error: stacker and slot variables must be integers"
                self.get_logger().error("------- Liconic Error message: " + str(error_msg) +  (" -------"))
            else:
                if stacker == None and slot == None:
                    # get location based on plate id
                    stacker, slot = self.resources.find_plate(plate_id)

                    stacker, slot = self.resources.convert_stack_and_slot_int(stacker, slot)
                else:
                    stacker = int(stacker)
                    slot = int(slot)
                
                if self.resources.is_location_occupied(stacker, slot) == False:
                    self.get_logger().error("unload_plate command cannot be completed, no plate in given position")

                # complete action if no other exceptions were raised 
                try: 

                    # check that transfer station is not already occupied
                    if not self.liconic.plate_handler.transfer_station_occupied: 
                        # complete action
                        self.liconic.plate_handler.move_plate_from_slot_to_transfer_station(stacker, slot)
                        sleep(18) # wait for plate to unload 
                        # log if plate is now on transfer station 
                        if self.liconic.plate_handler.transfer_station_occupied: 
                            self.get_logger().info("Liconic transfer station occupied")
                        else: 
                            self.get_logger().info("Liconic transfer station empty")
                        # format response
                        response.action_response = 0
                        response.action_msg = "Plate unloaded from liconic stack " + str(stacker) + ", slot " + str(slot)
                        self.resources.remove_plate(plate_id)
                    else: # transfer station already occupied 
                        response.action_response = -1
                        response.action_msg = "Error: Liconic cannot unload plate, transfer station is occupied"

                except Exception as error_msg: 
                    response.action_response = -1
                    response.action_msg = "Error: Liconic could not unload plate"
                    self.get_logger().error("------- Liconic Error message: " + str(error_msg) +  (" -------"))

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

    def check_resources_folder(self):
        '''
        checks if resource file path exists, if not, creates one
        '''
        isPathExist = os.path.exists(self.resources_folder_path)
        if not isPathExist:
            os.makedirs(self.resources_folder_path)
            self.get_logger().warn("Resource path doesn't exists")
            self.get_logger().info("Creating: " + self.resources_folder_path)

        # create json file within directory
        new_resources = self.resources.create_resource_file()
        with open(self.resources_folder_path+'liconic_resources.json', 'w') as f:
            json.dump(new_resources, f)


def main(args = None):
    NAME = "liconicNode"
    rclpy.init(args=args)  # initialize Ros2 communication
    node = liconicNode(NODE_NAME=NAME)
    rclpy.spin(node)     # keep Ros2 communication open for action node
    rclpy.shutdown()     # kill Ros2 communication

if __name__ == '__main__':
    main()
