"""REST-based client for the Liconic"""
import json
import os
import time
from argparse import ArgumentParser
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import FastAPI
from fastapi.datastructures import State
from fastapi.responses import JSONResponse
from stx import Stx
from wei.modules.rest_module import RESTModule
from wei.types import (
    ModuleStatus,
    ModuleState,
    StepResponse,
    StepStatus,
)

from liconic_interface.resource_tracker import Resource

liconic_module = RESTModule(
    name="liconic_node",
    description="A module for controlling a Liconic STX incubator",
    port=2010,
)

liconic_module.arg_parser.add_argument(
    "--device",
    type=str,
    default="/dev/ttyUSB0",
    help="Serial device for communicating with the device",
)
liconic_module.arg_parser.add_argument(
    "--resources_path",
    type=Path,
    default="~/liconic_temp/resources/liconic_resources.json",
    help="Path to resources directory",
)

# TODO: REMOVE
global status, module_resources, liconic


@liconic_module.startup()
def liconic_startup(state: State):
    state.liconic = None
    state.module_resource = None
    state.liconic = Stx(state.device)
    state.resources_path = Path(state.resources_path).expanduser().resolve()
    state.module_resource = Resource(state.resources_path)

@liconic_module.state_handler()
def liconic_state_handler(state: State):
    if state.status == ModuleStatus.IDLE:
        if state.liconic.ready:
            state.status = ModuleStatus.IDLE
        else:
            if state.liconic.has_error:
                state.status = ModuleStatus.ERROR
            else:
                state.status = ModuleStatus.BUSY
    return ModuleState(
        status=state.status,
        error=state.error,
        current_temperature=state.liconic.climate_controller.current_temperature,
        target_temperature=state.liconic.climate_controller.target_temperature,
        current_humidity=state.liconic.climate_controller.current_humidity,
        target_humidity=state.liconic.climate_controller.target_humidity,
    )

@liconic_module.action()
def get_current_temp(state: State) -> StepResponse:
    """Returns the current temperature of the incubator"""
    return StepResponse.step_succeeded(state.liconic.climate_controller.current_temperature)

@liconic_module.action()
def get_target_temp(state: State) -> StepResponse:
    """Returns the target temperature of the incubator"""
    return StepResponse.step_succeeded(state.liconic.climate_controller.target_temperature)

@liconic_module.action()
def set_target_temp(state: State, temp: float) -> StepResponse:
    """Sets the target temperature of the incubator"""
    try:
        liconic.climate_controller.target_temperature = temp
        return StepResponse.step_succeeded(f"Set temperature to {temp}")
    except ValueError:
        error_msg = "Error: temp argument must be a float"
        print(error_msg)
        return StepResponse.step_failed(error_msg)

@liconic_module.action()
def get_current_humidity(state: State) -> StepResponse:
    """Returns the current humidity of the incubator"""
    return StepResponse.step_succeeded(state.liconic.climate_controller.current_humidity)

@liconic_module.action()
def get_target_humidity(state: State) -> StepResponse:
    """Returns the target humidity of the incubator"""
    return StepResponse.step_succeeded(state.liconic.climate_controller.target_humidity)

@liconic_module.action()
def set_target_humidity(state: State, humidity: float) -> StepResponse:
    """Sets the target humidity of the incubator"""
    try:
        liconic.climate_controller.target_temperature = humidity
        return StepResponse.step_succeeded(f"Set humidity to {humidity}")
    except ValueError:
        error_msg = "Error: humidity argument must be a float"
        print(error_msg)
        return StepResponse.step_failed(error_msg)

@liconic_module.action()
def begin_shake(state: State, shaker_speed: int):
    """Activate the shaker in the liconic at the specified 'shaker_speed'"""
    try:
        liconic = state.liconic
        if liconic.shaker_controller.shaker_is_active:
            if (
                not shaker_speed
                == liconic.shaker_controller.shaker_speed
            ):
                """already shaking but not at the desired speed"""
                # stop shaking
                liconic.shaker_controller.stop_shaker()
                # set shaking speed to new value
                liconic.shaker_controller.shaker_speed = shaker_speed
                # restart shaking at new speed
                liconic.shaker_controller.activate_shaker()
        else:
            """not already shaking"""
            # set shaking speed to new value (regardless of if already set to new value)
            liconic.shaker_controller.shaker_speed = shaker_speed
            # start shaking
            liconic.shaker_controller.activate_shaker()
        return StepResponse.step_succeeded(f"Liconic shaker activated, shaker speed: {liconic.shaker_controller.shaker_speed}")
    except ValueError:
        error_msg = "Error: shaker_speed argument must be an int"
        print(error_msg)
        return StepResponse.step_failed(error_msg)

@liconic_module.action()
def end_shake(state: State):
    """Stop the liconic's shaker"""
    liconic.shaker_controller.stop_shaker()
    return StepResponse.step_succeeded(f"Liconic shaker stopped")

@liconic_module.action()
def load_plate(state: State,  plate_id: str, stacker: Optional[int] = None, slot: Optional[int] = None,):
    """Load a plate into the incubator"""
    try:
        if stacker is None or slot is None:
            stacker, slot = state.module_resources.get_next_free_slot_int()
        if state.module_resources.is_location_occupied(stacker, slot):
            return StepResponse.step_failed("load_plate command cannot be completed, already plate in given position")
        else:
            state.liconic.plate_handler.move_plate_from_transfer_station_to_slot(
                stacker, slot
            )
            time.sleep(20)
            state.module_resources.add_plate(plate_id, stacker, slot)
            return StepResponse.step_succeeded(
                f"Plate loaded into liconic stack {stacker}, slot {slot}"
            )
    except ValueError:
        return StepResponse.step_failed("Error: stacker and slot variables must be integers; plate_id required")

@liconic_module.action()
def unload_plate(state: State,  plate_id: str, stacker: Optional[int] = None, slot: Optional[int] = None,):
    """Unload a plate from the incubator"""
    try:
        if stacker == None or slot == None:
            # get location based on plate id
            stacker, slot = module_resources.find_plate(plate_id)

            stacker, slot = module_resources.convert_stack_and_slot_int(
                stacker, slot
            )
        if state.module_resources.is_location_occupied(stacker, slot):
            if plate_id is None:
                plate_id = module_resources.get_plate_id(stacker, slot)
            liconic.plate_handler.move_plate_from_slot_to_transfer_station(
                stacker, slot
            )   
            if liconic.plate_handler.transfer_station_occupied:
                print("Liconic transfer station occupied")
            else:
                print("Liconic transfer station empty")
            time.sleep(18)
            module_resources.remove_plate(plate_id)
            return StepResponse.step_succeeded(
                f"Plate unloaded from liconic stack {stacker}, slot {slot}"
            )
        else:
            return StepResponse.step_failed("No plate in location, can't unload.")
    except ValueError:
        return StepResponse.step_failed("Error: stacker and slot variables must be integers; plate_id required")

def check_resources_folder(resources_folder_path):
    """
    checks if resource file path exists, if not, creates one
    """
    if not os.path.exists(resources_folder_path):
        os.makedirs(resources_folder_path)
        print("Creating: " + str(resources_folder_path))

        # create json file within directory
        new_resources = create_resource_file()
        with open(resources_folder_path / "liconic_resources.json", "w") as f:
            json.dump(new_resources, f)


def create_resource_file():
    """
    if resource file does not exist, creates a blank one
    """
    resources = {}
    for stack in range(4):
        curr_stack = "Stack" + str(stack + 1)
        print(curr_stack)
        slot_dict = {}
        for slot in range(22):
            curr_slot = "Slot" + str(slot + 1)
            slot_dict[curr_slot] = {
                "occupied": False,
                "time_added": "0",
                "plate_id": "NONE",
            }

        resources[curr_stack] = slot_dict

    return resources


if __name__ == "__main__":

    liconic_module.start()