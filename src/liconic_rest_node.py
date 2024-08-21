"""REST-based client for the Liconic"""

import time
from pathlib import Path
from typing import Optional

from fastapi.datastructures import State
from liconic_interface import Stx
from liconic_interface.resource_tracker import ResourceTracker
from wei.modules.rest_module import RESTModule
from wei.types import StepResponse
from wei.types.module_types import ModuleState
from wei.types.step_types import StepSucceeded

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
    default=Path.home() / "liconic_temp/resources/liconic_resources.yaml",
    help="Path to resources directory",
)


@liconic_module.startup()
def liconic_startup(state: State):
    """Handles initializing the Liconic STX object and resources"""
    state.liconic = None
    state.module_resources = None
    state.liconic = Stx(state.device)
    state.resources_path = Path(state.resources_path).expanduser().resolve()
    state.module_resources = ResourceTracker(state.resources_path)


@liconic_module.shutdown()
def liconic_shutdown(state: State):
    """Handles cleaning up the Liconic STX object"""
    if state.liconic is not None:
        del state.liconic


@liconic_module.state_handler()
def liconic_state_handler(state: State):
    """Returns the state of the Liconic device and module"""
    liconic: Optional[Stx] = state.liconic
    if liconic is None:
        return ModuleState(
            status=state.status,
            error=state.error,
        )
    if liconic.is_busy:
        return ModuleState(
            status=state.status,
            error=state.error,
            current_temperature=state.cached_current_temperature,
            target_temperature=state.cached_target_temperature,
            current_humidity=state.cached_current_humidity,
            target_humidity=state.cached_target_humidity,
            shovel_occupied=state.cached_shovel_occupied,
            transfer_station_occupied=state.cached_transfer_station_occupied,
            transfer_station_2_occupied=state.cached_transfer_station_2_occupied,
        )
    else:
        state.cached_current_temperature = liconic.current_temperature
        state.cached_target_temperature = liconic.target_temperature
        state.cached_current_humidity = liconic.current_humidity
        state.cached_target_humidity = liconic.target_humidity
        state.cached_shovel_occupied = liconic.shovel_occupied
        state.cached_transfer_station_occupied = liconic.transfer_station_occupied
        state.cached_transfer_station_2_occupied = liconic.transfer_station_2_occupied
        return ModuleState(
            status=state.status,
            error=state.error,
            current_temperature=liconic.current_temperature,
            target_temperature=liconic.target_temperature,
            current_humidity=liconic.current_humidity,
            target_humidity=liconic.target_humidity,
            shovel_occupied=liconic.shovel_occupied,
            transfer_station_occupied=liconic.transfer_station_occupied,
            transfer_station_2_occupied=liconic.transfer_station_2_occupied,
        )


@liconic_module.action()
def set_target_temp(state: State, temp: float) -> StepResponse:
    """Sets the target temperature of the incubator"""
    liconic: Stx = state.liconic
    try:
        liconic.target_temperature = float(temp)
        return StepSucceeded()
    except ValueError:
        error_msg = "Error: temp argument must be a float"
        print(error_msg)
        return StepResponse.step_failed(error_msg)


@liconic_module.action()
def set_target_humidity(state: State, humidity: float) -> StepResponse:
    """Sets the target humidity of the incubator"""
    liconic: Stx = state.liconic
    liconic.target_humidity = float(humidity)
    return StepSucceeded()


@liconic_module.action()
def begin_shake(state: State, shaker_speed: int):
    """Activate the shaker in the liconic at the specified 'shaker_speed'"""
    liconic: Stx = state.liconic
    if not shaker_speed == int(liconic.shaker_speed):
        """already shaking but not at the desired speed"""
        liconic.shaker_active = False
        liconic.shaker_speed = int(shaker_speed)
    liconic.shaker_active = True
    return StepSucceeded()


@liconic_module.action()
def end_shake(state: State):
    """Stop the liconic's shaker"""
    liconic: Stx = state.liconic
    liconic.shaker_active = False
    return StepSucceeded()


@liconic_module.action()
def load_plate(
    state: State,
    plate_id: str,
    stacker: Optional[int] = None,
    slot: Optional[int] = None,
):
    """Load a plate into the incubator"""
    liconic: Stx = state.liconic
    module_resources: ResourceTracker = state.module_resources
    if stacker is None or slot is None:
        stacker, slot = module_resources.get_next_free_slot()
    else:
        stacker = int(stacker)
        slot = int(slot)
    if module_resources.is_location_occupied(stacker, slot):
        return StepResponse.step_failed(
            "load_plate command cannot be completed, already plate in given position"
        )
    if not liconic.transfer_station_occupied:
        return StepResponse.step_failed(
            "load_plate command cannot be completed, no plate in transfer station"
        )
    liconic.load_plate(stacker, slot)
    while liconic.is_busy:
        time.sleep(1)
    module_resources.add_plate(plate_id, stacker, slot)
    return StepSucceeded()


@liconic_module.action()
def unload_plate(
    state: State,
    plate_id: Optional[str] = None,
    stacker: Optional[int] = None,
    slot: Optional[int] = None,
):
    """Unload a plate from the incubator"""
    liconic: Stx = state.liconic
    module_resources: ResourceTracker = state.module_resources
    if stacker is None or slot is None and plate_id is not None:
        # * Get location based on plate id
        stacker, slot = module_resources.find_plate(plate_id)
    else:
        stacker = int(stacker)
        slot = int(slot)
    if liconic.transfer_station_occupied:
        return StepResponse.step_failed(
            "Transfer station occupied, please clear it before unloading."
        )
    if not module_resources.is_location_occupied(stacker, slot):
        if not liconic.slot_occupied(stacker, slot):
            return StepResponse.step_failed("No plate in location, can't unload.")
    if plate_id is None:
        try:
            plate_id = module_resources.get_plate_id(stacker, slot)
        except Exception:
            pass
    liconic.unload_plate(stacker, slot)
    while liconic.is_busy:
        time.sleep(1)
    if liconic.transfer_station_occupied:
        if plate_id:
            module_resources.remove_plate(plate_id=plate_id, stack=stacker, slot=slot)
        return StepSucceeded()
    else:
        return StepResponse.step_failed(
            f"Failed to unload plate from liconic stack {stacker}, slot {slot}"
        )


if __name__ == "__main__":
    liconic_module.start()
