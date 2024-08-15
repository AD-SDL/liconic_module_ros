"""Liconic Interface for interacting with the STX library"""

from ctypes import *
from pathlib import Path
import threading

stx = CDLL(Path(__file__).parent / "libstxlib.so")

serial_device = c_char_p(b"/dev/ttyUSB0")


class Stx:
    
    lib = CDLL(Path(__file__).parent / "libstxlib.so")
    
    def __init__(self, device="/dev/ttyUSB0"):
        self.lock = threading.Lock()
        self.device = device
        self.connect_and_initialize(self.device)
    
    def __del__(self):
        self.lib.Disconnect(self.id)
        print("Disconnected from Liconic")
    
    
    def connect_and_initialize(self, device="/dev/ttyUSB0"):
        with self.lock:
            self.id = self.lib.Connect(c_char_p(bytes(device, encoding="utf-8")))
            if self.id < 0:
                raise Exception(f"Failed to connect to Liconic with error code: {self.id}")
            print("Connected to Liconic")
            is_initialized = c_bool()
            self.lib.ReadIsInitialized(self.id, pointer(is_initialized))
            if not is_initialized:
                print("Liconic not initialized, initializing...")
                self.lib.Initialize(self.id)
                print("Liconic initialized.")
    
    @property
    def current_temperature(self):
        with self.lock:
            temp = c_float()
            result = self.lib.ReadTemperature(self.id, pointer(temp))
            if result < 0:
                raise Exception(f"Error reading temperature: {result}")
            return temp.value
    
    @property
    def target_temperature(self):
        with self.lock:
            temp = c_float()
            result = self.lib.ReadSetTemperature(self.id, pointer(temp))
            if result < 0:
                raise Exception(f"Error reading target temperature: {result}")
            return temp.value
    
    @target_temperature.setter
    def set_temperature(self, temperature: float) -> bool:
        result = self.lib.SetTemperature(self.id, c_float(temperature))
        if result < 0:
            raise Exception(f"Error setting temperature: {result}")
        
    @property
    def current_temperature(self):
        with self.lock:
            temp = c_float()
            result = self.lib.ReadTemperature(self.id, pointer(temp))
            if result < 0:
                raise Exception(f"Error reading temperature: {result}")
            return temp.value
    
    @property
    def target_temperature(self):
        with self.lock:
            temp = c_float()
            result = self.lib.ReadSetTemperature(self.id, pointer(temp))
            if result < 0:
                raise Exception(f"Error reading target temperature: {result}")
            return temp.value
    
    @target_temperature.setter
    def set_temperature(self, temperature: float) -> bool:
        result = self.lib.SetTemperature(self.id, c_float(temperature))
        if result < 0:
            raise Exception(f"Error setting temperature: {result}")


if __name__ == "__main__":
    
    stx = Stx()
    print(stx.read_temperature())
    
    
    