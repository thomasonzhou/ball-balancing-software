import serial
from enum import Enum
import re

DATA_START = '<'
DATA_END = '>'
DELIMITER = ','

TARE_CHAR = 't'
TARE_CONFIRM = "TARE"
ABSOLUTE_MOVE_CHAR = 'a'
RELATIVE_MOVE_CHAR = 'r'

format = re.compile(r"<(-?\d+\.\d+),\s*(-?\d+\.\d+)>")

# A class for devices that are communicating over serial. Eg, Joystick, or Motor Controller can be made instances of the SerialDevice class.
class SerialDevice():
    class DeviceType(Enum):
        JOYSTICK = 1
        MOTOR_CONTROLLER = 2
    class _Command(Enum):
        TARE = 1
        RELATIVE_MOVE = 2
        ABSOLUTE_MOVE = 3
    # Constructor: sets the COM port and Speed
    def __init__(self, port:str, speed:int, deviceType:DeviceType):
        self._port = port
        self._speed = speed
        self.type = deviceType
        # Create serial object
        self._serial = serial.Serial(port, speed, timeout=10)
        # If device is a joystick
        if self.type == self.DeviceType.JOYSTICK:
            self.joystickX = 0
            self.joystickY = 0
        # If device is motor controller:
        if self.type == self.DeviceType.MOTOR_CONTROLLER:
            self.tared = False
    # Tare command
    def tare(self, rad1:float, rad2:float, rad3:float)->bool:
        # Only run function if the device is a motor controller
        if self.type != self.DeviceType.MOTOR_CONTROLLER:
            return False
        # Send the command
        command = self._makeCommand(self._Command.TARE, rad1, rad2, rad3)
        self._serial.write(command)
        # Verify that the tare confirm string has been received
        if self._serial.in_waiting > 0:
            ack = self._serial.read(self._serial.in_waiting)
            if ack.find(TARE_CONFIRM) >= 0:
                self.tared = True
        return self.tared
    # Relative move command
    def relativeMove(self, rad1:float, rad2:float, rad3:float)->bool:
        # Only run function if the device is a motor controller
        if self.type != self.DeviceType.MOTOR_CONTROLLER:
            return False
        # Send the command
        command = self._makeCommand(self._Command.RELATIVE_MOVE, rad1, rad2, rad3)
        self._serial.write(command)
        return True
    # Absolute move command
    def absoluteMove(self, rad1:float, rad2:float, rad3:float)->bool:
        # Only run function if the device is a motor controller 
        if self.type != self.DeviceType.MOTOR_CONTROLLER:
            return False
        # Send the command
        command = self._makeCommand(self._Command.ABSOLUTE_MOVE, rad1, rad2, rad3)
        self._serial.write(command)
        # If we receive an error message, then return false
        if self._serial.in_waiting > 0:
            self._serial.reset_input_buffer()
            return False
        return True
    # Get joystick angle
    def getJoystickValues(self)->float:
        if self._serial.in_waiting > 0:
            data = self._serial.read(self._serial.in_waiting).decode('ascii').strip()
            x, y = self._parseCoord(data)
            if x is not None and y is not None:
                self.joystickX = x
                self.joystickY = y
        return self.joystickX, self.joystickY
    # Private function that creates command strings in the proper format
    def _makeCommand(self, command:_Command, rad1:float, rad2:float, rad3:float):
        # Add data start character
        cmdStr = DATA_START
        # Add command character
        match command:
            case self._Command.TARE:
                cmdStr += TARE_CHAR
            case self._Command.RELATIVE_MOVE:
                cmdStr += RELATIVE_MOVE_CHAR
            case self._Command.ABSOLUTE_MOVE:
                cmdStr += ABSOLUTE_MOVE_CHAR
        # Add three radian values
        cmdStr += "," + rad1
        cmdStr += "," + rad2
        cmdStr += "," + rad3
        # Add data end character
        cmdStr += ">"
        return cmdStr
    # Private function that parses x and y values from joystick
    def _parseCoord(data):
        match = format.match(data)
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            return x, y
        else:
            print("Data format error")
            return None