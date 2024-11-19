"""Arduino flashed as ball position"""

import serial
import re
from enum import Enum
from typing import Optional

class JoystickMode(Enum):
    JOYSTICK_AS_BALL_POSITION = 1
    JOYSTICK_AS_TILT_VECTOR = 2

# Expected ASCII data in the format <x,y>
BALL_COORDINATE_FORMAT = re.compile(r"<(-?\d+\.\d+),\s*(-?\d+\.\d+)>") # in centimeters

def parse_coord(data):
    match = BALL_COORDINATE_FORMAT.match(data)
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        return x, y
    else:
        return None

def joystick_decode(joystick_serial: serial.Serial, joystick_mode: JoystickMode) -> Optional[tuple[float, float]]:
    """Receive a packet with the shape <x, y> representing either ball position or tilt vector"""
    if joystick_serial.in_waiting > 0:
        data = joystick_serial.read(joystick_serial.in_waiting)
        try:
            decoded_data = data.decode('ascii').strip()
            res = parse_coord(decoded_data)
            if res is None:
                return None
            return res
        except UnicodeDecodeError:
            print("non-ASCII data") 
        return x, y
    else:
        return None

if __name__ == "__main__":
    DEFAULT_PORT = "/dev/cu.usbmodem1301"
    BAUD_RATE = 115200
    default_ser = serial.Serial(DEFAULT_PORT, BAUD_RATE, timeout=1)
    while True:
        xy = joystick_decode(default_ser, JoystickMode.JOYSTICK_AS_TILT_VECTOR)
        if xy:
            x, y = xy
            print(f"Joystick x: {x} y: {y}")
        