"""Arduino flashed as ball position"""

import serial
import re

# Expected ASCII data in the format <x,y>
ball_coord_format = re.compile(r"<(-?\d+\.\d+),\s*(-?\d+\.\d+)>")

def parse_ball_coord(data):
    match = ball_coord_format.match(data)
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        return x, y
    else:
        # print("Data format error")
        return None

def joystick_as_ball_decode(joystick_serial: serial.Serial):
    if joystick_serial.in_waiting > 0:
        data = joystick_serial.read(joystick_serial.in_waiting)
        try:
            decoded_data = data.decode('ascii').strip()
            actual_coord = parse_coord(decoded_data)
            if actual_coord:
                print(f"RX COORD: {decoded_data}")
                return actual_coord
        except UnicodeDecodeError:
            print("non-ASCII data") 

if __name__ == "__main__":
    DEFAULT_PORT = "/dev/cu.usbmodem1301"
    BAUD_RATE = 115200
    default_ser = serial.Serial(DEFAULT_PORT, BAUD_RATE, timeout=1)
    while True:
        joystick_as_ball_decode(default_ser)
