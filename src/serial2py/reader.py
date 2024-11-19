"""Simple move with absolute positions"""

import math
import serial
from serial2py.joystick_serial import joystick_decode

MAX_PLATFORM_TILT_RAD = math.pi / 12.0  # 15 degrees
JOYSTICK_MAX_SCALING_FACTOR = math.sqrt(2.0)
JOYSTICK_STEADY_STATE_X = 0.01
JOYSTICK_STEADY_STATE_Y = 0.02


def correct_steady_state_error(dir_x, dir_y):
    if -JOYSTICK_STEADY_STATE_X <= dir_x <= JOYSTICK_STEADY_STATE_X:
        dir_x = 0.0
    if -JOYSTICK_STEADY_STATE_Y <= dir_y <= JOYSTICK_STEADY_STATE_Y:
        dir_y = 0.0
    return dir_x, dir_y


def map_square_to_circle(square_x, square_y):
    """Remaps a square of inputs from -1 to 1 to a unit circle"""
    dir_x = square_x * math.sqrt(1.0 - ((square_y**2) / 2.0))
    dir_y = square_y * math.sqrt(1.0 - ((square_x**2) / 2.0))
    return dir_x, dir_y


def read_arduino_joystick(joystick_serial: serial.Serial) -> tuple[float, float, float]:
    """Translate x and y joystick position on a scale of -1.0 to 1.0 into plate values"""

    res = None
    while res is None:
        res = joystick_decode(joystick_serial)
    square_x, square_y = res

    dir_x, dir_y = map_square_to_circle(square_x, square_y)
    dir_x, dir_y = correct_steady_state_error(dir_x, dir_y)

    theta_rad = MAX_PLATFORM_TILT_RAD * math.sqrt(dir_x**2 + dir_y**2)
    return dir_x, dir_y, theta_rad


def read_wasd() -> tuple[tuple[float, float], float]:
    """Get WASD and arrow key inputs, print key presses, and return direction and tilt."""
    raise NotImplementedError
    dir_x, dir_y = 0.0, 0.0
    theta_rad = 0.0
    theta_rad = math.sqrt(dir_x**2 + dir_y**2) * MAX_PLATFORM_TILT_RAD
    return (dir_x, dir_y), theta_rad


if __name__ == "__main__":
    pass
