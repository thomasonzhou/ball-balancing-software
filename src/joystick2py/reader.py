"""Simple move with absolute positions"""
import math
import serial
from joystick2py.joystick_serial import joystick_decode, JoystickMode

MAX_PLATFORM_TILT_RAD = math.pi / 12.0
JOYSTICK_MAX_SCALING_FACTOR = math.sqrt(2.0)


def read_arduino_joystick(joystick_serial: serial.Serial) -> tuple[float, float, float]:
    """Translate x and y joystick position on a scale of -1.0 to 1.0 into plate values"""

    res = None
    while res is None:
        res = joystick_decode(joystick_serial, JoystickMode.JOYSTICK_AS_TILT_VECTOR)
    x, y = res

    scaling_factor = math.sqrt(x * x + y * y)
    theta_rad = MAX_PLATFORM_TILT_RAD * (scaling_factor / JOYSTICK_MAX_SCALING_FACTOR)

    scaled_x, scaled_y = x / scaling_factor, y/scaling_factor
    return scaled_x, scaled_y, theta_rad


def read_wasd() -> tuple[tuple[float, float], float]:
    """Get WASD and arrow key inputs, print key presses, and return direction and tilt."""
    raise NotImplementedError
    dir_x, dir_y = 0.0, 0.0
    theta_rad = 0.0
    theta_rad = math.sqrt(dir_x**2 + dir_y**2) * MAX_PLATFORM_TILT_RAD
    return (dir_x, dir_y), theta_rad


if __name__ == "__main__":
    pass
