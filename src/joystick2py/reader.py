import math

MAX_PLATFORM_TILT_RAD = math.pi / 12.0
JOYSTICK_MAX_SCALING_FACTOR = math.sqrt(2.0)


def read_arduino_joystick() -> tuple[tuple[float, float], float]:
    """Translate x and y joystick position on a scale of -1.0 to 1.0 into plate values"""

    raise NotImplementedError
    # from serial (stub)
    input_line = "1.0, 1.0"

    raw_xy = tuple(float(coord.strip()) for coord in input_line.split(","))

    scaling_factor = math.sqrt(sum(val * val for val in raw_xy))
    theta_rad = MAX_PLATFORM_TILT_RAD * (scaling_factor / JOYSTICK_MAX_SCALING_FACTOR)

    scaled_xy = tuple(val / scaling_factor for val in raw_xy)
    return scaled_xy, theta_rad


def read_wasd() -> tuple[tuple[float, float], float]:
    """Get WASD and arrow key inputs, print key presses, and return direction and tilt."""
    raise NotImplementedError
    dir_x, dir_y = 0.0, 0.0
    theta_rad = 0.0
    theta_rad = math.sqrt(dir_x**2 + dir_y**2) * MAX_PLATFORM_TILT_RAD
    return (dir_x, dir_y), theta_rad


if __name__ == "__main__":
    pass
