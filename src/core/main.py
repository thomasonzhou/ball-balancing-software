"""Full integration of software components with sanity checks"""

import joystick2py
import inverse_kinematics
import py2motor
import pid

import math
from enum import Enum

PLATFORM_TILT_MIN_RAD = 0.0
PLATFORM_TILT_MAX_RAD = math.pi / 12.0

MOTOR_MIN_RAD = -math.pi / 2.0
MOTOR_MAX_RAD = math.pi / 2.0


class OperationMode(Enum):
    COMPUTER_VISION = 1
    WASD_JOYSTICK = 2
    ARDUINO_JOYSTICK = 3


def main(operation_mode=OperationMode.COMPUTER_VISION):
    # --------------------------------------------------
    # Initialize Components
    # --------------------------------------------------
    controller = pid.Controller()

    while True:
        match operation_mode:
            case OperationMode.COMPUTER_VISION:
                # ball detection
                # ball_position_camera_view = get_ball_position()
                # ball_position_plate_view = camera_view_to_plate_view()

                # set target position
                target_position = (0.0, 0.0)

                # find theta_rad, the angle of tilt

                # target_direction_vector = (target) - (curr)
                # (dir_x, dir_y), theta_rad = controller.get_val

                # assert direction vectors match
            case OperationMode.WASD_JOYSTICK:
                (dir_x, dir_y), theta_rad = joystick2py.read_wasd()
            case OperationMode.ARDUINO_JOYSTICK:
                dir_x, dir_y, theta_rad = joystick2py.read_arduino_joystick()
        # --------------------------------------------------
        # Inverse Kinematics
        # --------------------------------------------------

        # Sanity Check
        assert PLATFORM_TILT_MIN_RAD <= theta_rad <= PLATFORM_TILT_MAX_RAD

        # IK
        abs_motor_angles = inverse_kinematics.translate_dir_to_motor_angles(dir_x, dir_y, theta_rad)

        # Sanity Check
        for angle in abs_motor_angles:
            assert MOTOR_MIN_RAD <= angle <= MOTOR_MAX_RAD

        py2motor.write_to_motors(abs_motor_angles)


if __name__ == "__main__":
    main(operation_mode=OperationMode.ARDUINO_JOYSTICK)
