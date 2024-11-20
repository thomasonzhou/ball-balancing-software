"""Full integration of software components with sanity checks"""

import computer_vision
import motion_planner
import serial2py
import inverse_kinematics
import py2motor
import pid

import math
from enum import Enum
import serial
import time

HOMING_STRING = "HOME"

PLATFORM_TILT_MIN_RAD = 0.0
PLATFORM_TILT_MAX_RAD = math.pi / 12.0
MOTOR_MIN_DEG = -52.0
MOTOR_MAX_DEG = 52.0

ANGLE_REDUCTION_FACTOR = 2.0

MOTOR_MIN_RAD = math.radians(MOTOR_MIN_DEG)
MOTOR_MAX_RAD = math.radians(MOTOR_MAX_DEG)

ARDUINO_PORT = "/dev/ttyACM0"
MOTOR_CONTROLLER_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200


class OperationMode(Enum):
    COMPUTER_VISION = 1
    WASD_JOYSTICK = 2
    ARDUINO_JOYSTICK = 3


def main(operation_mode=OperationMode.COMPUTER_VISION):
    # --------------------------------------------------
    # Initialize Components
    # --------------------------------------------------
    controller = pid.Controller()
    planner = motion_planner.MotionPlanner()
    if operation_mode == OperationMode.COMPUTER_VISION:
        ball_detector = computer_vision.BallDetector(preview=True)

    # experimental trajectory
    # planner.load_square_trajectory()

    homing_completed = False
    if operation_mode == OperationMode.ARDUINO_JOYSTICK:
        arduino_serial = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    motor_serial = serial.Serial(MOTOR_CONTROLLER_PORT, BAUD_RATE, timeout=1)
    motor_serial.write("<h>\r\n".encode("ascii"))

    while not homing_completed:
        motor_serial.write("<h>\r".encode("ascii"))
        if motor_serial.in_waiting > 0:
            homing_string = motor_serial.read(size=4)
            received = homing_string.decode("ascii").strip()
            if HOMING_STRING in received:
                print("HOME detected")
                homing_completed = True
    py2motor.write_to_motors(motor_serial, (0, 0, 0))

    last_command_sent = time.time()
    try:
        while True:
            match operation_mode:
                case OperationMode.COMPUTER_VISION:
                    # ball detection
                    ball_position_plate_view = ball_detector.get_ball_position_plate_view()

                    # path planning
                    target_position_plate_view = planner.update_target(ball_position_plate_view)

                    dir_x, dir_y, theta_rad = controller.calculate(
                        desired_pos=target_position_plate_view,
                        actual_pos=ball_position_plate_view,
                    )
                case OperationMode.WASD_JOYSTICK:
                    (dir_x, dir_y), theta_rad = serial2py.read_wasd()
                case OperationMode.ARDUINO_JOYSTICK:
                    dir_x, dir_y, theta_rad = serial2py.read_arduino_joystick(arduino_serial)
            print(f"{dir_x:.2f}, {dir_y:.2f}, {math.radians(theta_rad):.5f}")

            assert (
                PLATFORM_TILT_MIN_RAD <= theta_rad <= PLATFORM_TILT_MAX_RAD
            ), f"theta_rad OOB: {theta_rad}"

            theta_rad /= ANGLE_REDUCTION_FACTOR

            # IK
            abs_motor_angles = inverse_kinematics.translate_dir_to_motor_angles(
                dir_x, dir_y, theta_rad
            )
            for angle in abs_motor_angles:
                assert MOTOR_MIN_RAD <= angle <= MOTOR_MAX_RAD, f"angle {angle} OOB"

            curr_time = time.time()
            if curr_time - last_command_sent >= 1:
                last_command_sent = curr_time
                py2motor.write_to_motors(motor_serial, abs_motor_angles)

    except KeyboardInterrupt:
        ball_detector.close_stream()
        if operation_mode == OperationMode.ARDUINO_JOYSTICK:
            arduino_serial.close()
        motor_serial.close()


if __name__ == "__main__":
    main(operation_mode=OperationMode.ARDUINO_JOYSTICK)
