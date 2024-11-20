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

HOMING_STRING = "HOME"

PLATFORM_TILT_MIN_RAD = 0.0
PLATFORM_TILT_MAX_RAD = math.pi / 12.0
MOTOR_MIN_DEG = -51.0
MOTOR_MAX_DEG = 51.0

MOTOR_MIN_RAD = math.radians(MOTOR_MIN_DEG)
MOTOR_MAX_RAD = math.radians(MOTOR_MAX_DEG)

ARDUINO_PORT = "/dev/cu.usbmodem11301"
MOTOR_CONTROLLER_PORT = "/dev/cu.usbserial-10"
# other ports: 'COM5' '/dev/ttyUSB0' "/dev/cu.usbmodem1301" "/dev/cu.usbmodem1401" "/dev/cu.usbserial-130"
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
    motor_serial.write(("<h>\r\n").encode("ascii"))
    recieved = ""
    while not homing_completed:
        print(f"sent {HOMING_STRING}")
        # while len(motor_serial.)
        # while motor_serial.in_waiting == 0:
        #     # print("waiting")
        #     continue
        if motor_serial.in_waiting > 0:
            homing_string = motor_serial.read()
            decoded = homing_string.decode("ascii").strip()
            recieved += decoded
            print(f"got {recieved}")
        if recieved == HOMING_STRING:
            print("HOME detected")
            homing_completed = True
    py2motor.write_to_motors(motor_serial, (0, 0, 0))

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
                    print(dir_x, dir_y, theta_rad)
            # print(f"{dir_x:.2f}, {dir_y:.2f}, {theta_rad*180/math.pi:.2f}")

            assert PLATFORM_TILT_MIN_RAD <= theta_rad <= PLATFORM_TILT_MAX_RAD

            # IK
            abs_motor_angles = inverse_kinematics.translate_dir_to_motor_angles(
                dir_x, dir_y, theta_rad
            )
            for angle in abs_motor_angles:
                assert MOTOR_MIN_RAD <= angle <= MOTOR_MAX_RAD

            py2motor.write_to_motors(motor_serial, abs_motor_angles)
    except KeyboardInterrupt:
        ball_detector.close_stream()


if __name__ == "__main__":
    main(operation_mode=OperationMode.ARDUINO_JOYSTICK)
