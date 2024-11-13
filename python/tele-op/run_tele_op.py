"""Barebones function for running tele-op tomorrow"""
# Python imports
from subprocess import Popen
import time
import numpy as np

# Module imports
from hlc.homing import read_aruco_data
from kinematics.wrappers import translate_N_to_motor_angles
from serial_utils import *
from pid.position_feedback import Controller


def get_motor_angles_from_aruco() -> tuple[float, float, float] | tuple[None, None, None]:
    """Helper function to encapsulate the aruco/kinematics pipeline
    https://github.com/thomasonzhou/MTE380_Software/blob/03967e148c3493c3d2852fd17dae668f0a841446/python/run_homing.py
    
    Returns:
        tuple of 3 floats OR tuple of 3 Nones: Motor angles (MotorA, MotorB, MotorC) or failure
    """
    detect_aruco = Popen("./aruco_detection") # THIS IS THOMASON'S BINARY
    time.sleep(2)
    calibrated = False
    motor_angles = (0, 0, 0)
    try:
        while not calibrated:
            data = read_aruco_data()
            print(data)
            if data is None:
                continue
            x, y, z, distance = data
            motor_angles = translate_N_to_motor_angles(np.array([float(x), float(y), float(z)]), float(distance))
            return motor_angles

    except Exception as e:
        print(e)
        detect_aruco.kill()
    return (None, None, None)


if __name__ == "__main__":
    # Setup
    desired_coord = (0,0,0)
    pid = Controller()
    motor_serial, joystick_serial = init_serial()

    # Give time to the user to setup
    input("Press enter when ready to start hte homing sequence: ")

    # Homing
    print("Starting homing sequence")
    motor_angles = get_motor_angles_from_aruco()
    if None in motor_angles:
        raise ValueError("CV/Aruco -> Kinematics/Motor Angles pipeline failed")
    
    print("Taring motors")
    tare_motors(motor_angles)

    # Waiting for start from user
    print("Waiting for the joystick start")
    wait_for_user(joystick_serial)

    # Tele-op until keyboard cancellation
    print("Starting tele-op")
    try:
        while True:
            actual_coord = joystick_decode(joystick_serial)
            dir_x, dir_y, theta_mag = pid.calculate(desired_coord, actual_coord)
            motor_angles = translate_dir_to_motor_angles(dir_x, dir_y, theta_mag)
            send_encoded_motor_commands(motor_angles)
            
            print(f"PID VECTOR: {(dir_x, dir_y, theta_mag)}")
            print(f"TX ANGLE: {motor_angles}")    
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("User Interrupt")

    close_serial(motor_serial, joystick_serial)


