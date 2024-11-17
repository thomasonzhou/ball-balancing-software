# Python imports
import numpy as np
import time

# Module imports
from ball_position import get_ball_position
from kinematics.constants import REST_MOTOR_ANGLE
from kinematics.wrappers import translate_dir_to_motor_angles
from pid.position_feedback import Controller
from serial.motor_serial import MotorSerial


def main():
    # Setup
    desired_coord = np.array([0,0])
    pid = Controller()
    motor_serial = MotorSerial()

    # Homing
    print("Starting homing sequence")
    print("Taring motors")
    motor_serial.tare_motors((REST_MOTOR_ANGLE, REST_MOTOR_ANGLE, REST_MOTOR_ANGLE))

    # Main pipeline
    print("Starting the main pipeline")
    try:
        while True:
            desired_coord = np.array([0, 0])
            actual_coord = get_ball_position()
            dir_x, dir_y, theta_mag = pid.calculate(desired_coord, actual_coord)
            motor_angles = translate_dir_to_motor_angles(dir_x, dir_y, theta_mag)
            motor_serial.send_encoded_motor_commands(motor_angles)
            
            print(f"PID VECTOR: {(dir_x, dir_y, theta_mag)}")
            print(f"TX ANGLE: {motor_angles}")    
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("User Interrupt, exiting out of main loop")
    
    motor_serial.close()




if __name__ == "__main__":
    main()