#!/usr/bin/python3
from subprocess import Popen
import time
import numpy as np
from hlc.homing import read_aruco_data
from kinematics.wrappers import translate_N_to_motor_angles


def level_platform():
    detect_aruco = Popen("./aruco_detection")
    time.sleep(10)
    calibrated = False
    try:
        while not calibrated:
            data = read_aruco_data()
            print(data)
            if data is None:
                continue
            print(data)
            x, y, z, distance = data
            motor_angles = translate_N_to_motor_angles(np.array([float(x), float(y), float(z)]))
            print(motor_angles)

    except Exception as e:
        print(e)
        detect_aruco.kill()


if __name__ == "__main__":
    print("Warning: this script requires C++ binaries.")
    level_platform()
    print("done")
