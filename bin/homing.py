from subprocess import Popen
import os

KINEMATICS_INFILE_NAME = "kinematics_input.txt"


def read_last_line(filename):
    num_newlines = 0
    with open(filename, "rb") as f:
        try:
            f.seek(-2, os.SEEK_END)
            while num_newlines < 1:
                f.seek(-2, os.SEEK_CUR)
                if f.read(1) == b"\n":
                    num_newlines += 1
        except OSError:
            f.seek(0)
        last_line = f.readline().decode().strip()
    return last_line

def read_aruco_data():
    aruco_data = read_last_line(KINEMATICS_INFILE_NAME)
    aruco_data_list = aruco_data.split(",")
    if not aruco_data_list:
        return None
    return aruco_data_list # x, y, z, distance

def level_platform():
    detect_aruco = Popen("./aruco_detection")

    calibrated = False
    while not calibrated:
        x, y, z, distance = read_aruco_data()
        print(x, y, z, distance)

if __name__ == "__main__":
    print("Warning: this script requires C++ binaries.")
    level_platform()
