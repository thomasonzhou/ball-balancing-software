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


def level_platform():
    detect_aruco = Popen("./aruco_detection")

    calibrated = False
    while not calibrated:
        aruco_data = read_last_line(KINEMATICS_INFILE_NAME)
        x, y, z, distance = aruco_data.split(",")


if __name__ == "__main__":
    print("Warning: this script requires C++ binaries.")
    level_platform()
