# Refer to `plate_kinematics_diagram.png` for the visual on the vectors used below

import numpy as np
import numpy.typing as npt
import time

def calculate_angle_from_cosine(a: float, b: float, c: float) -> float:
    """Helper function to calculate angle from law of cosines
    
    Args:
        a (float): Opposite side of return angle
        b, c (floats): Other sides of triangle
    Returns:
        float: theta of A in RADIANS"""
    return np.arccos((np.square(b) + np.square(c) - np.square(a))/(2*b*c))

def calculate_li(T: npt.NDArray, theta_x: float, phi_y: float, pi: npt.NDArray, bi: npt.NDArray):
    R_P_wrt_B = np.array(
        [np.cos(theta_x), np.sin(theta_x)*np.sin(phi_y), np.sin(theta_x)*np.cos(phi_y)],
        [0, np.cos(phi_y), -np.sin(phi_y)],
        [-np.sin(theta_x), np.cos(theta_x)*np.sin(phi_y), np.cos(theta_x)*np.cos(phi_y)]
        )
    pi_body = R_P_wrt_B @ pi
    li = T + pi_body - bi
    return li

# To be changed
P_I = 5
B_I = P_I
MOTOR_LEG_LENGTH = 3
PLATE_LEG_LENGTH = 6

# Default Configuration Setup
T = np.array[0, 0, 7]

MOTOR_TO_PLATE_HEIGHT = np.sqrt(np.square(MOTOR_LEG_LENGTH) + np.square(PLATE_LEG_LENGTH))

ORG_TO_MOTOR_A = np.array([0, B_I, 0])
ORG_TO_MOTOR_B = np.array([
    -np.cos(np.pi/6)*B_I, 
    -np.sin(np.pi/6)*B_I,
    0])
ORG_TO_MOTOR_C = np.array([
    np.cos(np.pi/6)*B_I, 
    -np.sin(np.pi/6)*B_I,
    0])

MOTOR_A_START_THETA = 
# project li onto (0, 0, 1)
# do the arm calculation of hte local triangle
# find the difference, this is the motor angle



time.sleep(1)



