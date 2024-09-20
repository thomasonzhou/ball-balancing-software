# Refer to `plate_kinematics_diagram.png` for the visual on the vectors used below

import numpy as np
import numpy.typing as npt

### Can be changed
P_I = 5
B_I = P_I
MOTOR_LEG_LENGTH = 3
PLATE_LEG_LENGTH = 6

def calculate_angle_from_cosine(a: float, b: float, c: float) -> float:
    """Helper function to calculate angle from law of cosines
    
    Args:
        a (float): Opposite side of return angle
        b, c (floats): Other sides of triangle
    Returns:
        float: theta of A (rad)
    """
    return np.arccos((np.square(b) + np.square(c) - np.square(a))/(2*b*c))

def calculate_theta_phi_from_T(T: npt.NDArray) -> tuple[float, float]:
    """Given the desired height, find the tilt about the x and y axis
    
    Args: 
        T (3 float vector): The vector with the desired plane height and direction
    
    Returns:
        float: theta_x -> tilk about the x axis
        float: phi_y -> tilt about the y axis"""
    T_norm = T/np.linalg.norm(T)
    phi_y = np.arcsin(T_norm[1])
    theta_x = np.arcsin(T[0]/np.sqrt(1-np.square(T_norm[1])))
    return (theta_x, phi_y)

def calculate_li(T: npt.NDArray, theta_x: float, phi_y: float, pi: npt.NDArray, bi: npt.NDArray) -> npt.NDArray:
    """Calculate the vector between the plate mount and the motor mount, for any motor. The configuration of the motor
    is described with pi and bi.

    Args:
        T (3 float vector): The vector with mag/dir between the center of the motor mounts, and the center of the plate
        theta_x (float): The angle of the tilt of the plate about the x-axis (rad)
        phi_y (float): The angle of the tilt of the plane about the y-axis (rad)
        pi (3 float vector): The vector describing the distance between plate mount and center of plate, in the plate's 
        reference frame
        bi (3 float vector): The vector describing the distance between the motor mount and center of plate, in the 
        body's reference frame

    Returns:
        3 float vector: The vector from motor mount -> plate mount
    """
    # Rotation matrix of A*B where A is x rotation (theta_x), B is y rotation (phi_y)
    R_P_wrt_B = np.array(
        [np.cos(theta_x), np.sin(theta_x)*np.sin(phi_y), np.sin(theta_x)*np.cos(phi_y)],
        [0, np.cos(phi_y), -np.sin(phi_y)],
        [-np.sin(theta_x), np.cos(theta_x)*np.sin(phi_y), np.cos(theta_x)*np.cos(phi_y)]
        )
    # Rotate the pi vector out of the plate frame and into the body frame
    pi_body = R_P_wrt_B @ pi
    li = T + pi_body - bi
    return li

def calculate_abs_motor_angle_from_li(li: npt.NDArray) -> float:
    """Given li (vector between the plate mount and the motor mount) for any motor, calculate the absolute angle of the 
    motor. This is with reference to the axis of the motor = 0.
    
    Args:
        li (3 float vector): The vector from the motor mount -> plate mount

    Returns:
        float: Desired shaft angle in radians offset from 0 (shaft plane)
    """
    # Magnitude of the vector between the plate mount and the motor mount
    li_mag = np.linalg.norm(li)
    # Use co-sine law to calculate the angle between the li vector and the motor shaft
    rel_shaft_angle = calculate_angle_from_cosine(li_mag, MOTOR_LEG_LENGTH, PLATE_LEG_LENGTH)
    # Calculate the angle between li and the z-axis
    li_norm = li/li_mag
    li_angle_from_z = np.arccos(np.dot(UNIT_K, li_norm))
    # For example, when the li vector is pointing solely in the z-direction, the rel_shaft_angle = abs_shaft_angle
    # Otherwise, if the li vector is tilted toward the plate or away from the plate, the motor shaft angle will need
    # overrotate or under rotate to compensate
    abs_shaft_angle = li_angle_from_z + rel_shaft_angle
    return abs_shaft_angle


### Default Configuration Setup
# HELPER CONSTANTS
UNIT_K = np.array([0, 0, 1])
UNIT_XY_VEC = np.array([1, 1, 0])

# DEFAULT VARIABLES DEFINING SETUP AND REST STATE
# at rest, the shaft and plate legs will be perpendiculat
MOTOR_TO_PLATE_HEIGHT = np.sqrt(np.square(MOTOR_LEG_LENGTH) + np.square(PLATE_LEG_LENGTH))
T = np.array([0, 0, MOTOR_TO_PLATE_HEIGHT])

ORIENTATION_A = np.pi/2 # along the y-axis
ORIENTATION_B = 7*np.pi/6 # 120 deg, CCW from A
ORIENTATION_C = 11*np.pi/6 # 120, CW from A

# bi per motor
ORG_TO_MOTOR_A = np.multiply(UNIT_XY_VEC, [np.cos(ORIENTATION_A), np.sin(ORIENTATION_A), 0])
ORG_TO_MOTOR_B = np.multiply(UNIT_XY_VEC, [np.cos(ORIENTATION_B), np.sin(ORIENTATION_B), 0])
ORG_TO_MOTOR_C = np.multiply(UNIT_XY_VEC, [np.cos(ORIENTATION_C), np.sin(ORIENTATION_C), 0])



