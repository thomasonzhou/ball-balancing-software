# Refer to `plate_vectors.png` for the visual on the vectors used below

import numpy as np
import numpy.typing as npt
from kinematics.motor import Motor, MOTOR_LEG_LENGTH, PLATE_LEG_LENGTH

DEFAULT_PLATE_HEIGHT = 8 # cm, rel to shaft

# See `motor_orientations.png`
MOTOR_ORIENTATIONS = [
    np.pi/2, # Motor A: along the y-axis
    7*np.pi/6, # Motor B: 120 deg, CCW from A
    11*np.pi/6, # Motor C: 120, CW from A
]

# HELPER CONSTANTS
UNIT_K = np.array([0, 0, 1])

def calculate_normal_from_dir_vec(dir_vec: npt.NDArray, mag: float) -> npt.NDArray:
    """Helper function to calculate the normal vector of the place given a direction and magnitude

    Uses the Rodrigues rotation formula: https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
    
    Args:
        dir_vec (2 float): Direction vector describing the tilt, on the x-y plane
        mag: How much to tilt the plate, in radians
    
    Returns:
        3 float vector: The normal to the plate
    """
    
    # First, determine the axis of rotation. This is orthogonal to the dir_vec, in the x-y plane.
    # Recall that two vectors are orthogonal if their dot product is zero.

    # Add this, else, normalizing with a norm of 0 will give NaNs everywhere + redundant code if no movement
    if np.all(dir_vec==0):
        return UNIT_K
    if mag == 0:
        return UNIT_K

    rot_axis = np.array([-dir_vec[1], dir_vec[0], 0])
    rot_axis_norm = rot_axis / np.linalg.norm(rot_axis)

    # Rotate the direction vector about the orthogonal axis, with a angle of `mag` degrees
    dir_vec_rot = ( # Rodrigues rotation formula
        dir_vec * np.cos(mag) + 
        np.cross(rot_axis_norm, dir_vec) * np.sin(mag) + 
        rot_axis_norm * (np.dot(rot_axis_norm, dir_vec)) * (1 - np.cos(mag))
    )
    dir_vec_rot_norm = dir_vec_rot / np.linalg.norm(dir_vec_rot)

    N = np.cross(rot_axis_norm, dir_vec_rot_norm)

    # Always ensuring the vector is pointing in positive Z
    if N[2] < 0:
        N = N * -1

    return N


def calculate_xy_rotation_matrix(theta_y: float, phi_x: float) -> npt.NDArray:
    """Helper function to calculate the xy rotation matrix
    
    Args:
        theta_y: To rotate in Y
        phi_x: To rotate in X
    
    Returns:
        3x3 float matrix: Rotation matrix
    """
    R_p_wrt_b = np.array(
        [[np.cos(theta_y), np.sin(theta_y)*np.sin(phi_x), np.sin(theta_y)*np.cos(phi_x)],
        [0, np.cos(phi_x), -np.sin(phi_x)],
        [-np.sin(theta_y), np.cos(theta_y)*np.sin(phi_x), np.cos(phi_x)*np.cos(theta_y)]]
        )
    return R_p_wrt_b


def calculate_angle_from_cosine(a: float, b: float, c: float) -> float:
    """Helper function to calculate angle from law of cosines
    
    Args:
        a (float): Opposite side of return angle
        b, c (floats): Other sides of triangle
    Returns:
        float: Theta of A (rad)
    """
    return np.arccos((np.square(b) + np.square(c) - np.square(a))/(2*b*c))


def calculate_theta_phi_from_N(N: npt.NDArray) -> tuple[float, float]:
    """Given the normal vector of the plate, find the tilt about the x and y axis.
    See `rotation_math.png` for more details.
    
    Args:
        N (3 float vector): The normal vector with the desired plate tilt
    
    Returns:
        float: phi_x -> tilk about the x axis
        float: theta_y -> tilt about the y axis
    """
    N_norm = N/np.linalg.norm(N)
    phi_x = np.arcsin(-N_norm[1])
    theta_y = np.arcsin(N_norm[0]/np.sqrt(1-np.square(N_norm[1])))
    return (phi_x, theta_y)


def calculate_li(T: npt.NDArray, theta_y: float, phi_x: float, pi_plat: npt.NDArray, bi: npt.NDArray) -> npt.NDArray:
    """Calculate the vector between the plate mount and the motor mount, for any motor. The configuration of the 
    mounting on the plate, and the mounting of the motor, is described with pi and bi respectively. 
    See `rotation_math.png` for more details.

    Args:
        T (3 float vector): The vector with mag/dir between the center of the motor mounts, and the center of the plate
        phi_x (float): The angle of the tilt of the plate about the x-axis (rad)
        theta_y (float): The angle of the tilt of the plane about the y-axis (rad)
        pi_plat (3 float vector): The vector describing the distance between plate mount and center of plate, in the 
        plate's reference frame
        bi (3 float vector): The vector describing the distance between the motor mount and center of plate, in the 
        body's reference frame

    Returns:
        3 float vector: The vector from motor mount -> plate mount
    """
    # Rotation matrix of A*B where A is x rotation (phi_x), B is y rotation (theta_y)
    R_p_wrt_b = calculate_xy_rotation_matrix(theta_y, phi_x)
    # Rotate the pi vector with the same rotation matrix as the unit K -> new normal vector
    pi_body = R_p_wrt_b @ pi_plat
    li = T + pi_body - bi
    return li


def calculate_abs_motor_angle_from_li(li: npt.NDArray) -> float:
    """Given li (vector between the plate mount and the motor mount) for any motor, calculate the absolute angle of the 
    motor. This is with reference to the axis of the motor = 0. See angles from `relative_motor_angle.png` and 
    `absolute_motor_angle.png`.
    
    Args:
        li (3 float vector): The vector from the motor mount -> plate mount

    Returns:
        float: Desired shaft angle in radians offset from 0 (shaft plane)
    """
    # Magnitude of the vector between the plate mount and the motor mount
    li_mag = np.linalg.norm(li)
    # Use co-sine law to calculate the angle between the li vector and the motor shaft
    alpha = calculate_angle_from_cosine(PLATE_LEG_LENGTH, li_mag, MOTOR_LEG_LENGTH)
    # Calculate the angle between li and the z-axis
    beta = np.arccos(np.dot(UNIT_K, li)/li_mag)
    # For example, when the li vector is pointing solely in the z-direction, the rel_shaft_angle = abs_shaft_angle
    # Otherwise, if the li vector is tilted toward the plate or away from the plate, the motor shaft angle will need
    # overrotate or under rotate to compensate. 
    gamma = np.pi/2 - beta - alpha
    return gamma

if __name__ == "__main__":
    ### Default Configuration Setup
    # DEFAULT VARIABLES DEFINING SETUP AND REST STATE
    T = np.array([0, 0, DEFAULT_PLATE_HEIGHT]) # from the middle of the base of the motor mounts to the middle of the plate
    N = np.array([0, 0, 1]) # normal vector of plate

    # From here, the pipeline is:
    motors = [Motor(orientation) for orientation in MOTOR_ORIENTATIONS] # initializes motors a, b, c
    phi_x, theta_y = calculate_theta_phi_from_N(N)
    for motor in motors:
        li = calculate_li(T, theta_y, phi_x, pi_plat=motor.PLATE_ORIENTATION_VECTOR, bi = motor.MOTOR_ORIENTATION_VECTOR)
        abs_angle = calculate_abs_motor_angle_from_li(li)
        motor.set_desired_angle(abs_angle)


