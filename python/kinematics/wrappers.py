#!/usr/bin/python3
"""Module for the kinematics pipeline for tele-op"""

from kinematics.plate_kinematics import *
from hlc.homing import read_aruco_data


def get_plate_height():
    # File I/O based on Thomason's CV binary
    data = read_aruco_data()
    if data is None:
        return None
    _, _, _, height = data
    return float(height)


def translate_dir_to_motor_angles(
    x_dir: float, y_dir: float, mag: float
) -> tuple[float, float, float]:
    """Wrapper function for movement kinematics: Translate x-y coordiantes to a vector, then return motor angles
    that tilt the plane in that direction, about a specified magnitude

    Args:
        x_dir (float): X-direction
        y_dir (float): Y-direction
        mag (float): Angle to tilt in radians

    Returns:
        Tuple[float, float, float]: Absolute angles (float) of motor A, B, C
    """

    dir = np.array([x_dir, y_dir])
    N = calculate_normal_from_dir_vec(dir, mag)
    motor_angles = translate_N_to_motor_angles(N)
    return motor_angles


def translate_N_to_motor_angles(N_norm: npt.NDArray) -> tuple[float, float, float]:
    """Wrapper function for homing: Translate a normal vector and distance, and return motor angles that give the plane
    it's current tilt

    Args:
        N_norm (3 float vector): Normalized normal vector of the plate

    Returns:
    Tuple[float, float, float]: Absolute angles (float) of motor A, B, C
    """

    motors = [Motor(orientation) for orientation in MOTOR_ORIENTATIONS]
    phi_x, theta_y = calculate_theta_phi_from_N(N_norm)
    T = np.array([0, 0, get_plate_height()])
    for motor in motors:
        li = calculate_li(
            T,
            theta_y,
            phi_x,
            pi_plat=motor.PLATE_ORIENTATION_VECTOR,
            bi=motor.MOTOR_ORIENTATION_VECTOR,
        )
        abs_angle = calculate_abs_motor_angle_from_li(li)
        motor.set_desired_angle(abs_angle)
    motor_angles = (
        float(motors[0].get_set_angle()),
        float(motors[1].get_set_angle()),
        float(motors[2].get_set_angle()),
    )
    return motor_angles


if __name__ == "__main__":
    angles_dir = translate_dir_to_motor_angles(0, 0, 0)
    angles_N = translate_N_to_motor_angles(UNIT_K)

    for angle_d, angle_N in zip(angles_dir, angles_N):
        assert angle_d == angle_N
