import numpy as np
import pytest
from kinematics.motor import MOTOR_LEG_LENGTH, PLATE_LEG_LENGTH
from kinematics.plate_kinematics import (
    UNIT_K, 
    PIBI_LENGTH, 
    calculate_angle_from_cosine, 
    calculate_theta_phi_from_N, 
    calculate_xy_rotation_matrix, 
    calculate_li,
    calculate_abs_motor_angle_from_li
)

# Run all tests using `pytest`, or try `pytest -m helper`, `pytest -m stew`

### ALL `KNOWN` VALUES HAVE BEEN HAND-CALCULATED. ROUNDED TO 3 DECIMAL PLACES.
# See `diagrams/kinematics_test_values.png` for math background.

ROUND_DECIMALS = 3
KNOWN_N_ANGLE_PAIR = {
        "zero": {
            "known_N": UNIT_K,
            "known_angles": (0, 0)
        },
        "five": { # deg
            "known_N": np.array([0.087, -0.087, 0.992]),
            "known_angles": np.round(np.array([5 * np.pi / 180, 5 * np.pi / 180]), ROUND_DECIMALS)
        }
    }
T = np.round(np.sqrt(np.square(MOTOR_LEG_LENGTH) + np.square(PLATE_LEG_LENGTH)), ROUND_DECIMALS)
KNOWN_LI_ANGLE_PAIR = {
    "zero": {
            "known_T": np.array([0, 0, T]),
            "known_li": np.array([0, 0, T]),
            "known_angles": (0, 0),
            "known_pibi": (0, PIBI_LENGTH, 0)
        },
    "five": {
            "known_T": np.array([0, 0, T]),
            "known_li": np.array([0.038, -0.019, 7.142]),
            "known_angles": np.array([5 * np.pi / 180, 5 * np.pi / 180]),
            "known_pibi": (0, PIBI_LENGTH, 0)
    }
}
KNOWN_MOTOR_ANGLE_PAIR = {
    "zero": {
            "known_li": np.array([0, 0, T]),
            "known_abs_angle": np.round(np.pi/2 - np.arctan(PLATE_LEG_LENGTH/MOTOR_LEG_LENGTH), ROUND_DECIMALS)
        },
    "five": {
            "known_li": np.array([0.038, -0.019, 7.142]),
            "known_abs_angle": np.round(33.719 * np.pi / 180, ROUND_DECIMALS)
    }
}


@pytest.mark.helper
def test_calculate_cosine():
    """Tests the cosine helper function to ensure it outputs the correct angles"""
    known_a = [1, np.pi/2]
    known_b = [np.sqrt(3)/2, np.pi/3]
    known_c = [1/2, np.pi/6]

    known_sides = [known_a, known_b, known_c]

    for i, known_side in enumerate(known_sides):
        i_1 = (i+1)%len(known_sides)
        i_2 = (i+2)%len(known_sides)
        assert known_sides[i][1] == calculate_angle_from_cosine(known_side[0], known_sides[i_1][0], known_sides[i_2][0])


@pytest.mark.stew
def test_calculate_theta_phi_from_N():
    """Tests the calculation of the tilt angles given a normal vector"""
    for _, value in KNOWN_N_ANGLE_PAIR.items():
        angles = np.round(calculate_theta_phi_from_N(value["known_N"]), ROUND_DECIMALS)
        for known_angle, output_angle in zip(value["known_angles"], angles):
            assert known_angle == output_angle


@pytest.mark.helper
def test_calculate_xy_rotation_matrix():
    """Tests the calculation of the right rotation matrix given tilt angles"""
    for _, value in KNOWN_N_ANGLE_PAIR.items():
        rotMat = calculate_xy_rotation_matrix(value["known_angles"][0], value["known_angles"][1])
        rotated_N = rotMat @ UNIT_K
        for known_N_el, output_N_el in zip(value["known_N"], np.round(rotated_N, ROUND_DECIMALS)):
            assert known_N_el == output_N_el


@pytest.mark.stew
def test_calculate_li():
    """Tests the calculation of the li vector given tilt angles"""
    for _, value in KNOWN_LI_ANGLE_PAIR.items():
        li = calculate_li(
            T=value["known_T"], 
            theta_y=value["known_angles"][0], 
            phi_x=value["known_angles"][1], 
            pi_plat=value["known_pibi"], 
            bi=value["known_pibi"]
        )
        li = np.round(li, ROUND_DECIMALS)
        for known_li_el, output_li_el in zip(value["known_li"], np.round(li, ROUND_DECIMALS)):
            assert known_li_el == output_li_el

@pytest.mark.stew
def test_calculate_abs_motor_angle_from_li():
    """Tests the calulation of the absolute motor angle given vector li"""
    for _, value in KNOWN_MOTOR_ANGLE_PAIR.items():
        angle = calculate_abs_motor_angle_from_li(value["known_li"])
        angle = np.round(angle, ROUND_DECIMALS)
        assert value["known_abs_angle"] == angle
