import numpy as np
import pytest
from kinematics.motor import (
        MOTOR_DIST_FROM_ORIGIN, 
        PLATE_DIST_FROM_ORIGIN
)
from kinematics.plate_kinematics import (
    UNIT_K, 
    calculate_normal_from_dir_vec,
    calculate_angle_from_cosine, 
    calculate_theta_phi_from_N, 
    calculate_xy_rotation_matrix, 
    calculate_li,
    calculate_abs_motor_angle_from_li
)

# Run all tests using `pytest`, or try `pytest -m helper`, `pytest -m stew`

### ALL `KNOWN` VALUES HAVE BEEN HAND-CALCULATED. ROUNDED TO 3 DECIMAL PLACES.
# See `diagrams/MTE_380_Rotation_Equations.pdf` for math background.

ROUND_DECIMALS = 3
T = 8

KNOWN_VEC_MAG_N_PAIR = {
    "zero": {
        "known_dir": np.array([0, 0, 0]),
        "known_mag": 0,
        "known_N": [0, 0, 1]
    },
    "five": { # deg
        "known_dir": np.array([1, 0, 0]),
        "known_mag": np.round(5 * np.pi / 180, ROUND_DECIMALS),
        "known_N": np.array([0.087, 0, 0.996])
    }
}

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

KNOWN_LI_ANGLE_PAIR = {
    "zero": {
            "known_T": np.array([0, 0, T]),
            "known_li": np.array([0, PLATE_DIST_FROM_ORIGIN-MOTOR_DIST_FROM_ORIGIN, T]),
            "known_angles": (0, 0),
            "known_pi": (0, PLATE_DIST_FROM_ORIGIN, 0),
            "known_bi": (0, MOTOR_DIST_FROM_ORIGIN, 0),
        },
    "five": { # deg
            "known_T": np.array([0, 0, T]),
            "known_li": np.array([0.114, 2.943, 9.302]),
            "known_angles": np.array([5 * np.pi / 180, 5 * np.pi / 180]),
            "known_pi": (0, PLATE_DIST_FROM_ORIGIN, 0),
            "known_bi": (0, MOTOR_DIST_FROM_ORIGIN, 0),
    }
}

KNOWN_MOTOR_ANGLE_PAIR = {
    "zero": {
            "known_li": np.array([0, PLATE_DIST_FROM_ORIGIN-MOTOR_DIST_FROM_ORIGIN, T]),
            "known_abs_angle": np.round(2.89340950562 * np.pi / 180, ROUND_DECIMALS)
        },
    "five": { # deg
            "known_li": np.array([0.114, 2.943, 9.302]),
            "known_abs_angle": np.round(17.606 * np.pi / 180, ROUND_DECIMALS)
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
        known_angle = np.round(known_sides[i][1], ROUND_DECIMALS)
        calc_angle = np.round(calculate_angle_from_cosine(
            known_side[0], 
            known_sides[i_1][0], 
            known_sides[i_2][0]), ROUND_DECIMALS)
        assert known_angle == calc_angle


@pytest.mark.stew
def test_calculate_N():
    """Tests the calculation of the normal angle given a direction and magnitude of movement"""
    for value in KNOWN_VEC_MAG_N_PAIR.values():
        N = np.round(calculate_normal_from_dir_vec(value["known_dir"], value["known_mag"]), ROUND_DECIMALS)
        for known_N_el, output_N_el in zip(value["known_N"], N):
            assert known_N_el == output_N_el


@pytest.mark.stew
def test_calculate_theta_phi_from_N():
    """Tests the calculation of the tilt angles given a normal vector"""
    for value in KNOWN_N_ANGLE_PAIR.values():
        angles = np.round(calculate_theta_phi_from_N(value["known_N"]), ROUND_DECIMALS)
        for known_angle, output_angle in zip(value["known_angles"], angles):
            assert known_angle == output_angle


@pytest.mark.helper
def test_calculate_xy_rotation_matrix():
    """Tests the calculation of the right rotation matrix given tilt angles"""
    for value in KNOWN_N_ANGLE_PAIR.values():
        rotMat = calculate_xy_rotation_matrix(*value["known_angles"])
        rotated_N = rotMat @ UNIT_K
        for known_N_el, output_N_el in zip(value["known_N"], np.round(rotated_N, ROUND_DECIMALS)):
            assert known_N_el == output_N_el


@pytest.mark.stew
def test_calculate_li():
    """Tests the calculation of the li vector given tilt angles"""
    for value in KNOWN_LI_ANGLE_PAIR.values():
        li = calculate_li(
            T=value["known_T"], 
            theta_y=value["known_angles"][0], 
            phi_x=value["known_angles"][1], 
            pi_plat=value["known_pi"], 
            bi=value["known_bi"]
        )
        li = np.round(li, ROUND_DECIMALS)
        for known_li_el, output_li_el in zip(value["known_li"], np.round(li, ROUND_DECIMALS)):
            assert known_li_el == output_li_el


@pytest.mark.stew
def test_calculate_abs_motor_angle_from_li():
    """Tests the calulation of the absolute motor angle given vector li"""
    for value in KNOWN_MOTOR_ANGLE_PAIR.values():
        angle = calculate_abs_motor_angle_from_li(value["known_li"])
        angle = np.round(angle, ROUND_DECIMALS)
        assert value["known_abs_angle"] == angle
