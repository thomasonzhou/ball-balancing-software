import pytest
import numpy as np

from kinematics.plate_kinematics import calculate_angle_from_cosine

def test_calculate_cosine():
    # Defining some known values
    a = [1, np.pi/2]
    b = [np.sqrt(3)/2, np.pi/3]
    c = [1/2, np.pi/6]

    sides = [a, b, c]

    for i, side in enumerate(sides):
        print(side)
        i_1 = (i+1)%len(sides)
        i_2 = (i+2)%len(sides)
        assert sides[i][1] == calculate_angle_from_cosine(side[0], sides[i_1][0], sides[i_2][0])



if __name__ == "__main__":
    test_calculate_cosine()

