"""Some module that gets the ball position from CV"""
import numpy as np
import numpy.typing as npt

def get_ball_position() -> npt.NDArray:
    """Helper functino to interface with the CV binary to get the current ball position
    
    Returns:
        2 float vector of [x, y] position"""
    return np.array([0, 0])