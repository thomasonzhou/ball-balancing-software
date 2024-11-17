import numpy as np

"""Mechanical Constants"""
MOTOR_LEG_LENGTH = 5 # cm
PLATE_LEG_LENGTH = 8 # cm
MOTOR_DIST_FROM_ORIGIN = 11 # cm
PLATE_DIST_FROM_ORIGIN = 15 # cm

DEFAULT_PLATE_HEIGHT = 8 # cm, rel to shaft

# See `motor_orientations.png`
MOTOR_ORIENTATIONS = [
    np.pi/2, # Motor A: along the y-axis
    7*np.pi/6, # Motor B: 120 deg, CCW from A
    11*np.pi/6, # Motor C: 120, CW from A
]

# HELPER CONSTANTS
UNIT_K = np.array([0, 0, 1])


"""Homing Constants"""
REST_MOTOR_ANGLE = -60 # deg