import numpy as np

# HELPER CONSTANTS
UNIT_XY_VEC = np.array([1, 1, 0])

MOTOR_LEG_LENGTH = 3
PLATE_LEG_LENGTH = 6

class Motor:
    motor_leg_length = MOTOR_LEG_LENGTH
    plate_leg_length = PLATE_LEG_LENGTH

    def __init__(self, orientation: float, distance: float):
        self.orientation = orientation
        self.distance_from_origin = distance
        self.orientation_vector()

    def orientation_vector(self): 
        # This is bi & pi
        motor_orientation_vector = np.multiply(UNIT_XY_VEC, [np.cos(self.orientation), np.sin(self.orientation), 0])
        self.MOTOR_ORIENTATION_VECTOR = motor_orientation_vector * self.distance_from_origin
        self.PLATE_ORIENTATION_VECTOR = self.MOTOR_ORIENTATION_VECTOR

    def set_desired_angle(self, angle: float):
        self.desired_angle = angle

