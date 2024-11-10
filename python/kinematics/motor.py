#!/usr/bin/python3
import numpy as np

MOTOR_LEG_LENGTH = 5 # cm
PLATE_LEG_LENGTH = 8 # cm
MOTOR_DIST_FROM_ORIGIN = 11 # cm
PLATE_DIST_FROM_ORIGIN = 15 # cm

class Motor:
    """Class defining a motor's orientation and leg details"""
    motor_leg_length = MOTOR_LEG_LENGTH
    plate_leg_length = PLATE_LEG_LENGTH

    def __init__(self, orientation: float):
        self.orientation = orientation
        self.distance_from_origin = MOTOR_DIST_FROM_ORIGIN
        self.distance_from_origin_plate_mount = PLATE_DIST_FROM_ORIGIN
        self.orientation_vector()

    def orientation_vector(self): 
        # Defining pi and bi
        motor_orientation_vector = np.array([np.cos(self.orientation), np.sin(self.orientation), 0])
        self.MOTOR_ORIENTATION_VECTOR = motor_orientation_vector * self.distance_from_origin # bi
        self.PLATE_ORIENTATION_VECTOR = motor_orientation_vector * self.distance_from_origin_plate_mount # pi

    def set_desired_angle(self, angle: float):
        self.desired_angle = angle
    
    def get_set_angle(self):
        return self.desired_angle
