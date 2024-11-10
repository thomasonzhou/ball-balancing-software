#!/usr/bin/python3
import subprocess
import math


class Controller:
    def __init__(self):
        self.K_p = 1.0
        self.K_i = 0.0
        self.K_d = 0.0

        self.integral = 0.0

        self.prev_error = 0.0

        self.target_position = (0, 0)
        self.running = False

    def find_target_vector(self, curr_x, curr_y):
        return (self.target_position[0] - curr_x, self.target_position[1] - curr_y)

    def PID_step(self, error):
        proportional = self.K_p * error

        self.integral += error
        integral = self.K_i * self.integral

        derivative = self.K_d * (error - self.prev_error)

        self.prev_error = error
        return proportional + integral + derivative

    def start(self):
        self.running = True
        while self.running:
            # get ball position
            subprocess.run([""])
            ball_x, ball_y = get_ball_position()

            # generate target vector
            target_vector = self.find_target_vector(ball_x, ball_y)
            error = math.sqrt(target_vector[0] ** 2 + target_vector[1] ** 2)

            # run position through PID
            target = PID_step(error)

            # pass through IK
            serial_command = inverse_kinematics()

            # send to microcontroller over serial
            send_to_motors()
