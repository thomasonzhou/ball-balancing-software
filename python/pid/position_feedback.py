import math

class Controller:
    def __init__(self, kp, kd, ki, dt):
        # Define PID gains and time interval
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.dt = dt 

        self.prev_e_x = 0
        self.prev_e_y = 0
        self.int_x = 0
        self.int_y = 0

    def calculate(self, desired_pos, actual_pos):
        x_r, y_r = desired_pos
        x, y = actual_pos

        # Calculate error
        e_x = x_r - x
        e_y = y_r - y

        # Calculate proportional term
        p_x = self.kp * e_x
        p_y = self.kp * e_y

        # Calculate derivative term
        d_x = self.kd * ((e_x - self.prev_e_x) / self.dt)
        d_y = self.kd * ((e_y - self.prev_e_y) / self.dt)

        # Calculate integral term
        self.int_x += e_x * self.dt
        self.int_y += e_y * self.dt
        i_x = self.ki * self.int_x
        i_y = self.ki * self.int_y

        # Update error
        self.prev_e_x = e_x
        self.prev_e_y = e_y

        # Calculate control signal
        u_x = p_x + d_x + i_x
        u_y = p_y + d_y + i_y

        # Calculate theta as the magnitude of the control signal
        theta_mag = math.sqrt(u_x**2 + u_y**2)

        # Calculate the unit vector components
        if theta_mag != 0:
            dir_x = u_x / theta_mag
            dir_y = u_y / theta_mag
        else:
            dir_x, dir_y = 0,0

        return dir_x, dir_y, theta_mag