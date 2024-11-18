"""Full integration of software components with sanity checks"""

# from constants import PLATFORM_TILT_MIN_RAD, PLATFORM_TILT_MAX_RAD, MOTOR_MIN_RAD, MOTOR_MAX_RAD
from kinematics.wrappers import translate_dir_to_motor_angles


def init():
    pass


def main():
    # --------------------------------------------------
    # Computer Vision (Detection, Control)
    # --------------------------------------------------
    # ball detection
    # ball_position_camera_view = get_ball_position()
    # ball_position_plate_view = camera_view_to_plate_view()

    # set target position
    target_position = (0.0, 0.0)

    # find theta_rad, the angle of tilt
    # target_direction_vector = (target) - (curr)
    # control_direction_vector, theta_rad = controller.get_val

    # assert direction vectors match
    # --------------------------------------------------
    # Inverse Kinematics
    # --------------------------------------------------

    # debug: read from joystick
    # theta_rad = read_joystick()

    # Sanity Check
    # assert PLATFORM_TILT_MIN_RAD <= theta_rad <= PLATFORM_TILT_MAX_RAD

    motor_angles = translate_dir_to_motor_angles(*direction_vector, theta_rad)

    # Sanity Check
    # for angle in motor_angles:
    #     assert MOTOR_MIN_RAD <= angle <= MOTOR_MAX_RAD

    # send_to_motors_with_serial


if __name__ == "__main__":
    init()
    main()
