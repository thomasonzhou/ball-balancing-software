import serial

ABSOLUTE_MOVE_CHAR = "a"
RELATIVE_MOVE_CHAR = "r"


def write_to_motors(
    motor_serial: serial.Serial, absolute_motor_angles: tuple[float, float, float]
) -> None:
    """ASCII encodes a given motor command and set of motor angles in the expected format

    Args:
        motor_angles (tuple of 3 floats, (motorA, motorB, motorC)): Angles of the motors to be operated on (rad)
    """
    motorA, motorB, motorC = absolute_motor_angles
    motor_output = f"<{ABSOLUTE_MOVE_CHAR}, {motorA:.2f}, {motorB:.2f}, {motorC:.2f}>".encode(
        "ascii"
    )
    motor_serial.write(motor_output)
