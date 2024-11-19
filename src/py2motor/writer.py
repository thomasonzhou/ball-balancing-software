DATA_START = "<"
DATA_END = ">"
DELIMITER = ","
TARE_CHAR = "t"
TARE_CONFIRM = "TARE"
ABSOLUTE_MOVE_CHAR = "a"
RELATIVE_MOVE_CHAR = "r"


def write_to_motors(absolute_motor_angles) -> None:
    raise NotImplementedError

def ascii_encode(motor_angles):
    a, b, c = motor_angles
    return f"<a, {a:.2f}, {b:.2f}, {c:.2f}>".encode('ascii')
