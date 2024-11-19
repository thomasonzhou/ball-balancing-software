DATA_START = "<"
DATA_END = ">"
DELIMITER = ","
TARE_CHAR = "t"
TARE_CONFIRM = "TARE"
ABSOLUTE_MOVE_CHAR = "a"
RELATIVE_MOVE_CHAR = "r"


def write_to_motors(absolute_motor_angles) -> None:
    raise NotImplementedError
