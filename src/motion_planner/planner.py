from collections import deque
import math
from enum import Enum

DISTANCE_TOLERANCE_CM = 1
DEFAULT_TARGET = (0.0, 0.0)
TICKS_TO_CHANGE_SETPOINT = 5


class LoopType(Enum):
    NONE = 0
    SQUARE = 1
    CIRCLE = 2
    TRIANGLE = 3
    LINE = 4


class MotionPlanner:
    def __init__(self, loop_type: LoopType = LoopType.NONE):
        self._move_queue = deque()
        self.ticks_needed = TICKS_TO_CHANGE_SETPOINT
        self.loop_type = loop_type

    def update_target(self, current_position):
        """Advance to the next Movement if the end_condition of the current"""
        if len(self._move_queue) <= 1:
            self._append_to_move_queue()

        if all(
            abs(target - curr) < DISTANCE_TOLERANCE_CM
            for target, curr in zip(self._move_queue[0], current_position)
        ):
            self.ticks_needed -= 1
            if self.ticks_needed == 0:
                self._move_queue.popleft()
                self.ticks_needed = TICKS_TO_CHANGE_SETPOINT
        else:
            self.ticks_needed = TICKS_TO_CHANGE_SETPOINT

        return self._move_queue[0]

    def no_plan(self):
        return len(self._move_queue) == 0

    def _append_to_move_queue(self):
        match self.loop_type:
            case LoopType.SQUARE:
                side_length = 4.0
                half = side_length / 2.0
                self._move_queue.extend(
                    [
                        (half, half),
                        (half, -half),
                        (-half, -half),
                        (-half, half),
                        (half, half),
                    ]
                )
            case LoopType.TRIANGLE:
                side_length = 7.0
                TRIANGLE_HEIGHT_RATIO = math.sqrt(3) / 2.0
                half_height = TRIANGLE_HEIGHT_RATIO * side_length / 2.0
                self._move_queue.extend(
                    [
                        (0, -half_height),
                        (side_length / 2.0, half_height),
                        (-side_length / 2.0, half_height),
                        (0, -half_height),
                    ]
                )
            case LoopType.CIRCLE:
                radius = 4.0
                num_points = 36
                for i in range(num_points):
                    angle = (2 * math.pi / num_points) * i
                    x = radius * math.cos(angle)
                    y = radius * math.sin(angle)
                    self._move_queue.append((x, y))
            case LoopType.LINE:
                length = 5.0
                self._move_queue.extend([(0, length / 2.0), (0, -length / 2.0)])
