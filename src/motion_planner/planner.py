from collections import deque
import math

DISTANCE_TOLERANCE_CM = 2
DEFAULT_TARGET = (0.0, 0.0)
TICKS_TO_CHANGE_SETPOINT = 10


class MotionPlanner:
    def __init__(self):
        self._move_queue = deque()
        self.ticks_needed = 10

    def update_target(self, current_position):
        """Advance to the next Movement if the end_condition of the current"""
        if len(self._move_queue) == 0:
            return DEFAULT_TARGET

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

        # print(f"target {self._move_queue}")
        return self._move_queue[0]

    def no_plan(self):
        return len(self._move_queue) == 0

    def load_square_trajectory(self, side_length=8.0):
        """Square centered at (0,0)"""
        assert self.no_plan()
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

    def load_triangle_trajectory(self, side_length=3.0):
        """Unilateral triangle centered at (0,0)"""
        assert self.no_plan()
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

    def load_circle_trajectory(self, radius=4.0, num_points=36):
        """Circle centered at (0,0)"""
        assert self.no_plan()
        for i in range(num_points):
            angle = (2 * math.pi / num_points) * i
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            self._move_queue.append((x, y))
