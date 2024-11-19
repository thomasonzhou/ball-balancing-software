from collections import deque
import math

DISTANCE_TOLERANCE_CM = 1
DEFAULT_TARGET = (0.0, 0.0)


class MotionPlanner:
    def __init__(self):
        self._move_queue = deque()

    def update_target(self, current_position):
        """Advance to the next Movement if the end_condition of the current"""
        if len(self._move_queue) > 0:
            return DEFAULT_TARGET

        if all(
            abs(target - curr) < DISTANCE_TOLERANCE_CM
            for target, curr in zip(self._move_queue[0], current_position)
        ):
            self._move_queue.popleft()
            return self.update_target(current_position)

        return self._move_queue[0]

    def no_plan(self):
        return len(self._move_queue) == 0

    def load_square_trajectory(self):
        assert self.no_plan()
        self._move_queue.extend([(3, 3), (3, -3), (-3, -3), (-3, 3), (3, 3)])

    def load_triangle_trajectory(self):
        assert self.no_plan()
        self._move_queue.extend([])
        self._move_queue.extend([])

    def load_circle_trajectory(self):
        assert self.no_plan()
        radius = 5
        num_points = 36
        for i in range(num_points):
            angle = (2 * math.pi / num_points) * i
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            self._move_queue.append((x, y))
