"""Sensor-driven depth-first exploration, independent of the Webots runtime."""

from enum import IntEnum
from typing import Mapping


class Direction(IntEnum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3


Cell = tuple[int, int]
Observation = Mapping[str, bool | None]
DELTAS = ((0, 1), (1, 0), (0, -1), (-1, 0))
RELATIVE_ACTIONS = ("F", "R", "U", "L")


class NavigationError(RuntimeError):
    """Sensor or movement feedback contradicts the exploration state."""


def goal_cells(width: int, height: int) -> set[Cell]:
    if width < 1 or height < 1:
        raise ValueError("Maze dimensions must be positive.")
    return {
        (x, y)
        for x in {(width - 1) // 2, width // 2}
        for y in {(height - 1) // 2, height // 2}
    }


class RuleBasedNavigator:
    """Prefer unvisited left/front/right cells, then return to the parent."""

    def __init__(self, width: int, height: int, max_steps: int | None = None):
        self.goals = goal_cells(width, height)
        self.width = width
        self.height = height
        self.max_steps = 2 * width * height if max_steps is None else max_steps
        if self.max_steps < 1:
            raise ValueError("The step limit must be positive.")
        self.cell = (0, 0)
        self.heading = Direction.NORTH
        self.entry_heading = self.heading
        self.pending: Direction | None = None
        self.visited = {self.cell}
        self.path = [self.cell]
        self.walls: dict[tuple[Cell, Direction], bool] = {}
        self.steps = 0
        self.status = "running"
        self.last_rule = "start"

    def neighbor(self, direction: Direction) -> Cell:
        dx, dy = DELTAS[direction]
        return self.cell[0] + dx, self.cell[1] + dy

    def wall(self, direction: Direction) -> bool | None:
        x, y = self.neighbor(direction)
        if not (0 <= x < self.width and 0 <= y < self.height):
            return True
        return self.walls.get((self.cell, direction))

    def set_wall(self, direction: Direction, value: bool) -> None:
        if type(value) is not bool:
            raise ValueError("Wall observations must be boolean.")
        neighbor = self.neighbor(direction)
        if not (0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height):
            if not value:
                raise NavigationError("A sensor reported an opening outside the maze.")
            return
        self.walls[self.cell, direction] = value
        opposite = Direction((direction + 2) % 4)
        self.walls[neighbor, opposite] = value

    def observe(self, observation: Observation) -> None:
        for key, turn in (("front", 0), ("left", -1), ("right", 1)):
            value = observation[key]
            direction = Direction((self.heading + turn) % 4)
            # Late diagonal readings must not replace an earlier clear reading.
            if observation.get("late") and turn and self.wall(direction) is not None:
                continue
            if value is not None:
                self.set_wall(direction, value)

    def decide(self) -> str:
        if self.pending is not None:
            raise NavigationError("Movement must finish or be rejected before choosing again.")
        if self.cell in self.goals:
            self.status, self.last_rule = "goal", "goal_reached"
            return "STOP"
        if self.steps >= self.max_steps:
            self.status, self.last_rule = "step_limit", "step_limit"
            return "STOP"

        for turn, label in ((-1, "left"), (0, "front"), (1, "right"), (2, "back")):
            direction = Direction((self.entry_heading + turn) % 4)
            if self.wall(direction) is not True and self.neighbor(direction) not in self.visited:
                # Unknown sides can be verified after turning; unknown fronts cannot.
                if direction == self.heading and self.wall(direction) is None:
                    raise NavigationError("The front observation is missing.")
                return self._select(direction, f"unvisited_{label}")

        if len(self.path) == 1:
            self.status, self.last_rule = "exhausted", "no_reachable_goal"
            return "STOP"
        parent = self.path[-2]
        for direction in Direction:
            if self.neighbor(direction) == parent:
                if self.wall(direction) is True:
                    raise NavigationError("The previously traversed return passage is blocked.")
                return self._select(direction, "backtrack")
        raise NavigationError("The return path contains nonadjacent cells.")

    def _select(self, direction: Direction, rule: str) -> str:
        action = RELATIVE_ACTIONS[(direction - self.heading) % 4]
        self.pending = direction
        self.heading = direction
        self.last_rule = rule
        return action

    def arrive(self, observation: Observation) -> str:
        if self.pending is None:
            raise NavigationError("An arrival was reported without a pending movement.")
        direction = self.pending
        destination = self.neighbor(direction)
        self.set_wall(direction, False)
        if len(self.path) > 1 and destination == self.path[-2]:
            self.path.pop()
        elif destination not in self.visited:
            self.path.append(destination)
            self.visited.add(destination)
        else:
            raise NavigationError("Movement would create a loop in the return path.")
        self.cell = destination
        self.pending = None
        self.entry_heading = direction
        self.steps += 1
        self.observe(observation)
        return self.decide()

    def verify(self, is_wall: bool) -> None:
        if self.pending is None:
            raise NavigationError("Verification requires a pending movement.")
        self.set_wall(self.pending, is_wall)

    def reject(self) -> str:
        if self.pending is None:
            raise NavigationError("No pending movement can be rejected.")
        # Keep the path unchanged until the next arrival confirms a cell crossing.
        self.set_wall(self.pending, True)
        self.pending = None
        return self.decide()
