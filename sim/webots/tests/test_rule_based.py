"""Exercise exploration and recovery without starting a Webots process."""

import ast
import importlib.util
import json
import math
import sys
import unittest
from collections import deque
from pathlib import Path


WEBOTS = Path(__file__).resolve().parents[1]
REPO = WEBOTS.parents[1]
CONTROLLERS = WEBOTS / "micromouse_webots" / "controllers"
sys.path.insert(0, str(CONTROLLERS / "rule_based"))
sys.path.insert(0, str(WEBOTS / "tools"))

from navigation import Direction, NavigationError, RuleBasedNavigator, goal_cells
from maze_to_webots import generate_robot, parse_maze


def load_maze(path):
    width, height, horizontal, vertical = parse_maze(path)

    def wall(cell, direction):
        x, y = cell
        return (
            horizontal[height - y - 1][x],
            vertical[height - y - 1][x + 1],
            horizontal[height - y][x],
            vertical[height - y - 1][x],
        )[direction]

    return width, height, wall


def observation(navigator, wall):
    return {
        key: wall(navigator.cell, Direction((navigator.heading + turn) % 4))
        for key, turn in (("front", 0), ("left", -1), ("right", 1))
    }


def reachable_goal(width, height, wall):
    goals = goal_cells(width, height)
    seen = {(0, 0)}
    queue = deque(seen)
    while queue:
        cell = queue.popleft()
        if cell in goals:
            return True
        for direction, (dx, dy) in enumerate(((0, 1), (1, 0), (0, -1), (-1, 0))):
            neighbor = cell[0] + dx, cell[1] + dy
            if (
                0 <= neighbor[0] < width
                and 0 <= neighbor[1] < height
                and not wall(cell, direction)
                and neighbor not in seen
            ):
                seen.add(neighbor)
                queue.append(neighbor)
    return False


class NavigationTests(unittest.TestCase):
    def test_all_archived_mazes_match_reachability(self):
        archive = REPO / "sim" / "mazes" / "mazefiles"
        paths = sorted(path for category in ("classic", "halfsize", "training")
                       for path in (archive / category).glob("*.txt"))
        self.assertGreaterEqual(len(paths), 580)
        for path in paths:
            with self.subTest(maze=path.name):
                width, height, wall = load_maze(path)
                navigator = RuleBasedNavigator(width, height)
                navigator.observe(observation(navigator, wall))
                # A real robot also knows the wall behind its initial pose.
                navigator.set_wall(Direction.SOUTH, wall((0, 0), Direction.SOUTH))
                action = navigator.decide()
                while action != "STOP":
                    self.assertFalse(wall(navigator.cell, navigator.pending))
                    destination = navigator.neighbor(navigator.pending)
                    readings = {
                        key: wall(destination, Direction((navigator.heading + turn) % 4))
                        for key, turn in (("front", 0), ("left", -1), ("right", 1))
                    }
                    action = navigator.arrive(readings)
                expected = "goal" if reachable_goal(width, height, wall) else "exhausted"
                self.assertEqual(navigator.status, expected)
                self.assertLessEqual(navigator.steps, 2 * width * height - 2)

    def test_rejected_moves_do_not_change_visit_memory(self):
        navigator = RuleBasedNavigator(4, 4)
        navigator.observe({"front": False, "left": True, "right": False})
        self.assertEqual(navigator.decide(), "F")
        self.assertEqual(navigator.reject(), "R")
        self.assertEqual(navigator.path, [(0, 0)])
        self.assertEqual(navigator.visited, {(0, 0)})
        self.assertEqual(navigator.steps, 0)
        self.assertEqual(navigator.reject(), "STOP")
        self.assertEqual(navigator.status, "exhausted")

    def test_ambiguous_side_requires_verification(self):
        navigator = RuleBasedNavigator(4, 4)
        navigator.observe({"front": True, "left": True, "right": None})
        self.assertEqual(navigator.decide(), "R")
        navigator.verify(True)
        self.assertEqual(navigator.reject(), "STOP")

    def test_missing_front_and_invalid_feedback_fail_safely(self):
        navigator = RuleBasedNavigator(4, 4)
        with self.assertRaises(NavigationError):
            navigator.decide()
        with self.assertRaises(NavigationError):
            navigator.arrive({"front": False, "left": True, "right": True})
        with self.assertRaises(NavigationError):
            navigator.reject()
        with self.assertRaises(NavigationError):
            navigator.set_wall(Direction.WEST, False)

    def test_goal_parity_and_configuration(self):
        for width, height, count in ((4, 4, 4), (3, 4, 2), (4, 3, 2), (3, 3, 1)):
            self.assertEqual(len(goal_cells(width, height)), count)
        self.assertEqual(RuleBasedNavigator(1, 1).decide(), "STOP")
        for width, height, limit in ((0, 4, None), (4, -1, None), (4, 4, 0)):
            with self.assertRaises(ValueError):
                RuleBasedNavigator(width, height, limit)

    def test_step_limit_is_not_reported_as_success(self):
        navigator = RuleBasedNavigator(6, 6, max_steps=1)
        navigator.observe({"front": False, "left": True, "right": True})
        navigator.decide()
        self.assertEqual(navigator.arrive({"front": False, "left": True, "right": True}), "STOP")
        self.assertEqual(navigator.status, "step_limit")


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        source = CONTROLLERS / "micromouse_controller" / "micromouse_controller.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        function = next(node for node in tree.body
                        if isinstance(node, ast.FunctionDef) and node.name == "_verify_search_exit")
        self.turns = []
        self.readings = iter(())
        namespace = {
            "_point_turn": self.turns.append,
            "read_front_wall": lambda: (next(self.readings), 0),
            "log": lambda message: None,
            "StallError": RuntimeError,
        }
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), "exec"), namespace)
        self.verify_exit = namespace["_verify_search_exit"]

    def test_replacement_exits_are_also_verified(self):
        self.readings = iter((True, True, False))
        replacements = iter(("R", "U"))
        verified = []
        self.assertTrue(self.verify_exit("L", lambda: next(replacements), verified.append))
        self.assertEqual(verified, [True, True, False])
        self.assertEqual(self.turns, ["left", "right", "left", "left"])

    def test_stop_after_rejection_does_not_move(self):
        self.readings = iter((True,))
        self.assertFalse(self.verify_exit("F", lambda: "STOP", None))
        self.assertEqual(self.turns, [])

    def test_recovery_attempts_are_bounded(self):
        self.readings = iter((True,) * 4)
        with self.assertRaises(RuntimeError):
            self.verify_exit("L", lambda: "R", None)
        with self.assertRaises(ValueError):
            self.verify_exit("invalid", lambda: "STOP", None)

    def test_rejected_side_restores_correct_distance_along_final_heading(self):
        source = CONTROLLERS / "micromouse_controller" / "micromouse_controller.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                     and node.name in ("search_run", "_verify_search_exit")]
        lengths, shifts = [], []
        choices = iter(("L", "STOP"))
        fronts = iter((True, False))
        observations = iter((
            {"front": True, "left": None, "right": False},
            {"front": False, "left": True, "right": True},
        ))
        namespace = {
            "math": math, "CELL": 0.18, "DECIDE_BEFORE": 0.095,
            "LATE_DECIDE_BEFORE": 0.07, "V_SEARCH": 0.22, "SIDE_THRESHOLD": 145,
            "FRONT_STOP_VALUE": 1160, "SAFE_STOP_NOSE": 0.037,
            "PCB_FRONT_AHEAD": 0.066, "HALF_LANE": 0.084,
            "NUDGE_MIN": 0.01, "NUDGE_MAX": 0.04, "target_yaw": 0.0,
            "center": lambda: None, "settle": lambda: None,
            "log": lambda message: None, "StallError": RuntimeError,
            "_early_observation": lambda *args: next(observations),
            "read_front_wall": lambda: (next(fronts), 0),
            "_lateral_nudge": lambda shift, **kwargs: shifts.append(shift),
        }

        def point_turn(direction):
            namespace["target_yaw"] += math.pi / 2 if direction == "left" else -math.pi / 2

        def straight(length, *args, **kwargs):
            lengths.append(length)
            return {}

        namespace["_point_turn"] = point_turn
        namespace["_run_straight"] = straight
        exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), "exec"), namespace)
        cells = namespace["search_run"](lambda obs: next(choices), lambda: "R")
        self.assertEqual(cells, 2)
        self.assertEqual(shifts, [])
        self.assertAlmostEqual(namespace["target_yaw"], 0.0)
        self.assertAlmostEqual(lengths[2], 0.104)


class LateralMotionTests(unittest.TestCase):
    def test_both_passages_preserve_final_heading_and_longitudinal_position(self):
        source = CONTROLLERS / "micromouse_controller" / "micromouse_controller.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        function = next(node for node in tree.body
                        if isinstance(node, ast.FunctionDef) and node.name == "_lateral_nudge")
        for forward in (False, True):
            for shift in (-0.019, 0.019):
                with self.subTest(forward=forward, shift=shift):
                    pose = [0.0, 0.0, 0.0]
                    drives = []

                    def rotate(angle):
                        pose[2] += angle

                    def drive(distance):
                        drives.append(distance)
                        pose[0] += distance * math.cos(pose[2])
                        pose[1] += distance * math.sin(pose[2])

                    namespace = {
                        "math": math, "NUDGE_ANGLE": math.radians(15),
                        "_rotate_by": rotate, "_drive": drive,
                        "settle": lambda: None, "log": lambda message: None,
                    }
                    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), "exec"), namespace)
                    namespace["_lateral_nudge"](shift, forward=forward)
                    self.assertAlmostEqual(pose[0], 0.0)
                    self.assertAlmostEqual(pose[1], shift)
                    self.assertAlmostEqual(pose[2], 0.0)
                    self.assertEqual(drives[0] > 0, forward)


class WorldTests(unittest.TestCase):
    def test_generated_metadata_matches_controller_configuration(self):
        robot = generate_robot(0.18, "rule_based", maze_size=(16, 16))
        line = next(line.strip() for line in robot.splitlines() if "customData" in line)
        config = json.loads(json.loads(line.removeprefix("customData ")))
        self.assertEqual(config, {"maze_width": 16, "maze_height": 16, "cell_size": 0.18})
        self.assertIn('controller "rule_based"', robot)

    def test_controller_is_safe_to_import_without_webots(self):
        path = CONTROLLERS / "rule_based" / "rule_based.py"
        spec = importlib.util.spec_from_file_location("rule_based_entry", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertTrue(callable(module.main))


if __name__ == "__main__":
    unittest.main()
