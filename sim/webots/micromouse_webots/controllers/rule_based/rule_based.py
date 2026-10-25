"""Run explicit exploration rules over continuous differential-drive motion."""

import json
import math
import os
import sys
from pathlib import Path

from navigation import Direction, NavigationError, RuleBasedNavigator


HERE = Path(__file__).resolve().parent
WALL_CONTACT_MIN_HEIGHT = 0.003
sys.path.insert(0, str(HERE.parent / "micromouse_controller"))


def main() -> int:
    import micromouse_controller as motion

    motion.log_file.close()
    motion.log_path = str(HERE / "run_log.txt")
    motion.log_file = open(motion.log_path, "w", encoding="utf-8")
    started = motion.robot.getTime()
    original_step = motion.step
    wall_contact_steps = 0
    monitor_contacts = False
    contact_active = False
    contact_events = 0

    try:
        metadata = motion.robot.getCustomData()
        if not metadata:
            raise ValueError("World metadata is missing; regenerate the world with maze_to_webots.py.")
        config = json.loads(metadata)
        width, height = config["maze_width"], config["maze_height"]
        if type(width) is not int or type(height) is not int:
            raise ValueError("World maze dimensions must be integers.")
        if not math.isclose(config["cell_size"], motion.CELL, abs_tol=1e-9):
            raise ValueError("This controller requires classic 0.18 m cells.")
        if motion.robot.getBasicTimeStep() != motion.TIME_STEP:
            raise ValueError("The world and controller time steps must match.")
        max_steps = int(os.environ.get("MM_RULE_MAX_STEPS", str(2 * width * height)))
        timeout = float(os.environ.get("MM_RULE_TIMEOUT", "1800"))
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("MM_RULE_TIMEOUT must be finite and positive.")
        navigator = RuleBasedNavigator(width, height, max_steps)
        x, y, _ = motion.self_node.getPosition()
        orientation = motion.self_node.getOrientation()
        angle = math.atan2(orientation[3], orientation[0])
        position_error = math.hypot(x - motion.CELL / 2, y - motion.CELL / 2)
        if position_error > 0.005 or abs(angle - math.pi / 2) > 0.01:
            raise ValueError("The robot must start at cell (0, 0), facing north.")
        motion.self_node.enableContactPointsTracking(motion.TIME_STEP, True)

        def checked_step():
            nonlocal wall_contact_steps, contact_active, contact_events
            original_step()
            if motion.robot.getTime() - started > timeout:
                raise NavigationError("The simulation time limit was exceeded.")
            # Supervisor contact data is an evaluation metric, never a direction input.
            contacts = []
            if monitor_contacts:
                # Exclude floor contacts near z=0 from wall-contact metrics.
                contacts = [contact for contact in motion.self_node.getContactPoints(True)
                            if contact.point[2] > WALL_CONTACT_MIN_HEIGHT]
            if contacts:
                wall_contact_steps += 1
                if not contact_active and contact_events < 12:
                    contact_events += 1
                    motion.log(f"RULE CONTACT: time={motion.robot.getTime() - started:.2f}s "
                               f"points={[(point.point, point.node_id) for point in contacts]}")
                    motion.log_true_pose()
            contact_active = bool(contacts)

        motion.step = checked_step
        motion.log(f"RULE start: maze={width}x{height}, goals={sorted(navigator.goals)}")
        motion.log("RULE state: continuous pose and wheel motion; discrete visit memory")
        motion.log_true_pose()
        motion.square_to_back_wall()
        motion.center()
        monitor_contacts = True

        # The provided classic start has three closed sides and faces north.
        navigator.set_wall(Direction.SOUTH, True)
        front, _ = motion.read_front_wall()
        navigator.observe({"front": front, "left": True, "right": True})
        action = navigator.decide()

        def log_decision(action, observation=None):
            readings = None
            if observation is not None:
                readings = {key: observation[key] for key in ("front", "left", "right")}
            motion.log(
                f"RULE step={navigator.steps} cell={navigator.cell} "
                f"entry={navigator.entry_heading.name} target={navigator.heading.name} action={action} "
                f"rule={navigator.last_rule} visited={len(navigator.visited)} "
                f"walls={readings}"
            )

        def on_cell(observation):
            action = navigator.arrive(observation)
            # Memory may resolve an ambiguous late reading for the motion layer.
            for key, turn in (("left", -1), ("right", 1)):
                direction = Direction((navigator.entry_heading + turn) % 4)
                observation[key] = navigator.wall(direction)
            log_decision(action, observation)
            return action

        def on_verify(is_wall):
            navigator.verify(is_wall)
            motion.log(f"RULE verify: cell={navigator.cell} wall={is_wall}")

        def on_blocked():
            action = navigator.reject()
            log_decision(action)
            return action

        log_decision(action)
        if action != "STOP":
            if action != "F":
                raise NavigationError("The supplied start must open north.")
            motion.search_run(
                on_cell,
                on_blocked,
                late_arc_with_front=False,
                conservative_sides=True,
                on_verify=on_verify,
            )

        motion.settle()
        x, y, _ = motion.self_node.getPosition()
        actual_cell = (math.floor(x / motion.CELL), math.floor(y / motion.CELL))
        physical_goal = actual_cell in navigator.goals
        success = navigator.status == "goal" and physical_goal and wall_contact_steps == 0
        motion.log(
            f"RULE RESULT: status={navigator.status} success={success} "
            f"steps={navigator.steps} visited={len(navigator.visited)} "
            f"actual_cell={actual_cell} physical_goal={physical_goal} "
            f"wall_contact_steps={wall_contact_steps} "
            f"time={motion.robot.getTime() - started:.2f}s"
        )
        motion.log_true_pose()
        return 0 if success else 1
    except Exception as error:
        motion.log(f"RULE ERROR: {type(error).__name__}: {error}")
        return 1
    finally:
        motion.set_wheels(0.0, 0.0)
        motion.step = original_step
        motion.log_file.flush()


if __name__ == "__main__":
    exit_code = main()
    import micromouse_controller as motion

    sys.stdout.flush()
    if os.environ.get("MICROMOUSE_QUIT") == "1":
        motion.robot.simulationQuit(exit_code)
    else:
        while motion.robot.step(motion.TIME_STEP) != -1:
            pass
