# simulator-micromouse

Simulation environment and maze-solving algorithms for a micromouse robot
(target hardware: STM32F401CCU6). Uses [mms](https://github.com/mackorone/mms)
to test algorithms against real competition mazes before porting to firmware.

## Structure

```
sim/
  algorithms/
    flood-fill-c/   Working flood fill algorithm (C) — BFS distance map,
                     shortest-path visualization, adapts to any maze size
    mms-cpp/        Unmodified C++ template, kept for reference
  webots/           Continuous physics simulation, including a rule-based
                     exploration scenario for the robotics assignment
  mms-app/          Prebuilt mms simulator (Windows)
  mms-src/          mms simulator source, for reference
  mazes/mazefiles/  522 real competition mazes (All Japan, APEC, AAMC, ...),
                     classic (16x16) / halfsize (32x32) / training
```

## Running the simulator

1. Launch `sim/mms-app/mms/mms.exe`.
2. Click **+** to add a new algorithm, using:
   - **Directory**: `sim/algorithms/flood-fill-c`
   - **Build Command**: `gcc -std=c11 -O2 -o algo.exe main.c solver.c API.c queue.c`
   - **Run Command**: full path to `algo.exe` in that directory (Windows
     requires an absolute path here, a bare filename won't resolve)
3. Pick a maze (default examples, or **open** any `.txt` file from
   `sim/mazes/mazefiles/classic/`) and click **Run**.

Requires a C compiler on PATH (MinGW-w64 `gcc`/`g++`).

## Algorithm

`flood-fill-c` implements flood fill: it builds a BFS distance map from the
goal to every reachable cell as walls are discovered, and always moves toward
the neighboring cell with the lowest distance. It reads the maze's actual
width/height from the simulator at startup, so it works with any maze size in
the archive (not just 16x16 classic). The goal-cell rule mirrors mms's own
convention: the geometric center of the maze (4 cells for an even x even
maze, 2 for one even dimension, 1 for odd x odd).

`floodFill()` in `solver.c` is where the move-selection logic lives, if you
want to tune or replace it.

## Continuous rule-based simulation

Open `sim/webots/micromouse_webots/worlds/micromouse_rule_based.wbt` in
Webots R2025a and run it. The robot explores using sensor rules, visit
memory, and backtracking over continuous wheel control.

See [the assignment specification](sim/webots/RULE_BASED.md) for the state
space, actions, initial state, goal, rules, execution steps, and validation.
