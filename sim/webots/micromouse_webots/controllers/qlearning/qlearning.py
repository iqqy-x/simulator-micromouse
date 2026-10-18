#!/usr/bin/env python3
"""
Q-learning di Webots: algoritma yang sama dengan sim/algorithms/q-learning
(Q-learning standar tabular), dijalankan di atas gerak berkelanjutan
robot (search_run).

  - State  : posisi sel (x, y), dilacak dari langkah yang dijalankan.
  - Aksi   : arah mutlak {N, E, S, W} yang tidak berdinding menurut sensor
             (depan/kiri/kanan di titik keputusan tiap sel). Arah balik hanya
             dipilih kalau jalan buntu (tanpa itu robot sering berputar balik
             di tempat, yang lambat). Pembacaan None dianggap dinding.
  - Reward : -1 tiap langkah (pindah sel), +100 saat masuk sel tujuan
             (menggantikan -1).
  - Update : Q(s,a) <- Q(s,a) + ALPHA * (r + GAMMA * max Q(s',a') - Q(s,a)),
             s' terminal -> target = r.
  - Pilih aksi: epsilon-greedy, epsilon turun tiap episode.
  - Berhenti: early stopping (best_len tak membaik PATIENCE episode) atau
             EPISODES; lalu satu episode greedy dan lari cepat (run_path)
             di jalur tanpa loop dari kebijakan itu.

Beda dengan versi mms: antar episode robot DIPINDAHKAN ke start lewat
Supervisor (tidak pulang membalik jalur), dan langkah episode dibatasi
MAX_STEPS (lebih kecil dari 4*w*h karena tiap langkah ~1 detik simulasi).

File maze hanya dipakai untuk ukuran labirin dan menilai hasil (panjang
jalur optimal); dinding tidak pernah dipakai untuk menentukan aksi.

Variabel lingkungan (semua opsional):
  MM_EPISODES (500)  MM_PATIENCE (100)  MM_MAX_STEPS (400)  MM_EPS_DECAY (0.99)
  MM_ALPHA (0.3)  MM_GAMMA (0.95)  MM_SEED  MM_MAZE (file maze, bawaan practice-8x8-a)
  MICROMOUSE_QUIT=1 -> Webots keluar sendiri setelah selesai.

Keluaran di folder controller ini: episode_log.csv dan qtable_state.pkl.
"""

import csv
import math
import os
import pickle
import random
import sys
from collections import defaultdict, deque

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "micromouse_controller"))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..", "..", "tools")))

import micromouse_controller as c  # noqa: E402
from maze_to_webots import parse_maze  # noqa: E402

# Console (panel Webots) meniru versi mms (sim/algorithms/q-learning/main.py):
# "Ukuran maze", satu baris per episode, early stopping, run akhir. Log gerak
# pustaka dan keterangan tambahan hanya ke run_log.txt.
c.LOG_TO_CONSOLE = False


def out(msg):
    print(msg, flush=True)
    c.log_file.write(msg + "\n")
    c.log_file.flush()

DEFAULT_MAZE = os.path.abspath(os.path.join(
    HERE, "..", "..", "..", "..", "mazes", "mazefiles", "training", "practice-8x8-a.txt"))
MAZE_FILE = os.environ.get("MM_MAZE") or DEFAULT_MAZE

EPISODES = int(os.environ.get("MM_EPISODES", "500"))
PATIENCE = int(os.environ.get("MM_PATIENCE", "100"))
MAX_STEPS = int(os.environ.get("MM_MAX_STEPS", "400"))
ALPHA = float(os.environ.get("MM_ALPHA", "0.3"))
GAMMA = float(os.environ.get("MM_GAMMA", "0.95"))
EPS_START = 1.0
EPS_MIN = 0.05
EPS_DECAY = float(os.environ.get("MM_EPS_DECAY", "0.99"))
STEP_REWARD = -1.0
GOAL_REWARD = 100.0
if os.environ.get("MM_SEED"):
    random.seed(int(os.environ["MM_SEED"]))

N, E, S, W_ = 0, 1, 2, 3
DX = (0, 1, 0, -1)
DY = (1, 0, -1, 0)
NAME = "NESW"

m_width, m_rows, m_horiz, m_vert = parse_maze(MAZE_FILE)
WIDTH, HEIGHT = m_width, m_rows
LOG_FILE = os.path.join(HERE, "episode_log.csv")
STATE_FILE = os.path.join(HERE, "qtable_state.pkl")


def truth_wall(x, y, d):
    """Dinding sebenarnya menurut file maze -- HANYA untuk menilai hasil."""
    if d == N:
        return m_horiz[m_rows - y - 1][x]
    if d == S:
        return m_horiz[m_rows - y][x]
    if d == E:
        return m_vert[m_rows - 1 - y][x + 1]
    return m_vert[m_rows - 1 - y][x]


def goal_cells(w, h):
    xs = [w // 2 - 1, w // 2] if w % 2 == 0 else [w // 2]
    ys = [h // 2 - 1, h // 2] if h % 2 == 0 else [h // 2]
    return {(gx, gy) for gx in xs for gy in ys}


GOALS = goal_cells(WIDTH, HEIGHT)


def truth_shortest(start, goals):
    dist = {start: 0}
    queue = deque([start])
    while queue:
        x, y = queue.popleft()
        if (x, y) in goals:
            return dist[(x, y)]
        for d in range(4):
            nx, ny = x + DX[d], y + DY[d]
            if truth_wall(x, y, d) or (nx, ny) in dist or not (0 <= nx < WIDTH and 0 <= ny < HEIGHT):
                continue
            dist[(nx, ny)] = dist[(x, y)] + 1
            queue.append((nx, ny))
    return None


class QAgent:
    def __init__(self):
        self.q = defaultdict(float)   # (x, y, arah) -> nilai Q

    def best_action(self, x, y, dirs):
        best = max(self.q[(x, y, d)] for d in dirs)
        return random.choice([d for d in dirs if self.q[(x, y, d)] == best])

    def choose(self, x, y, dirs, epsilon):
        if random.random() < epsilon:
            return random.choice(dirs)
        return self.best_action(x, y, dirs)

    def update(self, x, y, a, reward, nx, ny, next_dirs, terminal):
        if terminal:
            target = reward
        else:
            target = reward + GAMMA * max(self.q[(nx, ny, d)] for d in next_dirs)
        self.q[(x, y, a)] += ALPHA * (target - self.q[(x, y, a)])


def reset_to_start():
    """Pindahkan robot ke tengah sel start menghadap Utara (Supervisor) dan
    reset keadaan pustaka gerak (arah acuan gyro, offset)."""
    c.set_wheels(0.0, 0.0)
    c.self_node.getField("translation").setSFVec3f([0.5 * c.CELL, 0.5 * c.CELL, c.WHEEL_R])
    c.self_node.getField("rotation").setSFRotation([0, 0, 1, math.pi / 2])
    c.self_node.resetPhysics()
    c.yaw = 0.0
    c.target_yaw = 0.0
    c.offset = 0.0
    for _ in range(10):
        c.step()


def open_dirs_from(obs, h):
    """Arah mutlak yang boleh dipilih di sel ini: yang tidak berdinding menurut
    sensor (None dianggap dinding); balik hanya kalau jalan buntu."""
    dirs = []
    if obs["front"] is False:
        dirs.append(h)
    if obs["left"] is not True:      # False atau None (ambigu: diverifikasi robot saat berbelok)
        dirs.append((h + 3) % 4)
    if obs["right"] is not True:
        dirs.append((h + 1) % 4)
    return dirs or [(h + 2) % 4]


def log_cell(ep, obs, action):
    """Catat keputusan tiap sel ke run_log.txt, tandai pembacaan yang salah."""
    h = ep["h"]
    parts = []
    for label, d_abs, key in (("depan", h, "front"), ("kiri", (h + 3) % 4, "left"),
                              ("kanan", (h + 1) % 4, "right")):
        seen = obs[key]
        truth = truth_wall(ep["x"], ep["y"], d_abs)
        flag = "" if seen is None or seen == truth else " <-- SALAH"
        parts.append(f"{label}={'?' if seen is None else int(seen)}{flag}")
    c.log(f"QL sel ({ep['x']},{ep['y']}) arah {NAME[h]}{' (terlambat)' if obs.get('late') else ''}: "
          + " ".join(parts) + f" -> {action}")


def run_episode(agent, epsilon, learn, walls):
    """Satu episode dari start sampai tujuan atau MAX_STEPS.
    Return dict: steps, reward, success, trail (urutan sel), sim_time."""
    reset_to_start()
    t0 = c.robot.getTime()
    ep = {"x": 0, "y": 0, "h": N, "steps": 0, "reward": 0.0, "success": False,
          "trail": [(0, 0)], "prev": (0, 0, N), "dirs": [N]}

    def record_walls(obs):
        x, y, h = ep["x"], ep["y"], ep["h"]
        cell = walls.setdefault((x, y), {})
        for d_abs, key in ((h, "front"), ((h + 3) % 4, "left"), ((h + 1) % 4, "right")):
            if obs[key] is not None:
                cell[NAME[d_abs]] = bool(obs[key])
        cell[NAME[(h + 2) % 4]] = False   # baru dilewati

    def rel_action(d):
        return {0: "F", 1: "R", 3: "L", 2: "U"}[(d - ep["h"]) % 4]

    def on_cell(obs):
        ep["x"] += DX[ep["h"]]
        ep["y"] += DY[ep["h"]]
        ep["steps"] += 1
        ep["trail"].append((ep["x"], ep["y"]))
        ep["entry_h"] = ep["h"]        # arah masuk sel ini: kebalikannya = tempat asal
        goal = (ep["x"], ep["y"]) in GOALS
        reward = GOAL_REWARD if goal else STEP_REWARD
        ep["reward"] += reward
        # Di sel "terlambat" (tepat sesudah busur) pembacaan samping kurang andal:
        # kalau sisi itu sudah pernah teramati di kunjungan lain, pakai ingatannya
        # (salah baca 'terbuka' membuat robot berbelok ke dinding).
        if obs.get("late"):
            obs = dict(obs)
            for key, d_abs in (("left", (ep["h"] + 3) % 4), ("right", (ep["h"] + 1) % 4)):
                remembered = walls.get((ep["x"], ep["y"]), {}).get(NAME[d_abs])
                if remembered is not None:
                    obs[key] = remembered
        record_walls(obs)
        dirs = open_dirs_from(obs, ep["h"])
        if learn:
            px, py, pa = ep["prev"]
            agent.update(px, py, pa, reward, ep["x"], ep["y"], dirs, goal)
        if goal:
            ep["success"] = True
            return "STOP"
        if ep["steps"] >= MAX_STEPS:
            return "STOP"
        d = agent.choose(ep["x"], ep["y"], dirs, epsilon)
        ep["prev"] = (ep["x"], ep["y"], d)
        ep["dirs"] = dirs
        action = rel_action(d)
        log_cell(ep, obs, action)
        ep["h"] = d
        return action

    def on_verify(is_wall):
        """Hasil verifikasi sisi ambigu: dinding depan setelah berputar ke sana."""
        walls.setdefault((ep["x"], ep["y"]), {})[NAME[ep["h"]]] = bool(is_wall)

    def on_blocked():
        """Dinding depan ada padahal terbaca terbuka (henti darurat): pilih arah
        lain dari pilihan sel ini; aksi yang gagal diganti di catatan Q."""
        blocked = ep["h"]
        # kalau tak ada pilihan lain, kembali ke sel asal (kebalikan arah MASUK sel ini),
        # bukan kebalikan arah yang diblokir (bisa saja dinding lain)
        dirs = [d for d in ep["dirs"] if d != blocked] or [(ep["entry_h"] + 2) % 4]
        d = agent.choose(ep["x"], ep["y"], dirs, epsilon)
        ep["prev"] = (ep["x"], ep["y"], d)
        action = rel_action(d)
        ep["h"] = d
        if action == "F":
            raise RuntimeError("henti darurat tapi aksi baru lurus")
        return action

    # Eksplorasi acak lebih rentan pada belokan sempit: tanpa busur di sel terlambat
    # kalau ada dinding depan (berhenti dan putar di tempat).
    try:
        c.search_run(on_cell, on_blocked, late_arc_with_front=False, conservative_sides=True,
                     on_verify=on_verify)
    except c.StallError as e:
        # robot tertahan (badan menekan dinding): episode GAGAL, robot dipindah ke start
        ep["success"] = False
        ep["stalled"] = True
        c.log(f"QL episode berhenti: {e} di sel ({ep['x']},{ep['y']})")
    ep["sim_time"] = c.robot.getTime() - t0
    return ep


def loop_erased(trail):
    """Buang loop dari urutan sel (seperti trail_pos di versi mms)."""
    out = []
    pos = {}
    for cell in trail:
        if cell in pos:
            i = pos[cell]
            for dropped in out[i + 1:]:
                pos.pop(dropped, None)
            del out[i + 1:]
        else:
            pos[cell] = len(out)
            out.append(cell)
    return out


def relative_plan(cells):
    plan = []
    heading = N
    for (x0, y0), (x1, y1) in zip(cells, cells[1:]):
        d = {(0, 1): N, (1, 0): E, (0, -1): S, (-1, 0): W_}[(x1 - x0, y1 - y0)]
        plan.append({0: "F", 1: "R", 3: "L"}[(d - heading) % 4])
        heading = d
    return "".join(plan)


def main():
    agent = QAgent()
    walls = {}
    optimal = truth_shortest((0, 0), GOALS)
    out(f"Ukuran maze: {WIDTH}x{HEIGHT}")
    c.log(f"QL labirin {os.path.basename(MAZE_FILE)}, tujuan {sorted(GOALS)}, "
          f"optimal menurut file maze = {optimal} langkah; EPISODES={EPISODES} PATIENCE={PATIENCE} "
          f"MAX_STEPS={MAX_STEPS} ALPHA={ALPHA} GAMMA={GAMMA} EPS_DECAY={EPS_DECAY}")
    epsilon = EPS_START
    best_len = None
    no_improve = 0
    t_begin = c.robot.getTime()

    with open(LOG_FILE, "w", newline="", encoding="utf-8") as f:
        log = csv.writer(f)
        log.writerow(["episode", "steps", "success", "reward", "epsilon", "best_len", "sim_time"])
        for episode in range(1, EPISODES + 1):
            ep = run_episode(agent, epsilon, True, walls)
            if ep["success"] and (best_len is None or ep["steps"] < best_len):
                best_len = ep["steps"]
                no_improve = 0
            elif best_len is not None:
                no_improve += 1
            log.writerow([episode, ep["steps"], int(ep["success"]), ep["reward"], f"{epsilon:.4f}",
                          best_len or "", f"{ep['sim_time']:.1f}"])
            f.flush()
            out(f"[Episode {episode}/{EPISODES}] "
                f"{'sampai tujuan' if ep['success'] else 'kehabisan langkah'} "
                f"dalam {ep['steps']} langkah, reward={ep['reward']:.1f}, "
                f"epsilon={epsilon:.3f}, terbaik={best_len}")
            c.log(f"QL episode {episode}: {ep['sim_time']:.0f} s simulasi")
            epsilon = max(EPS_MIN, epsilon * EPS_DECAY)
            if no_improve >= PATIENCE:
                out(f"Early stopping: best_len={best_len} tidak membaik "
                    f"selama {PATIENCE} episode (berhenti di episode {episode}).")
                break

    with open(STATE_FILE, "wb") as f:
        pickle.dump({"q": dict(agent.q), "walls": walls, "w": WIDTH, "h": HEIGHT}, f)
    c.log(f"QL training: {c.robot.getTime() - t_begin:.0f} s simulasi, "
          f"{len(agent.q)} pasangan (sel, arah) di Q-table")
    out("Training selesai. Run akhir (greedy, tanpa eksplorasi)...")

    # --- episode greedy + lari cepat -----------------------------------------------
    ep = run_episode(agent, 0.0, False, walls)
    if ep["success"]:
        out(f"Run akhir sampai tujuan dalam {ep['steps']} langkah.")
    else:
        out("Run akhir belum sampai tujuan -- coba naikkan NUM_EPISODES.")
    c.log(f"QL episode greedy: {ep['steps']} langkah (optimal {optimal}), "
          f"{ep['sim_time']:.0f} s simulasi")
    if ep["success"]:
        cells = loop_erased(ep["trail"])
        plan = relative_plan(cells)
        c.log(f"QL jalur tanpa loop: {len(cells) - 1} langkah, rencana {plan}")
        reset_to_start()
        t0 = c.robot.getTime()
        c.run_path(plan)
        run_time = c.robot.getTime() - t0
        x, y, _ = c.self_node.getPosition()
        cell = (int(x // c.CELL), int(y // c.CELL))
        c.log(f"QL lari cepat: {run_time:.1f} s, rata-rata {(len(cells) - 1) * c.CELL / run_time:.2f} m/s, "
              f"berhenti di sel {cell} ({'di tujuan' if cell in GOALS else 'BUKAN di tujuan'})")

    if os.environ.get("MICROMOUSE_QUIT"):
        c.robot.simulationQuit(0)
    out("Selesai. Menunggu (tekan reset di simulator jika ingin mengulang).")
    while c.robot.step(c.TIME_STEP) != -1:
        pass


try:
    main()
except BaseException:
    import traceback
    out("QL GALAT: " + " | ".join(traceback.format_exc().splitlines()))
    if os.environ.get("MICROMOUSE_QUIT"):
        c.robot.simulationQuit(1)
