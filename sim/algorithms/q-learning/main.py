#!/usr/bin/env python3
"""
Q-learning sederhana untuk simulator Micromouse "mms" (github.com/mackorone/mms).

Cara pakai:
1. Buka simulator mms -> klik tombol "+" untuk menambah algoritma baru.
2. Isi:
   - Name       : bebas, mis. "QLearning"
   - Directory  : folder tempat main.py ini berada
   - Run command: `python main.py`
3. Klik "Run". Program menjalankan banyak episode training, lalu satu run
   akhir memakai kebijakan yang sudah dipelajari (greedy).

Q-learning standar:
- State  : posisi sel (x, y)
- Aksi   : 4 arah mutlak {N, E, S, W}; robot berbelok otomatis sebelum maju.
           Hanya arah yang TIDAK ada dindingnya (hasil baca sensor) yang
           boleh dipilih, supaya robot tidak menabrak (di mms, nabrak = crash).
- Reward : -1 tiap langkah, +100 saat sampai goal (sel tengah maze).
- Update : Q(s,a) <- Q(s,a) + ALPHA * (r + GAMMA * max Q(s',a') - Q(s,a))
- Pilih aksi: epsilon-greedy, epsilon turun tiap episode.

Antar episode robot pulang ke start dengan memutar balik jalurnya sendiri
(loop dibuang), jadi tidak perlu klik Reset di simulator tiap episode.

Asumsi koordinat mms: (0,0) kiri-bawah, mouse mulai menghadap Utara (N),
X ke Timur, Y ke Utara.
"""

import csv
import pickle
import random
import sys
import time
from collections import defaultdict

# ============================================================
# 1) API komunikasi dengan simulator (stdin/stdout)
#    Diadaptasi dari template resmi mackorone/mms-python.
# ============================================================

class MouseCrashedError(Exception):
    pass


def _command(args, return_type=None):
    line = " ".join(str(a) for a in args) + "\n"
    sys.stdout.write(line)
    sys.stdout.flush()
    if return_type:
        response = sys.stdin.readline().strip()
        if return_type == bool:
            return response == "true"
        return return_type(response)


def mazeWidth():
    return _command(["mazeWidth"], return_type=int)


def mazeHeight():
    return _command(["mazeHeight"], return_type=int)


def wallFront():
    return _command(["wallFront"], return_type=bool)


def wallRight():
    return _command(["wallRight"], return_type=bool)


def wallLeft():
    return _command(["wallLeft"], return_type=bool)


def wallBack():
    return _command(["wallBack"], return_type=bool)


def moveForward(distance=1):
    response = _command(["moveForward", distance], return_type=str)
    if response == "crash":
        raise MouseCrashedError()


def turnRight():
    _command(["turnRight"], return_type=str)


def turnLeft():
    _command(["turnLeft"], return_type=str)


def setColor(x, y, color):
    _command(["setColor", x, y, color])


def clearAllColor():
    _command(["clearAllColor"])


def wasReset():
    return _command(["wasReset"], return_type=bool)


def ackReset():
    _command(["ackReset"], return_type=str)


def eprint(*args, **kwargs):
    # stdout dipakai khusus protokol simulator, jadi semua log lewat stderr.
    print(*args, file=sys.stderr, **kwargs)
    sys.stderr.flush()


# ============================================================
# 2) Konstanta & hyperparameter
# ============================================================

DIRS = ["N", "E", "S", "W"]
DELTA = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}
OPPOSITE = {"N": "S", "S": "N", "E": "W", "W": "E"}

NUM_EPISODES = 500
ALPHA = 0.3           # learning rate
GAMMA = 0.95          # discount factor
EPSILON_START = 1.0   # peluang aksi acak di awal
EPSILON_MIN = 0.05
EPSILON_DECAY = 0.99  # dikalikan ke epsilon setelah tiap episode
PATIENCE = 100        # early stopping: berhenti kalau best_len tak membaik sekian episode

STEP_REWARD = -1.0
GOAL_REWARD = 100.0

STATE_FILE = "qtable_state.pkl"       # dibaca view_qtable.py
EPISODE_LOG_FILE = "episode_log.csv"  # dibaca plot_training.py


# ============================================================
# 3) Sensor & gerak
# ============================================================

def sense(x, y, heading, walls):
    """Baca sensor di sel (x, y), catat ke `walls` (dipakai view_qtable.py),
    dan kembalikan daftar arah mutlak yang terbuka (tanpa dinding)."""
    ci = DIRS.index(heading)
    readings = {
        DIRS[ci]: wallFront(),
        DIRS[(ci + 1) % 4]: wallRight(),
        DIRS[(ci + 3) % 4]: wallLeft(),
        DIRS[(ci + 2) % 4]: wallBack(),
    }
    walls[(x, y)] = readings
    return [d for d in DIRS if not readings[d]]


def turn_to_heading(current, target):
    """Berbelok seminimal mungkin ke arah target; kembalikan heading baru."""
    diff = (DIRS.index(target) - DIRS.index(current)) % 4
    if diff == 1:
        turnRight()
    elif diff == 3:
        turnLeft()
    elif diff == 2:
        turnRight()
        turnRight()
    return target


def goal_cells(w, h):
    xs = [w // 2 - 1, w // 2] if w % 2 == 0 else [w // 2]
    ys = [h // 2 - 1, h // 2] if h % 2 == 0 else [h // 2]
    return {(gx, gy) for gx in xs for gy in ys}


# ============================================================
# 4) Agen Q-learning
# ============================================================

class QLearningAgent:
    def __init__(self):
        self.q = defaultdict(float)  # (x, y, arah) -> nilai Q

    def best_action(self, x, y, open_dirs):
        best_v = max(self.q[(x, y, d)] for d in open_dirs)
        # Seri dipilih acak, bukan selalu arah pertama.
        return random.choice([d for d in open_dirs if self.q[(x, y, d)] == best_v])

    def choose_action(self, x, y, open_dirs, epsilon):
        if random.random() < epsilon:
            return random.choice(open_dirs)
        return self.best_action(x, y, open_dirs)

    def update(self, x, y, a, reward, nx, ny, next_open_dirs, terminal):
        if terminal:
            target = reward
        else:
            target = reward + GAMMA * max(self.q[(nx, ny, d)] for d in next_open_dirs)
        self.q[(x, y, a)] += ALPHA * (target - self.q[(x, y, a)])


# ============================================================
# 5) Satu episode
# ============================================================

def run_episode(agent, goals, walls, max_steps, epsilon, heading, learn=True):
    """Jalan dari (0,0) sampai goal atau kehabisan langkah.
    Return (jumlah_langkah, total_reward, sukses, (x, y), heading, trail),
    trail = daftar aksi jalur dari start ke posisi akhir TANPA loop."""
    x, y = 0, 0
    open_dirs = sense(x, y, heading, walls)
    trail_pos = [(x, y)]  # posisi di jalur tanpa loop
    trail_act = []        # aksi yang membawa dari trail_pos[i] ke trail_pos[i+1]
    total_reward = 0.0
    steps = 0

    while steps < max_steps and open_dirs:
        action = agent.choose_action(x, y, open_dirs, epsilon)
        heading = turn_to_heading(heading, action)
        moveForward(1)
        steps += 1

        nx, ny = x + DELTA[action][0], y + DELTA[action][1]
        terminal = (nx, ny) in goals
        reward = GOAL_REWARD if terminal else STEP_REWARD
        total_reward += reward

        next_open = sense(nx, ny, heading, walls)
        if learn:
            agent.update(x, y, action, reward, nx, ny, next_open, terminal)

        # Perbarui jalur tanpa loop: kalau sel ini sudah ada di jalur, potong.
        if (nx, ny) in trail_pos:
            i = trail_pos.index((nx, ny))
            del trail_pos[i + 1:]
            del trail_act[i:]
        else:
            trail_pos.append((nx, ny))
            trail_act.append(action)

        x, y, open_dirs = nx, ny, next_open
        if terminal:
            return steps, total_reward, True, heading, trail_act

    return steps, total_reward, False, heading, trail_act


def go_home(trail_act, heading):
    """Pulang ke start dengan membalik jalur. Aman tanpa sensor: tiap langkah
    mundur adalah kebalikan dari langkah maju yang tadi sudah berhasil."""
    for a in reversed(trail_act):
        heading = turn_to_heading(heading, OPPOSITE[a])
        moveForward(1)
    return heading


def color_path(trail_act):
    clearAllColor()
    x, y = 0, 0
    setColor(x, y, "b")
    for a in trail_act:
        x, y = x + DELTA[a][0], y + DELTA[a][1]
        setColor(x, y, "g")


# ============================================================
# 6) Main
# ============================================================

def main():
    w, h = mazeWidth(), mazeHeight()
    eprint(f"Ukuran maze: {w}x{h}")

    goals = goal_cells(w, h)
    agent = QLearningAgent()
    walls = {}
    max_steps = 4 * w * h
    epsilon = EPSILON_START
    heading = "N"  # tidak ada reset sungguhan antar episode, jadi heading diteruskan
    best_len = None
    no_improve = 0  # episode berturut-turut tanpa perbaikan best_len

    log_f = open(EPISODE_LOG_FILE, "w", newline="", encoding="utf-8")
    log = csv.writer(log_f)
    log.writerow(["episode", "steps", "success", "reward", "epsilon", "best_len"])

    try:
        for ep in range(1, NUM_EPISODES + 1):
            steps, reward, success, heading, trail = run_episode(
                agent, goals, walls, max_steps, epsilon, heading
            )
            if success and (best_len is None or steps < best_len):
                best_len = steps
                no_improve = 0
                color_path(trail)
            elif best_len is not None:
                no_improve += 1

            log.writerow([ep, steps, int(success), reward, epsilon, best_len or ""])
            log_f.flush()
            eprint(
                f"[Episode {ep}/{NUM_EPISODES}] "
                f"{'sampai tujuan' if success else 'kehabisan langkah'} "
                f"dalam {steps} langkah, reward={reward:.1f}, "
                f"epsilon={epsilon:.3f}, terbaik={best_len}"
            )

            heading = go_home(trail, heading)
            epsilon = max(EPSILON_MIN, epsilon * EPSILON_DECAY)

            if no_improve >= PATIENCE:
                eprint(f"Early stopping: best_len={best_len} tidak membaik "
                       f"selama {PATIENCE} episode (berhenti di episode {ep}).")
                break

        with open(STATE_FILE, "wb") as f:
            pickle.dump({"q": dict(agent.q), "walls": walls, "w": w, "h": h}, f)

        eprint("Training selesai. Run akhir (greedy, tanpa eksplorasi)...")
        steps, _, success, heading, trail = run_episode(
            agent, goals, walls, max_steps, 0.0, heading, learn=False
        )
        if success:
            eprint(f"Run akhir sampai tujuan dalam {steps} langkah.")
            color_path(trail)
        else:
            eprint("Run akhir belum sampai tujuan -- coba naikkan NUM_EPISODES.")
    except MouseCrashedError:
        eprint("Mouse crash -- seharusnya tidak terjadi; cek arah/koordinat maze.")
        return
    finally:
        log_f.close()

    eprint("Selesai. Menunggu (tekan reset di simulator jika ingin mengulang).")
    while True:
        if wasReset():
            clearAllColor()
            ackReset()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
