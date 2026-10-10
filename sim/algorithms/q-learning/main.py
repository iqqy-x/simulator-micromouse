#!/usr/bin/env python3
"""
Algoritma Q-learning untuk simulator Micromouse "mms" (github.com/mackorone/mms).

Cara pakai:
1. Simpan file ini sebagai `main.py` di sebuah folder khusus (mis. `mm_rl/main.py`).
2. Buka simulator mms -> klik tombol "+" untuk menambah algoritma baru.
3. Isi:
   - Name       : bebas, mis. "QLearning"
   - Directory  : folder tempat main.py ini berada
   - Run command: `python3 main.py`  (atau `python main.py` di Windows)
4. Klik "Run". Program akan menjalankan banyak episode eksplorasi (training),
   lalu satu run akhir memakai kebijakan yang sudah dipelajari (greedy).

Desain singkat:
- State  : posisi sel (x, y) — arah hadap (heading) dilacak sendiri, tidak
           dimasukkan ke Q-table, supaya state space tetap kecil (W*H*4 aksi).
- Aksi   : 4 arah mutlak {N, E, S, W}. Robot otomatis berbelok seperlunya
           sebelum maju, jadi agen "berpikir" dalam arah mutlak, bukan
           relatif (maju/kiri/kanan).
- Reward : -1 tiap langkah (dorong jalur sependek mungkin), +100 saat
           mencapai sel tujuan (2x2 di tengah maze), plus reward shaping
           opsional (potential-based, lihat bfs_distances_to_goal()) supaya
           belajar lebih cepat pada maze besar (16x16). Potensinya = jarak
           BFS ke goal lewat dinding yang SUDAH DIKETAHUI robot (bukan
           jarak garis lurus/Manhattan -- itu buta dinding dan bisa membuat
           agen terjebak maju-mundur ke jalan buntu yang "kelihatan" dekat
           padahal sebenarnya harus memutar).
- Dinding diketahui lewat sensor (wallFront/Right/Left/Back) dan disimpan
  di peta internal (KnownMap) supaya tidak perlu sensor ulang untuk sel
  yang sudah pernah dikunjungi -- ini juga dipakai untuk mencari jalan
  pulang ke start di antara episode (lewat BFS di peta yang sudah diketahui).
- Q-table dan peta disimpan ke file lokal (qtable_state.pkl) supaya training
  bisa dilanjutkan di sesi berikutnya, tidak mulai dari nol setiap kali
  simulator dijalankan ulang.

Asumsi koordinat (umum dipakai di mms): (0,0) di pojok kiri-bawah, mouse
mulai menghadap Utara (N), sumbu X ke arah Timur (E), sumbu Y ke arah Utara.
Jika ternyata arah gerak robot di simulatormu terbalik, tinggal tukar tanda
pada dict DELTA di bawah.
"""

import csv
import os
import pickle
import random
import sys
import time
from collections import defaultdict, deque

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


def setWall(x, y, direction):
    _command(["setWall", x, y, direction])


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
# 2) Konstanta arah & hyperparameter
# ============================================================

DIRS = ["N", "E", "S", "W"]
DELTA = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}
OPPOSITE = {"N": "S", "S": "N", "E": "W", "W": "E"}

NUM_EPISODES = 300         # jumlah episode eksplorasi/training
ALPHA = 0.3                 # learning rate
GAMMA = 0.95                # discount factor
EPSILON_START = 1.0         # peluang aksi acak di awal (full eksplorasi)
EPSILON_MIN = 0.05
EPSILON_DECAY = 0.97        # dikalikan ke epsilon setelah tiap episode

STEP_REWARD = -1.0
GOAL_REWARD = 100.0

USE_REWARD_SHAPING = True   # bantu belajar lebih cepat di maze besar (16x16)
SHAPING_WEIGHT = 1.0        # set False/0 kalau mau Q-learning "murni"

STATE_FILE = "qtable_state.pkl"
EPISODE_LOG_FILE = "episode_log.csv"  # per-episode stats, overwritten each run -- see plot_training.py


# ============================================================
# 3) Peta internal (dinding yang sudah diketahui)
# ============================================================

class KnownMap:
    def __init__(self, w, h):
        self.w = w
        self.h = h
        self.walls = {}  # (x, y) -> {"N": True/False/None, ...}

    def _cell(self, x, y):
        if (x, y) not in self.walls:
            self.walls[(x, y)] = {d: None for d in DIRS}
        return self.walls[(x, y)]

    def get(self, x, y, d):
        return self._cell(x, y)[d]

    def set(self, x, y, d, is_wall):
        self._cell(x, y)[d] = is_wall
        dx, dy = DELTA[d]
        nx, ny = x + dx, y + dy
        if 0 <= nx < self.w and 0 <= ny < self.h:
            self._cell(nx, ny)[OPPOSITE[d]] = is_wall

    def open_dirs(self, x, y):
        cell = self._cell(x, y)
        return [d for d in DIRS if cell[d] is False]


def sense(x, y, heading, kmap):
    """Baca sensor dinding relatif (depan/kanan/kiri/belakang) lalu simpan
    sebagai dinding arah mutlak. Sel yang arahnya sudah diketahui (dari
    kunjungan sebelumnya / sel tetangga) tidak disensor ulang."""
    ci = DIRS.index(heading)
    checks = [
        (DIRS[ci], wallFront),
        (DIRS[(ci + 1) % 4], wallRight),
        (DIRS[(ci + 3) % 4], wallLeft),
        (DIRS[(ci + 2) % 4], wallBack),
    ]
    for abs_dir, sensor_fn in checks:
        if kmap.get(x, y, abs_dir) is None:
            is_wall = sensor_fn()
            kmap.set(x, y, abs_dir, is_wall)
            if is_wall:
                draw_wall(kmap, x, y, abs_dir)


def draw_wall(kmap, x, y, d):
    """Gambar dinding di simulator di kedua sisinya (sel ini dan tetangganya).
    Hanya dipanggil sekali per dinding karena sense() cuma membacanya saat
    pertama kali diketahui."""
    setWall(x, y, d.lower())
    dx, dy = DELTA[d]
    nx, ny = x + dx, y + dy
    if 0 <= nx < kmap.w and 0 <= ny < kmap.h:
        setWall(nx, ny, OPPOSITE[d].lower())


def turn_to_heading(current_heading, target_heading):
    """Berbelok seminimal mungkin (0, 1, atau 2 kali) dari current_heading
    ke target_heading, lalu mengembalikan heading baru."""
    if current_heading == target_heading:
        return current_heading
    ci = DIRS.index(current_heading)
    ti = DIRS.index(target_heading)
    diff = (ti - ci) % 4
    if diff == 1:
        turnRight()
    elif diff == 3:
        turnLeft()
    elif diff == 2:
        turnRight()
        turnRight()
    return target_heading


def compute_goal_cells(w, h):
    xs = [w // 2 - 1, w // 2] if w % 2 == 0 else [w // 2]
    ys = [h // 2 - 1, h // 2] if h % 2 == 0 else [h // 2]
    return {(gx, gy) for gx in xs for gy in ys}


def bfs_distances_to_goal(kmap, goal_cells):
    """Multi-source BFS from every goal cell, spread through the maze's
    CURRENTLY KNOWN open passages -- same idea as flood-fill-c's own flood
    fill (see its distances[][]), just used here for reward shaping instead
    of navigation. Returns {(x, y): distance}; a cell absent from the dict
    is unreachable from what's known so far (behind an undiscovered wall),
    same convention as flood-fill-c's distances[][] == -1.

    This replaces the old plain Manhattan-distance shaping, which doesn't
    know about walls: a dead-end cell can be Manhattan-closer to the goal
    than the cell next to it, which made the agent think stepping into that
    dead end was a great move, then bounce back and forth between the two
    forever once it hit the wall. BFS over known walls can't make that
    mistake -- a dead end's BFS distance reflects the real detour required.
    """
    dist = {g: 0 for g in goal_cells}
    q = deque(goal_cells)
    while q:
        cx, cy = q.popleft()
        d = dist[(cx, cy)]
        for direction in DIRS:
            if kmap.get(cx, cy, direction) is False:
                nx, ny = cx + DELTA[direction][0], cy + DELTA[direction][1]
                if (nx, ny) not in dist:
                    dist[(nx, ny)] = d + 1
                    q.append((nx, ny))
    return dist


# ============================================================
# 4) Agen Q-learning
# ============================================================

class QLearningAgent:
    def __init__(self):
        self.q = defaultdict(float)  # key: (x, y, arah) -> nilai Q

    def best_action(self, x, y, open_dirs):
        candidates = list(open_dirs)
        random.shuffle(candidates)  # supaya seri (tie) dipilih acak, bukan selalu arah pertama
        best_d, best_v = None, float("-inf")
        for d in candidates:
            v = self.q[(x, y, d)]
            if v > best_v:
                best_v, best_d = v, d
        return best_d

    def choose_action(self, x, y, open_dirs, epsilon):
        if random.random() < epsilon:
            return random.choice(open_dirs)
        return self.best_action(x, y, open_dirs)

    def update(self, x, y, d, reward, nx, ny, next_open_dirs, alpha, gamma, terminal):
        old = self.q[(x, y, d)]
        if terminal or not next_open_dirs:
            target = reward
        else:
            target = reward + gamma * max(self.q[(nx, ny, d2)] for d2 in next_open_dirs)
        self.q[(x, y, d)] = old + alpha * (target - old)


# ============================================================
# 5) Satu episode: dari start sampai goal (atau kehabisan langkah)
# ============================================================

def run_episode(agent, kmap, goal_cells, max_steps, alpha, gamma, epsilon, start_heading):
    # start_heading MUST reflect the mouse's real current heading in mms, not
    # just be hardcoded to "N" -- there is no actual reset between episodes
    # (see main()'s loop), so if navigate_home() left the mouse facing some
    # other direction, hardcoding "N" here would desync this file's heading
    # tracking from the real one, corrupting every sense() call's front/
    # right/left/back -> absolute-direction mapping for the rest of the
    # episode.
    x, y, heading = 0, 0, start_heading
    path = []
    total_reward = 0.0

    for _ in range(max_steps):
        sense(x, y, heading, kmap)
        open_dirs = kmap.open_dirs(x, y)
        if not open_dirs:
            # Seharusnya tidak pernah terjadi di maze yang valid (selalu ada
            # jalan masuk yang baru saja dilewati), tapi jaga-jaga saja.
            open_dirs = DIRS[:]

        action = agent.choose_action(x, y, open_dirs, epsilon)
        heading = turn_to_heading(heading, action)
        moveForward(1)  # aman: 'action' sudah dipastikan arah terbuka

        nx, ny = x + DELTA[action][0], y + DELTA[action][1]
        path.append(action)
        sense(nx, ny, heading, kmap)

        terminal = (nx, ny) in goal_cells
        reward = GOAL_REWARD if terminal else STEP_REWARD
        if USE_REWARD_SHAPING and not terminal:
            # Potential-based shaping F(s->s') = gamma*Phi(s') - Phi(s), with
            # Phi(s) = -distance_to_goal(s) (closer to goal = higher
            # potential). This exact form is what Ng et al. (1999) prove
            # leaves the optimal policy unchanged for ANY choice of
            # potential function -- so this is just as legitimate a part of
            # Q-learning as the base -1/+100 reward, not a shortcut around
            # it. distance_to_goal comes from bfs_distances_to_goal(), using
            # only walls the mouse has actually sensed so far.
            dist_map = bfs_distances_to_goal(kmap, goal_cells)
            d_here = dist_map.get((x, y))
            d_next = dist_map.get((nx, ny))
            if d_here is not None and d_next is not None:
                reward += SHAPING_WEIGHT * (d_here - gamma * d_next)

        next_open = kmap.open_dirs(nx, ny)
        agent.update(x, y, action, reward, nx, ny, next_open, alpha, gamma, terminal)
        total_reward += reward
        x, y = nx, ny

        if terminal:
            return path, total_reward, True, x, y, heading

    return path, total_reward, False, x, y, heading


# ============================================================
# 6) Kembali ke start di antara episode (BFS di peta yang diketahui)
# ============================================================

def bfs_path(kmap, start, goal):
    q = deque([start])
    prev = {start: None}
    while q:
        cur = q.popleft()
        if cur == goal:
            break
        cx, cy = cur
        for d in DIRS:
            if kmap.get(cx, cy, d) is False:
                nxt = (cx + DELTA[d][0], cy + DELTA[d][1])
                if nxt not in prev:
                    prev[nxt] = (cur, d)
                    q.append(nxt)
    if goal not in prev:
        return None
    moves, cur = [], goal
    while prev[cur] is not None:
        pcur, d = prev[cur]
        moves.append(d)
        cur = pcur
    moves.reverse()
    return moves


def navigate_home(kmap, x, y, heading):
    """Walk back to (0,0), re-sensing and re-running BFS before EVERY single
    step -- never trust a multi-step plan computed once and executed blind.

    This used to compute one bfs_path() up front and blindly walk it (or, if
    no path was known, replay the taken_path in reverse). That was the one
    place in this file that broke the same invariant run_episode() relies on
    for every one of its moveForward() calls: only ever move in a direction
    just confirmed open by a live sensor read for the CURRENT cell. A stale
    or partially-mirrored KnownMap entry could point this into a real wall,
    which raises MouseCrashedError -- and that's exactly what happened on a
    fresh 16x16 run with no prior qtable_state.pkl involved, so it wasn't a
    stale-state issue at all. Recomputing every step is cheap (a plain BFS
    over a <=32x32 grid) and matches flood-fill-c's own "replan every step"
    design, which never has this problem.
    """
    max_return_steps = 4 * (kmap.w + kmap.h)
    for _ in range(max_return_steps):
        if (x, y) == (0, 0):
            return heading

        sense(x, y, heading, kmap)
        moves = bfs_path(kmap, (x, y), (0, 0))
        if moves:
            d = moves[0]
        else:
            # Shouldn't happen once (x,y) has just been sensed in a valid,
            # fully-enclosed maze -- but don't get stuck if it somehow does.
            open_dirs = kmap.open_dirs(x, y)
            d = open_dirs[0] if open_dirs else DIRS[0]

        heading = turn_to_heading(heading, d)
        moveForward(1)
        x, y = x + DELTA[d][0], y + DELTA[d][1]

    return heading


def color_path(path):
    """Warnai jalur (dari start) di simulator: start biru, jalur hijau.
    Dipanggil hanya saat jalur terbaik baru ditemukan, dan untuk run akhir."""
    clearAllColor()
    x, y = 0, 0
    setColor(x, y, "b")
    for d in path:
        x, y = x + DELTA[d][0], y + DELTA[d][1]
        setColor(x, y, "g")


# ============================================================
# 7) Simpan / muat Q-table supaya training bisa dilanjutkan
# ============================================================

def load_state(agent, kmap, w, h):
    if os.path.exists(STATE_FILE):
        try:
            with open(STATE_FILE, "rb") as f:
                data = pickle.load(f)
            stored_w, stored_h = data.get("w"), data.get("h")
            if (stored_w, stored_h) != (w, h):
                # State from a different-size maze (e.g. switched from classic
                # 16x16 to a halfsize/training maze) is not just unhelpful --
                # navigate_home()'s BFS would trust "open" walls that may not
                # exist in the new maze and drive straight into a real one,
                # which raises MouseCrashedError nothing ever catches. Safer
                # to discard it and start fresh than risk that.
                eprint(
                    f"State lama untuk maze {stored_w}x{stored_h}, maze saat ini "
                    f"{w}x{h} -- berbeda, mulai dari nol (tidak dipakai)."
                )
                return
            agent.q = defaultdict(float, data.get("q", {}))
            kmap.walls = data.get("walls", {})
            eprint(f"Memuat state lama: {len(agent.q)} nilai-Q, {len(kmap.walls)} sel diketahui.")
        except Exception as e:
            eprint(f"Gagal memuat state lama ({e}), mulai dari nol.")


def save_state(agent, kmap, w, h):
    try:
        with open(STATE_FILE, "wb") as f:
            pickle.dump({"q": dict(agent.q), "walls": kmap.walls, "w": w, "h": h}, f)
        eprint(f"State disimpan ke {STATE_FILE}.")
    except Exception as e:
        eprint(f"Gagal menyimpan state: {e}")


# ============================================================
# 8) Main
# ============================================================

def main():
    w = mazeWidth()
    h = mazeHeight()
    eprint(f"Ukuran maze: {w}x{h}")

    goal_cells = compute_goal_cells(w, h)
    kmap = KnownMap(w, h)
    agent = QLearningAgent()
    load_state(agent, kmap, w, h)

    epsilon = EPSILON_START
    max_steps = max(200, 3 * w * h)
    best_len = None
    # Real starting heading, per mms's own INITIAL_STARTING_POSITION/rotation.
    # Threaded through the whole loop below (never hardcoded again) since
    # there's no actual reset between episodes -- whatever heading
    # navigate_home() ends an episode facing is what the NEXT run_episode()
    # must start from, or its sense() calls desync from the real mouse.
    heading = "N"

    # Per-episode stats (episode, steps, success, reward, epsilon, best_len)
    # for plot_training.py -- overwritten each run, so it always reflects the
    # most recent training session. Flushed after every row so a partial log
    # is still usable for plotting even if training is interrupted early.
    log_f = open(EPISODE_LOG_FILE, "w", newline="", encoding="utf-8")
    log_writer = csv.writer(log_f)
    log_writer.writerow(["episode", "steps", "success", "reward", "epsilon", "best_len"])

    try:
        for ep in range(1, NUM_EPISODES + 1):
            path, reward, success, x, y, heading = run_episode(
                agent, kmap, goal_cells, max_steps, ALPHA, GAMMA, epsilon, heading
            )
            if success and (best_len is None or len(path) < best_len):
                best_len = len(path)
                color_path(path)

            log_writer.writerow([ep, len(path), int(success), reward, epsilon, best_len or ""])
            log_f.flush()

            eprint(
                f"[Episode {ep}/{NUM_EPISODES}] "
                f"{'sampai tujuan' if success else 'kehabisan langkah'} "
                f"dalam {len(path)} langkah, reward={reward:.1f}, "
                f"epsilon={epsilon:.3f}, jalur terbaik={best_len}"
            )

            heading = navigate_home(kmap, x, y, heading)
            epsilon = max(EPSILON_MIN, epsilon * EPSILON_DECAY)

        save_state(agent, kmap, w, h)

        eprint("Training selesai. Menjalankan run akhir (greedy, tanpa eksplorasi)...")
        path, reward, success, x, y, heading = run_episode(
            agent, kmap, goal_cells, max_steps, alpha=0.0, gamma=GAMMA, epsilon=0.0,
            start_heading=heading,
        )
        if success:
            eprint(f"Run akhir sampai tujuan dalam {len(path)} langkah.")
            color_path(path)
        else:
            eprint("Run akhir belum sampai tujuan — coba naikkan NUM_EPISODES.")
    except MouseCrashedError:
        # navigate_home() now re-senses and re-plans every single step (same
        # invariant run_episode() already relies on for its own moves), so
        # this should be very rare. If it still happens, the most plausible
        # remaining cause is qtable_state.pkl carried over from a DIFFERENT
        # maze that happens to share this one's width/height (load_state()'s
        # check only catches a dimension mismatch, not a same-size layout
        # mismatch). Don't overwrite the saved state with data gathered
        # against a possibly-wrong maze -- just stop cleanly instead of an
        # unhandled traceback, and tell the user how to recover.
        eprint(
            "Mouse crashed -- coba hapus qtable_state.pkl di folder ini lalu "
            "jalankan ulang. Kalau masih terjadi dari state kosong (fresh run, "
            "tanpa qtable_state.pkl lama), ini kemungkinan bug baru -- laporkan "
            "beserta baris log [Episode N/...] terakhir sebelum crash."
        )
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