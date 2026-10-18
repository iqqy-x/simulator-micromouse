#!/usr/bin/env python3
"""
Flood fill di Webots: jelajah labirin dengan sensor robot, lalu lari cepat.

Alur:
  1. Luruskan di sel start: mundur sampai PCB belakang menempel dinding.
  2. Jelajah ke tengah, kembali ke start, ke tengah lagi, ... sampai jalur
     terpendek yang SUDAH DIKETAHUI (sel tak dikenal dianggap dinding) sama
     panjang dengan perkiraan optimis (sel tak dikenal dianggap terbuka) --
     artinya jalur itu terbukti optimal -- atau mencapai MAX_TRIPS.
  3. Pulang ke start, lalu lari cepat memakai run_path (belok halus).

Tiap langkah jelajah memakai gerak per sel (drive_forward_cells + turn)
karena sel tempat belok halus tidak teramati sensornya. Dinding dibaca HANYA
dari sensor (sense_walls): depan dari FL/FR di tengah sel, kiri/kanan dari
DL/DR saat masuk sel, belakang terbuka (baru dilewati).

File maze (MM_MAZE, bawaan practice-8x8-a.txt) dipakai untuk dua hal saja:
ukuran labirin (seperti mazeWidth() di mms) dan menilai hasil (peta salah,
jalur optimal). Dinding tidak pernah dipakai untuk menentukan gerakan.

Algoritmanya mengikuti sim/algorithms/flood-fill-c (BFS dari sel tujuan,
sel tak dikenal dianggap terbuka, melangkah ke tetangga berjarak lebih kecil,
lebih suka lurus). Aturan tujuan sama dengan mms: 1/2/4 sel di tengah.

Set MICROMOUSE_QUIT=1 agar Webots keluar sendiri setelah selesai.
"""

import math
import os
import sys
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "micromouse_controller"))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..", "..", "tools")))

import micromouse_controller as c  # noqa: E402
from maze_to_webots import parse_maze  # noqa: E402

DEFAULT_MAZE = os.path.abspath(os.path.join(
    HERE, "..", "..", "..", "..", "mazes", "mazefiles", "training", "practice-8x8-a.txt"))
MAZE_FILE = os.environ.get("MM_MAZE") or DEFAULT_MAZE
MAX_TRIPS = int(os.environ.get("MM_MAX_TRIPS", "4"))
MAX_STEPS = 600   # pengaman: batas langkah jelajah
# "continuous": jelajah tanpa berhenti (search_run); "cell": berhenti di tiap sel
SEARCH_MODE = os.environ.get("MM_SEARCH", "continuous")

N, E, S, W_ = 0, 1, 2, 3
DX = (0, 1, 0, -1)
DY = (1, 0, -1, 0)
NAME = "NESW"
INF = 10 ** 6

m_width, m_rows, m_horiz, m_vert = parse_maze(MAZE_FILE)
WIDTH, HEIGHT = m_width, m_rows


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


class MazeMap:
    """Peta dinding yang dipelajari: wall[x][y][d] = True / False / None (belum tahu)."""

    def __init__(self, w, h):
        self.w, self.h = w, h
        self.wall = [[[None] * 4 for _ in range(h)] for _ in range(w)]
        for x in range(w):
            self.set_wall(x, 0, S, True)
            self.set_wall(x, h - 1, N, True)
        for y in range(h):
            self.set_wall(0, y, W_, True)
            self.set_wall(w - 1, y, E, True)

    def set_wall(self, x, y, d, value):
        self.wall[x][y][d] = value
        nx, ny = x + DX[d], y + DY[d]
        if 0 <= nx < self.w and 0 <= ny < self.h:
            self.wall[nx][ny][(d + 2) % 4] = value

    def passable(self, x, y, d, unknown_open):
        v = self.wall[x][y][d]
        nx, ny = x + DX[d], y + DY[d]
        if not (0 <= nx < self.w and 0 <= ny < self.h):
            return False
        return v is False or (v is None and unknown_open)

    def distances(self, goals, unknown_open):
        """BFS dari sel tujuan: jumlah langkah ke tujuan tiap sel."""
        dist = [[INF] * self.h for _ in range(self.w)]
        queue = deque()
        for gx, gy in goals:
            dist[gx][gy] = 0
            queue.append((gx, gy))
        while queue:
            x, y = queue.popleft()
            for d in range(4):
                if self.passable(x, y, d, unknown_open):
                    nx, ny = x + DX[d], y + DY[d]
                    if dist[nx][ny] > dist[x][y] + 1:
                        dist[nx][ny] = dist[x][y] + 1
                        queue.append((nx, ny))
        return dist

    def path(self, start, goals, unknown_open, heading=N):
        """Jalur terpendek (daftar arah) dari start ke tujuan; lebih suka lurus."""
        dist = self.distances(goals, unknown_open)
        x, y = start
        if dist[x][y] >= INF:
            return None
        out = []
        while (x, y) not in goals:
            best = None
            for d in sorted(range(4), key=lambda d: (d != heading, d)):
                if self.passable(x, y, d, unknown_open) and dist[x + DX[d]][y + DY[d]] == dist[x][y] - 1:
                    best = d
                    break
            out.append(best)
            heading = best
            x, y = x + DX[best], y + DY[best]
        return out

    def known_edges(self):
        edges = {}
        for x in range(self.w):
            for y in range(self.h):
                for d in range(4):
                    v = self.wall[x][y][d]
                    if v is None:
                        continue
                    nx, ny = x + DX[d], y + DY[d]
                    key = frozenset({(x, y), (nx, ny)}) if 0 <= nx < self.w and 0 <= ny < self.h \
                        else ((x, y), d)
                    edges[key] = (x, y, d, v)
        return edges


class Explorer:
    def __init__(self):
        self.map = MazeMap(WIDTH, HEIGHT)
        self.x, self.y, self.h = 0, 0, N
        self.steps = 0
        self.blocked_events = 0
        self.goals_now = None
        self.start_cell = (0, 0)
        self.goals = goal_cells(WIDTH, HEIGHT)

    # --- gerak ---------------------------------------------------------------
    def turn_to(self, d):
        diff = (d - self.h) % 4
        if diff == 1:
            c.turn("right")
        elif diff == 3:
            c.turn("left")
        elif diff == 2:
            c.turn("right")
            c.turn("right")
        self.h = d

    def step_forward(self):
        c.drive_forward_cells(1)
        self.x += DX[self.h]
        self.y += DY[self.h]
        self.steps += 1

    def rotate_to(self, d):
        """Putar di tempat (kedua roda berlawanan arah) ke arah d, dari tengah sel."""
        c.center()
        diff = (d - self.h) % 4
        if diff == 1:
            c._point_turn("right")
        elif diff == 3:
            c._point_turn("left")
        elif diff == 2:
            c._point_turn("left")
            c._point_turn("left")
        self.h = d

    # --- sensor --------------------------------------------------------------
    def prepare_start(self):
        """Sel start: mundur sampai PCB belakang (datar) menempel dinding belakang
        untuk meluruskan robot, lalu maju ke tengah sel. Dinding kiri, kanan, dan
        belakang sel start diasumsikan ada (aturan kompetisi); dinding depan dibaca
        FL/FR dari tengah sel, tanpa berputar."""
        tilt = float(os.environ.get("MM_START_TILT_DEG", "0"))
        if tilt:   # uji: miringkan robot dulu untuk melihat apakah dinding meluruskannya
            c.self_node.getField("rotation").setSFRotation(
                [0, 0, 1, math.pi / 2 + math.radians(tilt)])
            c.self_node.resetPhysics()
            c.log(f"FLOOD uji: robot dimiringkan {tilt:.1f} derajat di sel start")
        c.square_to_back_wall()
        r = c.self_node.getOrientation()
        c.log(f"FLOOD setelah menempel dinding belakang: arah sebenarnya "
              f"{math.degrees(math.atan2(r[3], r[0])):.2f} derajat (seharusnya 90)")
        c.center()
        for d in (S, W_, E):
            self.map.set_wall(0, 0, d, True)
            if not truth_wall(0, 0, d):
                c.log(f"FLOOD PERINGATAN: sisi {NAME[d]} sel start ternyata tidak berdinding")
        seen, value = c.read_front_wall()
        self.map.set_wall(0, 0, N, seen)
        c.log(f"FLOOD sel start: kiri/kanan/belakang diasumsikan dinding; depan "
              f"{'dinding' if seen else 'kosong'} (nilai {value:.0f})")

    def log_stop_distance(self):
        """Jarak sebenarnya ke dinding depan saat rem darurat (hanya validasi)."""
        info = c.last_blocked
        if not info:
            return
        # bidang dinding depan sel (x, y) searah h, di sumbu gerak
        plane = {N: (self.y + 1) * c.CELL - 0.006, E: (self.x + 1) * c.CELL - 0.006,
                 S: self.y * c.CELL + 0.006, W_: self.x * c.CELL + 0.006}[self.h]
        sign = 1.0 if self.h in (N, E) else -1.0

        def gap(pose, ahead):
            axle = pose[1] if self.h in (N, S) else pose[0]
            return sign * (plane - axle) - ahead   # jarak titik ke dinding

        x, y, deg = info["pose_rem"]
        pose = c.self_node.getPosition()
        c.log("FLOOD rem darurat: nilai FL/FR rata-rata %.0f; sensor ke dinding %.1f mm, "
              "tepi depan PCB ke dinding %.1f mm saat berhenti; sudah menempuh %.0f mm sejak "
              "titik keputusan" % (info["nilai_fl_fr"], 1000 * gap(info["pose_rem"], 0.0475),
                                   1000 * gap(info["pose_rem"], 0.066), 1000 * info["jarak_tempuh"]))
        c.log("FLOOD   sebelum diputar: sensor ke dinding %.1f mm, tepi depan PCB ke dinding %.1f mm "
              "(batas aman %.0f mm dari tepi PCB)"
              % (1000 * gap((pose[0], pose[1], 0.0), 0.0475),
                 1000 * gap((pose[0], pose[1], 0.0), 0.066), 1000 * c.SAFE_STOP_NOSE))

    def on_blocked(self):
        """Dinding depan ternyata ada padahal diperkirakan terbuka (berhenti
        darurat): perbaiki peta, hitung ulang, kembalikan aksi relatif."""
        self.blocked_events += 1
        self.log_stop_distance()
        self.map.set_wall(self.x, self.y, self.h, True)
        c.log(f"FLOOD dinding depan sel ({self.x},{self.y}) arah {NAME[self.h]} "
              f"ditemukan mendadak dekat -- peta diperbaiki, hitung ulang")
        dist = self.map.distances(self.goals_now, True)
        d = self.next_dir(dist)
        if d is None:
            raise RuntimeError(f"buntu di ({self.x},{self.y}) setelah henti darurat")
        action = {0: "F", 1: "R", 3: "L", 2: "U"}[(d - self.h) % 4]
        if action == "F":
            raise RuntimeError("henti darurat tapi rute baru lurus -- peta tidak konsisten")
        self.h = d
        return action

    def on_cell(self, obs, goals):
        """Dipanggil search_run di titik keputusan tiap sel C yang dimasuki:
        perbarui peta dengan dinding C yang teramati, jalankan flood fill,
        kembalikan aksi relatif (F/L/R/U/STOP)."""
        self.x += DX[self.h]
        self.y += DY[self.h]
        self.steps += 1
        if self.steps > MAX_STEPS:
            raise RuntimeError("terlalu banyak langkah jelajah")
        blind = os.environ.get("MM_BLIND_FRONT")   # uji: anggap dinding depan sel ini TERBUKA
        if blind and obs["front"] and (self.x, self.y) == tuple(int(v) for v in blind.split(",")):
            c.log(f"FLOOD uji: dinding depan sel ({self.x},{self.y}) sengaja dianggap terbuka")
            obs = dict(obs, front=False)
        for d_abs, val, is_side in ((self.h, obs["front"], False),
                                    ((self.h + 3) % 4, obs["left"], True),
                                    ((self.h + 1) % 4, obs["right"], True)):
            if val is None:
                continue
            # Dinding samping di sel "terlambat" (tepat sesudah busur belok) kurang
            # andal: hanya dipakai kalau sisi itu belum diketahui; pembacaan
            # normal (di sel lain atau kunjungan lain) boleh menimpa.
            if is_side and obs.get("late") and self.map.wall[self.x][self.y][d_abs] is not None:
                continue
            self.map.set_wall(self.x, self.y, d_abs, val)
        self.map.set_wall(self.x, self.y, (self.h + 2) % 4, False)  # baru dilewati
        self.log_cell(obs)
        if (self.x, self.y) in goals:
            return "STOP"
        dist = self.map.distances(goals, True)
        d = self.next_dir(dist)
        if d is None:
            raise RuntimeError(f"buntu di ({self.x},{self.y}) -- peta salah?")
        action = {0: "F", 1: "R", 3: "L", 2: "U"}[(d - self.h) % 4]
        self.h = d
        return action

    def log_cell(self, obs):
        """Catat pengamatan sel dan tandai yang salah (hanya untuk validasi)."""
        parts = []
        for label, d_abs, key in (("depan", self.h, "front"), ("kiri", (self.h + 3) % 4, "left"),
                                  ("kanan", (self.h + 1) % 4, "right")):
            seen = obs[key]
            truth = truth_wall(self.x, self.y, d_abs)
            val = obs[key + "_value"]
            flag = "" if seen is None or seen == truth else " <-- SALAH"
            parts.append(f"{label}={'?' if seen is None else int(seen)}(nilai {val if val is None else round(val)},"
                         f" benar {int(truth)}){flag}")
        c.log(f"FLOOD sel ({self.x},{self.y}) arah {NAME[self.h]}"
              f"{' (terlambat)' if obs.get('late') else ''}: " + "  ".join(parts))
        if any("SALAH" in q for q in parts):
            c.log(f"FLOOD   urutan nilai kiri={obs.get('left_seq')} kanan={obs.get('right_seq')}")

    def search_to(self, goals):
        """Jelajah berkelanjutan dari tengah sel sekarang sampai sel tujuan."""
        if (self.x, self.y) in goals:
            return
        dist = self.map.distances(goals, True)
        d = self.next_dir(dist)
        if d is None:
            raise RuntimeError(f"buntu di ({self.x},{self.y}) -- peta salah?")
        self.rotate_to(d)
        self.goals_now = goals
        c.search_run(lambda obs: self.on_cell(obs, goals), self.on_blocked)

    def sense_cell(self):
        w = c.sense_walls()
        self.map.set_wall(self.x, self.y, self.h, w["front"])
        self.map.set_wall(self.x, self.y, (self.h + 3) % 4, w["left"])
        self.map.set_wall(self.x, self.y, (self.h + 1) % 4, w["right"])
        self.map.set_wall(self.x, self.y, (self.h + 2) % 4, False)  # baru dilewati

    # --- jelajah ---------------------------------------------------------------
    def next_dir(self, dist):
        cx, cy = self.x, self.y
        best = None
        for d in range(4):
            if not self.map.passable(cx, cy, d, True):
                continue
            nd = dist[cx + DX[d]][cy + DY[d]]
            if nd < dist[cx][cy]:
                turn_cost = 0 if d == self.h else (2 if (d - self.h) % 4 == 2 else 1)
                key = (nd, turn_cost)
                if best is None or key < best[0]:
                    best = (key, d)
        return None if best is None else best[1]

    def explore_to(self, goals):
        while (self.x, self.y) not in goals:
            if self.steps > MAX_STEPS:
                raise RuntimeError("terlalu banyak langkah jelajah")
            dist = self.map.distances(goals, True)
            d = self.next_dir(dist)
            if d is None:
                raise RuntimeError(f"buntu di ({self.x},{self.y}) -- peta salah?")
            self.turn_to(d)
            self.step_forward()
            self.sense_cell()


def relative_plan(path, heading=N):
    out = []
    for d in path:
        out.append({0: "F", 1: "R", 3: "L"}[(d - heading) % 4])
        heading = d
    return "".join(out)


def truth_shortest(start, goals):
    """Panjang jalur terpendek menurut file maze (hanya untuk menilai)."""
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


class Clearance:
    """Jarak bebas terkecil antara OUTLINE PCB dan dinding, dihitung tiap langkah
    simulasi dari pose sebenarnya (hanya untuk validasi; MM_CLEARANCE=1)."""

    def __init__(self):
        import maze_to_webots as m
        self.m = m
        cell, t = c.CELL, 0.012
        self.boxes = []
        for r in range(m_rows + 1):
            for x in range(m_width):
                if m_horiz[r][x]:
                    cy = (m_rows - r) * cell
                    self.boxes.append(((x + 0.5) * cell - cell / 2, cy - t / 2, (x + 0.5) * cell + cell / 2, cy + t / 2))
        for r in range(m_rows):
            yc = m_rows - 1 - r
            for x in range(m_width + 1):
                if m_vert[r][x]:
                    cx = x * cell
                    self.boxes.append((cx - t / 2, (yc + 0.5) * cell - cell / 2, cx + t / 2, (yc + 0.5) * cell + cell / 2))
        # titik outline: sudut tiap strip body, frame robot (x depan, y kiri), meter
        self.pts = []
        for lo, hi, hw in m.body_blocks():
            for py in (lo, hi):
                for sgn in (1, -1):
                    self.pts.append(((py - m.AXLE_PCB_MM[1]) / 1000.0, sgn * hw / 1000.0))
        self.min_all = (9.0, None)
        self.min_moving = (9.0, None)
        self.min_still = (9.0, None)
        self.last = None
        self.ev = None
        self.ev_count = 0

    def sample(self, label):
        x, y, _ = c.self_node.getPosition()
        r = c.self_node.getOrientation()
        v = c.self_node.getVelocity()
        speed = math.hypot(v[0], v[1])
        best = 9.0
        for px, py in self.pts:
            wx = x + r[0] * px + r[1] * py
            wy = y + r[3] * px + r[4] * py
            for (x0, y0, x1, y1) in self.boxes:
                if x0 - 0.1 < wx < x1 + 0.1 and y0 - 0.1 < wy < y1 + 0.1:
                    dx = max(x0 - wx, 0.0, wx - x1)
                    dy = max(y0 - wy, 0.0, wy - y1)
                    best = min(best, math.hypot(dx, dy))
        info = (round(best * 1000, 1), (round(x, 3), round(y, 3), round(math.degrees(math.atan2(r[3], r[0])), 0),
                                       round(speed, 2), round(c.robot.getTime(), 1)))
        if best < self.min_all[0] / 1000.0 * 1000 and best * 1000 < self.min_all[0]:
            self.min_all = (best * 1000, info[1])
        if speed < 0.03 and best * 1000 < self.min_still[0]:
            self.min_still = (best * 1000, info[1])
        if speed >= 0.03 and best * 1000 < self.min_moving[0]:
            self.min_moving = (best * 1000, info[1])
        # kejadian "dekat dinding" (< 30 mm): catat awal/akhir dan jarak terkecil
        t = c.robot.getTime()
        if best < 0.030:
            cell = (int(x // c.CELL), int(y // c.CELL))
            if self.ev is None:
                self.ev = {"t0": t, "min": best, "cell": cell, "speed": speed,
                           "head": math.degrees(math.atan2(r[3], r[0]))}
            elif best < self.ev["min"]:
                self.ev.update(min=best, cell=cell, speed=speed,
                               head=math.degrees(math.atan2(r[3], r[0])))
            self.ev["t1"] = t
        elif self.ev is not None:
            e = self.ev
            if self.ev_count < 60:
                c.log("FLOOD dekat dinding: t=%.1f-%.1f s  jarak min %.1f mm  sel %s  arah %.0f  kecepatan %.2f m/s"
                      % (e["t0"], e["t1"], e["min"] * 1000, e["cell"], e["head"], e["speed"]))
            self.ev_count += 1
            self.ev = None
        return best

    def report(self):
        for name, (d, where) in (("keseluruhan", self.min_all), ("saat bergerak", self.min_moving),
                                 ("saat diam/berputar di tempat", self.min_still)):
            c.log(f"FLOOD jarak bebas PCB-dinding terkecil {name}: {d:.1f} mm di (x,y,arah,kecepatan,waktu)={where}")


def main():
    clearance = None
    if os.environ.get("MM_CLEARANCE"):
        clearance = Clearance()
        orig_step = c.step

        def traced():
            orig_step()
            clearance.sample("")
        c.step = traced
    ex = Explorer()
    t_start = c.robot.getTime()
    c.log(f"FLOOD labirin {WIDTH}x{HEIGHT} dari {os.path.basename(MAZE_FILE)}, "
          f"tujuan {sorted(ex.goals)}, optimal menurut file maze = "
          f"{truth_shortest((0, 0), ex.goals)} langkah")

    explore = ex.search_to if SEARCH_MODE == "continuous" else ex.explore_to
    c.log(f"FLOOD mode jelajah: {SEARCH_MODE}")
    ex.prepare_start()
    if clearance:   # menempel dinding belakang di sel start disengaja: jangan dihitung
        clearance.min_all = clearance.min_moving = clearance.min_still = (9000.0, None)
    t_scan = c.robot.getTime()
    c.log(f"FLOOD persiapan sel start selesai ({t_scan - t_start:.1f} s)")

    certified = False
    for trip in range(1, MAX_TRIPS + 1):
        target = ex.goals if trip % 2 == 1 else {ex.start_cell}
        t0 = c.robot.getTime()
        s0 = ex.steps
        explore(target)
        c.log(f"FLOOD perjalanan {trip} ke {'tengah' if trip % 2 == 1 else 'start'}: "
              f"{ex.steps - s0} langkah, {c.robot.getTime() - t0:.1f} s")
        known = ex.map.path(ex.start_cell, ex.goals, unknown_open=False)
        optimistic = ex.map.path(ex.start_cell, ex.goals, unknown_open=True)
        c.log(f"FLOOD jalur diketahui = {len(known) if known else None} langkah, "
              f"perkiraan optimis = {len(optimistic) if optimistic else None} langkah")
        if known and optimistic and len(known) == len(optimistic):
            certified = True
            break

    if (ex.x, ex.y) != ex.start_cell:
        explore({ex.start_cell})
    t_explored = c.robot.getTime()
    if clearance:
        clearance.report()
    c.log(f"FLOOD henti darurat: {ex.blocked_events} kali")
    c.log(f"FLOOD jelajah selesai: {ex.steps} langkah, {t_explored - t_start:.1f} s, "
          f"jalur {'TERBUKTI optimal' if certified else 'belum terbukti optimal (batas perjalanan)'}")

    # --- nilai peta terhadap file maze -----------------------------------------
    edges = ex.map.known_edges()
    wrong = [(k, v) for k, v in edges.items() if truth_wall(v[0], v[1], v[2]) != v[3]]
    c.log(f"FLOOD peta: {len(edges)} sisi diketahui, {len(wrong)} salah")
    for _, (x, y, d, v) in wrong[:10]:
        c.log(f"FLOOD   salah: sel ({x},{y}) sisi {NAME[d]} terbaca "
              f"{'dinding' if v else 'kosong'}")

    # --- lari cepat ---------------------------------------------------------------
    path = ex.map.path(ex.start_cell, ex.goals, unknown_open=False)
    if not path:
        c.log("FLOOD tidak ada jalur diketahui ke tujuan -- lari cepat dibatalkan")
    else:
        ex.rotate_to(N)
        plan = relative_plan(path)
        c.log(f"FLOOD lari cepat: {len(path)} langkah, rencana {plan}")
        t0 = c.robot.getTime()
        c.run_path(plan)
        run_time = c.robot.getTime() - t0
        x, y, _ = c.self_node.getPosition()
        cell = (int(x // c.CELL), int(y // c.CELL))
        c.log(f"FLOOD lari cepat selesai: {run_time:.1f} s, rata-rata "
              f"{len(path) * c.CELL / run_time:.2f} m/s, berhenti di sel {cell} "
              f"({'di tujuan' if cell in ex.goals else 'BUKAN di tujuan'})")

    if os.environ.get("MICROMOUSE_QUIT"):
        c.robot.simulationQuit(0)
    while c.robot.step(c.TIME_STEP) != -1:
        pass


try:
    main()
except BaseException:
    import traceback
    c.log("FLOOD GALAT: " + " | ".join(traceback.format_exc().splitlines()))
    if os.environ.get("MICROMOUSE_QUIT"):
        c.robot.simulationQuit(1)
