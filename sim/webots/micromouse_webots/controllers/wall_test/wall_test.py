#!/usr/bin/env python3
"""
Uji pembacaan dinding dengan sensor PCB (FL/FR/DL/DR) di maze practice-8x8-a.

Robot menyusuri jalur terpendek dari start ke tengah sel demi sel. Di tiap
sel ia membaca dinding depan/kiri/kanan HANYA dari sensor (sense_walls()),
lalu hasilnya dibandingkan dengan file maze. File maze dipakai untuk dua
hal saja: memilih jalur bebas tabrakan (BFS) dan menilai hasil -- tidak
pernah untuk menentukan pembacaan.

Sel start dipindai dengan read_front_wall() sambil berputar di tempat
(DL/DR tidak bisa membaca dinding samping sel yang sedang ditempati).

Hasil lengkap ke run_log.txt (baris WALLTEST). Set variabel lingkungan
MICROMOUSE_QUIT=1 agar Webots keluar sendiri setelah selesai (untuk uji
headless); tanpa itu robot idle seperti biasa.
"""

import os
import sys
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "micromouse_controller"))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..", "..", "tools")))

import micromouse_controller as c  # noqa: E402
from maze_to_webots import parse_maze  # noqa: E402

MAZE = os.path.abspath(os.path.join(
    HERE, "..", "..", "..", "..", "mazes", "mazefiles", "training", "practice-8x8-a.txt"))

DIRS = ["N", "E", "S", "W"]
DELTA = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}

width, rows, horiz, vert = parse_maze(MAZE)


def truth_wall(cx, cy, d):
    """Dinding di sisi mutlak d dari sel (cx, cy) menurut file maze."""
    if d == "N":
        return horiz[rows - cy - 1][cx]
    if d == "S":
        return horiz[rows - cy][cx]
    if d == "E":
        return vert[rows - 1 - cy][cx + 1]
    return vert[rows - 1 - cy][cx]


def shortest_path(start, goals):
    """BFS pada peta kebenaran -- hanya untuk memilih jalur uji yang aman."""
    prev = {start: None}
    queue = deque([start])
    while queue:
        cur = queue.popleft()
        if cur in goals:
            path = []
            while prev[cur]:
                cur, d = prev[cur]
                path.append(d)
            return path[::-1]
        for d in DIRS:
            if truth_wall(cur[0], cur[1], d):
                continue
            nxt = (cur[0] + DELTA[d][0], cur[1] + DELTA[d][1])
            if nxt not in prev and 0 <= nxt[0] < width and 0 <= nxt[1] < rows:
                prev[nxt] = (cur, d)
                queue.append(nxt)
    raise RuntimeError("tidak ada jalur ke goal")


def turn_to(heading, target):
    diff = (DIRS.index(target) - DIRS.index(heading)) % 4
    if diff == 1:
        c.turn("right")
    elif diff == 3:
        c.turn("left")
    elif diff == 2:
        c.turn("right")
        c.turn("right")
    return target


def rel(heading, offset):
    return DIRS[(DIRS.index(heading) + offset) % 4]


mx, my = width // 2, rows // 2
goals = {(gx, gy) for gx in (mx - 1, mx) for gy in (my - 1, my)}
path = shortest_path((0, 0), goals)

c.log("WALLTEST maze=practice-8x8-a  jalur=" + " ".join(path) + f"  ({len(path)} langkah)")

checks = []  # (label, terbaca, sebenarnya, nilai)

# --- Sel start: pindai 4 sisi dengan FL/FR sambil berputar di tempat -------
heading = "N"
for d in ("N", "W", "E", "S"):
    heading = turn_to(heading, d)
    seen, value = c.read_front_wall()
    checks.append((f"start {d}", seen, truth_wall(0, 0, d), value))
heading = turn_to(heading, "N")

# --- Susuri jalur, baca dinding tiap sel hanya dari sensor ---------------------
x, y = 0, 0
for d in path:
    heading = turn_to(heading, d)
    c.drive_forward_cells(1)
    x, y = x + DELTA[d][0], y + DELTA[d][1]
    walls = c.sense_walls()
    _, front_value = c.read_front_wall()
    left_value, right_value = c.last_side_values
    for label, seen, d_abs, value in (
        ("depan", walls["front"], heading, front_value),
        ("kiri", walls["left"], rel(heading, 3), left_value),
        ("kanan", walls["right"], rel(heading, 1), right_value),
    ):
        checks.append((f"({x},{y}) {label}", seen, truth_wall(x, y, d_abs), value))

# --- Ringkasan -----------------------------------------------------------------
wrong = [k for k in checks if k[1] != k[2]]
for label, seen, truth, value in checks:
    flag = "" if seen == truth else "   <-- SALAH"
    c.log(f"WALLTEST {label:<12} terbaca={'dinding' if seen else 'kosong ':<7} "
          f"sebenarnya={'dinding' if truth else 'kosong ':<7} nilai={value:7.1f}{flag}")
c.log(f"WALLTEST selesai: {len(checks) - len(wrong)}/{len(checks)} benar")
for kind in ("depan", "kiri", "kanan", "start"):
    wall_v = [k[3] for k in checks if kind in k[0] and k[2]]
    free_v = [k[3] for k in checks if kind in k[0] and not k[2]]
    if wall_v and free_v:
        c.log(f"WALLTEST {kind:<6} dinding: {min(wall_v):.0f}..{max(wall_v):.0f}   "
              f"kosong: {min(free_v):.0f}..{max(free_v):.0f}")
    elif wall_v or free_v:
        c.log(f"WALLTEST {kind:<6} hanya satu kelas: {min(wall_v or free_v):.0f}..{max(wall_v or free_v):.0f}")

if os.environ.get("MICROMOUSE_QUIT"):
    c.robot.simulationQuit(0)
while c.robot.step(c.TIME_STEP) != -1:
    pass
