#!/usr/bin/env python3
"""
Uji gerak berkelanjutan dan pembacaan dinding di maze practice-8x8-a.

Robot menyusuri jalur terpendek dari start ke tengah labirin TANPA berhenti
di tiap sel (run_path: lurus dengan profil kecepatan trapesium, belok halus).
Dinding dibaca hanya dari sensor, lalu dibandingkan dengan file maze. File
maze dipakai untuk dua hal saja: memilih jalur bebas tabrakan (BFS) dan
menilai hasil -- tidak pernah untuk menentukan pembacaan.

Yang dilaporkan (baris WALLTEST di run_log.txt):
  - waktu tempuh, kecepatan rata-rata
  - galat posisi/arah SEBENARNYA saat melewati tengah tiap sel
  - benar/salah pembacaan dinding untuk sel yang teramati (sel tempat belok
    halus tidak teramati; sel start tidak dibaca)

Set variabel lingkungan MICROMOUSE_QUIT=1 agar Webots keluar sendiri setelah
selesai (untuk uji headless); tanpa itu robot idle seperti biasa.
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

MAZE = os.path.abspath(os.path.join(
    HERE, "..", "..", "..", "..", "mazes", "mazefiles", "training", "practice-8x8-a.txt"))

DIRS = ["N", "E", "S", "W"]
DELTA = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}
HEADING_DEG = {"E": 0.0, "N": 90.0, "W": 180.0, "S": -90.0}

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


def rel(heading, offset):
    return DIRS[(DIRS.index(heading) + offset) % 4]


def to_plan(path, heading="N"):
    """Arah mutlak per langkah -> aksi relatif 'F'/'L'/'R' per sel."""
    plan = []
    for d in path:
        diff = (DIRS.index(d) - DIRS.index(heading)) % 4
        plan.append({0: "F", 1: "R", 3: "L"}[diff])
        heading = d
    return "".join(plan)


mx, my = width // 2, rows // 2
goals = {(gx, gy) for gx in (mx - 1, mx) for gy in (my - 1, my)}
path = shortest_path((0, 0), goals)
plan = to_plan(path)
c.log("WALLTEST maze=practice-8x8-a  jalur=" + " ".join(path) + f"  ({len(path)} langkah)")
c.log("WALLTEST rencana relatif=" + plan)

t0 = c.robot.getTime()
obs = c.run_path(plan)
elapsed = c.robot.getTime() - t0
length = len(path) * c.CELL
c.log(f"WALLTEST waktu tempuh {elapsed:.1f} s (simulasi), {length:.2f} m, "
      f"kecepatan rata-rata {length / elapsed:.3f} m/s")

checks = []  # (label, terbaca, sebenarnya, nilai)
x, y = 0, 0
pos_err, head_err = [], []
for k, d in enumerate(path):
    x, y = x + DELTA[d][0], y + DELTA[d][1]
    o = obs[k]
    if o["pose"]:
        px, py, ph = o["pose"]
        dx, dy = px - (x + 0.5) * c.CELL, py - (y + 0.5) * c.CELL
        e = 1000 * math.hypot(dx, dy)
        hx, hy = DELTA[d]
        along = 1000 * (dx * hx + dy * hy)    # + = di depan tengah sel
        cross = 1000 * (-dx * hy + dy * hx)   # + = di kiri jalur tengah
        he = (ph - HEADING_DEG[d] + 180) % 360 - 180
        pos_err.append(e)
        head_err.append(he)
        c.log(f"WALLTEST ({x},{y}) {d} galat posisi {e:5.1f} mm "
              f"(sepanjang jalur {along:+5.1f}, ke kiri {cross:+5.1f})  galat arah {he:+5.1f} derajat")
    else:
        c.log(f"WALLTEST ({x},{y}) {d} belok halus (tidak teramati)")
    if o.get("pitch") is not None:
        c.log(f"WALLTEST ({x},{y}) kemiringan depan-belakang saat sampel samping {o['pitch']:+.2f} derajat")
    for label, seen, d_abs, value in (
        ("depan", o["front"], d, o["front_value"]),
        ("kiri", o["left"], rel(d, 3), o["left_value"]),
        ("kanan", o["right"], rel(d, 1), o["right_value"]),
    ):
        if seen is not None:
            checks.append((f"({x},{y}) {label}", seen, truth_wall(x, y, d_abs), value))

wrong = [k for k in checks if k[1] != k[2]]
for label, seen, truth, value in checks:
    flag = "" if seen == truth else "   <-- SALAH"
    c.log(f"WALLTEST {label:<12} terbaca={'dinding' if seen else 'kosong ':<7} "
          f"sebenarnya={'dinding' if truth else 'kosong ':<7} nilai={value:7.1f}{flag}")
c.log(f"WALLTEST selesai: {len(checks) - len(wrong)}/{len(checks)} pembacaan benar "
      f"(dari {3 * len(path)} kemungkinan; sisanya sel belok halus / tak teramati)")
if pos_err:
    c.log(f"WALLTEST galat posisi di tengah sel: maks {max(pos_err):.1f} mm, "
          f"rata-rata {sum(pos_err) / len(pos_err):.1f} mm; "
          f"galat arah maks {max(abs(h) for h in head_err):.1f} derajat")
for kind in ("depan", "kiri", "kanan"):
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
