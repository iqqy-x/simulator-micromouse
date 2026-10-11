#!/usr/bin/env python3
"""
Visualisasi jalur terbaik dari Q-table: satu gambar PNG per langkah.

Cara pakai (dari folder ini, SETELAH training selesai):
    python visualize_path.py                      # maze practice-8x8-a.txt
    python visualize_path.py path/maze.txt        # maze lain (format mms)
    python visualize_path.py --gif                # sekaligus buat jalur.gif

Jalur dibuat dengan mengikuti nilai Q tertinggi dari (0,0) sampai goal,
sama seperti run greedy akhir di main.py. Tiap gambar = satu keputusan:
- sel merah        : posisi robot saat ini
- angka di sel     : nilai Q untuk arah N (atas), E (kanan), S (bawah),
                     W (kiri); arah yang dipilih ditandai kotak kuning,
                     tanda x = ada dinding di arah itu
- sel merah muda   : sel yang sudah dilewati (panah = arah yang diambil)
- sel hijau (G)    : sel tujuan
Dinding digambar dari file maze, area digambar hanya sebatas bagian maze
yang terjangkau dari start. Hasil disimpan ke qpath_frames/.

Selain itu dibuat kebijakan.png: satu gambar grid dengan panah arah Q
tertinggi di setiap sel yang pernah dipelajari (jalur terbaik berwarna
merah muda).
"""

import pickle
import sys
from collections import deque
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # simpan ke file, tidak perlu jendela GUI
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

HERE = Path(__file__).resolve().parent
DEFAULT_MAZE = HERE.parents[1] / "mazes" / "mazefiles" / "training" / "practice-8x8-a.txt"
STATE_FILE = HERE / "qtable_state.pkl"
OUT_DIR = HERE / "qpath_frames"

DIRS = ["N", "E", "S", "W"]
DELTA = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}
ARROW = {"N": "↑", "E": "→", "S": "↓", "W": "←"}


class Maze:
    """Baca maze format mms: baris genap = dinding horizontal ('---'),
    baris ganjil = dinding vertikal ('|'); baris pertama = sisi atas."""

    def __init__(self, path):
        lines = [l.rstrip("\n") for l in open(path, encoding="utf-8") if l.strip()]
        self.lines = lines
        self.w = lines[0].count("o") - 1
        self.h = (len(lines) - 1) // 2

    def hwall(self, b, x):
        """Dinding horizontal di batas b (0 = bawah, h = atas) untuk kolom x."""
        return self.lines[2 * (self.h - b)][4 * x + 1:4 * x + 4] == "---"

    def vwall(self, y, b):
        """Dinding vertikal di batas b (0 = kiri, w = kanan) untuk baris y."""
        line = self.lines[2 * (self.h - 1 - y) + 1]
        return 4 * b < len(line) and line[4 * b] == "|"

    def blocked(self, x, y, d):
        return {
            "N": lambda: self.hwall(y + 1, x),
            "E": lambda: self.vwall(y, x + 1),
            "S": lambda: self.hwall(y, x),
            "W": lambda: self.vwall(y, x),
        }[d]()

    def open_dirs(self, x, y):
        return [d for d in DIRS if not self.blocked(x, y, d)]

    def reachable(self, start=(0, 0)):
        seen, q = {start}, deque([start])
        while q:
            x, y = q.popleft()
            for d in self.open_dirs(x, y):
                n = (x + DELTA[d][0], y + DELTA[d][1])
                if n not in seen:
                    seen.add(n)
                    q.append(n)
        return seen

    def shortest(self, goals):
        dist, q = {(0, 0): 0}, deque([(0, 0)])
        while q:
            c = q.popleft()
            for d in self.open_dirs(*c):
                n = (c[0] + DELTA[d][0], c[1] + DELTA[d][1])
                if n not in dist:
                    dist[n] = dist[c] + 1
                    q.append(n)
        return min((dist[g] for g in goals if g in dist), default=None)


def goal_cells(w, h):
    # Sama dengan goal_cells() di main.py / aturan goal mms.
    xs = [w // 2 - 1, w // 2] if w % 2 == 0 else [w // 2]
    ys = [h // 2 - 1, h // 2] if h % 2 == 0 else [h // 2]
    return {(gx, gy) for gx in xs for gy in ys}


def greedy_path(q, maze, goals):
    """Ikuti Q tertinggi dari (0,0). Kembalikan daftar (x, y, aksi) per
    langkah dan apakah goal tercapai. Berhenti kalau masuk loop."""
    x, y, steps, seen = 0, 0, [], {(0, 0)}
    for _ in range(4 * maze.w * maze.h):
        if (x, y) in goals:
            return steps, True
        opens = maze.open_dirs(x, y)
        best = max(q.get((x, y, d), 0.0) for d in opens)
        a = next(d for d in opens if q.get((x, y, d), 0.0) == best)
        steps.append((x, y, a))
        x, y = x + DELTA[a][0], y + DELTA[a][1]
        if (x, y) in seen:
            return steps, False  # loop: Q-table belum cukup matang
        seen.add((x, y))
    return steps, (x, y) in goals


def draw_walls(ax, maze, bbox):
    x0, y0, x1, y1 = bbox
    for y in range(y0, y1 + 2):
        for x in range(x0, x1 + 1):
            if maze.hwall(y, x):
                ax.plot([x, x + 1], [y, y], color="black", lw=3, solid_capstyle="round")
    for x in range(x0, x1 + 2):
        for y in range(y0, y1 + 1):
            if maze.vwall(y, x):
                ax.plot([x, x], [y, y + 1], color="black", lw=3, solid_capstyle="round")


def draw_policy(maze, q, goals, bbox, steps, out_path):
    """Satu gambar: panah arah Q tertinggi di tiap sel yang pernah dipelajari.
    Sel di jalur terbaik berwarna merah muda; sel kosong = belum dikunjungi."""
    x0, y0, x1, y1 = bbox
    on_path = {(sx, sy) for sx, sy, _ in steps}
    fig, ax = plt.subplots(figsize=(9, 9.6))
    ax.set_xlim(x0, x1 + 1)
    ax.set_ylim(y0, y1 + 1)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.subplots_adjust(left=0.02, right=0.98, top=0.94, bottom=0.07)

    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            color = "#c8e6c9" if (x, y) in goals else "#f8d7d4" if (x, y) in on_path else "white"
            ax.add_patch(Rectangle((x, y), 1, 1, facecolor=color, edgecolor="#cccccc", lw=0.6))
            if (x, y) in goals:
                ax.text(x + 0.5, y + 0.5, "G", ha="center", va="center", fontsize=16, color="#2e7d32", weight="bold")
                continue
            opens = maze.open_dirs(x, y)
            if not any((x, y, d) in q for d in opens):
                continue  # belum pernah dikunjungi: tidak ada nilai Q
            best = max(opens, key=lambda d: q.get((x, y, d), 0.0))
            ax.text(x + 0.5, y + 0.5, ARROW[best], ha="center", va="center", fontsize=26,
                    color="#b71c1c" if (x, y) in on_path else "#37474f")
            if (x, y) == (0, 0):
                ax.text(x + 0.12, y + 0.84, "S", ha="center", va="center", fontsize=11, color="#1565c0", weight="bold")

    draw_walls(ax, maze, bbox)
    ax.set_title("Kebijakan hasil Q-table: arah terbaik di tiap sel", fontsize=13, pad=10)
    fig.text(0.5, 0.025,
             "Panah = arah dengan Q tertinggi; merah muda = jalur terbaik dari start; "
             "kosong = belum pernah dikunjungi",
             ha="center", fontsize=9, color="#555555")
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def draw_frame(maze, q, goals, bbox, steps, k, total_ok, out_path):
    x0, y0, x1, y1 = bbox  # batas sel inklusif
    cur_x, cur_y, action = steps[k]
    fig, ax = plt.subplots(figsize=(9, 9.6))
    ax.set_xlim(x0, x1 + 1)
    ax.set_ylim(y0, y1 + 1)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.subplots_adjust(left=0.02, right=0.98, top=0.94, bottom=0.07)

    visited = {(sx, sy): a for sx, sy, a in steps[:k]}
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            color = "white"
            if (x, y) in goals:
                color = "#c8e6c9"
            if (x, y) in visited:
                color = "#f8d7d4"
            if (x, y) == (cur_x, cur_y):
                color = "#e53935"
            ax.add_patch(Rectangle((x, y), 1, 1, facecolor=color, edgecolor="#cccccc", lw=0.6))
            if (x, y) in goals and (x, y) != (cur_x, cur_y):
                ax.text(x + 0.5, y + 0.5, "G", ha="center", va="center", fontsize=16, color="#2e7d32", weight="bold")
            if (x, y) == (0, 0) and (x, y) != (cur_x, cur_y):
                # Huruf kecil di sudut supaya tidak menimpa panah jalur.
                ax.text(x + 0.12, y + 0.84, "S", ha="center", va="center", fontsize=11, color="#1565c0", weight="bold")
            if (x, y) in visited:
                ax.text(x + 0.5, y + 0.5, ARROW[visited[(x, y)]], ha="center", va="center", fontsize=18, color="#b71c1c")

    draw_walls(ax, maze, bbox)

    # Nilai Q untuk 4 arah di sel saat ini
    pos = {"N": (0.5, 0.80), "S": (0.5, 0.20), "E": (0.76, 0.5), "W": (0.24, 0.5)}
    for d in DIRS:
        tx, ty = cur_x + pos[d][0], cur_y + pos[d][1]
        if maze.blocked(cur_x, cur_y, d):
            ax.text(tx, ty, "×", ha="center", va="center", fontsize=14, color="#ffcdd2")
            continue
        val = q.get((cur_x, cur_y, d), 0.0)
        chosen = d == action
        ax.text(
            tx, ty, f"{val:.1f}", ha="center", va="center", fontsize=10,
            weight="bold" if chosen else "normal",
            color="black" if chosen else "white",
            bbox=dict(boxstyle="round,pad=0.15", fc="#ffeb3b", ec="none") if chosen else None,
        )

    qv = q.get((cur_x, cur_y, action), 0.0)
    ax.set_title(
        f"Langkah {k + 1}/{len(steps)}   sel ({cur_x},{cur_y}) → {action} ({ARROW[action]})   Q = {qv:.2f}",
        fontsize=13, pad=10,
    )
    fig.text(0.5, 0.025,
             "Angka di sel merah: Q untuk arah atas=N, kanan=E, bawah=S, kiri=W (kuning = dipilih, × = dinding)"
             + ("" if total_ok else "\nPERINGATAN: jalur dari Q-table masuk loop, belum sampai goal"),
             ha="center", fontsize=9, color="#555555")
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    make_gif = "--gif" in sys.argv[1:]
    maze_path = Path(args[0]) if args else DEFAULT_MAZE

    with open(STATE_FILE, "rb") as f:
        data = pickle.load(f)
    q = data["q"]
    maze = Maze(maze_path)
    if (data.get("w"), data.get("h")) != (maze.w, maze.h):
        print(f"PERINGATAN: Q-table untuk maze {data.get('w')}x{data.get('h')}, "
              f"file maze {maze.w}x{maze.h} -- hasil mungkin tidak cocok.")

    goals = goal_cells(maze.w, maze.h)
    steps, ok = greedy_path(q, maze, goals)
    reach = maze.reachable()
    bbox = (min(c[0] for c in reach), min(c[1] for c in reach),
            max(c[0] for c in reach), max(c[1] for c in reach))
    optimal = maze.shortest(goals)

    print(f"Maze   : {maze_path.name} ({maze.w}x{maze.h}), area terjangkau {bbox[2]-bbox[0]+1}x{bbox[3]-bbox[1]+1}")
    print(f"Jalur Q: {len(steps)} langkah, {'sampai goal' if ok else 'TIDAK sampai goal (loop)'}; "
          f"optimal BFS: {optimal} langkah")

    OUT_DIR.mkdir(exist_ok=True)
    for old in OUT_DIR.glob("langkah_*.png"):
        old.unlink()
    files = []
    for k in range(len(steps)):
        p = OUT_DIR / f"langkah_{k + 1:02d}.png"
        draw_frame(maze, q, goals, bbox, steps, k, ok, p)
        files.append(p)
    print(f"{len(files)} gambar disimpan ke: {OUT_DIR}")

    policy_png = OUT_DIR / "kebijakan.png"
    draw_policy(maze, q, goals, bbox, steps, policy_png)
    print(f"Gambar kebijakan (panah tiap sel): {policy_png}")

    if make_gif:
        from PIL import Image
        frames = [Image.open(p).convert("P", palette=Image.ADAPTIVE) for p in files]
        gif = OUT_DIR / "jalur.gif"
        frames[0].save(gif, save_all=True, append_images=frames[1:], duration=700, loop=0)
        print(f"Animasi disimpan ke: {gif}")


if __name__ == "__main__":
    main()
