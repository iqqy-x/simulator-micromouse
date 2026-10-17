#!/usr/bin/env python3
"""
Primitif gerak low-level untuk micromouse kontinu di Webots: "maju N sel" dan
"belok 90 derajat", pakai feedback encoder roda (PositionSensor) supaya
presisi -- bukan cuma kecepatan+waktu tebakan.

Ini LAPISAN KENDALI, bukan algoritma navigasi. Rencananya planner yang
sudah ada (flood-fill/A*/Q-table, format mms) dipakai ulang tanpa berubah --
dia cuma manggil drive_forward_cells(1)/turn("left"/"right") alih-alih
mengirim perintah "moveForward"/"turnLeft" lewat stdin/stdout ke mms.

Urutan di bawah ini cuma UJI COBA primitif (maju 2 sel, belok kanan, maju 1
sel) -- bukan algoritma solving maze. Lihat run_log.txt setelah dijalankan
untuk angka presisinya (target vs tercapai).
"""

import math
import os

from controller import Robot

TIME_STEP = 16

# Harus sama dengan generate_robot() di sim/webots/tools/maze_to_webots.py --
# kalau maze di-generate ulang dengan --category halfsize, CELL perlu
# diganti ke 0.09.
CELL = 0.18
WHEEL_R = 0.016
TRACK_HALF = 0.032

CRUISE_SPEED = 4.0   # rad/s roda saat maju lurus
TURN_SPEED = 3.0     # rad/s roda saat belok di tempat

FRONT_WALL_THRESHOLD = 600.0  # nilai sensor depan/kiri/kanan di atas ini -> dianggap ada dinding

log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "run_log.txt")
log_file = open(log_path, "w", encoding="utf-8")


def log(msg):
    print(msg)
    log_file.write(msg + "\n")
    log_file.flush()


robot = Robot()

left_motor = robot.getDevice("left_wheel_motor")
right_motor = robot.getDevice("right_wheel_motor")
left_motor.setPosition(float("inf"))  # mode kecepatan, bukan posisi target
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

left_enc = robot.getDevice("left_wheel_sensor")
right_enc = robot.getDevice("right_wheel_sensor")
left_enc.enable(TIME_STEP)
right_enc.enable(TIME_STEP)

ds_front = robot.getDevice("ds_front")
ds_left = robot.getDevice("ds_left")
ds_right = robot.getDevice("ds_right")
for ds in (ds_front, ds_left, ds_right):
    ds.enable(TIME_STEP)

# Satu step dulu supaya semua sensor punya nilai valid sebelum dipakai.
robot.step(TIME_STEP)


def read_walls():
    front, left, right = ds_front.getValue(), ds_left.getValue(), ds_right.getValue()
    walls = {
        "front": front > FRONT_WALL_THRESHOLD,
        "left": left > FRONT_WALL_THRESHOLD,
        "right": right > FRONT_WALL_THRESHOLD,
    }
    log(f"  sensor: front={front:.1f} left={left:.1f} right={right:.1f} -> {walls}")
    return walls


def drive_forward_cells(n=1):
    """Maju N sel lurus, berhenti berdasarkan rata-rata rotasi roda (bukan
    waktu/kecepatan tebakan). Encoder = radian kumulatif sejak start."""
    target = n * CELL / WHEEL_R
    l0, r0 = left_enc.getValue(), right_enc.getValue()
    left_motor.setVelocity(CRUISE_SPEED)
    right_motor.setVelocity(CRUISE_SPEED)

    avg = 0.0
    while robot.step(TIME_STEP) != -1:
        dl = left_enc.getValue() - l0
        dr = right_enc.getValue() - r0
        avg = (dl + dr) / 2.0
        if avg >= target:
            break

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    log(f"maju {n} sel: target={target:.3f} rad, tercapai={avg:.3f} rad "
        f"(kiri={dl:.3f}, kanan={dr:.3f})")


def turn(direction):
    """Belok 90 derajat di tempat. direction: 'left' atau 'right'.

    BELUM DIVALIDASI arah putarnya benar atau terbalik -- cek run_log.txt /
    lihat langsung di Webots, kalau 'right' ternyata belok kiri (atau
    sebaliknya), tinggal tukar tanda TURN_SPEED di dua baris setVelocity
    di bawah.
    """
    target = TRACK_HALF * (math.pi / 2) / WHEEL_R
    l0, r0 = left_enc.getValue(), right_enc.getValue()

    sign = 1.0 if direction == "right" else -1.0
    left_motor.setVelocity(sign * TURN_SPEED)
    right_motor.setVelocity(-sign * TURN_SPEED)

    avg = 0.0
    while robot.step(TIME_STEP) != -1:
        dl = abs(left_enc.getValue() - l0)
        dr = abs(right_enc.getValue() - r0)
        avg = (dl + dr) / 2.0
        if avg >= target:
            break

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    log(f"belok {direction}: target={target:.3f} rad, tercapai={avg:.3f} rad")


log("=== Uji primitif gerak (bukan solving maze) ===")
log("Urutan: maju 1 sel -> belok kanan -> maju 1 sel -> belok kiri -> maju 1 sel")
log("(sudah dicek lewat parser maze -- jalur ini bebas dinding di minimaze.txt)")
read_walls()
drive_forward_cells(1)
read_walls()
turn("right")
read_walls()
drive_forward_cells(1)
read_walls()
turn("left")
read_walls()
drive_forward_cells(1)
read_walls()
log("=== Selesai. Cek log di atas: tercapai harus dekat dengan target. ===")
log("=== Robot sekarang idle (motor berhenti). ===")

while robot.step(TIME_STEP) != -1:
    pass
