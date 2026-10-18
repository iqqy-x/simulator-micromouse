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

from controller import Supervisor

TIME_STEP = 16

# Harus sama dengan generate_robot() di sim/webots/tools/maze_to_webots.py --
# kalau maze di-generate ulang dengan --category halfsize, CELL perlu
# diganti ke 0.09.
CELL = 0.18
WHEEL_R = 0.017      # N20 wheel 34x7
TRACK_HALF = 0.0365  # jarak antar roda 73 mm (poros di PCB x=3.5 dan 76.5)

CRUISE_SPEED = 4.0   # rad/s roda saat maju lurus (maks motor 31.4 rad/s)
TURN_SPEED = 3.0     # rad/s roda saat belok di tempat

# Sensor PCB: FL/FR menghadap depan, DL/DR diagonal 45 derajat. Ambang
# dinding BELUM ada -- tunggu kalibrasi dari nilai mentah di run_log.txt.
SENSOR_NAMES = ("ds_fl", "ds_fr", "ds_dl", "ds_dr")

log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "run_log.txt")
log_file = open(log_path, "w", encoding="utf-8")


def log(msg):
    print(msg)
    log_file.write(msg + "\n")
    log_file.flush()


# Supervisor = Robot + akses ke posisi sebenarnya (untuk validasi dan reset
# antar episode). Posisi sebenarnya TIDAK dipakai untuk navigasi.
robot = Supervisor()
self_node = robot.getSelf()

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

sensors = {name: robot.getDevice(name) for name in SENSOR_NAMES}
for ds in sensors.values():
    ds.enable(TIME_STEP)

# Satu step dulu supaya semua sensor punya nilai valid sebelum dipakai.
robot.step(TIME_STEP)


def log_true_pose():
    """Posisi (m) dan arah hadap (derajat, 0 = Timur, 90 = Utara) sebenarnya."""
    x, y, _ = self_node.getPosition()
    r = self_node.getOrientation()
    log(f"  posisi sebenarnya: x={x:.3f} y={y:.3f} arah={math.degrees(math.atan2(r[3], r[0])):.1f}")


def read_sensors():
    """Nilai mentah 4 sensor (skala ~ADC 12-bit, makin besar makin dekat)."""
    log_true_pose()
    values = {name: ds.getValue() for name, ds in sensors.items()}
    log("  sensor: " + " ".join(f"{n[3:].upper()}={v:.0f}" for n, v in values.items()))
    return values


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
read_sensors()
drive_forward_cells(1)
read_sensors()
turn("right")
read_sensors()
drive_forward_cells(1)
read_sensors()
turn("left")
read_sensors()
drive_forward_cells(1)
read_sensors()
log("=== Selesai. Cek log di atas: tercapai harus dekat dengan target. ===")
log("=== Robot sekarang idle (motor berhenti). ===")

while robot.step(TIME_STEP) != -1:
    pass
