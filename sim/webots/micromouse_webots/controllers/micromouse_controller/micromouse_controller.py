#!/usr/bin/env python3
"""
Primitif gerak low-level untuk micromouse kontinu di Webots: "maju N sel" dan
"belok 90 derajat". Jarak dari encoder roda, SUDUT dari gyro (MPU6050 di PCB).

Kenapa gyro: belok di tempat hanya dengan hitungan encoder meleset (roda
slip; uji awal: encoder 'sampai' tapi badan baru berputar 59 dari 90 derajat).
Sama seperti robot asli, arah acuan (target_yaw) dijaga mutlak: tiap belok
menambah +-90 derajat ke acuan, dan saat maju robot dikoreksi supaya
yaw mengikuti acuan, jadi error tidak menumpuk.

Ini LAPISAN KENDALI, bukan algoritma navigasi. Rencananya planner yang sudah
ada (flood-fill/A*/Q-table, format mms) dipakai ulang tanpa berubah -- dia
cuma manggil drive_forward_cells(1)/turn("left"/"right") alih-alih mengirim
perintah "moveForward"/"turnLeft" lewat stdin/stdout ke mms.

Urutan di bawah ini cuma UJI COBA primitif (maju, belok kanan, maju, belok
kiri, maju) -- bukan algoritma solving maze. Lihat run_log.txt setelah
dijalankan: tiap baris mencatat posisi/arah SEBENARNYA dari Supervisor
(hanya untuk validasi, tidak dipakai navigasi) dan 4 nilai sensor mentah.
"""

import math
import os

from controller import Supervisor

TIME_STEP = 16
DT = TIME_STEP / 1000.0

# Harus sama dengan generate_robot() di sim/webots/tools/maze_to_webots.py --
# kalau maze di-generate ulang dengan --category halfsize, CELL perlu
# diganti ke 0.09.
CELL = 0.18
WHEEL_R = 0.017      # N20 wheel 34x7
TRACK_HALF = 0.0365  # jarak antar roda 73 mm (poros di PCB x=3.5 dan 76.5)

CRUISE_SPEED = 4.0   # rad/s roda saat maju lurus (maks motor 31.4 rad/s)
MIN_SPEED = 0.8      # rad/s roda saat merayap mendekati target
ACCEL = 20.0         # rad/s^2 batas percepatan roda
DECEL_GAIN = 2.0     # kecepatan = gain * sisa rotasi roda (rad), dibatasi di atas
STOP_TOL = 0.03      # rad rotasi roda (~0.5 mm) -- dianggap sampai

TURN_SPEED = 3.0     # rad/s roda maksimum saat belok di tempat
TURN_MIN = 0.5       # rad/s roda minimum saat belok (mendekati target)
TURN_GAIN = 6.0      # rad/s roda per radian sisa sudut
TURN_TOL = math.radians(0.5)
HEADING_KP = 10.0    # rad/s roda per radian error arah saat maju lurus

SETTLE_STEPS = 6     # langkah tunggu setelah berhenti supaya robot diam

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

gyro = robot.getDevice("gyro")
gyro.enable(TIME_STEP)

sensors = {name: robot.getDevice(name) for name in SENSOR_NAMES}
for ds in sensors.values():
    ds.enable(TIME_STEP)

yaw = 0.0         # integrasi gyro sumbu Z (rad), + = ke kiri
target_yaw = 0.0  # arah acuan mutlak; berubah +-90 derajat tiap belok


def step():
    """Satu langkah simulasi + integrasi gyro. Semua tunggu lewat sini."""
    global yaw
    if robot.step(TIME_STEP) == -1:
        raise SystemExit
    yaw += gyro.getValues()[2] * DT


def set_wheels(left, right):
    left_motor.setVelocity(left)
    right_motor.setVelocity(right)


def settle():
    set_wheels(0.0, 0.0)
    for _ in range(SETTLE_STEPS):
        step()


# Satu step dulu supaya semua sensor punya nilai valid sebelum dipakai.
step()


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
    """Maju N sel lurus. Jarak dari rata-rata rotasi roda (encoder), arah
    dijaga lewat gyro, kecepatan dihaluskan (ramp naik, melambat mendekati
    target) supaya roda tidak slip."""
    target = n * CELL / WHEEL_R
    l0, r0 = left_enc.getValue(), right_enc.getValue()
    speed = 0.0
    avg = 0.0

    while True:
        avg = ((left_enc.getValue() - l0) + (right_enc.getValue() - r0)) / 2.0
        remaining = target - avg
        if remaining <= STOP_TOL:
            break
        wanted = min(CRUISE_SPEED, max(MIN_SPEED, DECEL_GAIN * remaining))
        speed = min(wanted, speed + ACCEL * DT)
        correction = HEADING_KP * (target_yaw - yaw)  # + = perlu belok kiri
        set_wheels(speed - correction, speed + correction)
        step()

    settle()
    dl = left_enc.getValue() - l0
    dr = right_enc.getValue() - r0
    log(f"maju {n} sel: target={target:.3f} rad, tercapai={avg:.3f} rad "
        f"(kiri={dl:.3f}, kanan={dr:.3f}), yaw={math.degrees(yaw):.1f}")


def turn(direction):
    """Belok 90 derajat di tempat. direction: 'left' atau 'right'. Berhenti
    berdasarkan sudut gyro terhadap arah acuan, bukan rotasi roda."""
    global target_yaw
    target_yaw += (math.pi / 2) * (1.0 if direction == "left" else -1.0)

    while True:
        err = target_yaw - yaw  # + = perlu belok kiri
        if abs(err) <= TURN_TOL:
            break
        w = min(TURN_SPEED, max(TURN_MIN, TURN_GAIN * abs(err)))
        w = math.copysign(w, err)
        set_wheels(-w, w)
        step()

    settle()
    log(f"belok {direction}: yaw={math.degrees(yaw):.1f} "
        f"(acuan {math.degrees(target_yaw):.1f})")


if __name__ == "__main__":
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
    log("=== Selesai. Bandingkan posisi/arah sebenarnya dengan yang diharapkan. ===")
    log("=== Robot sekarang idle (motor berhenti). ===")

    while robot.step(TIME_STEP) != -1:
        pass
