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
WHEEL_R = 0.0145     # roda dari model 3D: diameter 29 mm
WHEEL_W = 0.0086     # lebar 8.6 mm
TRACK_HALF = 0.0373  # jarak antar pusat roda 74.6 mm

CRUISE_SPEED = 4.0   # rad/s roda saat maju lurus (maks motor 31.4 rad/s)
MIN_SPEED = 0.8      # rad/s roda saat merayap mendekati target
ACCEL = 20.0         # rad/s^2 batas percepatan roda
DECEL_GAIN = 2.0     # kecepatan = gain * sisa rotasi roda (rad), dibatasi di atas
STOP_TOL = 0.03      # rad rotasi roda (~0.5 mm) -- dianggap sampai

TURN_SPEED = 3.0     # rad/s roda maksimum saat belok di tempat
TURN_MIN = 0.5       # rad/s roda minimum saat belok (mendekati target)
TURN_GAIN = 6.0      # rad/s roda per radian sisa sudut
TURN_TOL = math.radians(0.5)
# Gaya belok: "pivot" = satu roda saja yang berputar (roda lain diam),
# "point" = kedua roda berlawanan arah (belok di tempat). Lihat turn().
TURN_STYLE = "pivot"
# Roda yang diam berputar di titik kontaknya di sisi LUAR silinder (lebar 7 mm),
# bukan di tengah, jadi jari-jari pivot efektif = TRACK_HALF + lebar roda / 2.
# Terukur: dengan TRACK_HALF saja, tiap belok meleset ~4 mm ke depan dan ~3 mm
# ke samping; dengan nilai ini hasilnya cocok.
PIVOT_RADIUS = TRACK_HALF + WHEEL_W / 2.0
PIVOT_SPEED = 6.0    # rad/s roda yang berputar (rotasi badan = w*R/(2*TRACK_HALF))
PIVOT_MIN = 1.0
PIVOT_GAIN = 12.0
HEADING_KP = 10.0    # rad/s roda per radian error arah saat maju lurus

SETTLE_STEPS = 6     # langkah tunggu setelah berhenti supaya robot diam

# Sensor PCB: FL/FR menghadap depan, DL/DR diagonal 45 derajat ke luar.
SENSOR_NAMES = ("ds_fl", "ds_fr", "ds_dl", "ds_dr")

# Pembacaan dinding (nilai sensor ~ADC 12-bit, kurva respons masih perkiraan).
# Dinding DEPAN: FL/FR saat robot berhenti di tengah sel. Dinding ada ->
# ~36 mm dari sensor (nilai ~2100); tidak ada -> dinding berikutnya >200 mm
# (nilai <60). Ambang di tengah-tengah dengan margin besar.
FRONT_THRESHOLD = 800.0
FRONT_SAMPLES = 5
# Dinding SAMPING: DL/DR menghadap diagonal 45 derajat dan dipasang di depan
# (+-9.5 mm dari sumbu, 60 mm di depan poros roda), jadi dari tengah sel
# sinarnya mengenai dinding samping sel DI DEPAN. Sinar baru mengenai dinding
# samping sel yang SEDANG dimasuki kalau robot ~135 mm sebelum tengah sel
# (45 mm setelah meninggalkan tengah sel sebelumnya). Karena itu DL/DR
# disampel saat bergerak di jendela jarak tempuh itu. Dinding ada -> sinar
# ~105 mm (nilai ~400); tidak ada -> sinar keluar sel, >250 mm (nilai 0).
SIDE_SAMPLE_AT = 0.045   # m jarak tempuh dari tengah sel sebelumnya
SIDE_WINDOW = 0.040      # m lebar jendela (rata-rata semua sampel di dalamnya)
SIDE_THRESHOLD = 120.0

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


last_side_walls = None   # (kiri, kanan) sel terakhir yang dimasuki, atau None
last_side_values = None  # nilai rata-rata DL/DR di jendela sampel (untuk kalibrasi)

# Posisi robot terhadap TENGAH sel, searah arah hadap (m). + = di depan
# tengah sel. Belok satu-roda (pivot) membuat sumbu roda berpindah, jadi
# posisi ini harus dilacak (lihat turn()).
offset = 0.0


def _drive(dist, side_cells=0):
    """Gerak lurus sejauh dist meter (+ maju, - mundur). Jarak dari rata-rata
    rotasi roda (encoder), arah dijaga lewat gyro, kecepatan dihaluskan (ramp
    naik, melambat mendekati target) supaya roda tidak slip.

    side_cells > 0: DL/DR disampel untuk dinding kiri/kanan tiap sel yang
    dimasuki (lihat SIDE_*). Return (sampel per sel, rotasi roda tercapai)."""
    sign = 1.0 if dist >= 0 else -1.0
    target = abs(dist) / WHEEL_R
    l0, r0 = left_enc.getValue(), right_enc.getValue()
    speed = 0.0
    avg = 0.0
    samples = [[] for _ in range(side_cells)]  # per sel: daftar (DL, DR)

    while True:
        avg = sign * ((left_enc.getValue() - l0) + (right_enc.getValue() - r0)) / 2.0
        if side_cells:
            from_center = offset + avg * WHEEL_R  # jarak dari tengah sel sebelumnya
            for k in range(side_cells):
                lo = k * CELL + SIDE_SAMPLE_AT - SIDE_WINDOW / 2.0
                if lo <= from_center <= lo + SIDE_WINDOW:
                    samples[k].append((sensors["ds_dl"].getValue(),
                                       sensors["ds_dr"].getValue()))
        remaining = target - avg
        if remaining <= STOP_TOL:
            break
        wanted = min(CRUISE_SPEED, max(MIN_SPEED, DECEL_GAIN * remaining))
        speed = min(wanted, speed + ACCEL * DT)
        # Selisih kecepatan roda memutar badan ke arah yang sama baik maju
        # maupun mundur, jadi koreksi arah memakai rumus yang sama.
        correction = HEADING_KP * (target_yaw - yaw)  # + = perlu belok kiri
        set_wheels(sign * speed - correction, sign * speed + correction)
        step()

    settle()
    return samples, avg


def drive_forward_cells(n=1):
    """Maju N sel lurus dari posisi sekarang sampai tengah sel ke-N. Sambil
    jalan, DL/DR disampel untuk dinding kiri/kanan tiap sel yang dimasuki.
    Return: daftar (kiri, kanan) satu per sel."""
    global offset, last_side_walls, last_side_values
    l0, r0 = left_enc.getValue(), right_enc.getValue()
    samples, avg = _drive(n * CELL - offset, side_cells=n)
    offset = 0.0
    log(f"maju {n} sel: tercapai={avg:.3f} rad "
        f"(kiri={left_enc.getValue() - l0:.3f}, kanan={right_enc.getValue() - r0:.3f}), "
        f"yaw={math.degrees(yaw):.1f}")

    result = []
    for cell_samples in samples:
        if not cell_samples:
            raise RuntimeError("jendela sampel DL/DR kosong -- naikkan SIDE_WINDOW")
        mean_l = sum(s[0] for s in cell_samples) / len(cell_samples)
        mean_r = sum(s[1] for s in cell_samples) / len(cell_samples)
        result.append((mean_l > SIDE_THRESHOLD, mean_r > SIDE_THRESHOLD, mean_l, mean_r))
    last_side_walls = (result[-1][0], result[-1][1])
    last_side_values = (result[-1][2], result[-1][3])  # nilai rata-rata, untuk log/kalibrasi
    return [(r[0], r[1]) for r in result]


def center():
    """Mundur ke tengah sel (offset -> 0). Perlu sebelum membaca dinding
    depan setelah belok pivot, karena robot berhenti di depan tengah sel."""
    global offset
    if abs(offset) > 1e-4:
        _drive(-offset)
        offset = 0.0


def read_front_wall():
    """Dinding di depan sel tempat robot berdiri. Robot dibawa ke tengah sel
    dulu (center()), lalu rata-rata FL dan FR dari beberapa langkah.
    Return (ada_dinding, nilai)."""
    center()
    total = 0.0
    for _ in range(FRONT_SAMPLES):
        step()
        total += (sensors["ds_fl"].getValue() + sensors["ds_fr"].getValue()) / 2.0
    value = total / FRONT_SAMPLES
    return value > FRONT_THRESHOLD, value


def sense_walls():
    """Dinding sel sekarang: dict front/left/right (relatif arah hadap).
    Depan dibaca sekarang; kiri/kanan berasal dari lintasan masuk sel ini
    (drive_forward_cells), jadi hanya sah tepat setelah masuk, sebelum
    belok. Setelah belok, depan baru tetap bisa dibaca dengan
    read_front_wall(). Sel start: kiri/kanan belum diketahui (None)."""
    front, _ = read_front_wall()
    left, right = last_side_walls if last_side_walls else (None, None)
    return {"front": front, "left": left, "right": right}


def _point_turn(direction):
    """Belok di tempat (kedua roda berlawanan arah), hanya dari tengah sel."""
    global target_yaw
    if abs(offset) > 1e-4:
        raise RuntimeError("belok di tempat hanya dari tengah sel -- panggil center()")
    target_yaw += (math.pi / 2) * (1.0 if direction == "left" else -1.0)
    while True:
        err = target_yaw - yaw
        if abs(err) <= TURN_TOL:
            break
        w = math.copysign(min(TURN_SPEED, max(TURN_MIN, TURN_GAIN * abs(err))), err)
        set_wheels(-w, w)
        step()
    settle()
    log(f"belok {direction} (di tempat): yaw={math.degrees(yaw):.1f} "
        f"(acuan {math.degrees(target_yaw):.1f})")


def turn(direction):
    """Belok 90 derajat. direction: 'left' atau 'right'. Berhenti berdasarkan
    sudut gyro terhadap arah acuan, bukan rotasi roda.

    TURN_STYLE "pivot": hanya SATU roda yang berputar -- belok kanan: roda
    kiri maju, roda kanan diam; belok kiri sebaliknya. Pusat robot berputar
    mengelilingi roda yang diam, jadi bergeser PIVOT_RADIUS ke depan dan ke
    samping. Agar akhirnya persis di jalur tengah sel, robot mundur dulu
    (offset + PIVOT_RADIUS) lalu berputar; hasilnya robot berhenti
    PIVOT_RADIUS di depan tengah sel searah arah baru
    (offset = PIVOT_RADIUS), dan
    drive_forward_cells() otomatis menghitung sisa jaraknya.

    TURN_STYLE "point": kedua roda berputar berlawanan arah (belok di
    tempat, tanpa pergeseran); hanya benar kalau robot di tengah sel."""
    global target_yaw, offset
    delta = (math.pi / 2) * (1.0 if direction == "left" else -1.0)

    if TURN_STYLE == "pivot":
        # Mundur DULU dengan arah acuan lama (kalau acuan sudah diubah,
        # penjaga arah ikut memutar badan selama mundur dan jalurnya melengkung).
        _drive(-(offset + PIVOT_RADIUS))
        target_yaw += delta
        while True:
            err = target_yaw - yaw  # + = perlu belok kiri
            if abs(err) <= TURN_TOL:
                break
            w = min(PIVOT_SPEED, max(PIVOT_MIN, PIVOT_GAIN * abs(err)))
            # err > 0: roda kanan maju, kiri diam (pivot di roda kiri)
            set_wheels(0.0, w) if err > 0 else set_wheels(w, 0.0)
            step()
        offset = PIVOT_RADIUS
    else:
        _point_turn(direction)
        return

    settle()
    log(f"belok {direction} ({TURN_STYLE}): yaw={math.degrees(yaw):.1f} "
        f"(acuan {math.degrees(target_yaw):.1f})")


# ---------------------------------------------------------------------------
# Gerak berkelanjutan: lurus dengan profil kecepatan trapesium + belok halus.
# Robot tidak berhenti di tiap sel; hanya berhenti di akhir lintasan.
# ---------------------------------------------------------------------------

V_CRUISE = 0.30      # m/s kecepatan jelajah di jalur lurus
V_TURN = 0.20        # m/s kecepatan di belokan halus
ACCEL_LIN = 2.0      # m/s^2 percepatan dan perlambatan di jalur lurus
V_MIN = 0.04         # m/s kecepatan terendah saat mendekati titik berhenti
YAW_KP = 8.0         # (rad/s) per rad: koreksi arah saat lurus dan di belokan
END_TOL = 0.0005     # m toleransi akhir segmen (encoder)


# --- Pemusatan lateral dengan DL/DR (PID bertingkat) -------------------------
# Galat lateral e (m, + = robot di KIRI tengah lorong) dihitung dari jarak sinar
# DL/DR ke dinding. Hasil PID pada e menjadi OFFSET arah (rad) yang ditambahkan
# ke yaw acuan; loop yaw (gyro, YAW_KP) di dalamnya menjalankan offset itu.
# Hanya aktif di jalur lurus dan hanya bila ada dinding yang terukur.
CENTER_ENABLED = True
CENTER_KP = 6.0        # rad per m galat
CENTER_KI = 2.0        # rad per (m*s)
CENTER_KD = 0.3        # rad per (m/s)
CENTER_MAX = 0.20      # rad batas offset arah (~11 derajat)
CENTER_I_MAX = 0.02    # m*s batas integral (anti-windup)
CENTER_FILTER = 0.4    # bobot sampel baru pada penyaring galat
CENTER_VALID_MIN = 70.0   # nilai sensor di bawah ini dianggap "tidak ada dinding"
HALF_LANE = 0.084      # m, setengah lebar lorong dalam (CELL/2 - tebal dinding/2)
SENSOR_LAT = 0.0095    # m, DL/DR di samping sumbu tengah (sama dengan generator)
SIN45 = math.sin(math.pi / 4)
# Kurva respons sensor perkiraan -- HARUS sama dengan maze_to_webots.py
# (SENSOR_ADC_MAX, SENSOR_D0, SENSOR_RANGE).
SENSOR_ADC_MAX = 4095.0
SENSOR_D0 = 0.04
SENSOR_FLOOR = SENSOR_ADC_MAX / (1.0 + (0.25 / SENSOR_D0) ** 2)


def _sensor_distance(value):
    """Jarak sinar (m) dari nilai sensor, kebalikan kurva respons."""
    return SENSOR_D0 * math.sqrt(max(SENSOR_ADC_MAX / (value + SENSOR_FLOOR) - 1.0, 0.0))


def _lateral_error():
    """Galat lateral (m, + = robot di kiri tengah lorong) dari DL dan DR,
    atau None kalau tak ada dinding yang terukur. Dua dinding: setengah
    selisih jarak ke kiri dan kanan; satu dinding: terhadap setengah lebar
    lorong nominal (sama nilainya kalau lorong nominal, jadi tidak melompat
    saat salah satu dinding hilang)."""
    vl = sensors["ds_dl"].getValue()
    vr = sensors["ds_dr"].getValue()
    y_left = SENSOR_LAT + SIN45 * _sensor_distance(vl) if vl > CENTER_VALID_MIN else None
    y_right = SENSOR_LAT + SIN45 * _sensor_distance(vr) if vr > CENTER_VALID_MIN else None
    if y_left is not None and y_right is not None:
        return (y_right - y_left) / 2.0
    if y_left is not None:
        return HALF_LANE - y_left
    if y_right is not None:
        return y_right - HALF_LANE
    return None


class _Centering:
    """PID galat lateral -> offset arah (rad). Dibuat baru di tiap segmen lurus."""

    def __init__(self):
        self.e = None        # galat tersaring
        self.integral = 0.0
        self.cmd = 0.0

    def update(self):
        if not CENTER_ENABLED:
            return 0.0
        e = _lateral_error()
        if e is None:
            # tak ada dinding: offset memudar halus ke nol
            self.cmd *= 0.9
            self.integral *= 0.95
            return self.cmd
        if self.e is None:
            self.e = e
            de = 0.0
        else:
            prev = self.e
            self.e += CENTER_FILTER * (e - self.e)
            de = (self.e - prev) / DT
        self.integral = max(-CENTER_I_MAX, min(CENTER_I_MAX, self.integral + self.e * DT))
        raw = -(CENTER_KP * self.e + CENTER_KI * self.integral + CENTER_KD * de)
        self.cmd = max(-CENTER_MAX, min(CENTER_MAX, raw))
        return self.cmd


def _smoothstep(t):
    return t * t * (3.0 - 2.0 * t)


def _smoothstep_rate(t):
    return 6.0 * t * (1.0 - t)


def _arc_length():
    """Panjang busur belok halus. Arah (yaw) dibuat mengikuti smoothstep
    terhadap jarak (laju putar nol di awal dan akhir, tanpa hentakan), dan
    panjangnya dipilih sehingga robot bergeser tepat CELL/2 ke depan dan ke
    samping: masuk di tengah sisi sel, keluar di tengah sisi sel yang
    bersebelahan (setara jari-jari 90 mm untuk sel 180 mm)."""
    n = 2000
    integral = sum(math.cos((math.pi / 2) * _smoothstep((i + 0.5) / n)) for i in range(n)) / n
    return (CELL / 2.0) / integral


ARC_LEN = _arc_length()


def _distance_since(l0, r0):
    """Jarak tempuh pusat robot (m) sejak encoder bernilai (l0, r0)."""
    return ((left_enc.getValue() - l0) + (right_enc.getValue() - r0)) / 2.0 * WHEEL_R


def _set_speed(v, omega):
    """Kecepatan pusat v (m/s) dan laju putar omega (rad/s, + = kiri)."""
    set_wheels((v - omega * TRACK_HALF) / WHEEL_R, (v + omega * TRACK_HALF) / WHEEL_R)


def _true_pose():
    x, y, _ = self_node.getPosition()
    r = self_node.getOrientation()
    return (x, y, math.degrees(math.atan2(r[3], r[0])))


def _run_straight(length, v_in, v_out, watch):
    """Lurus sejauh length (m): naik dari v_in, jelajah V_CRUISE, turun ke
    v_out. watch = daftar (kunci, d_awal, d_akhir): sampel sensor diambil saat
    jarak tempuh d di dalam jendela; return dict kunci -> daftar sampel."""
    l0, r0 = left_enc.getValue(), right_enc.getValue()
    got = {key: [] for key, _, _ in watch}
    centering = _Centering()
    while True:
        d = _distance_since(l0, r0)
        for key, lo, hi in watch:
            if lo <= d <= hi:
                got[key].append((sensors["ds_fl"].getValue(), sensors["ds_fr"].getValue(),
                                 sensors["ds_dl"].getValue(), sensors["ds_dr"].getValue(),
                                 _true_pose(), math.degrees(math.asin(self_node.getOrientation()[6]))))
        if d >= length - END_TOL:
            break
        v = min(V_CRUISE,
                math.sqrt(v_in * v_in + 2.0 * ACCEL_LIN * d),
                math.sqrt(v_out * v_out + 2.0 * ACCEL_LIN * max(length - d, 0.0)))
        offset_yaw = centering.update()
        _set_speed(max(v, V_MIN), YAW_KP * (target_yaw + offset_yaw - yaw))
        step()
    return got


def _run_arc(direction):
    """Belok halus 90 derajat dengan V_TURN: yaw acuan mengikuti smoothstep
    terhadap jarak busur, ditambah umpan maju laju putar."""
    global target_yaw
    sign = 1.0 if direction == "left" else -1.0
    yaw_start = target_yaw
    l0, r0 = left_enc.getValue(), right_enc.getValue()
    while True:
        s = _distance_since(l0, r0)
        if s >= ARC_LEN - END_TOL:
            break
        t = s / ARC_LEN
        yaw_ref = yaw_start + sign * (math.pi / 2) * _smoothstep(t)
        omega_ff = sign * (math.pi / 2) * _smoothstep_rate(t) / ARC_LEN * V_TURN
        _set_speed(V_TURN, omega_ff + YAW_KP * (yaw_ref - yaw))
        step()
    target_yaw = yaw_start + sign * math.pi / 2


def _build_segments(plan):
    """Ubah rencana ('F','L','R' per sel yang dilalui) menjadi segmen gerak:
    ('S', panjang, watch) untuk lurus dan ('A', arah) untuk belok halus.
    watch = daftar ((jenis, k), d_awal, d_akhir): jendela sensor untuk sel
    ke-(k+1); jenis 'front' (FL/FR di tengah sel) atau 'side' (DL/DR)."""
    n = len(plan)
    segments = []
    seg_watch = []
    cur_len = CELL / 2.0   # dari tengah sel awal ke sisi keluarnya
    prev_center = 0.0      # posisi tengah sel sebelumnya di segmen ini (None = tak ada)
    for k in range(n):     # masuk ke sel ke-(k+1) di jarak cur_len dalam segmen
        if prev_center is not None:
            lo = prev_center + SIDE_SAMPLE_AT - SIDE_WINDOW / 2.0
            seg_watch.append((("side", k), lo, lo + SIDE_WINDOW))
        center = cur_len + CELL / 2.0
        if k == n - 1:
            seg_watch.append((("front", k), center - 0.008, center + 0.008))
            segments.append(("S", center, seg_watch))
            break
        if plan[k + 1] == "F":
            seg_watch.append((("front", k), center - 0.008, center + 0.008))
            prev_center = center
            cur_len = center + CELL / 2.0
        else:
            segments.append(("S", cur_len, seg_watch))
            segments.append(("A", plan[k + 1]))
            seg_watch = []
            cur_len = 0.0
            prev_center = None
    return segments


def run_path(plan):
    """Jalankan rencana tanpa berhenti di tiap sel. plan = daftar aksi per sel
    yang dilalui, relatif arah hadap sekarang: 'F' = lurus ke sel berikutnya,
    'L'/'R' = belok halus ke kiri/kanan lalu ke sel berikutnya. Mulai di
    tengah sel, berhenti di tengah sel terakhir. Contoh: "FFRFL".

    Belok pertama (di sel awal) tidak bisa halus dari tengah sel, jadi
    diputar di tempat dulu.

    Return: daftar pengamatan per sel yang dimasuki (len(plan) buah), dict
    dengan kunci front/left/right (True/False, atau None kalau tak teramati:
    sel tempat belok halus tidak teramati), front_value/left_value/
    right_value (rata-rata sensor) dan pose (posisi/arah SEBENARNYA saat
    melewati tengah sel; hanya untuk validasi)."""
    global offset, last_side_walls
    plan = list(plan)
    center()
    if plan and plan[0] in ("L", "R"):
        _point_turn("left" if plan[0] == "L" else "right")
        plan[0] = "F"

    segments = _build_segments(plan)
    n = len(plan)
    samples = {}
    v_now = 0.0
    for i, seg in enumerate(segments):
        if seg[0] == "A":
            _run_arc("left" if seg[1] == "L" else "right")
            continue
        _, length, watch = seg
        v_out = V_TURN if i + 1 < len(segments) else 0.0
        samples.update(_run_straight(length, v_now, v_out, watch))
        v_now = v_out
    settle()

    obs = [{"front": None, "left": None, "right": None, "front_value": None,
            "left_value": None, "right_value": None, "pose": None} for _ in range(n)]
    for (kind, k), lst in samples.items():
        if not lst:
            continue
        if kind == "front":
            val = sum((s[0] + s[1]) / 2.0 for s in lst) / len(lst)
            obs[k]["front"] = val > FRONT_THRESHOLD
            obs[k]["front_value"] = val
            obs[k]["pose"] = lst[len(lst) // 2][4]
        else:
            ml = sum(s[2] for s in lst) / len(lst)
            mr = sum(s[3] for s in lst) / len(lst)
            obs[k]["left"], obs[k]["right"] = ml > SIDE_THRESHOLD, mr > SIDE_THRESHOLD
            obs[k]["left_value"], obs[k]["right_value"] = ml, mr
            obs[k]["pitch"] = sum(s[5] for s in lst) / len(lst)
    if obs:
        obs[-1]["pose"] = _true_pose()  # posisi akhir setelah berhenti
    offset = 0.0
    last_side_walls = (obs[-1]["left"], obs[-1]["right"]) if obs else None
    return obs


if __name__ == "__main__":
    log("=== Uji primitif gerak (bukan solving maze) ===")
    log("Rencana run_path('FRL'): maju, belok halus kanan, belok halus kiri, berhenti")
    log("(sudah dicek lewat parser maze -- jalur ini bebas dinding di minimaze.txt;")
    log(" robot mulai di tengah sel (0,0) menghadap Utara, berhenti di tengah sel (1,2))")
    log_true_pose()
    t0 = robot.getTime()
    run_path("FRL")
    log(f"waktu tempuh: {robot.getTime() - t0:.2f} s (simulasi)")
    log_true_pose()
    log("diharapkan: x=0.270 y=0.450 arah=90.0")
    log("=== Selesai. Bandingkan posisi/arah sebenarnya dengan yang diharapkan. ===")
    log("=== Robot sekarang idle (motor berhenti). ===")

    while robot.step(TIME_STEP) != -1:
        pass
