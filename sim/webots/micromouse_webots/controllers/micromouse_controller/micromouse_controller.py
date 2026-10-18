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
SIDE_THRESHOLD = 145.0

log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "run_log.txt")
log_file = open(log_path, "w", encoding="utf-8")


LOG_TO_CONSOLE = True   # False: log() hanya ke run_log.txt (controller boleh mengatur console sendiri)


def log(msg):
    if LOG_TO_CONSOLE:
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


MAX_WHEEL = 31.416       # rad/s, maxVelocity motor di generator (N20 300 RPM)
saturation_events = 0   # berapa kali perintah roda melebihi MAX_WHEEL


def set_wheels(left, right):
    """Kirim kecepatan roda. Kalau salah satu melebihi MAX_WHEEL, KEDUANYA diskala
    bersama (rasio, jadi kelengkungan jalur, tetap) -- dulu hanya dipotong oleh
    Webots (roda luar saja) sambil membanjiri console dengan peringatan."""
    global saturation_events
    biggest = max(abs(left), abs(right))
    if biggest > MAX_WHEEL:
        saturation_events += 1
        if saturation_events <= 20:
            import sys
            err = target_yaw - yaw
            log(f"SATURASI roda #{saturation_events}: diminta kiri={left:.1f} kanan={right:.1f} rad/s "
                f"(maks {MAX_WHEEL}), dari {sys._getframe(2).f_code.co_name}, "
                f"galat arah {math.degrees(err):+.1f} derajat, t={robot.getTime():.1f} s, "
                f"pose sebenarnya x={self_node.getPosition()[0]:.3f} y={self_node.getPosition()[1]:.3f}")
        scale = MAX_WHEEL / biggest
        left, right = left * scale, right * scale
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
    """Maju N sel lurus dari posisi sekarang sampai tengah sel ke-N, mulai dan
    berhenti dari diam (profil trapesium) dengan pemusatan DL/DR aktif.
    Sambil jalan, DL/DR disampel untuk dinding kiri/kanan tiap sel yang
    dimasuki. Return: daftar (kiri, kanan) satu per sel."""
    global offset, last_side_walls, last_side_values
    # jendela sampel dalam jarak tempuh d dari posisi sekarang (offset = jarak
    # robot di depan tengah sel sebelumnya)
    watch = [(("side", k),
              k * CELL + SIDE_SAMPLE_AT - SIDE_WINDOW / 2.0 - offset,
              k * CELL + SIDE_SAMPLE_AT + SIDE_WINDOW / 2.0 - offset) for k in range(n)]
    got = _run_straight(n * CELL - offset, 0.0, 0.0, watch)
    settle()
    samples = [[(s[2], s[3]) for s in got[("side", k)]] for k in range(n)]
    offset = 0.0
    log(f"maju {n} sel: yaw={math.degrees(yaw):.1f}")

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


REAR_TO_AXLE = 0.029   # m, tepi belakang PCB (datar) di belakang sumbu roda
BACK_SPEED = 0.05      # m/s saat mundur ke dinding belakang


def square_to_back_wall(extra=0.02):
    """Luruskan robot dengan mundur sampai tepi belakang PCB (datar) menempel
    dinding belakang sel start. Jarak ke dinding dari tengah sel = HALF_LANE;
    mundur sedikit lebih jauh (extra) supaya pasti menempel; roda boleh
    tergelincir sebentar. Penjaga arah dimatikan selama mundur agar dinding
    yang meluruskan badan. Sesudahnya arah acuan (gyro) di-reset ke 'lurus
    pada dinding' dan posisi diketahui (offset = -(HALF_LANE - REAR_TO_AXLE)),
    jadi center() tinggal maju ke tengah sel. Hanya untuk sel start yang
    punya dinding belakang."""
    global offset, yaw
    gap = HALF_LANE - REAR_TO_AXLE
    l0, r0 = left_enc.getValue(), right_enc.getValue()
    while -_distance_since(l0, r0) < gap + extra:
        set_wheels(-BACK_SPEED / WHEEL_R, -BACK_SPEED / WHEEL_R)
        step()
    settle()
    yaw = target_yaw
    offset = -gap
    log(f"luruskan ke dinding belakang: mundur {gap + extra:.3f} m, arah di-reset")


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
CENTER_FRONT_BLOCK = 400.0   # nilai FL/FR di atas ini: dinding depan < ~100 mm, DL/DR melihat dinding depan
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
        # Dinding depan dekat (nilai FL/FR tinggi): sinar diagonal DL/DR mengenai
        # dinding DEPAN, bukan dinding samping (terjadi bila robot < ~50 mm dari
        # tengah sel dan ada dinding depan), sehingga galat lateral palsu dan
        # robot dibelokkan keras. Pemusatan dimatikan, offset memudar.
        if (sensors["ds_fl"].getValue() + sensors["ds_fr"].getValue()) / 2.0 > CENTER_FRONT_BLOCK:
            self.cmd *= 0.85
            return self.cmd
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


_ARC_SHAPES = {}


class StallError(Exception):
    """Gerak tertahan (mis. badan menekan dinding) sampai melewati batas waktu."""


ARC_FAIL_DEG = 10.0   # galat arah sesudah busur di atas ini dianggap busur gagal


def _arc_shape(forward, lateral=CELL / 2.0):
    """Bentuk busur yang menempuh `forward` m ke depan dan `lateral` m ke samping.
    Yaw acuan = (pi/2) * smoothstep(t)**p terhadap jarak busur; p=1 -> simetris
    (maju = samping, busur standar); p<1 memutar lebih awal (lebih banyak
    ke samping dibanding ke depan), dipakai kalau busur harus dimulai lebih
    dekat ke tengah sel. Return (p, panjang) atau None kalau tak mungkin."""
    key = round(forward, 4)
    if key in _ARC_SHAPES:
        return _ARC_SHAPES[key]
    n = 600

    def integrals(p):
        cs = sn = 0.0
        for i in range(n):
            th = (math.pi / 2) * _smoothstep((i + 0.5) / n) ** p
            cs += math.cos(th)
            sn += math.sin(th)
        return cs / n, sn / n

    target = lateral / forward
    lo, hi = 0.5, 2.5
    r_lo = integrals(lo)[1] / integrals(lo)[0]
    r_hi = integrals(hi)[1] / integrals(hi)[0]
    if not (r_hi <= target <= r_lo):
        _ARC_SHAPES[key] = None
        return None
    for _ in range(40):
        mid = (lo + hi) / 2.0
        cs, sn = integrals(mid)
        if sn / cs > target:
            lo = mid
        else:
            hi = mid
    p = (lo + hi) / 2.0
    result = (p, lateral / integrals(p)[1])
    _ARC_SHAPES[key] = result
    return result


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


def _run_straight(length, v_in, v_out, watch, vmax=None, front_stop=None):
    """Lurus sejauh length (m): naik dari v_in, jelajah V_CRUISE, turun ke
    v_out. watch = daftar (kunci, d_awal, d_akhir): sampel sensor diambil saat
    jarak tempuh d di dalam jendela; return dict kunci -> daftar sampel."""
    l0, r0 = left_enc.getValue(), right_enc.getValue()
    got = {key: [] for key, _, _ in watch}
    centering = _Centering()
    vmax = V_CRUISE if vmax is None else vmax
    got["_d"] = 0.0
    t_run0 = robot.getTime()
    max_t = 3.0 * length / max(0.5 * vmax, V_MIN) + 3.0
    while True:
        d = _distance_since(l0, r0)
        if robot.getTime() - t_run0 > max_t:
            set_wheels(0.0, 0.0)
            raise StallError("lurus tertahan (waktu habis)")
        got["_d"] = d
        if front_stop is not None and d > 0.02 and \
                (sensors["ds_fl"].getValue() + sensors["ds_fr"].getValue()) / 2.0 > front_stop:
            # dinding depan jauh lebih dekat daripada yang diperkirakan: rem
            got["_blocked"] = True
            set_wheels(0.0, 0.0)
            break
        for key, lo, hi in watch:
            if lo <= d <= hi:
                got[key].append((sensors["ds_fl"].getValue(), sensors["ds_fr"].getValue(),
                                 sensors["ds_dl"].getValue(), sensors["ds_dr"].getValue(),
                                 _true_pose(), math.degrees(math.asin(self_node.getOrientation()[6]))))
        if d >= length - END_TOL:
            break
        v = min(vmax,
                math.sqrt(v_in * v_in + 2.0 * ACCEL_LIN * d),
                math.sqrt(v_out * v_out + 2.0 * ACCEL_LIN * max(length - d, 0.0)))
        offset_yaw = centering.update()
        _set_speed(max(v, V_MIN), YAW_KP * (target_yaw + offset_yaw - yaw))
        step()
    return got


def _run_arc(direction, v=None, forward=None):
    """Belok halus 90 derajat dengan V_TURN: yaw acuan mengikuti smoothstep
    terhadap jarak busur, ditambah umpan maju laju putar."""
    global target_yaw
    v = V_TURN if v is None else v
    p, arc_len = 1.0, ARC_LEN
    if forward is not None and abs(forward - CELL / 2.0) > 0.002:
        shape = _arc_shape(forward)
        if shape is None:
            raise ValueError(f"busur tak mungkin untuk maju={forward:.3f} m")
        p, arc_len = shape
    sign = 1.0 if direction == "left" else -1.0
    yaw_start = target_yaw
    l0, r0 = left_enc.getValue(), right_enc.getValue()
    t_arc0 = robot.getTime()
    while True:
        s = _distance_since(l0, r0)
        if s >= arc_len - END_TOL:
            break
        if robot.getTime() - t_arc0 > 3.0 * arc_len / v + 1.0:
            set_wheels(0.0, 0.0)
            raise StallError("busur tertahan (waktu habis)")
        t = s / arc_len
        sm = max(_smoothstep(t), 1e-9)
        yaw_ref = yaw_start + sign * (math.pi / 2) * sm ** p
        omega_ff = sign * (math.pi / 2) * p * sm ** (p - 1.0) * _smoothstep_rate(t) / arc_len * v
        _set_speed(v, omega_ff + YAW_KP * (yaw_ref - yaw))
        step()
    target_yaw = yaw_start + sign * math.pi / 2
    if abs(target_yaw - yaw) > math.radians(ARC_FAIL_DEG):
        # busur tidak selesai (badan tertahan, mis. menyentuh dinding): luruskan
        # arah ke acuan supaya pelacakan arah tetap benar
        log(f"BUSUR GAGAL: arah {math.degrees(yaw - target_yaw):+.0f} derajat dari acuan, diputar di tempat")
        _rotate_by(0.0)


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
    for key, lst in samples.items():
        if not isinstance(key, tuple) or not lst:   # "_d"/"_blocked": bukan sampel
            continue
        kind, k = key
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


# ---------------------------------------------------------------------------
# Jelajah berkelanjutan (search run): keputusan arah diambil SAMBIL BERGERAK.
# ---------------------------------------------------------------------------
# Titik keputusan untuk sel C ada 95 mm sebelum tengah C (kira-kira batas
# masuk C, di situ busur belok halus harus dimulai). Pada titik itu:
#   - dinding kiri/kanan C dari DL/DR (jendela 140-95 mm sebelum tengah C; sinar
#     diagonal mengenai dinding samping C pada posisi itu),
#   - dinding depan C dari FL/FR (jendela 130-95 mm sebelum tengah C; jarak ke
#     dinding depan 125-165 mm, nilai ~120-270 vs 0 kalau terbuka).
# Sel pertama sesudah busur belok: robot baru keluar busur tepat di batas
# masuk, jendela di atas sudah lewat. Jendela diganti 90-70 mm sebelum tengah
# dan keputusan diambil 70 mm sebelum tengah ("terlambat"; lebih dekat dari 50 mm
# sinar DL/DR sudah melewati batas sel dan melihat sel berikutnya): lurus tetap bisa,
# tapi belok harus berhenti dulu di tengah sel lalu diputar di tempat.
V_SEARCH = 0.22          # m/s kecepatan jelajah (lurus dan busur)
DECIDE_BEFORE = 0.095    # m sebelum tengah sel: titik keputusan
FRONT_EARLY_THRESHOLD = 60.0   # nilai FL/FR rata-rata: dinding depan ada (jarak 125-165 mm)
LATE_DECIDE_BEFORE = 0.07
# Berhenti darurat: di jalur lurus sesudah aksi "F" (dinding depan diperkirakan
# terbuka), kalau rata-rata FL/FR melewati nilai pemicu robot langsung direm
# dan peta diperbaiki lewat on_blocked(). Jarak berhenti aman ditetapkan dari
# TEPI DEPAN PCB ke dinding (diukur pengguna di model 3D: 37 mm). Sensor ada
# SENSOR_TO_NOSE di belakang ujung PCB, dan rem masih menempuh BRAKE_MARGIN
# (terukur 1-2 mm + noise sensor), jadi pemicu dipasang lebih jauh dari itu.
SAFE_STOP_NOSE = 0.037     # m, jarak berhenti aman: tepi depan PCB ke dinding
PCB_FRONT_AHEAD = 0.066     # m, ujung depan PCB di depan sumbu roda
SENSOR_TO_NOSE = 0.0185    # m, ujung PCB (66 mm) - sensor FL/FR (47.5 mm) dari sumbu roda
BRAKE_MARGIN = 0.004       # m, jarak rem + toleransi pembacaan
SAFE_STOP_SENSOR = SAFE_STOP_NOSE + SENSOR_TO_NOSE            # jarak sensor-dinding minimum
FRONT_STOP_VALUE = (SENSOR_ADC_MAX / (1.0 + ((SAFE_STOP_SENSOR + BRAKE_MARGIN) / SENSOR_D0) ** 2)
                    - SENSOR_FLOOR)                           # nilai FL/FR pemicu (~1160)
# Di sel "terlambat" robot lebih dekat ke batas sel: sinar DL/DR yang lolos
# lewat sisi terbuka mengenai dinding DEPAN sel tetangga pada jarak 0.15-0.17 m
# (nilai ~180-210), mirip dinding samping (nilai >300 di sini). Ambangnya
# dinaikkan supaya tidak salah baca.
SIDE_THRESHOLD_LATE = 260.0


def _sensor_value(distance):
    """Nilai sensor untuk jarak sinar tertentu (kurva respons)."""
    return SENSOR_ADC_MAX / (1.0 + (distance / SENSOR_D0) ** 2) - SENSOR_FLOOR


SIDE_LATE_MIN = 140.0   # nilai rata-rata minimum agar dianggap dinding di sel terlambat
RISING_RATIO = 1.12     # nilai separuh akhir jendela > rasio * separuh awal -> sinar makin dekat


E_EST_MIN = 300.0         # nilai sisi yang cukup jelas dinding untuk menaksir galat lateral
NUDGE_MIN = 0.010         # m: geser lateral dulu bila |galat| lebih dari ini sebelum putar di tempat
NUDGE_MAX = 0.040
NUDGE_ANGLE = math.radians(15)
VERY_CLOSE_WALL = 700.0   # nilai sisi lawan di atas ini: robot sangat dekat ke dinding itu (>~22 mm dari tengah)
AMBIGUOUS_MIN = 180.0     # di bawah ini (naik) lebih mungkin sinar lolos (terbaca 125-165) daripada dinding jauh
AMBIGUOUS_MAX = 330.0     # di bawah ini (dan di atas SIDE_LATE_MIN) nilai tak bisa dibedakan dari salah baca


def _side_wall_late(values, other_mean=0.0, conservative=False):
    """Dinding samping di sel tepat sesudah busur dari urutan nilai sensor di jendela.
    Dinding samping sungguhan sejajar arah gerak: jaraknya (nilainya) tetap selama
    robot maju. Sinar yang lolos lewat sisi terbuka mengenai dinding DEPAN sel
    tetangga: jaraknya mengecil ~1.4 mm per mm robot maju, jadi nilainya NAIK
    cepat di sepanjang jendela dan jatuh di 125-165 (maks teramati ~290).
    Dinding asli yang sedang dikoreksi ke tengah juga bisa terlihat naik (240-340),
    jadi nilai naik yang > AMBIGUOUS_MIN tak bisa dibedakan.
    Kesalahan TIDAK simetris: salah baca 'terbuka' bisa membuat robot berbelok
    ke dinding (busur macet, pelacakan posisi rusak), salah baca 'dinding' hanya
    menutup satu pilihan. Jadi yang ambigu dianggap DINDING."""
    n = len(values)
    mean = sum(values) / n
    if mean <= SIDE_LATE_MIN:
        return False
    rising = False
    if n >= 4:
        h = n // 2
        first = sum(values[:h]) / h
        last = sum(values[-h:]) / h
        rising = last > RISING_RATIO * first
    if not rising:
        return True
    if conservative:
        # zona ambigu (140-330, naik): tak bisa dibedakan -> None = "belum tahu";
        # search_run lalu memverifikasi lewat putar di tempat + FL/FR sebelum belok ke sana
        return True if mean >= AMBIGUOUS_MAX else None
    # Seimbang: nilai naik yang ambigu dianggap dinding hanya kalau sisi lawan
    # sangat dekat (robot sangat miring, jadi dinding sisi ini memang terbaca kecil
    # dan naik karena dikoreksi); selain itu dianggap sinar yang lolos (terbuka).
    return AMBIGUOUS_MIN < mean < AMBIGUOUS_MAX and other_mean > VERY_CLOSE_WALL


def _early_observation(got, key_side, key_front, side_threshold=SIDE_THRESHOLD, late=False,
                       conservative=False):
    out = {"front": None, "left": None, "right": None,
           "front_value": None, "left_value": None, "right_value": None}
    front = got.get(key_front) or []
    if front:
        val = sum((x[0] + x[1]) / 2.0 for x in front) / len(front)
        out["front"], out["front_value"] = val > FRONT_EARLY_THRESHOLD, val
    side = got.get(key_side) or []
    if side:
        ml = sum(x[2] for x in side) / len(side)
        mr = sum(x[3] for x in side) / len(side)
        if late:
            out["left"] = _side_wall_late([x[2] for x in side], mr, conservative)
            out["right"] = _side_wall_late([x[3] for x in side], ml, conservative)
        else:
            out["left"], out["right"] = ml > side_threshold, mr > side_threshold
        out["left_value"], out["right_value"] = ml, mr
        es = []
        if ml > E_EST_MIN:
            es.append(HALF_LANE - (SENSOR_LAT + SIN45 * _sensor_distance(ml)))
        if mr > E_EST_MIN:
            es.append((SENSOR_LAT + SIN45 * _sensor_distance(mr)) - HALF_LANE)
        out["e_est"] = sum(es) / len(es) if es else None   # m, + = robot di kiri tengah lorong
        out["left_seq"] = [round(x[2]) for x in side]
        out["right_seq"] = [round(x[3]) for x in side]
    return out


last_blocked = None   # info henti darurat terakhir (untuk validasi)


def _rotate_by(delta):
    """Putar di tempat sebesar delta rad (+ = kiri) relatif arah acuan."""
    global target_yaw
    target_yaw += delta
    while True:
        err = target_yaw - yaw
        if abs(err) <= TURN_TOL:
            break
        w = math.copysign(min(TURN_SPEED, max(TURN_MIN, TURN_GAIN * abs(err))), err)
        set_wheels(-w, w)
        step()
    settle()


def _lateral_nudge(shift):
    """Geser robot ke samping sejauh shift m (+ = kiri) tanpa mengubah arah akhir
    dan posisi sepanjang sumbu: putar NUDGE_ANGLE menjauhi arah geser, mundur
    sejauh shift/sin(sudut) (belakang lorong terbuka), putar kembali, maju
    sejauh komponen sumbunya. Dipakai sebelum putar di tempat di lorong sempit:
    radius putar PCB ~66 mm di lorong berjari 84 mm hanya menyisakan 18 mm."""
    psi = -NUDGE_ANGLE if shift > 0 else NUDGE_ANGLE
    d = abs(shift) / math.sin(NUDGE_ANGLE)
    _rotate_by(psi)
    _drive(-d)
    settle()
    _rotate_by(-psi)
    _drive(d * math.cos(NUDGE_ANGLE))
    settle()
    log(f"geser lateral {shift * 1000:+.0f} mm sebelum putar di tempat")


def search_run(on_cell, on_blocked=None, late_arc_with_front=True, conservative_sides=False,
               on_verify=None):
    """Jelajah tanpa berhenti di tiap sel. Mulai di tengah sel dan diam,
    menghadap arah gerak. Di titik keputusan tiap sel C yang dimasuki,
    on_cell(obs) dipanggil dengan dinding C yang teramati
    (obs: front/left/right = True/False/None, *_value) dan harus
    mengembalikan aksi relatif arah hadap:
        "F" lurus terus, "L"/"R" belok halus ke sel berikutnya,
        "U" berhenti di tengah C lalu putar balik di tempat,
        "STOP" berhenti di tengah C dan selesai.
    Belok di sel yang terlambat diputuskan (lihat di atas) menjadi
    berhenti di tengah C + putar di tempat. Return: jumlah sel dimasuki."""
    global offset, last_blocked
    center()
    to_center = CELL      # jarak dari awal segmen sekarang ke tengah sel berikutnya
    v_in = 0.0
    cells = 0
    prev_before = None
    while True:
        late = to_center - DECIDE_BEFORE < 0.02
        before = LATE_DECIDE_BEFORE if late else DECIDE_BEFORE
        s_dec = to_center - before
        side_lo = max(to_center - (0.09 if late else 0.14), 0.0)
        front_lo = max(to_center - (0.09 if late else 0.13), 0.0)
        watch = [(("side", 0), side_lo, s_dec), (("front", 0), front_lo, s_dec)]
        got = _run_straight(s_dec, v_in, V_SEARCH, watch, vmax=V_SEARCH,
                            front_stop=FRONT_STOP_VALUE if (v_in > 0 and on_blocked) else None)
        if got.get("_blocked"):
            # Dinding depan mendadak dekat padahal diperkirakan terbuka: sudah
            # direm. Maju/mundur ke tengah sel yang sedang dilewati, minta
            # peta diperbaiki dan arah baru, lalu putar di tempat.
            settle()
            last_blocked = {
                "pose_rem": _true_pose(),     # posisi sebenarnya setelah berhenti
                "nilai_fl_fr": (sensors["ds_fl"].getValue() + sensors["ds_fr"].getValue()) / 2.0,
                "jarak_tempuh": got["_d"],
            }
            # JANGAN mendekat ke dinding setelah rem: kalau terlalu dekat (di bawah
            # jarak aman), mundur saja; kalau tidak, diam di tempat dan putar. Jarak
            # sensor-dinding diperkirakan dari nilai FL/FR (kebalikan kurva respons).
            gap_now = _sensor_distance((sensors["ds_fl"].getValue() + sensors["ds_fr"].getValue()) / 2.0)
            last_blocked["sensor_ke_dinding"] = gap_now
            if gap_now < SAFE_STOP_SENSOR:
                _drive(-(SAFE_STOP_SENSOR - gap_now))
                settle()
            action = on_blocked()
            log(f"BERHENTI DARURAT: dinding depan mendadak dekat, aksi baru {action}")
            if action in ("L", "R"):
                _point_turn("left" if action == "L" else "right")
            elif action == "U":
                _point_turn("left")
                _point_turn("left")
            else:
                raise ValueError(f"aksi tidak dikenal setelah henti darurat: {action}")
            to_center = CELL
            v_in = 0.0
            prev_before = None
            continue
        obs = _early_observation(got, ("side", 0), ("front", 0), SIDE_THRESHOLD, late,
                                 conservative_sides)
        obs["late"] = late
        cells += 1
        action = on_cell(obs)
        prev_before = before

        if action == "F":
            to_center = (to_center - s_dec) + CELL
            v_in = V_SEARCH
            continue
        # Busur di sel "terlambat" yang berputar karena ada dinding depan lewat sangat
        # dekat dinding itu (terukur ~10 mm, normal 22-29 mm) dan bisa macet.
        # late_arc_with_front=False: di situ berhenti dan putar di tempat saja.
        side_unknown = ((action == "L" and obs.get("left") is None)
                        or (action == "R" and obs.get("right") is None))   # sisi ambigu: jangan busur
        arc_ok = not (late and obs.get("front") and not late_arc_with_front) and not side_unknown
        if action in ("L", "R") and arc_ok and _arc_shape(before) is not None:
            # busur dimulai `before` m sebelum tengah sel dan berakhir tepat di
            # tengah sisi sel (bentuk disesuaikan, titik akhir tetap di tengah
            # lorong sel berikutnya, juga di sel "terlambat")
            _run_arc("left" if action == "L" else "right", v=V_SEARCH, forward=before)
            to_center = CELL / 2.0   # keluar busur di batas masuk sel berikutnya
            v_in = V_SEARCH
            continue
        # berhenti di tengah sel C. Kalau ada dinding depan, berhenti LEBIH AWAL
        # supaya tepi depan PCB tetap >= SAFE_STOP_NOSE dari dinding (di tengah sel
        # tepi PCB hanya HALF_LANE - 0.066 = 18 mm dari dinding depan). Sesudah
        # diputar di tempat, selisihnya jadi galat lateral yang dikoreksi pemusatan.
        stop_back = max(0.0, SAFE_STOP_NOSE + PCB_FRONT_AHEAD - HALF_LANE) if obs.get("front") else 0.0
        _run_straight(max(to_center - s_dec - stop_back, 0.0), V_SEARCH, 0.0, [], vmax=V_SEARCH)
        settle()
        if action == "STOP":
            return cells
        e_est = obs.get("e_est")
        if e_est is not None and NUDGE_MIN < abs(e_est) <= NUDGE_MAX:
            _lateral_nudge(-e_est)
        if action in ("L", "R"):
            _point_turn("left" if action == "L" else "right")
            if stop_back > 0.005:
                # berhenti stop_back lebih awal sepanjang arah LAMA: setelah diputar
                # selisih itu jadi galat lateral (kiri bila belok kiri); geser kembali
                # supaya jalur berikutnya tidak lewat dekat dinding.
                _lateral_nudge(-stop_back if action == "L" else stop_back)
            if side_unknown and on_blocked:
                # Sisi tujuan ambigu: lihat dinding depan (arah baru) dari tengah sel
                # dengan FL/FR yang andal. Kalau dinding, laporkan dan pilih aksi lain.
                front_wall, _ = read_front_wall()
                if on_verify:
                    on_verify(front_wall)
                if front_wall:
                    log("VERIFIKASI: sisi ambigu ternyata dinding, aksi dipilih ulang")
                    action2 = on_blocked()
                    if action2 in ("L", "R"):
                        _point_turn("left" if action2 == "L" else "right")
                    elif action2 == "U":
                        _point_turn("left")
                        _point_turn("left")
            to_center = CELL
        elif action == "U":
            _point_turn("left")
            _point_turn("left")
            to_center = CELL - stop_back   # sesudah putar balik robot stop_back di DEPAN tengah sel
        else:
            raise ValueError(f"aksi tidak dikenal: {action}")
        v_in = 0.0


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
