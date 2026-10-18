#!/usr/bin/env python3
"""
Konversi file maze ASCII (format sama dengan sim/mazes/mazefiles/, dipakai
juga oleh flood-fill-c/astar-c) menjadi dunia Webots (.wbt), lengkap dengan
robot differential-drive dan 4 sensor jarak (FL, FR, DL, DR) yang posisinya
diambil dari desain PCB (hardware/PickAndPlace_MICE_PCB_2026_10_18.xlsx).

Cara pakai:
    python maze_to_webots.py <maze.txt> <output.wbt> [--category classic|halfsize]
                             [--controller NAMA]   (bawaan: micromouse_controller)

Dimensi mengikuti docs/micromouse-rules.md:
    classic : sel 18x18 cm, dinding tinggi 5 cm, tebal 1.2 cm
    halfsize: sel 9x9 cm,  dinding tinggi 2.5 cm, tebal 0.6 cm
(Robot di sini ~8 cm lebar, jadi hanya masuk akal untuk kategori classic.)

Konvensi koordinat (disamakan dengan proyek flood-fill-c/q-learning):
    - (0,0) di pojok kiri-bawah, x ke arah Timur, y ke arah Utara.
    - Dunia Webots: sumbu Z = atas (bawaan Webots sejak versi ini, ENU).
    - Posisi dunia sel (cx, cy) = pusatnya di ((cx+0.5)*CELL, (cy+0.5)*CELL).
    - Robot mulai dengan SUMBU RODA di tengah sel (0,0), menghadap Utara.
    - Frame lokal robot: x = depan, y = kiri, z = atas, origin = tengah sumbu
      roda pada ketinggian poros roda.

CATATAN: belum pernah dibuka di Webots asli (tidak ada GUI Webots di sesi
pembuatannya). Yang berasal dari data nyata: posisi sensor (PCB), jari-jari
dan lebar roda (N20 wheel 34x7), jarak antar roda, kecepatan motor (N20
300 RPM). Outline body diukur dari hardware/image.png (+-0.5 mm). Yang masih ASUMSI: tinggi body, massa, gesekan, tinggi
sensor dari lantai, dan kurva respons sensor (SENSOR_* di bawah).
"""

import math
import sys
from pathlib import Path

DIMENSIONS = {
    "classic": {"cell": 0.18, "wall_height": 0.05, "wall_thick": 0.012},
    "halfsize": {"cell": 0.09, "wall_height": 0.025, "wall_thick": 0.006},
}

# ---------------------------------------------------------------------------
# Geometri robot (satuan meter kecuali disebut lain)
# ---------------------------------------------------------------------------

# Sumbu roda di PCB (mm): motor di x=12 dan x=68, y=29 -> tengah (40, 29).
# PCB: x ke kanan, y ke depan (sensor ada di y besar).
AXLE_PCB_MM = (40.0, 29.0)

WHEEL_R = 0.017        # N20 wheel 34x7 -> diameter 34 mm
WHEEL_W = 0.007        #                    lebar 7 mm
TRACK_HALF = 0.0365    # poros roda di PCB x=3.5 dan x=76.5 -> jarak 73 mm
MAX_MOTOR_VEL = 31.416  # N20 300 RPM = 31.4 rad/s (tanpa beban)

# Outline PCB (diukur dari hardware/image.png, +-0.5 mm): persegi panjang
# 80 x 94.9 mm, sudut atas bulat radius 35 mm (bagian datar atas selebar 10
# mm), dua takik tempat roda di y_pcb 14.6-43.2 (sisi dalam di x=7.9 dan
# 72.0). Body disusun dari kotak-kotak (strip) mengikuti outline itu, tapi
# strip lebar penuh dipotong di luar rentang roda (y_pcb 12-46) supaya tidak
# menimpa roda (roda 34 mm lebih tinggi dari takik 28.6 mm).
BODY_NOTCH_HALF_MM = 32.05   # setengah lebar bagian takik (sisi dalam roda 33.0)
BODY_DOME_Y0_MM = 59.9       # mulai lengkung sudut atas
BODY_TOP_Y_MM = 94.9
BODY_CORNER_R_MM = 35.0
BODY_FLAT_HALF_MM = 5.0      # setengah lebar bagian datar di ujung depan
BODY_DOME_STRIPS = 7

# Tinggi body dari lantai -- ASUMSI: sisi bawah PCB ~10 mm (poros roda 17 mm
# dikurangi setengah tinggi motor N20), atas komponen ~30 mm. Sensor di 5 mm
# dari lantai sehingga sinar lewat di BAWAH PCB (tidak berawal di dalam
# boundingObject).
BODY_BOTTOM_FLOOR = 0.010
BODY_TOP_FLOOR = 0.030
ROBOT_MASS = 0.08      # ASUMSI
# Pusat massa tepat di atas sumbu roda (sedikit di bawahnya agar stabil).
# Kalau dibiarkan default (dihitung dari bounding object) pusat massa bergeser
# ke depan, skid menanggung beban dan gesekannya membuat roda slip saat
# belok di tempat (hanya berputar 59 dari 90 derajat pada uji).
ROBOT_COM_Z = -0.004
MPU6050_PCB_MM = (40.0, 70.75)

SENSOR_HEIGHT = 0.005  # dari lantai -- perkiraan user, belum ditentukan

# Pasangan emitter SFH4545 + penerima TEFT4300 dipasang TUMPANG TINDIH di
# dalam casing 3D yang diletakkan di kotak kuning (silkscreen arsir) di PCB;
# footprint pad-nya sendiri (y_pcb 68-71 dan 83-85) hanya titik sambungan
# kaki. Jadi posisi sensor efektif = pusat kotak arsir (diukur dari
# hardware/image.png, +-0.5 mm; kotak depan 7x7 mm, kotak diagonal berupa
# belah ketupat dengan diagonal 10 mm), dan emitter = penerima (satu titik).
# Arah hadap: FL/FR lurus ke depan, DL/DR diagonal 45 derajat ke luar.
SENSORS_PCB = {
    "ds_fl": {"center": (10.0, 76.5), "facing_deg": 0.0},
    "ds_fr": {"center": (70.0, 76.5), "facing_deg": 0.0},
    "ds_dl": {"center": (30.5, 89.1), "facing_deg": 45.0},
    "ds_dr": {"center": (49.5, 89.1), "facing_deg": -45.0},
}

# Kurva respons sensor ASUMSI (belum dikalibrasi): nilai ~ ADC 12-bit,
# turun mengikuti 1/(1+(d/D0)^2). Ganti dengan hasil ukur sensor asli.
SENSOR_ADC_MAX = 4095.0
SENSOR_D0 = 0.04
SENSOR_RANGE = 0.25
SENSOR_NOISE = 0.03          # fraksi nilai respons
SENSOR_RAYS = 1  # >1 sinar menyebar ke atas/bawah juga: pada tinggi 5 mm sinar
                 # miring bisa kena lantai dan memberi pembacaan palsu


def body_blocks():
    """Strip kotak pembentuk body: (y_pcb_awal, y_pcb_akhir, setengah_lebar), mm."""
    blocks = [
        (0.0, 12.0, 40.0),                      # belakang: baterai, regulator
        (12.0, 46.0, BODY_NOTCH_HALF_MM),       # antara roda (takik)
        (46.0, BODY_DOME_Y0_MM, 40.0),          # bahu
    ]
    step = (BODY_TOP_Y_MM - BODY_DOME_Y0_MM) / BODY_DOME_STRIPS
    for i in range(BODY_DOME_STRIPS):
        lo = BODY_DOME_Y0_MM + i * step
        hi = lo + step
        dy = (lo + hi) / 2.0 - BODY_DOME_Y0_MM
        hw = BODY_FLAT_HALF_MM + math.sqrt(BODY_CORNER_R_MM ** 2 - dy ** 2)
        blocks.append((lo, hi, hw))
    return blocks


def pcb_to_local(px_mm, py_mm):
    """Titik PCB (mm) -> frame lokal robot (x depan, y kiri), meter."""
    ax, ay = AXLE_PCB_MM
    return (py_mm - ay) / 1000.0, (ax - px_mm) / 1000.0


def sensor_pose(spec):
    """Posisi lokal (m) pusat casing sensor dan sudut hadap (rad, + = ke kiri)."""
    x, y = pcb_to_local(*spec["center"])
    return x, y, math.radians(spec["facing_deg"])


def lookup_table_rows():
    # Webots mengembalikan nilai BARIS TERAKHIR kalau tidak ada benda dalam
    # jangkauan, jadi kurva dikurangi nilainya di SENSOR_RANGE supaya "tidak
    # ada apa-apa" = 0 (bukan ~100).
    floor = SENSOR_ADC_MAX / (1.0 + (SENSOR_RANGE / SENSOR_D0) ** 2)
    rows = []
    for d in (0.0, 0.01, 0.02, 0.03, 0.04, 0.06, 0.08, 0.10, 0.12, 0.15,
              0.20, SENSOR_RANGE):
        value = SENSOR_ADC_MAX / (1.0 + (d / SENSOR_D0) ** 2) - floor
        rows.append(f"        {d:.3f} {value:.1f} {SENSOR_NOISE}")
    return "\n".join(rows)


def parse_maze(path):
    with open(path, "r", encoding="utf-8") as f:
        lines = [l.rstrip("\r\n") for l in f if l.strip("\r\n") != ""]

    width = (len(lines[0]) - 1) // 4
    rows = (len(lines) - 1) // 2  # jumlah baris sel (= height)

    def pad(s):
        return s.ljust(4 * width + 1)

    post_rows = [pad(lines[2 * r]) for r in range(rows + 1)]
    corridor_rows = [pad(lines[2 * r + 1]) for r in range(rows)]

    # horiz[r][x]: True = ada dinding horizontal di baris-teks r, kolom x
    horiz = [[post_rows[r][4 * x + 1:4 * x + 4] == "---" for x in range(width)]
             for r in range(rows + 1)]
    # vert[r][x]: True = ada dinding vertikal di baris-teks r, post ke-x
    vert = [[corridor_rows[r][4 * x] == "|" for x in range(width + 1)]
            for r in range(rows)]

    return width, rows, horiz, vert


def wall_box(name, cx, cy, cz, sx, sy, sz):
    return f"""Solid {{
  translation {cx:.4f} {cy:.4f} {cz:.4f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.85 0.1 0.1
        roughness 1
        metalness 0
      }}
      geometry Box {{
        size {sx:.4f} {sy:.4f} {sz:.4f}
      }}
    }}
  ]
  name "{name}"
  boundingObject Box {{
    size {sx:.4f} {sy:.4f} {sz:.4f}
  }}
}}"""


def generate_walls(width, rows, horiz, vert, cell, wall_h, wall_t):
    walls = []
    idx = 0
    # Dinding horizontal: baris-teks r=0..rows (post_rows), r -> y_world = (rows-r)*cell
    for r in range(rows + 1):
        y_world = (rows - r) * cell
        for x in range(width):
            if horiz[r][x]:
                cx = (x + 0.5) * cell
                walls.append(wall_box(f"wall_h_{idx}", cx, y_world, wall_h / 2,
                                       cell, wall_t, wall_h))
                idx += 1
    # Dinding vertikal: baris-teks r=0..rows-1 (corridor_rows), r -> y = rows-1-r (sel mms)
    for r in range(rows):
        y_cell = rows - 1 - r
        cy = (y_cell + 0.5) * cell
        for x in range(width + 1):
            if vert[r][x]:
                x_world = x * cell
                walls.append(wall_box(f"wall_v_{idx}", x_world, cy, wall_h / 2,
                                       wall_t, cell, wall_h))
                idx += 1
    return walls


def wheel_node(side, y):
    return f"""    DEF {side.upper()}_WHEEL HingeJoint {{
      jointParameters HingeJointParameters {{
        axis 0 1 0
        anchor 0 {y:.4f} 0
      }}
      device [
        RotationalMotor {{
          name "{side}_wheel_motor"
          maxVelocity {MAX_MOTOR_VEL}
        }}
        PositionSensor {{
          name "{side}_wheel_sensor"
        }}
      ]
      endPoint Solid {{
        translation 0 {y:.4f} 0
        children [
          Transform {{
            rotation 1 0 0 1.5707963
            children [
              Shape {{
                appearance PBRAppearance {{
                  baseColor 0.05 0.05 0.05
                  roughness 0.9
                }}
                geometry Cylinder {{
                  height {WHEEL_W:.4f}
                  radius {WHEEL_R:.4f}
                }}
              }}
            ]
          }}
        ]
        name "{side}_wheel"
        boundingObject Transform {{
          rotation 1 0 0 1.5707963
          children [
            Cylinder {{
              height {WHEEL_W:.4f}
              radius {WHEEL_R:.4f}
            }}
          ]
        }}
        physics Physics {{
        }}
      }}
    }}"""


def sensor_node(name, spec):
    x, y, angle = sensor_pose(spec)
    z = SENSOR_HEIGHT - WHEEL_R  # origin robot = ketinggian poros roda
    return f"""    DistanceSensor {{
      translation {x:.5f} {y:.5f} {z:.4f}
      rotation 0 0 1 {angle:.5f}
      children [
        Shape {{
          appearance PBRAppearance {{ baseColor 1 1 0 }}
          geometry Box {{ size 0.005 0.005 0.005 }}
        }}
      ]
      name "{name}"
      lookupTable [
{lookup_table_rows()}
      ]
      type "generic"
      numberOfRays {SENSOR_RAYS}
    }}"""


def body_nodes(visual):
    """Satu Transform per strip body (Shape berwarna, atau Box saja untuk bounding)."""
    h = BODY_TOP_FLOOR - BODY_BOTTOM_FLOOR
    cz = (BODY_TOP_FLOOR + BODY_BOTTOM_FLOOR) / 2.0 - WHEEL_R
    nodes = []
    for lo, hi, hw in body_blocks():
        x_lo, _ = pcb_to_local(40.0, lo)
        x_hi, _ = pcb_to_local(40.0, hi)
        size = f"{x_hi - x_lo:.4f} {2 * hw / 1000.0:.4f} {h:.4f}"
        if visual:
            geom = (f"Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.1 0.85 roughness 0.6 }} "
                    f"geometry Box {{ size {size} }} }}")
        else:
            geom = f"Box {{ size {size} }}"
        nodes.append(f"      Transform {{ translation {(x_lo + x_hi) / 2.0:.4f} 0 {cz:.4f} "
                     f"children [ {geom} ] }}")
    return "\n".join(nodes)


def generate_robot(cell, controller="micromouse_controller"):
    skid_r = 0.006
    sensors = "\n".join(sensor_node(n, s) for n, s in SENSORS_PCB.items())
    # MPU6050 di PCB (40, 70.75): gyro sumbu Z dipakai untuk sudut belok dan
    # menjaga arah lurus. Rentang +-500 deg/s (8.73 rad/s), noise 1%.
    gx, gy = pcb_to_local(*MPU6050_PCB_MM)
    sensors += f"""
    Gyro {{
      translation {gx:.5f} {gy:.5f} 0
      name "gyro"
      lookupTable [ -8.727 -8.727 0.01  8.727 8.727 0.01 ]
    }}"""
    return f"""Robot {{
  translation {0.5 * cell:.4f} {0.5 * cell:.4f} {WHEEL_R:.4f}
  rotation 0 0 1 1.5707963
  children [
    Group {{
      children [
{body_nodes(True)}
      ]
    }}
{wheel_node("left", TRACK_HALF)}
{wheel_node("right", -TRACK_HALF)}
    Solid {{
      translation 0.030 0 {-(WHEEL_R - skid_r):.4f}
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor 0.2 0.2 0.2
          }}
          geometry Sphere {{
            radius {skid_r:.4f}
          }}
        }}
      ]
      name "skid"
    }}
{sensors}
  ]
  name "micromouse"
  boundingObject Group {{
    children [
{body_nodes(False)}
      Transform {{ translation 0.030 0 {-(WHEEL_R - skid_r):.4f} children [ Sphere {{ radius {skid_r:.4f} }} ] }}
    ]
  }}
  physics Physics {{
    density -1
    mass {ROBOT_MASS}
    centerOfMass [ 0 0 {ROBOT_COM_Z} ]
  }}
  supervisor TRUE
  controller "{controller}"
}}"""


def main():
    if len(sys.argv) < 3:
        print("Pakai: python maze_to_webots.py <maze.txt> <output.wbt> [--category classic|halfsize]")
        sys.exit(1)

    maze_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2])
    category = "classic"
    if "--category" in sys.argv:
        category = sys.argv[sys.argv.index("--category") + 1]
    dims = DIMENSIONS[category]
    cell, wall_h, wall_t = dims["cell"], dims["wall_height"], dims["wall_thick"]

    width, rows, horiz, vert = parse_maze(maze_path)
    walls = generate_walls(width, rows, horiz, vert, cell, wall_h, wall_t)
    controller = "micromouse_controller"
    if "--controller" in sys.argv:
        controller = sys.argv[sys.argv.index("--controller") + 1]
    robot = generate_robot(cell, controller)

    floor_w, floor_h = width * cell, rows * cell

    world = f"""#VRML_SIM R2025a utf8

WorldInfo {{
  basicTimeStep 16
}}
Viewpoint {{
  orientation -0.25 0.2 0.95 1.3
  position {-floor_w * 0.3:.3f} {-floor_h * 0.3:.3f} {max(floor_w, floor_h) * 1.1:.3f}
}}
Background {{
  skyColor [ 0.75 0.82 0.92 ]
}}
DirectionalLight {{
  direction -0.3 -0.4 -1
  intensity 1.6
  ambientIntensity 0.5
}}
Solid {{
  translation {floor_w / 2:.4f} {floor_h / 2:.4f} -0.005
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.05 0.05 0.05
        roughness 1
      }}
      geometry Box {{
        size {floor_w:.4f} {floor_h:.4f} 0.01
      }}
    }}
  ]
  name "floor"
  boundingObject Box {{
    size {floor_w:.4f} {floor_h:.4f} 0.01
  }}
}}
{chr(10).join(walls)}
{robot}
"""

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(world, encoding="utf-8")
    print(f"Maze {width}x{rows} ({category}): {len(walls)} segmen dinding.")
    for name, spec in SENSORS_PCB.items():
        x, y, a = sensor_pose(spec)
        print(f"  {name}: x={x*1000:+.2f} mm  y={y*1000:+.2f} mm  hadap={math.degrees(a):+.0f} derajat")
    print(f"Ditulis ke: {out_path}")


if __name__ == "__main__":
    main()
