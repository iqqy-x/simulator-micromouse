#!/usr/bin/env python3
"""
Konversi file maze ASCII (format sama dengan sim/mazes/mazefiles/, dipakai
juga oleh flood-fill-c/astar-c) menjadi dunia Webots (.wbt), lengkap dengan
robot differential-drive dan 3 sensor jarak.

Cara pakai:
    python maze_to_webots.py <maze.txt> <output.wbt> [--category classic|halfsize]

Dimensi mengikuti docs/micromouse-rules.md:
    classic : sel 18x18 cm, dinding tinggi 5 cm, tebal 1.2 cm
    halfsize: sel 9x9 cm,  dinding tinggi 2.5 cm, tebal 0.6 cm

Konvensi koordinat (disamakan dengan proyek flood-fill-c/q-learning):
    - (0,0) di pojok kiri-bawah, x ke arah Timur, y ke arah Utara.
    - Dunia Webots: sumbu Z = atas (bawaan Webots sejak versi ini, ENU).
    - Posisi dunia sel (cx, cy) = pusatnya di ((cx+0.5)*CELL, (cy+0.5)*CELL).
    - Robot mulai di tengah sel (0,0), menghadap Utara (+Y dunia).

CATATAN: ini draft pertama (belum pernah dibuka di Webots asli, karena tidak
ada cara saya menjalankan GUI Webots dari sini). Body/roda/sensor pakai
parameter dugaan yang wajar untuk micromouse kecil -- cek & sesuaikan massa,
gesekan, dan lookupTable sensor setelah dibuka, terutama kalau robotnya
terasa licin/terbang/menembus dinding.
"""

import sys
from pathlib import Path

DIMENSIONS = {
    "classic": {"cell": 0.18, "wall_height": 0.05, "wall_thick": 0.012},
    "halfsize": {"cell": 0.09, "wall_height": 0.025, "wall_thick": 0.006},
}


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


ROBOT_TEMPLATE = """Robot {{
  translation {start_x:.4f} {start_y:.4f} {wheel_r:.4f}
  rotation 0 0 1 1.5707963
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.1 0.1 0.85
        roughness 0.6
      }}
      geometry Box {{
        size {body_len:.4f} {body_wid:.4f} {body_h:.4f}
      }}
    }}
    DEF LEFT_WHEEL HingeJoint {{
      jointParameters HingeJointParameters {{
        axis 0 1 0
        anchor {wheel_x:.4f} {track_half:.4f} 0
      }}
      device [
        RotationalMotor {{
          name "left_wheel_motor"
          maxVelocity {max_motor_vel}
        }}
        PositionSensor {{
          name "left_wheel_sensor"
        }}
      ]
      endPoint Solid {{
        translation {wheel_x:.4f} {track_half:.4f} 0
        children [
          Shape {{
            appearance PBRAppearance {{
              baseColor 0.05 0.05 0.05
              roughness 0.9
            }}
            geometry Cylinder {{
              height {wheel_w:.4f}
              radius {wheel_r:.4f}
            }}
          }}
        ]
        name "left_wheel"
        boundingObject Cylinder {{
          height {wheel_w:.4f}
          radius {wheel_r:.4f}
        }}
        physics Physics {{
        }}
      }}
    }}
    DEF RIGHT_WHEEL HingeJoint {{
      jointParameters HingeJointParameters {{
        axis 0 1 0
        anchor {wheel_x:.4f} {neg_track_half:.4f} 0
      }}
      device [
        RotationalMotor {{
          name "right_wheel_motor"
          maxVelocity {max_motor_vel}
        }}
        PositionSensor {{
          name "right_wheel_sensor"
        }}
      ]
      endPoint Solid {{
        translation {wheel_x:.4f} {neg_track_half:.4f} 0
        children [
          Shape {{
            appearance PBRAppearance {{
              baseColor 0.05 0.05 0.05
              roughness 0.9
            }}
            geometry Cylinder {{
              height {wheel_w:.4f}
              radius {wheel_r:.4f}
            }}
          }}
        ]
        name "right_wheel"
        boundingObject Cylinder {{
          height {wheel_w:.4f}
          radius {wheel_r:.4f}
        }}
        physics Physics {{
        }}
      }}
    }}
    Solid {{
      translation {skid_x:.4f} 0 {skid_z:.4f}
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
      boundingObject Sphere {{
        radius {skid_r:.4f}
      }}
    }}
    DistanceSensor {{
      translation {sensor_x:.4f} 0 0
      rotation 0 0 1 0
      children [
        Shape {{
          appearance PBRAppearance {{ baseColor 1 1 0 }}
          geometry Box {{ size 0.005 0.005 0.005 }}
        }}
      ]
      name "ds_front"
      lookupTable [
        0 1000 0
        0.001 1000 0
        {ds_range:.3f} 0 0
      ]
      type "infra-red"
    }}
    DistanceSensor {{
      translation {sensor_x:.4f} 0 0
      rotation 0 0 1 0.6109
      children [
        Shape {{
          appearance PBRAppearance {{ baseColor 1 1 0 }}
          geometry Box {{ size 0.005 0.005 0.005 }}
        }}
      ]
      name "ds_left"
      lookupTable [
        0 1000 0
        0.001 1000 0
        {ds_range:.3f} 0 0
      ]
      type "infra-red"
    }}
    DistanceSensor {{
      translation {sensor_x:.4f} 0 0
      rotation 0 0 1 -0.6109
      children [
        Shape {{
          appearance PBRAppearance {{ baseColor 1 1 0 }}
          geometry Box {{ size 0.005 0.005 0.005 }}
        }}
      ]
      name "ds_right"
      lookupTable [
        0 1000 0
        0.001 1000 0
        {ds_range:.3f} 0 0
      ]
      type "infra-red"
    }}
  ]
  name "micromouse"
  boundingObject Box {{
    size {body_len:.4f} {body_wid:.4f} {body_h:.4f}
  }}
  physics Physics {{
    density -1
    mass 0.08
  }}
  controller "micromouse_controller"
}}"""


def generate_robot(cell, wall_h):
    wheel_r = 0.016
    wheel_w = 0.008
    track_half = 0.032
    body_len = 0.07
    body_wid = 0.07
    body_h = 0.03
    return ROBOT_TEMPLATE.format(
        start_x=0.5 * cell, start_y=0.5 * cell, wheel_r=wheel_r,
        wheel_x=-0.01, track_half=track_half, neg_track_half=-track_half,
        wheel_w=wheel_w, max_motor_vel=20,
        skid_x=0.025, skid_z=-(wheel_r - 0.006), skid_r=0.006,
        sensor_x=0.045, ds_range=min(2 * cell, 0.3),  # sedikit di luar tepi body (0.035)
        body_len=body_len, body_wid=body_wid, body_h=body_h,
    )


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
    robot = generate_robot(cell, wall_h)

    floor_w, floor_h = width * cell, rows * cell

    world = f"""#VRML_SIM R2025a utf8

WorldInfo {{
  basicTimeStep 16
}}
Viewpoint {{
  orientation -0.25 0.2 0.95 1.3
  position {-floor_w * 0.3:.3f} {-floor_h * 0.3:.3f} {max(floor_w, floor_h) * 1.1:.3f}
}}
TexturedBackground {{
}}
TexturedBackgroundLight {{
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
    print(f"Ditulis ke: {out_path}")


if __name__ == "__main__":
    main()
