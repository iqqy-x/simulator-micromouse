#!/usr/bin/env python3
"""
Pecah file .obj hasil Tinkercad menjadi satu .obj per material (warna).

Kenapa: Webots tidak memakai warna dari .mtl untuk node Mesh (hasilnya putih
polos), jadi tiap warna dijadikan Shape sendiri dengan PBRAppearance. Skrip
ini menyiapkan potongannya; maze_to_webots.py yang membuat Shape-nya.

Cara pakai (dijalankan sekali, atau lagi kalau model 3D diganti):
    python split_obj_by_material.py

Masukan : micromouse_webots/meshes/{body,wheel_left,wheel_right}.obj + .mtl
Keluaran: micromouse_webots/meshes/parts/<nama>__<material>.obj
          (koordinat tidak diubah, hanya dipilah per material)
"""

from pathlib import Path

MESHES = Path(__file__).resolve().parent.parent / "micromouse_webots" / "meshes"
PARTS = MESHES / "parts"
NAMES = ("body", "wheel_left", "wheel_right")


def split(name):
    vertices = []
    faces = {}  # material -> daftar baris indeks (1-based, indeks vertex global)
    current = None
    with open(MESHES / f"{name}.obj", encoding="utf-8") as f:
        for line in f:
            if line.startswith("v "):
                vertices.append(line.split()[1:4])
            elif line.startswith("usemtl"):
                current = line.split()[1]
            elif line.startswith("f "):
                idx = [int(tok.split("/")[0]) for tok in line.split()[1:]]
                faces.setdefault(current, []).append(idx)

    written = []
    for material, face_list in faces.items():
        remap = {}
        out_vertices = []
        out_faces = []
        for idx in face_list:
            new = []
            for i in idx:
                if i not in remap:
                    remap[i] = len(out_vertices) + 1
                    out_vertices.append(vertices[i - 1])
                new.append(remap[i])
            out_faces.append(new)
        path = PARTS / f"{name}__{material}.obj"
        with open(path, "w", encoding="utf-8") as f:
            f.write(f"# {name} / {material}\n")
            for v in out_vertices:
                f.write("v " + " ".join(v) + "\n")
            for face in out_faces:
                f.write("f " + " ".join(str(i) for i in face) + "\n")
        written.append((material, len(out_vertices), len(out_faces)))
    return written


def main():
    PARTS.mkdir(parents=True, exist_ok=True)
    for old in PARTS.glob("*.obj"):
        old.unlink()
    for name in NAMES:
        parts = split(name)
        print(f"{name}: {len(parts)} material")
        for material, nv, nf in sorted(parts, key=lambda p: -p[2])[:20]:
            print(f"   {material}: {nv} titik, {nf} segitiga")


if __name__ == "__main__":
    main()
