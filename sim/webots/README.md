# Webots -- eksperimen gerak kontinu

Simulasi micromouse dengan gerak **kontinu** (kecepatan roda real-valued,
bukan perintah diskret seperti `mms`). Lihat diskusi di riwayat chat untuk
alasan kenapa `mms` tidak cocok diedit jadi kontinu (model geraknya
"tentukan tujuan diskret dulu, baru animasikan", bukan integrasi kecepatan
real-time).

## Struktur

```
sim/webots/
  tools/
    maze_to_webots.py        konverter: file maze ASCII -> dunia Webots (.wbt)
  micromouse_webots/         project Webots (buka worlds/micromouse.wbt di Webots)
    worlds/
      micromouse.wbt          hasil generate dari maze training/minimaze.txt
    controllers/
      micromouse_controller/
        micromouse_controller.py   primitif gerak (maju N sel / belok 90 derajat
                                    pakai feedback encoder) + urutan uji coba
        run_log.txt                 (dibuat otomatis tiap run, gitignored)
```

## Cara membuka

1. Buka Webots.
2. **File > Open World...** lalu pilih `sim/webots/micromouse_webots/worlds/micromouse.wbt`.
3. Klik tombol Play/Run. Robot menjalankan urutan uji (maju-belok kanan-maju-
   belok kiri-maju), lalu idle. Buka `run_log.txt` di folder controller untuk
   lihat hasil presisinya (target vs tercapai, dalam radian roda) dan nilai
   sensor di tiap titik.

## Status

- **Fisika dasar (roda, gesekan, bentuk badan): divalidasi jalan.** Bug yang
  sempat bikin robot tidak bisa belok/kontrol sudah diperbaiki: roda
  (`Cylinder`) sempat salah orientasi (rotasi tambahan yang tidak perlu
  membuat sumbu putarnya tidak sejajar dengan `HingeJoint`) -- sudah
  dihapus.
- **Primitif gerak (`drive_forward_cells`, `turn`): divalidasi lewat
  `run_log.txt` dari urutan uji (maju-kanan-maju-kiri-maju).** Semua
  "maju 1 sel" mencapai 11.259-11.260 rad vs target 11.250 rad (overshoot
  <0.1%), kedua belokan 3.167 vs target 3.142 rad (<1%) -- presisi encoder
  bagus. Arah belok (`turn("right")`/`turn("left")`) kemungkinan besar
  SUDAH BENAR: urutan uji ini satu-satunya kombinasi arah yang bebas
  tabrakan di maze yang dipakai (sudah dicek lewat parser sebelum run), dan
  tidak ada satu pun langkah yang undershoot (tanda khas nabrak sesuatu) --
  jadi arah belok terbalik seharusnya sudah akan ketahuan dari crash/
  undershoot, tapi ini masih inferensi tidak langsung, bukan konfirmasi
  visual langsung "robot terlihat belok kanan".
- **Sensor kiri/kanan dipasang diagonal ~35 derajat ke depan** (meniru tata
  letak sensor micromouse asli yang memang jarang tegak lurus 90 derajat),
  BUKAN pengganti langsung `wallLeft()`/`wallRight()` versi `mms` yang
  instan ya/tidak persis di sisi. Perlu logika interpretasi berbeda nanti
  (bukan threshold sederhana) saat dipakai untuk navigasi.
- Parameter fisika lain (massa, gesekan, kalibrasi `lookupTable` sensor
  jarak) masih dugaan awal generik -- akan diganti begitu robot 3D asli
  (lihat bawah) masuk, karena geometrinya beda dari box placeholder ini.
- Kalau robot "menembus" dinding tipis (1.2 cm) di kecepatan tinggi: physics
  engine diskrit tiap `basicTimeStep` (16 ms) bisa tunneling -- kecilkan
  `CRUISE_SPEED`/`TURN_SPEED` atau `basicTimeStep`.

## Rencana: ganti body/roda placeholder dengan model 3D robot asli

User punya file `.obj` (export dari Tinkercad, terpisah per bagian: body,
roda kiri, roda kanan). Rencana integrasinya:

1. File `.obj`/`.mtl` ditaruh di `micromouse_webots/meshes/`.
2. `maze_to_webots.py` diubah: `Shape { geometry Box {...} }` placeholder
   body/roda diganti jadi `Shape { geometry Mesh { url "meshes/....obj" } }`,
   nilai `boundingObject`/massa/posisi sensor disesuaikan dimensi mesh asli
   (bukan dugaan 0.07x0.07x0.03 m seperti sekarang).
3. Perlu dicek skala satuan (Tinkercad biasanya ekspor mm, Webots pakai
   meter) dan titik origin tiap mesh (pusat rotasi roda harus pas di anchor
   `HingeJoint`) -- lihat OBJ-nya langsung (parsing vertex) untuk estimasi
   bounding box sebelum pasang skala.

**BELUM dikerjakan** -- menunggu file mesh ditaruh di folder di atas + info
dimensi nyata/nama file dari user.

## Membuat ulang dunia dari maze lain

```
cd sim/webots
python tools/maze_to_webots.py ../mazes/mazefiles/classic/001.txt micromouse_webots/worlds/micromouse.wbt
```

Tambahkan `--category halfsize` untuk maze 32x32 (dimensi sel 9 cm).

## Rencana selanjutnya (belum dikerjakan)

- Ganti body/roda placeholder dengan mesh 3D asli (lihat bagian di atas).
- Tambah fungsi baca dinding dari 3 `DistanceSensor` (perlu logika beda dari
  threshold sederhana karena posisi sensor diagonal, bukan tegak lurus),
  menggantikan `wallFront()`/`wallLeft()`/`wallRight()` versi mms --
  setelah ini, planner (flood-fill/A*/Q-table, format sama dengan versi
  mms) bisa dipakai ulang hampir tanpa perubahan, cuma ganti pemanggilan
  moveForward/turnLeft/turnRight ke drive_forward_cells/turn di atas.
- Reset posisi robot antar episode (untuk Q-learning) lewat Supervisor API
  Webots -- lebih fleksibel daripada tombol Reset manual di `mms`.
