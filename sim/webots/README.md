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
      micromouse_practice8x8.wbt   dari training/practice-8x8-a.txt
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

- **Fisika dasar divalidasi lewat posisi SEBENARNYA (Supervisor), bukan
  encoder saja.** `Cylinder` di Webots R2025a (ENU) bersumbu Z, jadi roda
  diputar 90 derajat di sumbu X agar sejajar `HingeJoint` (sebelumnya
  tergeletak). Pusat massa diset eksplisit di atas sumbu roda
  (`ROBOT_COM_Z`); kalau dibiarkan default, skid menanggung beban dan roda
  slip: belok di tempat hanya memutar badan 59 dari 90 derajat padahal
  encoder sudah "sampai".
- **Primitif gerak (`drive_forward_cells`, `turn`): jarak dari encoder,
  sudut dari gyro (MPU6050), arah acuan mutlak.** Hasil uji (posisi
  sebenarnya): maju 1 sel = 0.180 m; 10 putaran pulang-pergi (maju, belok
  kiri x2, maju, belok kanan x2 = 40 belokan) berakhir 2 mm dari titik awal
  dan arah meleset <1 derajat -- tidak menumpuk karena tiap belok
  menambah +-90 derajat ke arah acuan dan saat maju yaw dikoreksi ke acuan.
- **Robot kini mengikuti desain PCB** (`hardware/PickAndPlace_MICE_PCB_2026_10_18.xlsx`),
  bukan placeholder lagi: 4 sensor (`ds_fl`, `ds_fr` menghadap depan di x=+-30 mm,
  `ds_dl`, `ds_dr` diagonal 45 derajat dekat tengah), roda 34x7 mm, jarak antar
  roda 73 mm, motor maks 31.4 rad/s (N20 300 RPM). Emitter dan penerima tiap
  pasangan dipasang tumpang tindih di casing 3D di atas kotak arsir kuning di
  PCB, jadi posisi sensor = pusat kotak itu (bukan pad footprint): FL/FR di
  (x=+-30, depan 47.5 mm) menghadap lurus, DL/DR di (x=+-9.5, depan 60.1 mm)
  menghadap diagonal 45 derajat. Kotak depan terukur 7x7 mm, kotak diagonal
  belah ketupat berdiagonal 10 mm (ukuran asli dari desain bisa dicek ulang).
- **Sensor diagonal TIDAK melihat dinding samping sel tempat robot berdiri.**
  Dari tengah sel, sinar DL/DR mengenai dinding samping sel DI DEPAN (titik kena
  135 mm di depan tengah sel, jarak sinar 105 mm). Dinding samping sel sendiri
  baru terbaca kalau robot sedang ~60-180 mm di belakang tengah sel (paling pas
  ~135 mm). Artinya DL/DR harus disampel SAAT BERGERAK (berdasarkan encoder),
  atau robot berhenti agak di belakang tengah. FL membaca dinding depan pada
  36.5 mm dari tengah sel, DL/DR pada 33.8 mm.
- **Outline body** diukur dari `hardware/image.png` (+-0.5 mm): 80 x 94.9 mm,
  sudut atas bulat radius 35 mm, takik roda di y_pcb 14.6-43.2. Body disusun
  dari strip kotak mengikuti outline itu (`body_blocks()`).
- **Dugaan, belum dikalibrasi:** kurva respons sensor (`SENSOR_D0`, noise 3%; tipe `generic`, BUKAN `infra-red`, karena tipe infra-red memodulasi nilai oleh warna benda dan jarak sehingga kurva tabel tidak terbaca apa adanya),
  tinggi sensor 5 mm dari lantai, massa 80 g, tinggi body (bawah PCB 10 mm,
  atas 30 mm dari lantai -- supaya sinar sensor di 5 mm lewat di bawah PCB).
  Sensor memakai 1 sinar: sinar ganda menyebar ke atas/bawah dan pada tinggi
  5 mm bisa mengenai lantai. Ganti dengan hasil ukur asli begitu ada.
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
- Kalibrasi ambang dinding dari nilai mentah 4 sensor (`read_sensors()` kini
  hanya mencatat nilai mentah ke `run_log.txt`), lalu tulis fungsi baca dinding
  yang menggantikan `wallFront()`/`wallLeft()`/`wallRight()` versi mms.
  Webots di sini adalah lingkungan micromouse umum (bukan khusus satu
  algoritma): planner apa pun cukup memanggil `drive_forward_cells`/`turn`
  alih-alih moveForward/turnLeft/turnRight.
- Reset posisi robot ke start lewat Supervisor API (opsional, untuk
  mengulang uji tanpa membuka ulang dunia).
