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
    split_obj_by_material.py memecah .obj Tinkercad per warna (untuk meshes/parts/)
  micromouse_webots/         project Webots (buka worlds/micromouse.wbt di Webots)
    worlds/
      micromouse.wbt          hasil generate dari maze training/minimaze.txt
      micromouse_practice8x8.wbt   dari training/practice-8x8-a.txt; controller
                                   wall_test (uji pembacaan dinding)
    meshes/                    model 3D Tinkercad + potongan per warna (parts/)
    controllers/
      micromouse_controller/
        micromouse_controller.py   pustaka: gerak (maju N sel: encoder + gyro;
                                    belok 90 derajat: gyro), pembacaan dinding,
                                    + urutan uji gerak bila dijalankan langsung
        run_log.txt                 (dibuat otomatis tiap run, gitignored)
      wall_test/
        wall_test.py               uji baca dinding di practice-8x8-a
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
  tergeletak). Pusat massa diset eksplisit setinggi sumbu roda dan 8 mm di
  depannya (`ROBOT_COM_X/Z`). Default (dihitung dari bounding object) membuat
  skid menanggung beban dan roda slip: belok di tempat hanya memutar badan 59
  dari 90 derajat padahal encoder sudah "sampai". Pusat massa DI BAWAH sumbu
  (pendulum) juga buruk: saat melaju moncong terangkat 7-20 derajat dan sinar
  sensor (tinggi 5 mm) lewat di atas dinding; dengan pusat massa di sumbu
  kemiringan saat melaju ~0 derajat.
- **Gerak berkelanjutan (`run_path("FFRFL")`)**: robot tidak berhenti di
  tiap sel. Lurus memakai profil kecepatan trapesium (jelajah 0.30 m/s,
  percepatan 2 m/s^2), belok memakai busur halus 90 derajat yang dimulai di
  tengah sisi sel dan berakhir di tengah sisi sel bersebelahan (setara
  jari-jari 90 mm): yaw acuan mengikuti smoothstep terhadap jarak busur
  (laju putar nol di awal dan akhir) dengan umpan maju laju putar, kecepatan
  di busur 0.20 m/s. Jalur lurus antar belokan digabung jadi satu segmen.
  Belok pertama di sel start diputar di tempat (tidak bisa halus dari tengah
  sel). Hasil di practice-8x8-a (22 langkah, 11 belokan halus): 15.3 s
  simulasi, kecepatan rata-rata 0.26 m/s (gerak per sel sebelumnya 0.07 m/s
  jelajah dengan berhenti tiap sel). Galat arah di tengah sel <1 derajat;
  galat posisi menumpuk ke SAMPING (maks 27 mm dari tengah sel setelah 11
  belokan, rata-rata 16 mm) karena belum ada koreksi dinding -- langkah
  berikutnya (pemusatan dengan DL/DR dan penyelarasan dinding depan).
  Sel tempat belok halus tidak teramati sensornya.
- **Gerak per sel (`drive_forward_cells`, `turn`, tetap tersedia): jarak dari encoder,
  sudut dari gyro (MPU6050), arah acuan mutlak.** Hasil uji (posisi
  sebenarnya): maju 1 sel = 0.180 m; 10 putaran pulang-pergi (maju, belok
  kiri x2, maju, belok kanan x2 = 40 belokan) berakhir 2 mm dari titik awal
  dan arah meleset <1 derajat -- tidak menumpuk karena tiap belok
  menambah +-90 derajat ke arah acuan dan saat maju yaw dikoreksi ke acuan.
- **Pembacaan dinding: dengan `run_path` 33/33 benar pada sel yang teramati
  (sel lurus; sel belok halus dan sel start tidak teramati); dengan gerak per
  sel (`sense_walls()`, `read_front_wall()`) 66/66 benar** di practice-8x8-a (`wall_test`: robot menyusuri jalur
  terpendek 22 langkah; dinding terbaca hanya dari sensor, dibandingkan
  dengan file maze). Depan: FL/FR di tengah sel (dinding 1839-2814, kosong
  0-38, ambang 800). Samping: DL/DR disampel SAAT BERGERAK di jarak tempuh
  45 mm setelah tengah sel sebelumnya (dinding 294-612, kosong 0-57, ambang
  200). Sel start tidak bisa dibaca sisinya dari tengah sel; `wall_test`
  tidak memindainya. Margin ini hanya berlaku untuk kurva sensor perkiraan;
  ulangi uji setelah kurva diganti hasil ukur asli.
- **Gaya belok (`TURN_STYLE`)**: `"pivot"` (bawaan) = hanya SATU roda berputar
  (belok kanan: roda kiri maju, kanan diam; belok kiri sebaliknya);
  `"point"` = kedua roda berlawanan arah, belok di tempat. Pivot menggeser
  pusat robot sejauh `PIVOT_RADIUS` (40 mm = setengah jarak roda + setengah
  lebar roda, terukur; roda diam berputar di tepi luar silinder), jadi
  `turn()` mundur dulu lalu memutar, dan robot berhenti `PIVOT_RADIUS` di
  depan tengah sel (`offset`); `drive_forward_cells()` memotong jaraknya,
  `read_front_wall()` mundur ke tengah dulu. Akurasi: pivot menumpuk galat
  ~1.3 mm per belokan (14 mm setelah 11 belokan di wall_test) karena roda
  diam sedikit tergelincir; belok di tempat ~0.05 mm per belokan (2 mm
  setelah 36 belokan). Pivot belum dikoreksi sensor.
- **Robot kini mengikuti desain PCB** (`hardware/PickAndPlace_MICE_PCB_2026_10_18.xlsx`),
  bukan placeholder lagi: 4 sensor (`ds_fl`, `ds_fr` menghadap depan di x=+-30 mm,
  `ds_dl`, `ds_dr` diagonal 45 derajat dekat tengah), roda 29x8.6 mm (dari model 3D), jarak antar
  pusat roda 74.6 mm, motor maks 31.4 rad/s (N20 300 RPM). Emitter dan penerima tiap
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
- **Dari model 3D (`meshes/`, diukur dari titik-titiknya):** roda diameter
  29 mm x lebar 8.6 mm (bukan 34x7), pusat roda 74.6 mm, poros 14 mm dari
  lantai; bidang bawah PCB 6.5 mm dari lantai, komponen tertinggi ~22.5 mm
  (ditambah satu benda kecil setinggi 52 mm yang diabaikan untuk tabrakan);
  casing sensor membentang 8-18 mm, jadi pusat pasangan sensor ~13.5 mm dari
  lantai (bukan 5 mm). Posisi casing sensor di model cocok dengan perhitungan
  dari kotak arsir PCB (selisih <0.5 mm).
- **Dugaan, belum dikalibrasi:** kurva respons sensor (`SENSOR_D0`, noise 3%; tipe `generic`, BUKAN `infra-red`, karena tipe infra-red memodulasi nilai oleh warna benda dan jarak sehingga kurva tabel tidak terbaca apa adanya),
  massa 80 g. Sensor memakai 1 sinar: sinar ganda menyebar ke atas/bawah dan
  bisa mengenai lantai. Ganti dengan hasil ukur asli begitu ada.
- Kalau robot "menembus" dinding tipis (1.2 cm) di kecepatan tinggi: physics
  engine diskrit tiap `basicTimeStep` (16 ms) bisa tunneling -- kecilkan
  `CRUISE_SPEED`/`TURN_SPEED` atau `basicTimeStep`.

## Model 3D robot (`micromouse_webots/meshes/`)

Model dari Tinkercad dipakai untuk TAMPILAN saja; bentuk fisik (tabrakan)
tetap kotak/silinder sederhana supaya simulasi cepat dan stabil.

| File | Isi |
|---|---|
| `body.obj/.mtl`, `wheel_left.obj/.mtl`, `wheel_right.obj/.mtl` | file asli dari Tinkercad (satuan mm, Z ke atas, X ke samping, Y ke depan, titik asal x/y = sumbu roda) |
| `parts/<nama>__<material>.obj` | hasil `tools/split_obj_by_material.py`: satu file per warna |
| `../images/robot-atas.png`, `robot-samping.png` | contoh tampilan hasil render |

- **Kenapa dipecah per warna:** Webots tidak memakai warna `.mtl` untuk node
  `Mesh` (hasilnya putih polos). `maze_to_webots.py` membuat satu `Shape`
  ber-`PBRAppearance` untuk tiap warna dari `Kd` di `.mtl`. Jalankan ulang
  `python tools/split_obj_by_material.py` kalau model diganti, lalu generate
  ulang dunia.
- **Orientasi:** mesh diputar -90 derajat sekitar Z (x_robot = y_mesh,
  y_robot = -x_mesh), diskala 0.001, dan digeser turun setinggi poros roda
  mesh. Dicek lewat gambar kamera (tampak atas dan samping).
- **Dunia memakai mesh kalau `meshes/parts/` ada;** `--no-mesh` memaksa
  kotak/silinder polos.
- **Berat:** body 289 ribu segitiga (satu bagian gelap berisi 132 ribu titik)
  -- Webots menampilkan peringatan "lebih dari 100'000 vertices" dan bayangan
  dimatikan (`castShadows FALSE`) untuk body. Kalau GUI lambat, kurangi
  detail di Tinkercad atau pakai `--no-mesh`.
- Folder `meshes/` ~20 MB (file asli + potongan).

## Membuat ulang dunia dari maze lain

```
cd sim/webots
python tools/maze_to_webots.py ../mazes/mazefiles/classic/001.txt micromouse_webots/worlds/micromouse.wbt
```

Tambahkan `--category halfsize` untuk maze 32x32 (dimensi sel 9 cm).

## Rencana selanjutnya (belum dikerjakan)

- Ganti kurva respons sensor perkiraan dengan hasil ukur TEFT4300+SFH4545
  asli, lalu ulangi `wall_test` (ambang `FRONT_THRESHOLD`/`SIDE_THRESHOLD`
  di `micromouse_controller.py` mungkin perlu disesuaikan).
  Webots di sini adalah lingkungan micromouse umum (bukan khusus satu
  algoritma): planner apa pun cukup memanggil `drive_forward_cells`/`turn`
  alih-alih moveForward/turnLeft/turnRight.
- Reset posisi robot ke start lewat Supervisor API (opsional, untuk
  mengulang uji tanpa membuka ulang dunia).
