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
      micromouse_floodfill.wbt     practice-8x8-a; controller floodfill
      micromouse_qlearning.wbt     practice-8x8-a; controller qlearning
    meshes/                    model 3D Tinkercad + potongan per warna (parts/)
    controllers/
      micromouse_controller/
        micromouse_controller.py   pustaka: gerak (maju N sel: encoder + gyro;
                                    belok 90 derajat: gyro), pembacaan dinding,
                                    + urutan uji gerak bila dijalankan langsung
        run_log.txt                 (dibuat otomatis tiap run, gitignored)
      wall_test/
        wall_test.py               uji baca dinding di practice-8x8-a
      floodfill/
        floodfill.py               flood fill: jelajah dengan sensor + lari cepat
      qlearning/
        qlearning.py               Q-learning tabular di atas search_run + lari cepat
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
  jelajah dengan berhenti tiap sel). Tanpa pemusatan: galat arah di tengah sel <1 derajat tapi galat
  posisi menumpuk ke SAMPING (maks 30 mm, rata-rata 17 mm setelah 11 belokan).
  Dengan pemusatan DL/DR (di bawah): maks 14 mm, rata-rata 7.6 mm.
  Sel tempat belok halus tidak teramati sensornya.
- **Gerak per sel (`drive_forward_cells`, `turn`, tetap tersedia): jarak dari encoder,
  sudut dari gyro (MPU6050), arah acuan mutlak.** Hasil uji (posisi
  sebenarnya): maju 1 sel = 0.180 m; 10 putaran pulang-pergi (maju, belok
  kiri x2, maju, belok kanan x2 = 40 belokan) berakhir 2 mm dari titik awal
  dan arah meleset <1 derajat -- tidak menumpuk karena tiap belok
  menambah +-90 derajat ke arah acuan dan saat maju yaw dikoreksi ke acuan.
- **Flood fill (`controllers/floodfill/floodfill.py`, dunia
  `micromouse_floodfill.wbt`)**: sel start diluruskan dengan MUNDUR sampai PCB
  belakang (datar) menempel dinding belakang (`square_to_back_wall`; kiri,
  kanan, belakang sel start diasumsikan dinding sesuai aturan, depan dibaca
  FL/FR tanpa berputar), lalu jelajah bolak-balik tengah/start sampai jalur terbukti
  optimal (jalur diketahui = perkiraan optimis), lalu lari cepat `run_path`.
  Dinding hanya dari sensor; file maze hanya untuk ukuran labirin dan
  penilaian. Dua mode (`MM_SEARCH`): `continuous` (bawaan) dan `cell`.
  Hasil di practice-8x8-a (deterministik, hasil sama tiap run):

  | | `cell` | `continuous` |
  |---|---|---|
  | Jelajah | 114 langkah, 345 s simulasi | 114 langkah, 126 s |
  | Putar di tempat saat jelajah | tiap belokan | 16 |
  | Peta (sisi diketahui / salah) | 201 / 0 | 201 / 0 |
  | Lari cepat | 22 langkah (optimal), 15.3 s, sampai tujuan | sama |

  `continuous` memakai `search_run` di pustaka: tidak berhenti di tiap sel,
  belok halus, keputusan flood fill diambil SAMBIL BERGERAK 95 mm sebelum
  tengah tiap sel yang dimasuki (di situ busur belok dimulai): dinding
  kiri/kanan dari DL/DR (jendela 140-95 mm sebelum tengah sel) dan dinding
  depan dari FL/FR (jendela 130-95 mm; jarak 125-165 mm, nilai 120-270 vs 0
  kalau terbuka; ambang 60).
  **Belokan beruntun (belok lagi tepat sesudah belok):** sel pertama sesudah
  busur ("terlambat") baru dimasuki di batas masuk, jendela di atas sudah
  lewat. Dipakai jendela 90-70 mm sebelum tengah dan keputusan di 70 mm. Busur
  di sel itu tetap dijalankan halus dengan BENTUK DISESUAIKAN
  (`_arc_shape`: yaw acuan = (pi/2)*smoothstep(t)^p; p dipilih supaya ujung
  busur tetap tepat di tengah lorong sel berikutnya walau mulai 70 mm, bukan
  90 mm, sebelum tengah; p~0.69, panjang ~132 mm). Sebelumnya belokan
  beruntun berhenti di tengah sel dan diputar di tempat (36 putaran per
  jelajah; sekarang 16, sisanya jalan buntu dan arah awal tiap perjalanan).
  **Pembacaan samping di sel terlambat** (`_side_wall_late`): nilai dinding
  asli (240-800) dan sinar yang lolos lewat sisi terbuka ke dinding DEPAN sel
  tetangga (180-290) tumpang tindih, tapi nilai sinar lolos NAIK ~30-50% di
  sepanjang jendela (jaraknya mengecil ~1.4 mm per mm robot maju) sedangkan
  dinding samping sungguhan tetap atau turun. Dinding = nilai rata-rata >140
  dan tidak naik >12%. Kalau sisi lawan sangat dekat (>700, robot miring >22
  mm) dan sisi ini ambigu (naik, 140-330), dianggap DINDING (aman; pembacaan
  normal boleh menimpa, pembacaan terlambat hanya mengisi sisi yang belum
  diketahui). Hasil: 1 pembacaan salah di log (dianggap dinding padahal
  terbuka), peta akhir 0 salah.
  **Meluruskan di sel start:** robot mundur 75 mm (jarak ke dinding belakang
  55 mm + 20 mm); kalau miring 6 derajat, dinding meluruskannya kembali ke
  90.00 derajat (uji `MM_START_TILT_DEG=6`); arah acuan gyro di-reset lalu
  maju ke tengah sel (persiapan 3.6 s, sebelumnya pemindaian 24 s).
  **Berhenti darurat:** di jalur lurus sesudah aksi "F" (dinding depan
  diperkirakan terbuka) FL/FR dipantau; kalau rata-ratanya melewati ~1170
  (sensor ~59.5 mm dari dinding) robot langsung direm. Jarak berhenti aman
  ditetapkan **37 mm dari TEPI DEPAN PCB ke dinding** (diukur pengguna di
  model 3D; konstanta `SAFE_STOP_NOSE`; sensor 18.5 mm di belakang ujung PCB,
  jadi 55.5 mm sensor-dinding, ditambah `BRAKE_MARGIN` 4 mm untuk jarak rem
  dan toleransi pembacaan). Setelah rem robot TIDAK maju lagi (sebelumnya
  maju ke tengah sel dan sempat tinggal 13 mm dari dinding); kalau di bawah
  jarak aman cukup mundur, lalu peta diperbaiki (`on_blocked`), rute
  dihitung ulang, dan robot diputar di tempat. Diuji dengan
  `MM_BLIND_FRONT=0,2` (dinding depan sel itu sengaja dianggap terbuka):
  berhenti pada 38.1-39.2 mm dari tepi PCB, reroute jalan, dan tetap sampai
  di tujuan dengan jalur akhir optimal. Pada kunjungan normal tidak pernah
  terpicu (0 kali). Pantauan tidak aktif di busur belok dan di tahap
  berhenti terencana (jalan buntu, tujuan; di tengah sel tepi PCB 18 mm dari
  dinding).
  **Jarak bebas PCB-dinding (`MM_CLEARANCE=1` mencatat tiap kejadian < 30 mm
  dari pose sebenarnya):** di belok halus tepi PCB (sudut depan sisi luar)
  lewat 22-29 mm dari dinding depan (geometri busur 90 mm; di bawah batas
  berhenti 37 mm, tapi itu jarak berhenti, bukan jarak lewat). Berhenti
  terencana (jalan buntu, belok di sel terlambat) dulu berhenti di tengah sel
  (tepi PCB 18 mm dari dinding depan, putar di tempat menyenggol dinding
  samping sampai 0 mm); sekarang kalau ada dinding depan robot berhenti
  `SAFE_STOP_NOSE + PCB_FRONT_AHEAD - HALF_LANE` = 19 mm LEBIH AWAL (tepi PCB
  37 mm dari dinding), putar di tempat tersisa jarak bebas 17-23 mm di
  sebagian besar jalan buntu. Kasus macet 6 s (sel (8,0): galat lateral ~22 mm
  saat tiba; lorong 168 mm dengan radius putar PCB ~66 mm hanya menyisakan 18
  mm toleransi lateral) diperbaiki dengan **geser lateral** (`_lateral_nudge`)
  sebelum putar di tempat bila |galat| > 10 mm (galat dari nilai DL/DR di
  jendela keputusan, `e_est`): putar 15 derajat menjauhi arah geser, mundur
  sejauh galat/sin(15), putar kembali, maju lagi (posisi sumbu dan arah akhir
  tidak berubah). Jarak bebas diam terkecil sekarang 10.6 mm (saat memutar 15
  derajat) dan >= 15.9 mm sesudahnya. Pemusatan DL/DR juga dimatikan bila
  FL/FR melihat dinding depan dekat (nilai > 400), karena di jarak itu sinar
  DL/DR mengenai dinding DEPAN dan galat lateralnya palsu. Strip tabrakan kubah depan
  dinaikkan 7 -> 24 supaya outline simulasi lebih dekat ke outline PCB.
  **Keterbatasan:** keputusan samping di sel terlambat bergantung pada pola nilai
  sensor dan ambang yang dikalibrasi ke kurva perkiraan; pada robot asli
  perlu diuji ulang. Pembacaan ambigu dibuat konservatif (dinding), jadi
  rute bisa lebih panjang dari optimal bila dinding tampak ada padahal
  tidak (belum terjadi di uji ini). Putar balik (jalan buntu) berhenti di
  tengah sel dan diputar 180 derajat di tempat. Tidak ada deteksi macet.
  `MM_MAZE=<file>` memilih labirin lain (generate dunianya dari file yang
  sama), `MM_MAX_TRIPS` membatasi jumlah perjalanan.
- **Q-learning (`controllers/qlearning/qlearning.py`, dunia
  `micromouse_qlearning.wbt`)**: algoritma yang sama dengan
  `sim/algorithms/q-learning` (state (x,y), aksi N/E/S/W yang tidak
  berdinding menurut sensor, reward -1 per langkah dan +100 di tujuan,
  alpha 0.3, gamma 0.95, epsilon 1.0 turun x0.99 per episode, early stopping
  patience 100), dijalankan di atas `search_run` (gerak mulus, belok halus,
  pengaman). Beda dengan versi `mms`: antar episode robot DIPINDAHKAN ke start
  lewat Supervisor (bukan pulang membalik jalur), batas langkah per episode
  `MM_MAX_STEPS` (400, bukan 4*w*h; tiap langkah ~1 s simulasi), arah balik
  hanya dipilih di jalan buntu. Hasil di
  practice-8x8-a (`MM_SEED=7`, setelah perbaikan di bawah): early stopping di
  episode 139, jalur 22 langkah (optimal) pertama kali di episode 39, 0
  saturasi roda, 0 rem darurat, 0 busur gagal; episode greedy sesudahnya 24
  langkah (Q belum konvergen penuh, sama seperti sebagian seed di versi
  `mms`), lari cepat dari jalur tanpa loop 24 langkah 16.9 s sampai tujuan.
  Uji 25 episode: seed 2 24/25 berhasil, seed 3 25/25, seed 4 25/25.
  **Perbaikan keandalan yang dipicu peringatan `requested velocity ...
  exceeds 'maxVelocity'` di console:** (1) peringatan itu berasal dari belok
  halus yang TERTAHAN dinding (galat arah 20-30 derajat memerintahkan roda
  luar >31.4 rad/s): `set_wheels` kini menskala KEDUA roda bersama dan mencatat
  `SATURASI` ke `run_log.txt`; (2) penyebab tertahannya: eksplorasi acak
  memilih belok ke sisi yang salah baca atau busur di sel terlambat dekat
  dinding depan (jarak bebas ~10 mm) -- `search_run(late_arc_with_front=False)`
  berhenti dan putar di tempat bila ada dinding depan; (3) sisi ambigu di sel
  terlambat (`conservative_sides=True`) dibaca None dan DIVERIFIKASI: robot
  berhenti, berputar ke sana, membaca dinding depan dengan FL/FR yang andal,
  dan memilih ulang bila dinding (`on_verify`/`on_blocked`); sebelumnya
  ambigu=dinding membuat jalan yang sah tertutup selamanya dan episode
  berputar di tempat; (4) bug pelacakan: sesudah verifikasi gagal, pilihan
  terakhir adalah kebalikan arah MASUK sel (bukan kebalikan arah yang diblokir,
  yang bisa dinding lain); (5) batas waktu busur/lurus (`StallError`) supaya
  gerak yang tertahan tidak menggantung selamanya (episode dihitung gagal dan
  robot dipindah ke start); (6) kompensasi lateral sesudah berhenti 19 mm lebih
  awal lalu putar di tempat; (7) cek arah sesudah busur (`BUSUR GAGAL`). Variabel:
  `MM_EPISODES`, `MM_PATIENCE`, `MM_MAX_STEPS`, `MM_EPS_DECAY`, `MM_ALPHA`,
  `MM_GAMMA`, `MM_SEED`, `MM_MAZE`. Console Webots dibuat sama dengan versi
  `mms` ("Ukuran maze: 16x16", `[Episode n/N] sampai tujuan dalam ... langkah,
  reward=..., epsilon=..., terbaik=...`, "Early stopping: ...", "Training
  selesai. Run akhir (greedy, tanpa eksplorasi)...", "Run akhir sampai tujuan
  dalam ... langkah."); log gerak pustaka dan keterangan tambahan (waktu
  simulasi, jalur tanpa loop, lari cepat) hanya ke `run_log.txt`
  (`LOG_TO_CONSOLE` di pustaka). Keluaran (diabaikan git):
  `controllers/qlearning/episode_log.csv` dan `qtable_state.pkl`. Pengaman
  hanya untuk dinding depan: kalau pembacaan samping salah, robot bisa
  menyenggol dinding.
- **Pemusatan lateral dengan DL/DR (PID bertingkat, `CENTER_*`)**: di jalur
  lurus, galat lateral e (m, + = robot di kiri tengah lorong) dihitung dari
  jarak sinar DL/DR ke dinding (kebalikan kurva respons sensor; dua dinding:
  setengah selisih jarak kiri-kanan; satu dinding: terhadap setengah lebar
  lorong nominal 84 mm; nilai sensor <70 dianggap tak ada dinding, kalau
  keduanya tak ada koreksi memudar ke nol). PID pada e (kp 6 rad/m, ki 2,
  kd 0.3, offset maks 0.2 rad, integral dibatasi) menghasilkan OFFSET yaw
  yang ditambahkan ke yaw acuan; loop yaw gyro (`YAW_KP`) di dalamnya
  menjalankannya. Tidak aktif di busur belok. Hasil di wall_test (22 sel,
  11 belokan halus): galat posisi di tengah sel maks 29.6 -> 14.4 mm,
  rata-rata 17.3 -> 7.6 mm; galat arah maks 2.4 derajat (sengaja miring saat
  mengoreksi). Dari awal digeser 20 mm: 20 -> 10 mm setelah satu sel (tanpa
  pemusatan tetap 20 mm). Gain 6-14 hampir sama hasilnya; dipakai yang
  konservatif. Sisa galat terbesar ada tepat setelah belokan (lurus 1 sel
  terlalu pendek untuk mengoreksi penuh); galat SEPANJANG jalur (maks ~11 mm)
  belum dikoreksi (butuh tepi dinding / dinding depan). Env var uji:
  `MM_NO_CENTER=1`, `MM_START_SHIFT_MM=20`, `MM_GAINS=kp,ki,kd`.
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
