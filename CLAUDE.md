# micromouse

Robot micromouse (maze-solving robot). Target hardware: **STM32F401CCU6**
(WeAct Blackpill). Repo git sendiri, terpisah dari `PROJEK/` (yang bukan git
repo) — remote: https://github.com/zrhnd/simulator-micromouse (branch `main`).

## Struktur

```
micromouse/
  README.md          ringkasan project untuk GitHub
  CLAUDE.md           file ini
  docs/
    micromouse-rules.md   aturan kompetisi (maze, robot, misi, skor) — referensi
  hardware/
    BOM_Board_MICROMOUSE.csv
    SCH_MICROMOUSE.png     skematik EasyEDA, board "Board1"
  sim/
    algorithms/
      flood-fill-c/   ALGORITMA AKTIF — flood fill (BFS) di C, sudah dikomentari
                       lengkap untuk belajar. Ini yang dipakai, bukan mms-cpp.
      astar-c/        A* di C, satu file (astar.c) + API.h/API.c. Kandidat
                       alternatif flood-fill-c. Lihat catatan di bawah.
      mms-cpp/        template C++ kosong, disimpan sebagai referensi kalau
                       nanti porting firmware pakai C++ (belum dipakai)
      q-learning/     EKSPERIMEN — Q-learning di Python, hanya untuk dicoba di
                       simulator (bukan kandidat firmware/robot nyata, jadi
                       sengaja Python bukan C). Lihat catatan di bawah.
    mms-app/          simulator mms siap pakai (mms.exe + DLL, Windows)
    mms-src/          source code simulator mms, untuk dibaca/referensi
    mazes/mazefiles/  522 maze kompetisi asli (All Japan, APEC, AAMC, dst),
                       classic (16x16) / halfsize (32x32) / training
    webots/           EKSPERIMEN — gerak kontinu (bukan diskret seperti mms).
                       Lihat catatan di bawah dan sim/webots/README.md.
  firmware/           BELUM ADA — rencana kode STM32F401CCU6 asli,
                       masih di device lain milik user
```

Semua folder di atas (termasuk `sim/mms-app`, `sim/mms-src`,
`sim/mazes/mazefiles`) **di-track di git**, bukan di-gitignore — user minta
repo GitHub-nya self-contained. `.gitignore` cuma exclude build artifact
(`sim/algorithms/**/*.exe`, `.o`, `.obj`) dan `__pycache__/`.

## Toolchain

- **Compiler C/C++**: MinGW-w64 (WinLibs, `gcc`/`g++`) ter-install
  system-wide via winget (`BrechtSanders.WinLibs.POSIX.UCRT`). PATH sistem
  sudah update, tapi shell yang sudah lama terbuka perlu direstart/PATH
  di-refresh manual dulu sebelum `gcc`/`g++` kekenali.
- **Git identity**: pakai global config user (`zurinhnd` / `zurinhnd@gmail.com`),
  tidak ada override lokal di repo ini.

## Cara pakai simulator

1. Jalankan `sim/mms-app/mms/mms.exe`.
2. Tambah algoritma baru (tombol **+**), isi:
   - Directory: `sim/algorithms/flood-fill-c`
   - Build Command: `gcc -std=c11 -O2 -o algo.exe main.c solver.c API.c queue.c`
   - Run Command: **path lengkap/absolut** ke `algo.exe` (di Windows, mms
     tidak bisa resolve nama file relatif untuk Run Command — harus full path)
3. Pilih maze (default, atau buka file dari `sim/mazes/mazefiles/classic/`),
   klik Run.

`mms` sendiri **hardcode goal = tengah geometris maze** (lihat
`Maze::getCenterPositions()` di `mms-src`) — tidak ada cara mengatur target
custom lewat file maze atau UI, walau maze half-size aslinya boleh goal di
mana saja. Jadi simulator ini cuma cocok buat latihan gaya "classic".

## flood-fill-c — catatan implementasi

- `mazeWidth`/`mazeHeight` diambil otomatis dari `API_mazeWidth()`/
  `API_mazeHeight()` saat `initialize()` — bukan hardcode 16. Array kapasitas
  tetap `MAZE_MAX_SIZE=32` (cukup untuk classic & halfsize).
- Aturan goal (`resetDistances()`) mengikuti persis logika `mms` sendiri:
  4 sel kalau width & height genap, 2 sel kalau salah satu genap, 1 sel
  kalau ganjil-ganjil.
- Run-output: satu baris log per langkah lewat `debug_log()` (muncul di
  panel log simulator), format:
  `F:<wall/open> R:<wall/open> L:<wall/open>\tACTION: <label>\tMOVE-TO: [x,y]\tREROUTE: <kosong atau "lama->baru">`
  REROUTE terisi kalau dinding baru bikin jarak-ke-goal dari sel sekarang
  naik dibanding sebelumnya (rencana lama kena blokir).
- `showPath()` mewarnai (kuning) jalur terpendek yang sedang diketahui dari
  posisi robot ke goal, update tiap langkah.
- **Known issue (belum dibenahi)**: `queue_create()` di `updateDistances()`
  tidak pernah di-`queue_destroy()` — memory leak kecil tiap langkah. Tidak
  masalah di simulator PC, tapi WAJIB dibenahi sebelum porting ke firmware
  STM32 (RAM jauh lebih terbatas).
- Ada sisa dead code di `updateMaze()` (variabel `north` tidak dipakai,
  komentar `// REMOVE LATER`) — aman diabaikan/dibersihkan kapan saja.

## q-learning — catatan implementasi

- Eksperimen, bukan algoritma aktif kompetisi (itu tetap flood-fill-c).
  Python dipilih (bukan C) karena khusus untuk dicoba di simulator `mms`.
  Target maze: 8x8 (goal 2x2 di tengah). Sengaja dibuat **Q-learning standar
  yang sederhana**: tanpa reward shaping, tanpa BFS, tanpa peta internal.
- State = (x,y); aksi = 4 arah mutlak N/E/S/W (robot berbelok otomatis).
  Hanya arah tanpa dinding (dari sensor) yang boleh dipilih, karena di `mms`
  menabrak = crash. `max Q(s',a')` juga dihitung hanya atas arah terbuka.
- Reward: -1 per langkah, +100 di goal. ε-greedy, ε turun x0.99 per episode.
  Hyperparameter ada di konstanta atas `main.py` (NUM_EPISODES=500 sebagai
  batas atas, ALPHA=0.3, GAMMA=0.95).
- Early stopping: training berhenti kalau `best_len` (langkah episode sukses
  terpendek) tak membaik selama PATIENCE=100 episode berturut-turut (dihitung
  setelah pertama kali sampai goal); kalau tidak, berhenti di NUM_EPISODES.
  Setelah itu: simpan Q-table, satu run greedy, lalu diam menunggu Reset.
- Antar episode robot pulang dengan membalik jalurnya sendiri (loop dibuang
  lewat `trail_pos`/`trail_act`) -- aman tanpa sensor, tidak perlu klik Reset.
  Heading diteruskan antar episode karena tidak ada reset sungguhan.
- Tidak ada persistensi antar sesi: Q-table mulai dari nol tiap run (menghindari
  state dari maze lain). `qtable_state.pkl` tetap ditulis di akhir training
  hanya untuk `view_qtable.py`; `episode_log.csv` untuk `plot_training.py`.
- Diuji dengan simulator tiruan (maze acak 8x8, dan `main()` utuh di
  `training/practice-8x8-a.txt`): tidak ada crash, pulang ke start selalu
  pas. Dengan early stopping (diuji dengan patience 50; default sekarang 100)
  training berhenti di episode ~80-200; run greedy akhir = jalur optimal BFS (22 langkah) di 11 dari 12
  seed, 1 seed berakhir 24 langkah (Q belum konvergen penuh saat berhenti).
  Run nyata di `mms` (`practice-8x8-a.txt`, patience 50): berhenti di episode
  194 dari batas 500, 192 sukses, jalur terbaik 22 langkah = optimal.
- Maze 9x9 yang cocok: `training/practice-8x8-a.txt` (namanya "8x8" tapi area
  bermainnya 9x9 di pojok kiri-bawah grid 16x16; goal `mms` 2x2 di (7..8,7..8)
  = pojok kanan-atas area itu).
- Riwayat: versi lama punya KnownMap + shaping BFS + `navigate_home()` BFS;
  dibuang karena shaping dari peta yang berubah melanggar syarat potential-based
  shaping dan terlalu kompleks untuk tujuan eksperimen ini.
- Tools analisis offline: `plot_training.py` (grafik reward/steps ->
  `training_plot.png`) dan `view_qtable.py` (grid panah kebijakan + ekspor
  CSV/JSON). Semua file hasil run (pkl, csv, png, gif) ikut di-track
  di git (atas permintaan user); hanya `__pycache__` yang di-gitignore.
  - `visualize_path.py` — satu PNG per langkah jalur greedy dari Q-table
    (sel saat ini merah, nilai Q arah N/E/S/W tertulis di sel, dinding dari
    file maze; default `practice-8x8-a.txt`), ke `qpath_frames/`. Opsi
    `--gif` untuk animasi `jalur.gif`. Menghitung juga jalur optimal BFS
    sebagai pembanding. Juga membuat `kebijakan.png`: grid dengan panah arah
    Q tertinggi di tiap sel yang pernah dipelajari (jalur terbaik berwarna
    merah muda).

## astar-c — catatan implementasi

- Kandidat alternatif dari `flood-fill-c`, algoritma A* (bukan BFS). Satu
  file (`astar.c`, konsolidasi dari `main.c`+`solver.h`+`solver.c` yang
  dulu terpisah, biar gampang dibaca) + `API.h`/`API.c` yang identik
  boilerplate-nya dengan `flood-fill-c`.
- Build Command: `gcc -std=c11 -O2 -o algo.exe astar.c API.c` (2 file, bukan
  3, karena sudah digabung).
- `aStarSearch()` dijalankan ulang dari nol tiap langkah (`gCost[][]`,
  `cameFrom[][]`, `closedSet[][]`, `inOpenSet[][]` di-reset penuh tiap
  panggilan) — beda dari `distances[][]` di flood-fill-c yang menyebar ke
  seluruh labirin, A* cuma menjelajah sebagian sel yang diarahkan
  heuristik (jarak Manhattan ke goal cell terdekat) sampai goal ketemu.
  Open/closed set diimplementasi lewat larik biasa + pemindaian linear
  (bukan priority queue/heap), cukup cepat untuk labirin ≤32×32.
- Aturan goal (`getGoalCells()`) sama persis logikanya dengan
  `resetDistances()` di flood-fill-c (1/2/4 sel tengah tergantung
  paritas lebar/tinggi, plus siklus tengah↔start lewat `reached_center`).
- Tampilan `gCost` di tiap sel (`main()`) pakai `API_clearText()` untuk
  sel yang belum pernah disentuh pencarian pada langkah itu (bukan
  ditampilkan sebagai `-1`), karena A* memang tidak mengisi seluruh
  labirin seperti BFS.
- **Known quirk (belum diubah)**: kolom `REROUTE` di log jauh lebih
  sering terisi dibanding `flood-fill-c`, karena heuristiknya mengarah
  ke goal cell mana pun yang *saat itu* paling dekat dari posisi robot,
  dan cell mana yang paling dekat bisa berpindah antar langkah walau
  tidak ada dinding baru sama sekali — jadi field ini tidak semurni
  penanda "dinding baru ditemukan" seperti di flood-fill-c.
- Sempat terhambat build di Windows karena kebijakan Application Control
  organisasi memblokir `collect2.exe` (bagian linker MinGW), sudah
  dibereskan sendiri oleh user.

## webots — catatan implementasi

- Eksperimen gerak **kontinu**, beda fundamental dari `mms`: di `mms`,
  algoritma kirim perintah diskret ("maju 1 sel", "belok 90°") dan posisi
  tujuan + cek tabrakan ditentukan SEBELUM animasi mulai (lihat
  `Window::updateMouseProgress` di `mms-src`) — animasinya mulus tapi
  keputusannya tetap grid. Di Webots, robot dikirim kecepatan roda
  (`v_kiri`, `v_kanan` rad/s) dan fisika/tabrakan dihitung tiap
  `basicTimeStep` (16 ms) secara real-time — ini yang bikin `mms` tidak
  cocok diedit jadi kontinu tanpa merombak total, sementara Webots memang
  dirancang untuk ini dari awal.
- `sim/webots/tools/maze_to_webots.py`: konverter file maze ASCII (format
  sama dengan `sim/mazes/mazefiles/`, dipakai juga `flood-fill-c`/`astar-c`)
  menjadi dunia Webots (`.wbt`). Dimensi sel/dinding diambil dari
  `docs/micromouse-rules.md` (classic 18cm / halfsize 9cm).
- Webots di sini adalah **lingkungan micromouse umum** (bukan khusus satu
  algoritma). Robot didefinisikan LANGSUNG di `.wbt` yang di-generate,
  mengikuti desain PCB (`hardware/PickAndPlace_MICE_PCB_2026_10_18.xlsx`
  + `image.png`): roda 29x8.6 mm (dari model 3D), jarak antar pusat roda 74.6 mm, body disusun
  dari strip kotak mengikuti outline PCB, 4 `DistanceSensor` (FL/FR depan,
  DL/DR diagonal 45°, di pusat casing 3D — bukan pad footprint), `Gyro`
  (MPU6050), 1 skid. Pusat massa diset eksplisit (`ROBOT_COM_Z`), sensor
  tipe `generic` (BUKAN `infra-red`, yang memodulasi nilai oleh warna/jarak).
- `controllers/micromouse_controller/micromouse_controller.py` = pustaka
  gerak + sensor. Utama: `run_path("FFRFL")` = gerak BERKELANJUTAN (lurus
  profil trapesium 0.30 m/s, belok busur halus 90°, tanpa berhenti tiap
  sel; sel belok tak teramati sensornya). Per sel: `drive_forward_cells(n)`
  (jarak encoder, arah dijaga gyro), `turn("left"/"right")` (sudut gyro,
  arah acuan mutlak), `read_front_wall()`, `sense_walls()`. Planner apa pun
  cukup memanggil ini alih-alih moveForward/turnLeft/turnRight versi mms.
  Pusat massa robot di sumbu roda + 8 mm ke depan (`ROBOT_COM_X/Z` di
  generator): pusat massa di bawah sumbu membuat moncong terangkat 7-20°
  saat melaju. Hasil `run_path` di practice-8x8-a: 15.3 s, rata-rata
  0.26 m/s, galat arah <1°. Pemusatan lateral PID dengan DL/DR
  (`CENTER_*`, di jalur lurus): galat posisi ke samping maks 30 -> 14 mm,
  rata-rata 17 -> 7.6 mm; galat sepanjang jalur belum dikoreksi.
- **Gaya belok** `TURN_STYLE`: `"pivot"` (bawaan, satu roda berputar, roda
  lain diam; pusat bergeser `PIVOT_RADIUS` 40 mm sehingga `turn()` mundur
  dulu dan melacak `offset`; galat menumpuk ~1.3 mm/belokan) atau
  `"point"` (belok di tempat, ~0.05 mm/belokan).
- **Pembacaan dinding**: dinding depan dari FL/FR di tengah sel; dinding
  kiri/kanan dari DL/DR yang disampel SAAT BERGERAK (jarak tempuh 45 mm
  setelah tengah sel sebelumnya), karena sinar diagonal dari tengah sel
  mengenai sel di depan. Sel start: kiri/kanan tak bisa dibaca dari tengah
  (pindai dengan berputar + `read_front_wall()`).
- `controllers/wall_test/wall_test.py` (controller dunia
  `micromouse_practice8x8.wbt`): menyusuri jalur BFS di practice-8x8-a dan
  membandingkan dinding terbaca vs file maze. Hasil uji headless:
  66/66 benar ( depan 1839-2814 vs kosong 0-38; samping 294-612 vs kosong 0-57).
  Kurva respons sensor masih PERKIRAAN — ganti dengan hasil ukur asli.
- `controllers/floodfill/floodfill.py` (dunia `micromouse_floodfill.wbt`):
  flood fill Python di Webots, hanya dari sensor. Sel start diluruskan dengan mundur ke dinding belakang (tanpa
  berputar), jelajah
  bolak-balik tengah/start sampai jalur terbukti optimal, lalu lari cepat
  `run_path`. Mode `MM_SEARCH=continuous` (bawaan; `search_run` di pustaka:
  tanpa berhenti, belok halus, keputusan diambil 95 mm sebelum tengah sel
  dari jendela DL/DR dan FL/FR) vs `cell` (berhenti di tiap sel). Hasil di
  practice-8x8-a: continuous 114 langkah/176 s simulasi, peta 0 salah;
  cell 114 langkah/351 s, peta 0 salah; lari cepat 22 langkah (optimal)
  15.3 s, sampai di tujuan. Pembacaan sisi di sel tepat sesudah busur
  ("terlambat") kurang andal (nilai tumpang tindih) -- lihat README.
  Berhenti darurat bila FL/FR melihat dinding depan mendadak <55 mm saat
  seharusnya terbuka (`on_blocked`, reroute). `MM_MAZE` memilih labirin lain;
  file maze hanya untuk ukuran dan penilaian.
- **Status** (diuji headless lewat `webots.exe --batch --no-rendering
  --mode=fast`; controller berupa Supervisor sehingga posisi SEBENARNYA
  tercatat di `run_log.txt` untuk validasi, tidak dipakai navigasi):
  - `Cylinder` di Webots R2025a (ENU) bersumbu **Z**: roda harus diputar 90°
    di sumbu X agar sejajar `HingeJoint`. Pusat massa default (dihitung dari
    bounding object) bergeser ke depan, skid menanggung beban, roda slip:
    belok di tempat hanya memutar 59 dari 90°. Sudah diperbaiki.
  - Gerak: 10 putaran pulang-pergi (36 belokan) berakhir ~2 mm dari titik
    awal, arah meleset <1°. Belum dikonfirmasi visual oleh user di GUI.
  - Dunia butuh `Background`/`DirectionalLight` bawaan (bukan
    `TexturedBackground`, PROTO-nya harus diunduh dari GitHub).
  - Lihat `sim/webots/README.md` untuk daftar asumsi yang perlu disetel
    user (kurva sensor, tinggi PCB/sensor, massa, dll).
- **Model 3D** (`sim/webots/micromouse_webots/meshes/`, dari Tinkercad):
  dipakai untuk tampilan; tabrakan tetap kotak/silinder sederhana. Webots
  tidak memakai warna `.mtl` untuk `Mesh`, jadi `tools/split_obj_by_material.py`
  memecah tiap `.obj` per warna (`meshes/parts/`) dan generator membuat satu
  `Shape` per warna. Ukuran dari model: roda 29x8.6 mm, poros 14 mm dari
  lantai, pusat sensor ~13.5 mm dari lantai, PCB 6.5 mm dari lantai.
  Orientasi dicek dengan gambar kamera (hasil di `sim/webots/images/`).
  Body 289 ribu segitiga (peringatan Webots >100k vertex); `--no-mesh` di
  generator untuk kotak polos.
- Webots Python API (`controller` module) bawaannya pure Python + ctypes
  (`wb.py` load DLL native via `WEBOTS_HOME`), BUKAN ekstensi terkompilasi
  per-versi Python — jadi tidak perlu `pip install` apa pun, Python
  manapun (termasuk 3.13 yang dipakai di mesin ini) langsung jalan selama
  dilaunch oleh Webots sendiri (bukan dari terminal luar/`extern`).

## Hardware — catatan dari review skematik

- MCU: STM32F401CCU6 Blackpill. Footprint EasyEDA yang dipakai bernama
  `Blackpill_STM32F401CEU6` (beda varian chip) — **ini aman**, PCB modul
  Blackpill CCU6 dan CEU6 identik secara fisik (dimensi, pin header, USB-C),
  cuma beda kapasitas flash/RAM chip di dalamnya.
- Buzzer di BOM (`SEA-1295Y-0520-42Ω`, LCSC C2687681) itu elektromagnetik,
  **50mA rated** — melebihi batas arus GPIO STM32F401 (~25mA/pin). Kalau
  dipakai langsung tanpa transistor driver, berisiko rusak pin. User sedang
  test di breadboard dulu.
- Motor driver TB6612FNG: designator beda antara skematik (`U2`) dan BOM
  (`U14`) — belum direkonsiliasi, cuma masalah penomoran/paperwork.

## Konvensi kerja

- Selalu test-build (`gcc ...`) sebelum bilang perubahan kode selesai.
- Commit + push ke GitHub dilakukan atas persetujuan eksplisit user tiap kali
  (bukan otomatis tiap edit).
- Attribusi commit pakai identitas git yang sudah di-set user sendiri.
