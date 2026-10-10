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
  firmware/           BELUM ADA — rencana kode STM32F401CCU6 asli,
                       masih di device lain milik user
```

Semua folder di atas (termasuk `sim/mms-app`, `sim/mms-src`,
`sim/mazes/mazefiles`) **di-track di git**, bukan di-gitignore — user minta
repo GitHub-nya self-contained. `.gitignore` cuma exclude build artifact
(`sim/algorithms/**/*.exe`, `.o`, `.obj`).

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
  Python dipilih (bukan C) karena tujuannya khusus dicoba di simulator `mms`,
  bukan untuk di-porting ke firmware STM32.
- State = posisi sel (x,y) saja; heading TIDAK masuk Q-table (aksi = arah
  mutlak N/E/S/W, robot berbelok otomatis sebelum maju) — state space jadi
  W×H×4 nilai-Q saja, bukan W×H×4×4.
- Training jalan lewat gerakan fisik sungguhan di `mms` (bukan simulasi
  virtual di memori): tiap episode robot benar-benar jalan dari start, lalu
  dipandu BFS (di atas peta yang sudah diketahui) untuk kembali ke start —
  jadi tidak perlu klik tombol Reset simulator berulang-ulang untuk setiap
  episode training (`wasReset()`/`ackReset()` cuma dipakai di akhir, untuk
  menunggu user reset manual kalau mau mengulang seluruh proses).
- Q-table + peta dinding yang sudah diketahui disimpan ke `qtable_state.pkl`
  (gitignored) supaya training lanjut dari sesi sebelumnya, bukan mulai dari
  nol tiap kali simulator dibuka ulang.
- **Bug nyata yang sudah diperbaiki (ditemukan dari crash log user, run
  pertama, maze 16x16, TANPA `qtable_state.pkl` lama sama sekali — jadi
  bukan soal state lama)**: `navigate_home()` dulu menghitung SATU rencana
  BFS di awal lalu mengeksekusi semua langkahnya membuta, tanpa sensor ulang
  di tiap langkah — beda dari `run_episode()` (dan dari `flood-fill-c`) yang
  selalu sensor+replan tiap langkah. Itu satu-satunya tempat yang melanggar
  invariant "hanya bergerak ke arah yang baru saja dikonfirmasi terbuka",
  dan bisa memicu `MouseCrashedError` yang sebelumnya tidak pernah ditangani
  (proses mati dengan traceback mentah). Sekarang `navigate_home()`
  sense+BFS ulang di SETIAP langkah, sama seperti `flood-fill-c`.
- **Bug terkait yang juga diperbaiki**: `run_episode()` dulu selalu asumsi
  mouse mulai menghadap `"N"`, padahal tidak ada reset sungguhan di `mms`
  antar episode (cuma jalan fisik pulang lewat `navigate_home()`) — kalau
  robot berhenti menghadap arah lain, asumsi `"N"` di episode berikutnya
  bikin heading yang dilacak Python desync dari heading asli robot, merusak
  semua pemetaan sensor depan/kanan/kiri/belakang→arah mutlak sesudahnya.
  Sekarang heading asli diteruskan (threaded) lewat seluruh loop di `main()`.
- **Proteksi tambahan (defense in depth, bukan fix utama)**: `qtable_state.pkl`
  dari maze lain yang KEBETULAN beda ukuran (lebar/tinggi) otomatis dibuang
  saat dimuat (`load_state()`), dan seluruh training/run akhir dibungkus
  `try/except MouseCrashedError` supaya kalau masih ada crash (mis. maze
  lain yang ukurannya SAMA tapi tata letaknya beda), programnya berhenti
  dengan pesan jelas alih-alih traceback mentah.
- `sense()` sekarang juga memanggil `setWall()` (lewat `draw_wall()`) begitu
  sebuah dinding baru pertama kali diketahui, digambar di kedua sisinya
  (sel ini dan tetangganya) — dulu tidak digambar sama sekali (cuma
  mewarnai tiap sel yang dikunjungi, tidak menampilkan peta dinding).
  Kosmetik saja, tidak memengaruhi korektnes algoritma.
- **Bug nyata lain yang sudah diperbaiki (ditemukan dari pengamatan user:
  robot "jerking" di dua sel yang sama sampai kehabisan langkah, pada
  epsilon rendah/mendekati greedy)**: reward shaping dulu pakai jarak
  **Manhattan** (garis lurus) ke goal, yang buta terhadap dinding. Sel jalan
  buntu bisa Manhattan-nya sangat dekat ke goal padahal jalur asli (lewat
  dinding yang diketahui) jauh memutar -- agen jadi mengira masuk ke jalan
  buntu itu langkah bagus, lalu terjebak maju-mundur di situ karena
  satu-satunya jalan keluar ya balik lagi. Sekarang shaping pakai
  `bfs_distances_to_goal()` (BFS multi-source dari goal, sama persis dengan
  ide `distances[][]` di flood-fill-c, tapi dihitung dari peta yang sudah
  diketahui robot) -- tidak bisa lagi salah kira jalan buntu itu dekat.
  Rumusnya juga dibetulkan jadi bentuk potential-based shaping yang benar
  (Ng, Harada, Russell 1999): `F(s->s') = gamma*Phi(s') - Phi(s)` dengan
  `Phi(s) = -jarak_ke_goal(s)` -- sebelumnya rumusnya `Phi(s) - Phi(s')`
  tanpa faktor `gamma`, jadi bukan potential-based shaping yang benar-benar
  sah (yang menjamin kebijakan optimal tidak berubah apapun potensinya).
- **Logging + tools analisis (offline, tidak perlu `mms` jalan)**:
  `main.py` sekarang menulis `episode_log.csv` (kolom: episode, steps,
  success, reward, epsilon, best_len), ditimpa tiap run dan di-flush tiap
  episode supaya tetap kebaca walau training terhenti di tengah. Dua
  script baru mengonsumsi file ini + `qtable_state.pkl`:
  - `plot_training.py` — grafik reward & steps per episode (plus moving
    average dan kurva best-path-so-far), disimpan ke `training_plot.png`.
  - `view_qtable.py` — ringkasan + grid panah ASCII dari kebijakan yang
    sudah dipelajari, plus ekspor Q-table/peta dinding mentah ke CSV/JSON.
  Semua file hasil (`episode_log.csv`, `training_plot.png`,
  `qtable_export.csv/json`) sudah di-gitignore.

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
