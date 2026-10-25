# Simulasi micromouse kontinu dengan rule based

Skenario tugas tersedia di
`micromouse_webots/worlds/micromouse_rule_based.wbt`, menggunakan maze
`training/practice-8x8-a.txt` dan controller `rule_based`.

Robot bergerak dengan fisika Webots dan perintah kecepatan roda bernilai
riil. Aturan navigasi menggunakan ingatan sel dan jalur kembali. Dengan
demikian, implementasinya **hybrid**: keadaan dan gerak fisik kontinu,
sedangkan ingatan serta keputusan navigasi diskret. Simulasi numeriknya
diperbarui setiap 16 ms. Kontinu tidak berarti robot tidak pernah berhenti:
putar balik, verifikasi sensor, dan berhenti di goal memang membutuhkan rem.

## 1. State space

Keadaan gerak planar dapat dinyatakan sebagai:

\[
s_c = (x, y, \theta, v, \omega)
\]

- `x`, `y`: posisi sumbu roda dalam meter, pada wilayah maze yang bebas tabrakan.
- `theta`: orientasi dunia dalam radian, 0 ke timur dan pi/2 ke utara.
- `v`: kecepatan linear badan dalam m/s.
- `omega`: kecepatan putar badan dalam rad/s.

State internal navigasi meliputi sel sekarang, heading, himpunan sel yang
dikunjungi, stack jalur kembali, ingatan dinding, gerakan yang belum
dikonfirmasi selesai, dan status eksekusi. Loop kontrol juga menyimpan
integrasi gyro dan state PID pemusatan. Pose saja tidak mewakili seluruh
ingatan controller.

Nilai FL/FR/DL/DR, encoder, dan gyro adalah **observasi**. Heading gerak
dijaga dengan gyro dan jarak tempuh diukur dengan encoder. Posisi sel
diperbarui dari gerakan yang dijalankan. Pose sebenarnya dan kontak dari
Supervisor hanya digunakan untuk validasi konfigurasi, log, dan penilaian
hasil; keduanya tidak menentukan arah eksplorasi.

## 2. Aksi

Aksi aktuator kontinu adalah `a = (omega_L, omega_R)`, kecepatan sudut roda
kiri dan kanan dalam rad/s. Batas motor masing-masing sekitar
`[-31.416, 31.416] rad/s`. Pustaka menskala kedua perintah bersama jika batas
terlampaui agar rasio kecepatan tetap terjaga.

Dengan jari-jari roda `r = 0.0145 m` dan jarak antar pusat roda
`b = 0.0746 m`, pemetaan dari kecepatan badan adalah:

\[
\omega_L = \frac{v - \omega b/2}{r}, \qquad
\omega_R = \frac{v + \omega b/2}{r}
\]

Aturan memilih abstraksi aksi `F` (maju), `L` (kiri), `R` (kanan), `U`
(putar balik), atau `STOP`. Pustaka gerak menerjemahkannya menjadi profil
kecepatan roda, koreksi arah, dan belokan halus bila pembacaan sensor serta
geometrinya memungkinkan. Kecepatan jelajah pencarian bawaan adalah
`0.22 m/s`.

## 3. S0

Sebelum persiapan start:

\[
s_{c,0} = (0.09\text{ m}, 0.09\text{ m}, \pi/2, 0, 0)
\]

Sel awal `(0, 0)`, heading utara, visited `{(0, 0)}`, stack `[(0, 0)]`, dan
belum ada gerakan tertunda. World memakai sel classic sepanjang `0.18 m`.
`yaw = 0` pada pustaka adalah acuan relatif terhadap orientasi awal utara,
bukan sudut dunia 0 ke timur.

Controller mengasumsikan start classic dengan dinding barat, timur, dan
selatan tertutup. Robot mundur untuk meluruskan badan ke dinding belakang,
kembali ke tengah sel, lalu membaca dinding depan. Kontak belakang pada
tahap persiapan ini disengaja dan tidak dihitung sebagai tabrakan eksplorasi.

## 4. Goal

Untuk world 16x16 yang disediakan:

\[
G = \{(7,7), (7,8), (8,7), (8,8)\}
\]

Untuk ukuran lain, controller memilih satu, dua, atau empat sel tengah
sesuai paritas dimensinya, mengikuti konvensi simulator yang sudah ada.
Goal tidak dibaca dari huruf `G` di file maze.

Aturan berhenti ketika sel yang dilacak mencapai goal. Setelah robot
benar-benar berhenti, pose Supervisor diperiksa: run baru dinyatakan
`success=True` jika status navigasi `goal`, posisi sebenarnya berada di
salah satu sel goal, dan tidak ada kontak dinding selama eksplorasi.

## 5. Rule based

Aturan dievaluasi dalam urutan berikut:

| Prioritas | Kondisi | Aksi |
|---|---|---|
| 1 | Sel sekarang termasuk goal | Berhenti dengan status `goal` |
| 2 | Batas langkah tercapai | Berhenti dengan status `step_limit` |
| 3 | Ada tetangga belum dikunjungi yang tidak diketahui berdinding | Pilih kiri, depan, kanan, lalu belakang relatif arah masuk sel |
| 4 | Tidak ada tetangga baru dan stack memiliki parent | Kembali ke parent |
| 5 | Seluruh cabang habis dan robot sudah kembali ke root | Berhenti dengan status `exhausted` |

Ini merupakan eksplorasi depth-first dengan ingatan kunjungan dan
backtracking. Tidak ada BFS, A*, Q-table, reward, training, atau pembacaan
peta dinding asli untuk memilih aksi. Controller hanya menerima ukuran sel
dan maze dari `customData` world yang dihasilkan generator.

Sisi ambigu (`None`) diverifikasi dengan FL/FR setelah robot berhenti dan
menghadap sisi tersebut. Jika tertutup, arah itu dicatat sebagai dinding
dan pilihan penggantinya juga diverifikasi. Posisi serta stack baru berubah
pada konfirmasi masuk sel berikutnya; verifikasi gagal tidak dianggap
sebagai perpindahan. Dinding depan mendadak memicu rem dan pemilihan ulang.

Dengan pembacaan dinding dan eksekusi gerak yang benar, setiap sel dikunjungi
sebagai cabang baru paling banyak sekali. Backtracking menjaga robot tidak
berputar tanpa akhir pada loop. Batas langkah bawaan `2 * width * height`
menampung penelusuran pohon tersebut. Strategi ini mencari goal, **tidak
menjamin jalur terpendek**.

## Menjalankan

1. Buka Webots R2025a.
2. Pilih **File > Open World**, lalu buka
   `sim/webots/micromouse_webots/worlds/micromouse_rule_based.wbt`.
3. Klik Run. Perhatikan keputusan `RULE` dan hasil `RULE RESULT` di console.
4. Log lengkap berada di
   `controllers/rule_based/run_log.txt` (diabaikan Git).
5. Gunakan Reload World untuk mengulang dari S0.

Uji batch dari root repository, menggunakan PowerShell:

```powershell
$env:MICROMOUSE_QUIT = '1'
& 'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe' --batch --no-rendering --mode=fast --minimize --stdout --stderr '.\sim\webots\micromouse_webots\worlds\micromouse_rule_based.wbt'
```

`MICROMOUSE_QUIT=1` menutup Webots dengan exit code 0 untuk run berhasil
dan 1 untuk kegagalan. Tanpa variabel ini, robot tetap diam sesudah selesai.

| Variabel opsional | Bawaan | Makna |
|---|---|---|
| `MM_RULE_MAX_STEPS` | `2 * width * height` | Batas perpindahan sel, integer positif |
| `MM_RULE_TIMEOUT` | `1800` | Batas waktu simulasi dalam detik, termasuk persiapan |

Batas waktu juga melindungi primitif gerak yang tertahan saat berputar atau
mundur. Kegagalan sensor, konfigurasi, batas waktu, dan gerak akan menghentikan
motor serta mencatat `RULE ERROR` atau hasil dengan `success=False`.

Untuk membuat world dari maze classic lain, jalankan dari root repository:

```powershell
python sim/webots/tools/maze_to_webots.py sim/mazes/mazefiles/classic/001.txt sim/webots/micromouse_webots/worlds/micromouse_rule_based.wbt --controller rule_based
```

Perintah ini mengganti world tugas dengan maze pilihan. Generator menyertakan
ukuran maze melalui `customData`, sehingga controller tidak membutuhkan
`MM_MAZE`. Robot dan controller tugas ini memakai dimensi classic; world
halfsize ditolak karena dimensi gerak dan lebar robot belum sesuai.

## Validasi dan batas hasil

Uji logika dan pemulihan dapat dijalankan tanpa Webots:

```powershell
python -B -m unittest discover -s sim/webots/tests -v
```

Uji mencakup 580 file maze yang tersedia, membandingkan hasil eksplorasi
dengan keterjangkauan goal, serta memeriksa batas langkah, verifikasi
berulang, gerakan yang dibatalkan, dan geometri koreksi lateral. BFS hanya
digunakan sebagai pembanding di pengujian, bukan pada controller tugas.

Uji fisika pada world bawaan di Webots R2025a mencapai sel `(7, 7)` dalam
36 langkah, mengunjungi 30 sel berbeda, dengan nol kontak dinding setelah
persiapan. Waktu simulasi sekitar 93 detik. Jalurnya lebih panjang daripada
jalur optimal 22 langkah karena aturan memprioritaskan eksplorasi kiri.

Perbaikan pustaka gerak mencakup verifikasi setiap exit pengganti serta
koreksi lateral setelah belok melalui lorong keluar yang terbuka. Sebelumnya
koreksi tersebut dapat mundur ke sisi berdinding. Uji regresi flood fill
tetap menghasilkan peta akhir tanpa kesalahan dan lari cepat 22 langkah
yang mencapai goal.

Uji regresi singkat Q-learning (seed 7, tiga episode, batas 80 langkah)
juga selesai tanpa error gerak; run greedy mencapai goal dalam 34 langkah.
Ini merupakan smoke test integrasi, bukan evaluasi konvergensi training.
Batas langkah diuji dengan nilai 1: robot berhenti dengan status
`step_limit`, `success=False`, dan exit code 1. Batas waktu 0.1 detik juga
diuji: controller mencatat timeout, menghentikan motor, dan keluar dengan
exit code 1. Pengujian default setelahnya kembali menghasilkan keberhasilan.

Hasil 580 maze membuktikan logika pada observasi ideal, bukan keandalan fisik
di semua maze. Kurva sensor dan massa masih mengikuti asumsi simulasi lama.
False positive dinding dapat membuat cabang terlewat, sedangkan slip dan
galat sensor dapat mengganggu pelacakan sel. Kontak Supervisor hanya menjadi
metrik penilaian. Kalibrasi dan uji fisika ulang diperlukan saat geometri,
kurva sensor, atau maze diganti.
