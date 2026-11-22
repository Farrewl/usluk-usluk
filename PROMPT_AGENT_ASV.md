# PROMPT AGENT: Aterkia ASV (usluk-usluk + Web_ASV)

Salin seluruh isi file ini ke agent coding sebagai instruksi awal. Bisa juga disimpan sebagai `AGENTS.md` di root repo.

---

## 0. Peran dan tujuan

Kamu adalah Principal Robotics Software Engineer yang bekerja di dua repo milik saya:

1. **`usluk-usluk`**: backend kapal (Autonomous Surface Vessel), Python (`app/`) + inti C (`core/`, dimuat via ctypes), berjalan di mini-PC Linux (Intel NUC) bersama Pixhawk 6C lewat MAVLink.
2. **`Web_ASV`**: dashboard statis (HTML/CSS/JS) di GitHub Pages: https://farrewl.github.io/Web_ASV/ . Akan dipasangi domain .com sendiri.

Tujuan jangka panjang:
- Kapal otonom yang menang di lomba ASV (acuan: Kontes Kapal Indonesia 2026, sub-kategori ASV).
- Mengurangi Python di jalur kontrol/persepsi. Target: **Rust** untuk inti otonomi, supervisor, dan jaringan; **C++** untuk kamera dan inferensi; Python hanya untuk tooling offline (training, analisis log).
- Mengganti GUI PyQt di kapal dengan **WebSocket ke website**, sehingga dashboard bisa diakses dari mana saja.

Gaya kerja: langsung ke inti, first-principles, selalu sebut trade-off (komputasi, latensi, daya, ketahanan terhadap noise, skenario gagal). Tanpa `.unwrap()`/panic di jalur kritis. Setiap klaim teknis harus punya tes atau pengukuran.

---

## 1. Konteks lomba (acuan: Panduan KKI 2026, ASV)

Aturan edisi berikutnya bisa berubah. **Jadikan aturan misi dan fungsi skor sebagai konfigurasi, bukan hard-code.**

**Misi (berurutan):** start dari dermaga; lewati 10 pasang bola merah-hijau; surface imaging (kotak hijau); underwater imaging (kotak biru); docking di finish dengan menyentuh 3 bola biru (jarak antar pusat 30 cm).

**Aturan penting:**
- Otonom penuh, tanpa kendali manual selama misi.
- Sistem monitoring web real-time **wajib**: posisi, heading, kecepatan, dan gambar. Harus bisa diakses panitia dan peserta lain. Tanpa monitoring aktif, tim tidak boleh bertanding. Koneksi putus saat run dapat membuat panitia menghentikan run.
- Keselamatan: E-stop onboard yang bisa dipicu jarak jauh (≤ 3 detik saat TSI), geofence, **fail-safe berhenti di tempat saat sinyal hilang** (bukan RTL), pelindung propeller.
- Run maksimum 20 menit termasuk 5 menit setup dan 5 menit clearing.
- Spesifikasi minimum prototipe: panjang ≥ 60 cm, lebar ≥ 20 cm, berat ≥ 7 kg termasuk baterai.

**Skor (NM = 20 hanya bila navigasi, imaging, dan docking semuanya berhasil):**
- NM = 20: `S = 100 × (40 − P) / T + IM + DC`. `P = 5 × k` (k = jumlah pelanggaran). `T` = waktu tempuh (detik). `IM` ≤ 10 (kualitas gambar permukaan + bawah air, dinilai dari bentuk kotak dan warna). `DC` = 5 × jumlah bola biru tersentuh **saat kontak pertama** (maks 15).
- NM < 20: `S = 10 × (2·NM − P) / 900` ≤ 0,44. Tebing besar: menyelesaikan seluruh misi bernilai sekitar 100×.
- Pelanggaran > 5 kali: kapal dikembalikan ke START.
- Pada T ≈ 90 s: 1 detik ≈ 0,5 poin; 1 sentuhan ≈ 5,6 poin ≈ 11 s; docking penuh ≈ 30 s; imaging penuh ≈ 20 s.
- **Prioritas desain: P(selesai) > docking dan imaging > waktu > sentuhan.**

Fungsi tujuan yang harus dioptimalkan lewat simulasi:
`E[S] = P_selesai(v) · (100·(40 − 5·E[k]) / T(v) + E[IM] + E[DC])`

---

## 2. Kondisi kode saat ini (baca dulu, jangan berasumsi)

**`usluk-usluk`:**
- `main.py` (GUI PyQt5), `app/navigator.py` (loop 30 Hz tunggal: kamera, YOLO async, state machine, kendali, emit ke GUI), `app/output_link.py` (SET_ATTITUDE_TARGET: thrust + quaternion yaw), `app/yolo_onnx.py` (YOLOv8 ONNX via `cv2.dnn`, CPU), `app/yolo_worker.py`, `app/detection_validation.py`, `app/safety.py`, `app/monitor_export.py` (file bridge ke `data/monitor/`), `app/telemetry_snapshot.py`, `app/settings.py` (+ `config/tuning_params.json`, `config/site.json`, `config/trackA|B.json`).
- `core/src/*.c`: `nav_math`, `gate_vision`, `fuzzy`, `state_machine` (15 state), `control_filters` (PID/komplementer/EKF), `thruster_mixer`, `mavlink_bridge`, `ecu_link` (STM32, belum dipakai di navigator), `arbitrator`, `manual_control`, `rc_decode`, `mode_manager`, `avoidance`, `gamepad_input.cpp`. Build: `make -C core linux`.
- Ekspor monitoring (sudah ada): `data/monitor/telemetry.json` (skema v1, termasuk `params` berisi semua atribut Config), `frame.jpg`, `frame_bawah.jpg`, `photos.json` (geo-tag), `events.jsonl`. Ditulis atomik (`.tmp` lalu `os.replace`).

**`Web_ASV`:** `index.html` (login), `admin.html`, `user.html`, `script.js`, `style.css`, `robot_bridge.py` (stub serial). Semua statis.

Catatan: salinan Windows dan Linux repo kapal sudah divergen (`navigator`, `camera_manager`, `qgc_offboard`, `settings`). **Tetapkan satu sumber kebenaran (Linux) sebelum mengubah apa pun.**

---

## 3. Temuan baseline: VERIFIKASI DENGAN TES DULU, baru perbaiki

Temuan ini berasal dari pembacaan statis. Tulis tes yang gagal terlebih dulu. Jika ada temuan yang ternyata salah, laporkan dan jangan memperbaikinya.

1. **`core/src/fuzzy.c`: gain gate kemungkinan selalu 0.**
   - `fuzzy_trapmf` mengembalikan 0 untuk `x >= d`, padahal MF bahu kanan (`{0.7,0.8,1.0,1.0}` pada jarak, `{110,150,320,320}` pada error piksel) seharusnya tetap 1. Bahu kiri (`a == b`) juga harus 1 pada `x <= b`.
   - Semesta jarak MF hanya 0..1 m, tetapi `navigator.py` memanggilnya dengan jarak yang di-clamp ke 0..2 m, dan target gate hanya dipilih bila jarak ≥ `GATE_PASS_DISTANCE_M` (1,2 m).
   - Akibat: `p_gain = 0`, koreksi visi 0, `target_yaw` hanya menahan heading saat ini dan mengabaikan bearing WP selama gate terlihat.
   - Tes: `fuzzy_gate_p_gain(2.0, 100.0)` tidak boleh 0.0. Tentukan semesta jarak dalam meter secara eksplisit, dokumentasikan, dan jangan ubah nilai singleton tanpa data. Error piksel harus relatif terhadap lebar frame, bukan konstanta 320.
2. **EKF dan filter komplementer melingkar** (`_get_gate_nav_yaw`): `gyro_rate` dihitung dari turunan yaw hasil ukur yang sama, lalu dipakai untuk `predict()`. Tidak menambah informasi, hanya lag. Gunakan `ATTITUDE.yawspeed` (atau pesan IMU) sebagai input gyro sungguhan, atau hapus kedua filter itu.
3. **Loop tunggal terlalu terikat.** Kamera, emit GUI, kontrol, dan stream setpoint berbagi satu loop. Ada `time.sleep(0.2)` di `TAKE_PHOTO` dan `prepare_offboard(3.0)` yang memblokir. Banyak `except Exception: pass` yang menelan error.
4. **Failsafe tidak sesuai aturan.** `SafetyManager` memakai `request_rtl()` (kapal malah bergerak). Jalur manual RC/QGC/gamepad masih aktif. KILL hanya tombol GUI → disarm lewat companion. `ecu_link` (STM32) tidak terpakai.
5. **Misi berbasis waypoint GPS per venue** (`trackA/B.json` berisi koordinat venue lama, leg foto hard-code 6 dan 8). Mission KKI berbeda dan venue baru tidak punya koordinat. Misi harus berbasis geometri relatif yang bisa dikonfigurasi.
6. **Asosiasi gate rapuh:** `GATE_AREA_SIMILARITY_RATIO = 0.0` mematikan cek ukuran, dan tidak ada constraint sisi merah/hijau sehingga bola dari gerbang berbeda bisa terpasang salah.
7. **Kalibrasi kamera:** `FOCAL_LENGTH_PX = 310` pada frame 1280 px menyiratkan HFOV ≈ 128°. Verifikasi bahwa f itu hasil kalibrasi pada resolusi yang sama. Belum ada koreksi distorsi.
8. **Avoidance:** ambang lebar ≥ 25% frame praktis baru aktif saat nyaris menabrak.
9. **Inferensi:** `cv2.dnn` backend CPU. Ganti ke OpenVINO/ONNX Runtime.
10. **Mix firmware:** kode memakai mode ArduRover (GUIDED = 15, RTL = 11) sementara ada spesifikasi parameter PX4. Pilih satu (rekomendasi: ArduRover frame motorboat, kecuali sudah terkunci di PX4; **tanyakan ke saya**).

---

## 4. Arsitektur target

```
 Kamera(V4L2) -> [perception  (C++)] --Detections(t_capture)--> shm/Zenoh --+
                                                                            v
 Pixhawk <--MAVLink--> [autonomy_core (Rust)]  FSM misi, estimator, guidance,
   (inner loop,          supervisor keselamatan, watchdog, control allocation
   E-stop, failsafe)             |
                                 +--state/event--> [telemetry_gateway (Rust)] --WSS (outbound)--> [relay] <--WSS-- Browser
```

Prinsip:
- **Isolasi kegagalan:** `perception` atau `telemetry_gateway` mati tidak boleh menghentikan `autonomy_core`. Tiap proses di-supervise (systemd), dengan watchdog. Perintah basi → kirim HOLD / thrust 0.
- **Setpoint streamer terpisah** (thread/proses prioritas tinggi, 20-50 Hz) yang selalu mengirim perintah terbaru dan menjatuhkan ke thrust 0 bila perintah basi lebih dari ambang.
- **Jalur keselamatan tidak boleh lewat companion:** E-stop dan failsafe via RC/Pixhawk (ArduPilot: opsi RC Motor E-Stop; `FS_ACTION = Hold`) plus STM32 bila terpasang.
- **Mission sebagai data:** urutan task (gerbang, imaging, docking), geometri course, dan fungsi skor berada di file konfigurasi.
- **Rust (`forbid(unsafe_code)` di inti):** FSM dengan `enum` + `match` eksaustif (typestate bila perlu), estimator, guidance/kontrol, MAVLink (`mavlink` crate), gateway (`tokio`, `tokio-tungstenite`), relay (`axum`), serialisasi (`serde`), log (`mcap`), matematika (`nalgebra`), solver (`clarabel`/`osqp` bila MPC dipakai).
- **C++:** capture V4L2 (mmap, libjpeg-turbo), inferensi (OpenVINO atau ONNX Runtime; INT8), OpenCV.
- **IPC:** shared memory ring (`iceoryx2`) atau Zenoh; struct POD berversi (`#[repr(C)]`) atau FlatBuffers/Protobuf. Hindari FFI yang menyebar.
- **Migrasi bertahap (strangler):** jangan menulis ulang sekaligus. Jadikan `core/*.c` beserta tes lintas-bahasa yang ada sebagai oracle golden-vector. Python lama tetap jalan sampai versi baru lulus paritas.
- Catatan jujur: bahasa saja tidak menaikkan skor. Bottleneck sebenarnya: inferensi CPU, loop yang saling terikat, dan GUI.

---

## 5. Konektivitas: Gateway ↔ Relay ↔ Web (PRIORITAS PERTAMA)

### 5.1 Topologi (dan alasannya)
- GitHub Pages hanya hosting statis; **tidak bisa** jadi server WebSocket. Dashboard tetap di Pages (dengan domain .com custom + HTTPS). Server WebSocket adalah **relay** terpisah yang selalu nyala, di subdomain (mis. `wss://relay.<domain>`).
- Kapal **dial-out** ke relay (modem seluler biasanya di belakang CGNAT, tidak menerima koneksi masuk). Browser juga menyambung ke relay. Browser tidak pernah menyambung langsung ke kapal.
- Relay ditulis dengan Rust + `axum`, kecil dan hampir stateless (hanya cache snapshot terakhir untuk klien yang baru bergabung). Deploy di Fly.io (region Singapura) atau VPS dengan Caddy (TLS otomatis), container Docker, endpoint `/healthz`.
- Browser halaman HTTPS tidak boleh membuka `ws://` non-TLS (mixed content). Untuk tuning lokal di lapangan, sajikan halaman terpisah dari NUC; jangan memaksa dari domain publik.

### 5.2 Protokol
Mulai dengan JSON untuk teks (mudah di-debug). Sediakan flag untuk beralih ke MessagePack bila bandwidth jadi masalah. Semua pesan memakai envelope:
`{ "v": 1, "kind": "...", "seq": N, "t_unix": ..., "boat_id": "..." }`

| kind | Arah | Isi |
|---|---|---|
| `hello` | semua → relay | role, versi, `resume_seq` |
| `telemetry` | kapal → relay | 5-10 Hz: pose, motion, power, link, mission, ringkasan vision (ikuti skema v1 `telemetry.json`, **tanpa** `params`) |
| `params_snapshot` | kapal → relay | `{rev, params}` saat connect dan saat berubah (bukan tiap tik) |
| `params_set` / `params_ack` | admin ↔ kapal | `{req_id, key, value}` / `{req_id, ok, applied, reason}` |
| `event` + `event_ack` | kapal → relay | **andal**: ada `id`, retry, ACK. Dipakai untuk snapshot imaging, perubahan state, failsafe |
| `media` (biner) | kapal → relay | header: 1 byte channel (0 nav, 1 bawah, 2 snapshot-surface, 3 snapshot-under) + 4 byte seq + 8 byte timestamp + byte JPEG/WebP. **Bukan base64** |
| `ping`/`pong` | dua arah | 1 Hz, deteksi basi |

Aturan kualitas layanan:
- Telemetri dan video: **buang yang lama** (latest wins). Event: andal dan berurutan, buffer terbatas di disk (store-and-forward) bila link putus.
- Reconnect dengan exponential backoff + jitter. Relay mengirim snapshot terakhir ke klien baru.
- Anggaran bandwidth: telemetri ≈ 2-5 KB/s; video ≈ 0,5-1,5 Mbps (640×360, ~5 fps, kualitas adaptif).
- **Gateway tidak boleh memblokir inti otonomi:** state terbaru lewat `tokio::sync::watch`, event lewat `mpsc` terbatas dengan `try_send`. Tes wajib: matikan gateway di tengah misi dan buktikan timing loop kontrol tidak berubah.

### 5.3 Parameter kapal
- **Baca:** web menampilkan seluruh parameter (tabel dengan pencarian, selisih terhadap default, kelompok per tab sesuai `settings_dialog.py`).
- **Tulis:** hanya bila **semua** syarat ini terpenuhi: (a) `competition_mode = false`; (b) kapal HOLD atau disarm; (c) key ada di whitelist (`TUNING_PARAM_KEYS`) dengan rentang tervalidasi (ambil dari `settings_dialog.py`); (d) token admin valid; (e) rate limit (maks 5/detik); (f) gateway membalas `params_ack` dengan nilai yang benar-benar diterapkan. Setiap perubahan dicatat di `data/monitor/param_audit.jsonl`. Penyimpanan permanen lewat `params_commit` eksplisit (pakai `save_params`), bukan otomatis.
- `competition_mode = true` menonaktifkan seluruh penulisan parameter dan jalur manual (RC/QGC/gamepad), dan memaksa failsafe = HOLD.

### 5.4 Keamanan
- Token terpisah: `PUBLISHER_TOKEN` (kapal), `ADMIN_TOKEN` (tulis parameter), via env di relay. Kirim di pesan `hello` pertama atau sub-protocol, **bukan di query URL** (masuk log).
- Viewer publik read-only tanpa login (aturan lomba: panitia dan peserta lain harus bisa melihat). Allowlist `Origin` untuk browser. Catatan: Origin bisa dipalsukan oleh klien non-browser, jadi token tetap wajib untuk semua hal yang menulis.
- **Kredensial `aterkia / asv123` sudah terpajang publik. Anggap bocor, ganti, dan jangan simpan rahasia apa pun di JS statis.**
- Tidak ada perintah gerak (thrust/yaw/arm) lewat internet.

### 5.5 Perubahan di Web_ASV
- Hapus login hardcode dan gating `localStorage`. Halaman `/` = viewer publik read-only. `/admin` meminta token yang divalidasi **di relay** (UI saja tidak cukup); token di `sessionStorage`.
- Ganti `Math.random()` dengan data nyata. Pill status berdasarkan keadaan WS **dan** kesegaran data (umur > 2 s = STALE).
- Tile: roll, pitch, yaw, SOG, heading, tegangan, koordinat, COG (sesuai tile yang sudah ada); peta (Leaflet/OSM) dengan trajektori; panel video NAV/BAWAH dari frame biner (`Blob` → `URL.createObjectURL`); galeri foto dengan geo-tag (`coord_dd`, `coord_dm`, hari/tanggal/jam, SOG, COG); log event; tabel parameter.
- Vanilla ES modules tanpa framework supaya tetap sederhana di GitHub Pages. Responsif. Reconnect otomatis.

### 5.6 Jalur bertahap (jangan menunggu rewrite)
- **Fase A (cepat):** `ws_bridge` (Rust) yang membaca `data/monitor/telemetry.json` (5 Hz), `frame.jpg`, `frame_bawah.jpg`, `photos.json`, `events.jsonl` lalu mendorongnya ke relay. **Tanpa mengubah navigator.** Tangani JSON parsial/file belum ada (`FileNotFoundError`).
- **Fase B:** gateway natif yang membaca state dari `autonomy_core` lewat IPC, bukan lewat file.

### 5.7 Domain dan deploy
- Pages: file `CNAME`, record DNS sesuai dokumentasi resmi GitHub Pages untuk custom domain, aktifkan Enforce HTTPS.
- Relay: record A/AAAA `relay.<domain>` ke server, TLS otomatis (Caddy). Variabel konfigurasi (token, origin) lewat env.

---

## 6. Riset dan algoritma (implementasi bertahap, setiap item perlu bukti pengukuran)

**A. Persepsi tahan gangguan air**
- Kalibrasi intrinsik dan distorsi (OpenCV), lalu undistort **hanya titik pusat deteksi**.
- Kunci exposure, gain, dan white balance (auto-exposure mengubah warna dan merusak validasi HSV serta skor warna imaging). Rekomendasikan polarizing filter di lensa.
- Crop pita horizon berdasarkan roll/pitch IMU agar resolusi efektif bola jauh naik tanpa menambah komputasi.
- Detektor: YOLO nano atau RT-DETR/RTMDet INT8 via OpenVINO, dengan set kalibrasi dari citra air nyata. Perhatikan lisensi (Ultralytics AGPL). Evaluasi dengan *leave-one-venue-out*.
- Timestamp capture untuk setiap frame; sinkronkan dengan jam MAVLink.

**B. Estimator: peta landmark lokal (bukan path planning)**
- Bearing dunia: `β = ψ(t_capture) + atan((u − c_x)/f_x)` (yaw diinterpolasi pada waktu capture).
- Dua estimasi jarak yang difusikan (inverse-variance): (1) `r = f·D/w_px` dengan diameter bola yang diketahui (lebih tahan sudut pandang dibanding lebar pasangan); (2) bidang air: `r = h / tan(θ_cam + θ_imu + atan((v − c_y)/f_y))`.
- Simpan posisi bola di frame lokal (smoother fixed-lag, ≤ 25 landmark) dan cocokkan ke template geometri course dari juknis (robust fit/RANSAC). Hasilnya: asosiasi merah/hijau yang benar, gerbang bisa "ditebak" saat glare/oklusi, dan lepas dari waypoint GPS per venue.
- Odometri visual di permukaan air tidak layak (tekstur rendah, pantulan). Pakai IMU + GPS-velocity untuk dead-reckoning di antara observasi.

**C. Guidance dan kontrol**
- Identifikasi model manuver (Nomoto orde 1/2 atau Fossen 3-DOF) dari data uji zigzag/turning.
- Guidance ALOS dengan integral untuk angin/arus: `ψ_d = χ_p − atan((y_e + σ·ŷ_int)/Δ)`, `χ_p` = arah garis antar pusat gerbang dari peta landmark.
- Kontrol kecepatan (bukan thrust terbuka). Jadwal kecepatan: `u_d = min(u_max, √(a_lat/κ), g(ketidakpastian_persepsi))`.
- MPC (horizon ≈ 1-2 s, constraint koridor gerbang) hanya setelah ALOS + feedforward terbukti kurang, terutama untuk docking. Tanpa model yang baik MPC tidak lebih baik.
- Dengan 2 azimuth thruster, rancang **control allocation** `(T1, α1, T2, α2) → (surge, sway, yaw)` untuk docking sejajar dermaga.

**D. Docking dan imaging**
- DC dihitung pada kontak pertama: pendekatan harus sejajar dermaga agar 3 bola tersentuh sekaligus. Cek orientasi sandar di Gambar 2 panduan. Pertimbangkan fender datar > 60 cm (co-design mekanik).
- Imaging: ambil burst, pilih frame terbaik (ketajaman Laplacian, kotak utuh dalam frame, rasio mendekati persegi, warna sesuai), kirim lewat kanal event andal dengan ACK. Putuskan *stop-and-shoot* atau sambil jalan berdasarkan hitungan biaya waktu vs poin.

**E. Kecepatan sadar-risiko:** sweep kecepatan per segmen, margin clearance, dan titik mulai melambat untuk memaksimalkan `E[S]` (bagian 1).

**F. Validasi**
- Log semua input mentah (frame, MAVLink, timestamp) dalam MCAP; replay deterministik sebagai regression test.
- Simulasi: model Fossen 3-DOF + angin/arus + model kamera sintetis dengan dropout glare; ribuan run Monte-Carlo untuk `P(NM=20)`, distribusi `T`, dan kecepatan optimal. ArduPilot SITL (`sim_vehicle.py -v Rover -f motorboat`) untuk jalur firmware.
- Metrik: `P(NM=20)`, `T` p50/p90, jumlah sentuhan, latensi capture→aktuasi p99, FP per menit.

---

## 7. Fase kerja, deliverable, kriteria lulus

Kerjakan **Fase 0-2 sekarang**. Fase 3 dan seterusnya hanya setelah saya konfirmasi.

**Fase 0: baseline dan keselamatan**
- Tes untuk temuan 1-2; perbaiki bila terbukti.
- `competition_mode` (menonaktifkan jalur manual dan penulisan parameter; failsafe = HOLD, bukan RTL).
- Logging MCAP dasar. Satu sumber kebenaran repo (Linux).
- Lulus: tes lolos di CI; `competition_mode` terbukti menolak `params_set` dan input manual.

**Fase 1: telemetri live ke web**
- `relay` (Rust/axum) + `ws_bridge` (Rust, Fase A) + perubahan Web_ASV (bagian 5.5), read-only termasuk parameter.
- Lulus: latensi telemetri p95 < 500 ms lewat seluler; reconnect < 10 s setelah link putus; gambar biner tampil ≥ 5 fps pada 640×360; viewer publik tanpa login; tidak ada kredensial di repo web; dashboard menandai data STALE.

**Fase 2: penulisan parameter (mode perawatan)**
- Sesuai bagian 5.3 dan 5.4 (whitelist, rentang, token, audit, ack, `params_commit`).
- Lulus: nilai di luar rentang ditolak dengan alasan; penulisan saat kapal tidak HOLD ditolak; audit log lengkap; tes bahwa `competition_mode` menolak semua penulisan.

**Fase 3 (setelah konfirmasi):** arsitektur tiga proses, setpoint streamer terpisah, port bertahap `core/*.c` ke Rust dengan golden-vector.
**Fase 4:** persepsi (6A) dan estimator (6B).
**Fase 5:** guidance, kontrol, dan control allocation (6C).
**Fase 6:** docking dan imaging (6D).
**Fase 7:** simulasi Monte-Carlo dan tuning ke `E[S]` (6E-6F).

---

## 8. Aturan kerja

- Satu perubahan, satu PR/commit kecil, dengan tes. Jangan hapus kode Python lama sebelum versi baru lulus paritas.
- Jangan ubah perilaku keselamatan (failsafe, kill, arming) tanpa menandainya jelas di deskripsi PR dan menyediakan tes.
- Jalur kritis: tanpa `.unwrap()`/`panic!`, tanpa alokasi di loop real-time (prealokasi), error eksplisit.
- Semua angka tuning masuk konfigurasi, bukan konstanta tersembunyi. Dokumentasikan satuan.
- Jangan menelan exception diam-diam. Log dengan throttle.
- Jangan menaruh rahasia (token, password) di repo atau JS statis.
- Bila spesifikasi ambigu atau butuh data lapangan, **tanya saya, jangan menebak**.
- Setiap akhir fase: tulis ringkasan temuan, keputusan, trade-off, dan hal yang belum terverifikasi.

---

## 9. Konfirmasi yang saya butuhkan sebelum Fase 3+

1. Firmware Pixhawk: ArduRover atau PX4?
2. Lebar gerbang dan diameter bola sebenarnya (juknis ASV), serta tata letak course.
3. Orientasi docking (Gambar 2 panduan) dan apakah fender datar boleh.
4. Model kamera dan data kalibrasi intrinsik; apakah ada polarizing filter.
5. Konfigurasi thruster dan cara Pixhawk mengendalikan azimuth/servo (custom mixing, script, atau output langsung).
6. Apakah STM32 ECU terpasang dan terhubung ke NUC?
7. Edisi lomba berikutnya yang ditargetkan (aturan bisa berbeda).
8. Nama domain .com dan pilihan hosting relay (Fly.io atau VPS).
