# Panduan Komunikasi LoRa P2P Ultra-Fast (Sub-Detik)
## 2x Raspberry Pi 5 & RAKwireless RAK3272S

Proyek ini menyediakan implementasi komunikasi nirkabel **LoRa Point-to-Point (P2P) Kecepatan Tinggi (Sub-Detik)** antar dua perangkat **Raspberry Pi 5** menggunakan modul **RAK3272S** (chip STM32WLE5CC RUI3).

---

### ⚡ Optimalisasi Kecepatan Sub-Detik:
1. **Bandwidth Maksimum (500 kHz):** 4x lebih cepat dari LoRa standar 125 kHz.
2. **Time-on-Air (ToA) Singkat:** Hanya **~15 - 25 milidetik** per paket di udara!
3. **Throughput Tinggi:** Mampu mengirim & menerima **5 hingga 20 paket per detik (5 - 20 Hz)**.
4. **Latency Real-time:** Receiver menampilkan statistik kecepatan (pkt/s), RSSI, dan SNR secara live.

---

### 📡 Arsitektur Node & Pengkabelan

#### 1. Node Transmitter (TX) — Menggunakan UART1 (Header J5)
*Sesuai jalur printed board PCB Anda:*

| Dari Raspberry Pi 5 (TX) | Ke Modul RAK3272S (TX) | Keterangan |
| :--- | :--- | :--- |
| **Pin 24 (TXD3)** | $\rightarrow$ **Header J5 Pin 5 (RX1)** | Jalur TX Pi $\rightarrow$ RX1 RAK |
| **Pin 21 (RXD3)** | $\leftarrow$ **Header J5 Pin 6 (TX1)** | Jalur RX Pi $\leftarrow$ TX1 RAK |
| **Pin 1 (3.3V)** | $\rightarrow$ **Header J5 Pin 9 (3V3)** | Tegangan 3.3V |
| **Pin 6 (GND)** | $\rightarrow$ **Header J5 Pin 7 (GND)** | Ground bersama |

#### 2. Node Receiver (RX) — Menggunakan UART2 (Header J4)
*Port default pabrik modul RAK (Header J4):*

| Dari Raspberry Pi (RX) | Ke Modul RAK3272S (RX) | Keterangan |
| :--- | :--- | :--- |
| **Pin 8 (GPIO 14 - TXD0)** | $\rightarrow$ **Header J4 Pin 8 (RX2)** | TX Pi $\rightarrow$ RX2 RAK |
| **Pin 10 (GPIO 15 - RXD0)** | $\leftarrow$ **Header J4 Pin 7 (TX2)** | RX Pi $\leftarrow$ TX2 RAK |
| **Pin 1 (3.3V)** | $\rightarrow$ **Header J5 Pin 9 (3V3)** | Tegangan 3.3V |
| **Pin 6 (GND)** | $\rightarrow$ **Header J4 Pin 4 (GND)** | Ground bersama |

*(Catatan: Jika Pi Receiver juga menggunakan port lain seperti `/dev/ttyAMA3` atau `/dev/ttyUSB0`, cukup sebutkan nama port saat menjalankan script).*

---

### ⚙️ Parameter Radio LoRa P2P Ultra-Fast

* **Frekuensi:** `923.2 MHz` (Band AS923)
* **Spreading Factor:** `SF7`
* **Bandwidth:** `500 kHz` (`P2P_BW = 2`, Mode Tercepat!)
* **Coding Rate:** `4/5` (`P2P_CR = 0`)
* **Preamble:** `8`
* **Tx Power:** `20 dBm`

---

### 🚀 Cara Menjalankan

#### 1. Tarik Pembaruan di Kedua Raspberry Pi:
```bash
cd ~/idrisLoRa
git pull origin main
```

#### 2. Jalankan Receiver Terlebih Dahulu:
Di Raspberry Pi **Receiver**:
```bash
python3 rx.py
```
*(Atau jika port Receiver berbeda: `python3 rx.py /dev/ttyAMA0` atau `python3 rx.py /dev/ttyAMA3`)*

#### 3. Jalankan Transmitter dengan Pilihan Kecepatan:
Di Raspberry Pi **Transmitter**:

- **Pilihan A: Kecepatan 5 Paket/detik (Interval 0.2 detik - Rekomendasi):**
  ```bash
  python3 tx.py /dev/ttyAMA3 0.2
  ```

- **Pilihan B: Kecepatan 10 Paket/detik (Interval 0.1 detik / 100 ms):**
  ```bash
  python3 tx.py /dev/ttyAMA3 0.1
  ```

- **Pilihan C: Kecepatan 20 Paket/detik (Interval 50 ms):**
  ```bash
  python3 tx.py /dev/ttyAMA3 0.05
  ```

- **Pilihan D: Mode Streaming Maksimum (Zero Delay / 0 jeda):**
  ```bash
  python3 tx.py /dev/ttyAMA3 0
  ```
  *(Modul akan langsung menembakkan paket berikutnya begitu paket sebelumnya selesai terkirim!)*
