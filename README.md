# Panduan Komunikasi LoRa P2P: 2x Raspberry Pi 5 (UART3) & RAK3272S (UART1)

Proyek ini menyediakan implementasi lengkap komunikasi nirkabel **LoRa Point-to-Point (P2P)** antar dua perangkat **Raspberry Pi 5** yang masing-masing menggunakan modul **RAKwireless RAK3272S Breakout Board** (berbasis chip STM32WLE5CC dengan firmware RUI3).

Konfigurasi Port:
* **Raspberry Pi 5:** Menggunakan **UART3** (Pin 24 TX & Pin 21 RX) $\rightarrow$ `/dev/ttyAMA3`
* **RAK3272S:** Menggunakan **UART1** (Header J5: Pin 6 TX1 & Pin 5 RX1)

---

## 📡 Skema Pengkabelan (Wiring Diagram)

Semua koneksi ke modul RAK3272S berada di satu deret header yang sama, yaitu **Header J5**:

| Pin Raspberry Pi 5 (40-Pin Header) | Fungsi Pin Pi 5 | Pin Modul RAK3272S (Header J5) | Fungsi RAK3272S | Keterangan |
| :--- | :--- | :--- | :--- | :--- |
| **Pin 1** (atau Pin 17) | **3.3V Power** | **Header J5 Pin 9** | **3V3 (VDD)** | Tegangan kerja 3.3V (**JANGAN hubungkan ke 5V!**) |
| **Pin 6** (atau Pin 9, 14, 20, 25) | **GND** | **Header J5 Pin 7** | **GND** | Ground bersama |
| **Pin 24** (GPIO 8) | **UART3 TXD** | **Header J5 Pin 5** | **RX1** (UART1 RX) | Jalur TX Pi 5 $\rightarrow$ RX modul RAK |
| **Pin 21** (GPIO 9) | **UART3 RXD** | **Header J5 Pin 6** | **TX1** (UART1 TX) | Jalur RX Pi 5 $\leftarrow$ TX modul RAK |

> 💡 **Kelebihan Menggunakan UART1 (Header J5):**
> Keempat pin (3V3, GND, TX1, RX1) terkumpul rapi di **Header J5 saja**, tidak perlu menyilang atau memakai dua header berbeda.
>
> ⚠️ **PERINGATAN KRUSIAL:**
> Pasang **Antena LoRa** pada konektor IPEX/SMA RAK3272S **SEBELUM** menyalakan modul atau memprogram transmisi untuk mencegah kerusakan amplifier radio internal (PA).

---

## ⚙️ Konfigurasi Raspberry Pi 5 (Aktifkan UART3)

Tambahkan overlay **`uart3-pi5`** ke file konfigurasi bootloader Pi 5:

1. Buka file konfigurasi:
   ```bash
   sudo nano /boot/firmware/config.txt
   ```
2. Pastikan baris berikut ada di bawah blok `[all]`:
   ```ini
   [all]
   dtparam=nvme
   dtparam=pciex1_gen=3
   dtoverlay=uart3-pi5
   ```
3. Simpan (`Ctrl+O`, `Enter`, `Ctrl+X`) lalu reboot:
   ```bash
   sudo reboot
   ```
4. Setelah reboot, pastikan port serial aktif:
   ```bash
   ls -l /dev/ttyAMA3
   ```

---

## 🚀 Langkah Uji Coba

### 1. Uji Komunikasi AT dengan Modul
Jalankan script verifikasi:
```bash
python3 cek.py
```
*Jika sukses, akan langsung muncul:*
```text
Raw Data Diterima : b'\r\nOK\r\n'
Teks Terbaca      : 'OK'
🎉 BERHASIL! RAK3272S merespon 'OK' via UART3 (/dev/ttyAMA3)!
```

### 2. Jalankan Receiver pada Pi 5 Pertama
```bash
python3 rx.py
```

### 3. Jalankan Transmitter pada Pi 5 Kedua
```bash
python3 tx.py
```

Transmitter akan memancarkan data secara berkala (JSON telemetri / 120 sensor parameter) pada frekuensi AS923 (923.2 MHz), dan Receiver akan menangkapnya beserta indikator RSSI dan SNR secara realtime!
