# Panduan Komunikasi LoRa P2P: 2x Raspberry Pi 5 & RAK3272S

Proyek ini menyediakan implementasi komunikasi nirkabel **LoRa Point-to-Point (P2P)** antar dua perangkat **Raspberry Pi 5** dan modul **RAKwireless RAK3272S** (berbasis chip STM32WLE5CC dengan firmware RUI3).

### Arsitektur Node:
1. **Node Transmitter (TX):**
   * **Raspberry Pi 5 :** Menggunakan **UART3** (Pin 24 TX & Pin 21 RX) $\rightarrow$ `/dev/ttyAMA3`
   * **RAK3272S       :** Menggunakan **UART1** (Header J5: Pin 5 RX1 & Pin 6 TX1)
   * *(Sesuai desain printed board PCB)*
2. **Node Receiver (RX):**
   * **Raspberry Pi 5 :** Menggunakan **UART0** (Pin 8 TX & Pin 10 RX) $\rightarrow$ `/dev/ttyAMA0` *(atau port serial lain)*
   * **RAK3272S       :** Menggunakan **UART2** (Header J4: Pin 7 TX2 & Pin 8 RX2)
   * *(Menggunakan port default pabrik RUI3)*

---

## 📡 Skema Pengkabelan (Wiring Diagram)

### 1. Node Transmitter (TX) — Menggunakan UART1 (Header J5)

| Pin Raspberry Pi 5 (40-Pin Header) | Fungsi Pin Pi 5 | Pin Modul RAK3272S (Header J5) | Fungsi RAK3272S | Keterangan |
| :--- | :--- | :--- | :--- | :--- |
| **Pin 1** (atau Pin 17) | **3.3V Power** | **Header J5 Pin 9** | **3V3** | Tegangan 3.3V (**JANGAN hubungkan ke 5V!**) |
| **Pin 6** (atau Pin 9, 14, 20) | **GND** | **Header J5 Pin 7** | **GND** | Ground bersama |
| **Pin 24** (GPIO 8) | **UART3 TXD** | **Header J5 Pin 5** | **RX1** (UART1 RX) | TX Pi 5 $\rightarrow$ RX1 RAK |
| **Pin 21** (GPIO 9) | **UART3 RXD** | **Header J5 Pin 6** | **TX1** (UART1 TX) | RX Pi 5 $\leftarrow$ TX1 RAK |

### 2. Node Receiver (RX) — Menggunakan UART2 (Header J4)

| Pin Raspberry Pi 5 (40-Pin Header) | Fungsi Pin Pi 5 | Pin Modul RAK3272S (Header J4) | Fungsi RAK3272S | Keterangan |
| :--- | :--- | :--- | :--- | :--- |
| **Pin 1** (atau Pin 17) | **3.3V Power** | **Header J5 Pin 9** | **3V3** | Tegangan 3.3V |
| **Pin 6** (atau Pin 9, 14, 20) | **GND** | **Header J4 Pin 4** | **GND** | Ground bersama |
| **Pin 8** (GPIO 14) | **UART0 TXD** | **Header J4 Pin 8** | **RX2** (UART2 RX) | TX Pi 5 $\rightarrow$ RX2 RAK |
| **Pin 10** (GPIO 15) | **UART0 RXD** | **Header J4 Pin 7** | **TX2** (UART2 TX) | RX Pi 5 $\leftarrow$ TX2 RAK |

*(Catatan: Jika pada Pi Receiver Anda juga menggunakan UART3, cukup jalankan `python3 rx.py /dev/ttyAMA3`).*

> ⚠️ **PERINGATAN ANTENA:**
> Pasang **Antena LoRa** pada konektor IPEX/SMA RAK3272S **SEBELUM** menyalakan modul atau memprogram transmisi untuk mencegah kerusakan amplifier radio internal (PA).

---

## ⚙️ Parameter Radio LoRa P2P (Sinkron Antara TX & RX)

* **Frekuensi:** `923.2 MHz` (Band AS923 Indonesia)
* **Spreading Factor:** `SF7`
* **Bandwidth:** `125 kHz` (Kode: `0`)
* **Coding Rate:** `4/5` (Kode: `0`)
* **Preamble Length:** `8`
* **Tx Power:** `20 dBm`

---

## 🚀 Cara Menjalankan

### Langkah 1: Pengecekan AT pada Masing-Masing Node

- **Pada Pi Transmitter (TX):**
  ```bash
  python3 cek.py /dev/ttyAMA3
  ```
- **Pada Pi Receiver (RX):**
  ```bash
  python3 cek.py /dev/ttyAMA0
  ```

### Langkah 2: Jalankan Receiver Terlebih Dahulu
Pada Raspberry Pi **Receiver**:
```bash
python3 rx.py
```
*(Atau tentukan port jika bukan ttyAMA0, misal: `python3 rx.py /dev/ttyAMA0`)*

### Langkah 3: Jalankan Transmitter
Pada Raspberry Pi **Transmitter**:
```bash
python3 tx.py
```
*(Atau tentukan port jika berbeda, misal: `python3 tx.py /dev/ttyAMA3`)*

Transmitter akan memancarkan telemetri JSON secara berkala setiap 5 detik, dan Receiver akan menangkapnya secara realtime beserta indikator kualitas sinyal **RSSI** dan **SNR**!
