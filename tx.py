#!/usr/bin/env python3
"""
=============================================================================
RAK3272S LoRa P2P Transmitter untuk Raspberry Pi 5
=============================================================================
Penulis : Pasangan Pair-Programming Antigravity & User
Deskripsi: Script pengirim data LoRa Point-to-Point (P2P) menggunakan modul
          RAKwireless RAK3272S (STM32WLE5CC RUI3) yang terhubung ke pin UART
          Raspberry Pi 5 (/dev/ttyAMA0).
=============================================================================
"""

import serial
import time
import struct
import json
import sys

# ================= Konfigurasi Port & Radio =================
# Pada Pi 5: GPIO 14 (TX) & 15 (RX) adalah /dev/ttyAMA0 setelah dtparam=uart0=on
SERIAL_PORT = '/dev/ttyAMA0'
BAUDRATE    = 115200

# Pengaturan Radio LoRa P2P (WAJIB PERSIS SAMA DENGAN RECEIVER)
# Frekuensi: 923.2 MHz (Alokasi LoRa Indonesia AS923)
# SF: 7, Bandwidth: 0 (125 kHz), Coding Rate: 0 (4/5), Preamble: 8, TxPower: 20 dBm
P2P_FREQ     = 923200000  # 923.2 MHz
P2P_SF       = 7          # SF7 (Spread Factor 6-12)
P2P_BW       = 0          # 0: 125kHz, 1: 250kHz, 2: 500kHz
P2P_CR       = 0          # 0: 4/5, 1: 4/6, 2: 4/7, 3: 4/8
P2P_PREAMBLE = 8          # Preamble length (default 8)
P2P_TXPOWER  = 20         # Max 22 dBm

TX_INTERVAL  = 5          # Interval pengiriman paket (detik)
NODE_ID      = 1          # ID Node Pengirim

# Pilihan Mode Payload: 'TEXT', 'JSON', atau 'BINARY_120' (120 sensor parameters)
PAYLOAD_MODE = 'JSON'     # 'JSON' / 'TEXT' / 'BINARY_120'
# =============================================================


def send_at_command(ser, cmd, timeout=1.5, expected=None):
    """
    Mengirim perintah AT dan membaca respon hingga timeout atau expected respon ditemukan.
    """
    ser.reset_input_buffer()
    ser.write((cmd + '\r\n').encode('utf-8'))
    
    start_time = time.time()
    responses = []
    
    while (time.time() - start_time) < timeout:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                responses.append(line)
                if expected and expected in line:
                    break
                if not expected and ("OK" in line or "AT_ERROR" in line or "AT_PARAM_ERROR" in line):
                    break
        time.sleep(0.02)
        
    return responses


def init_rak3272s_tx(port, baudrate):
    """
    Inisialisasi modul RAK3272S:
    1. Tes AT
    2. Cek/set mode kerja ke LoRa P2P (AT+NWM=0)
    3. Atur parameter RF P2P
    4. Pastikan receiver disabled (AT+PRECV=0) agar siap mengirim
    """
    print(f"\n{'='*60}")
    print(f"🚀 INISIALISASI RAK3272S TRANSMITTER (Port: {port})...")
    print(f"{'='*60}")
    
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1)
    except Exception as e:
        print(f"❌ GAGAL membuka port serial {port}!")
        print(f"   Detail Error : {e}")
        print(f"   Solusi Cepat :")
        print(f"   1. Pastikan UART aktif: sudo raspi-config -> Interface Options -> Serial")
        print(f"   2. Pastikan izin dialout: sudo usermod -a -G dialout $USER")
        return None

    # 1. Tes Komunikasi AT
    print("[1/5] Menguji komunikasi AT dasar...")
    res = send_at_command(ser, "AT", timeout=1.0)
    if not any("OK" in line for line in res):
        # Coba kirim sekali lagi barangkali modul baru bangun
        time.sleep(0.5)
        res = send_at_command(ser, "AT", timeout=1.0)
        if not any("OK" in line for line in res):
            print("   ❌ ERROR: Modul RAK3272S tidak merespon perintah AT.")
            print("   👉 Periksa kabel: TX Pin 8 (Pi) -> RX2 (RAK), RX Pin 10 (Pi) -> TX2 (RAK).")
            ser.close()
            return None
    print("   ✅ Respon AT OK! Modul terdeteksi.")

    # Tampilkan Versi Firmware
    ver = send_at_command(ser, "AT+VER=?", timeout=1.0)
    print(f"   ℹ️  Firmware Modul: {' '.join(ver)}")

    # 2. Cek Mode Kerja (Harus LoRa P2P / NWM=0)
    print("[2/5] Memeriksa Mode Kerja Modul (LoRa P2P)...")
    res_nwm = send_at_command(ser, "AT+NWM=?", timeout=1.0)
    current_mode = None
    for line in res_nwm:
        if "AT+NWM=" in line:
            current_mode = line.split("=")[-1].strip()
            
    if current_mode != "0":
        print("   ⚠️ Modul saat ini bukan mode P2P! Mengubah ke AT+NWM=0...")
        ser.write(b"AT+NWM=0\r\n")
        # Perubahan NWM menyebabkan modul reboot otomatis
        time.sleep(2.5)
        ser.reset_input_buffer()
        res_check = send_at_command(ser, "AT", timeout=1.0)
        if not any("OK" in line for line in res_check):
            print("   ❌ Gagal reboot ke mode P2P!")
            ser.close()
            return None
        print("   ✅ Sukses beralih ke Mode P2P (Reboot selesai).")
    else:
        print("   ✅ Modul sudah berada dalam Mode P2P (NWM=0).")

    # 3. Nonaktifkan mode listen jika sebelumnya aktif
    send_at_command(ser, "AT+PRECV=0", timeout=0.5)

    # 4. Konfigurasi Parameter Radio P2P
    p2p_cmd = f"AT+P2P={P2P_FREQ}:{P2P_SF}:{P2P_BW}:{P2P_CR}:{P2P_PREAMBLE}:{P2P_TXPOWER}"
    print(f"[3/5] Mengatur Parameter P2P: {p2p_cmd}")
    res_p2p = send_at_command(ser, p2p_cmd, timeout=1.5)
    if not any("OK" in line for line in res_p2p):
        print(f"   ❌ Gagal mengatur parameter P2P! Respon: {res_p2p}")
        ser.close()
        return None
        
    print(f"   ✅ Parameter Radio Terkonfigurasi:")
    print(f"      - Frekuensi   : {P2P_FREQ / 1e6} MHz")
    print(f"      - Spreading F : SF{P2P_SF}")
    print(f"      - Bandwidth   : 125 kHz")
    print(f"      - Coding Rate : 4/5")
    print(f"      - Tx Power    : {P2P_TXPOWER} dBm")

    print("[4/5] Memeriksa status final...")
    p2p_status = send_at_command(ser, "AT+P2P=?", timeout=1.0)
    print(f"   ℹ️  Status P2P Aktif : {' '.join(p2p_status)}")

    print(f"{'='*60}")
    print("📡 TRANSMITTER RAK3272S SIAP MENGIRIM PAKET!")
    print(f"{'='*60}\n")
    return ser


def generate_payload(mode, counter, node_id):
    """
    Membuat data payload dan mengembalikannya dalam format HEX string:
    - JSON        : Payload IoT umum (suhu, kelembapan, counter, id)
    - TEXT        : Pesan teks sederhana
    - BINARY_120  : Format binary struct 241 byte (Node ID + 120 sensor data uint16)
    """
    if mode == 'JSON':
        data = {
            "node": node_id,
            "cnt": counter,
            "temp": round(25.0 + (counter % 10) * 0.5, 2),
            "hum": round(60.0 + (counter % 5) * 1.2, 2),
            "volt": 3.31
        }
        raw_bytes = json.dumps(data).encode('utf-8')
        human_readable = json.dumps(data)
    elif mode == 'TEXT':
        text = f"Hello from Pi5 Node {node_id}! Packet #{counter}"
        raw_bytes = text.encode('utf-8')
        human_readable = text
    elif mode == 'BINARY_120':
        # 1 Byte Node ID + 120 x 2 Byte uint16 = 241 Byte payload
        scaled_params = [2550 + (i % 20) for i in range(120)]
        raw_bytes = struct.pack('<B120H', node_id, *scaled_params)
        human_readable = f"Binary Struct: Node {node_id}, 120 parameters (Total: {len(raw_bytes)} Bytes)"
    else:
        raw_bytes = f"Ping #{counter}".encode('utf-8')
        human_readable = f"Ping #{counter}"

    hex_string = raw_bytes.hex().upper()
    return hex_string, human_readable, len(raw_bytes)


def send_packet(ser, hex_payload, timeout=5.0):
    """
    Kirim payload hex via AT+PSEND dan tunggu konfirmasi transmisi:
    1. 'OK' -> modul menerima perintah
    2. '+EVT:TXP2P DONE' -> transmisi radio selesai
    """
    ser.reset_input_buffer()
    cmd = f"AT+PSEND={hex_payload}\r\n"
    ser.write(cmd.encode('utf-8'))
    
    start_time = time.time()
    ok_received = False
    done_received = False
    log_responses = []

    while (time.time() - start_time) < timeout:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                log_responses.append(line)
                if "OK" in line:
                    ok_received = True
                if "+EVT:TXP2P DONE" in line:
                    done_received = True
                    break
                if "AT_BUSY_ERROR" in line:
                    return False, f"Modul Sibuk ({line})"
                if "AT_PARAM_ERROR" in line or "AT_ERROR" in line:
                    return False, f"Error Parameter ({line})"
        time.sleep(0.02)

    elapsed = time.time() - start_time

    if done_received:
        return True, f"Berhasil terkirim via radio (+EVT:TXP2P DONE dalam {elapsed:.2f}s)"
    elif ok_received:
        return True, f"Perintah diterima modul (OK dalam {elapsed:.2f}s, radio finish)"
    else:
        return False, f"Timeout! Respon: {log_responses}"


def main():
    ser = init_rak3272s_tx(SERIAL_PORT, BAUDRATE)
    if not ser:
        sys.exit(1)

    counter = 1
    print(f"Mulai transmisi berkala setiap {TX_INTERVAL} detik (Mode: {PAYLOAD_MODE})...")
    print("Tekan Ctrl+C untuk menghentikan pengiriman.\n")

    try:
        while True:
            hex_payload, desc, byte_len = generate_payload(PAYLOAD_MODE, counter, NODE_ID)
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

            print(f"[{timestamp}] 📤 [Paket #{counter}]")
            print(f"   Payload  : {desc}")
            print(f"   Ukuran   : {byte_len} Byte ({len(hex_payload)} karakter Hex)")

            # Eksekusi Pengiriman
            success, msg = send_packet(ser, hex_payload, timeout=6.0)

            if success:
                print(f"   STATUS   : ✅ {msg}")
            else:
                print(f"   STATUS   : ⚠️ {msg}")

            print("-" * 65)
            counter += 1
            time.sleep(TX_INTERVAL)

    except KeyboardInterrupt:
        print("\n[TRANSMITTER] Dihentikan oleh pengguna (Ctrl+C).")
    finally:
        # Kembalikan modul ke idle dan tutup koneksi
        if ser and ser.is_open:
            ser.write(b"AT+PRECV=0\r\n")
            time.sleep(0.1)
            ser.close()
            print("Serial port berhasil ditutup dengan aman.")


if __name__ == '__main__':
    main()
