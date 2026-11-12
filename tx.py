#!/usr/bin/env python3
"""
=============================================================================
RAK3272S LoRa P2P Transmitter (TX) Node
=============================================================================
Konfigurasi Perangkat Keras:
  - Raspberry Pi 5  : UART3 (/dev/ttyAMA3)
                      Pin 24 (GPIO 8 - TXD3) & Pin 21 (GPIO 9 - RXD3)
  - Modul RAK3272S  : UART1 (Header J5)
                      Pin 5 (RX1) & Pin 6 (TX1)
                      Tegangan 3.3V (Header J5 Pin 9) & GND (Header J5 Pin 7)
=============================================================================
"""

import serial
import time
import struct
import json
import sys
import os

# ================= Konfigurasi Port & Radio =================
# Port serial Pi 5 (bisa di-override lewat argumen: python3 tx.py /dev/ttyAMA3)
DEFAULT_PORT = '/dev/ttyAMA3'
SERIAL_PORT  = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
BAUDRATE     = 115200

# Pengaturan Radio LoRa P2P (WAJIB PERSIS SAMA DENGAN RECEIVER)
# Alokasi Frekuensi Indonesia AS923: 923.2 MHz
P2P_FREQ     = 923200000  # 923.2 MHz
P2P_SF       = 7          # SF7 (Spread Factor 6-12)
P2P_BW       = 0          # 0: 125kHz, 1: 250kHz, 2: 500kHz
P2P_CR       = 0          # 0: 4/5, 1: 4/6, 2: 4/7, 3: 4/8
P2P_PREAMBLE = 8          # Preamble length (default 8)
P2P_TXPOWER  = 20         # Tx Power 20 dBm (Max 22 dBm)

# Interval pengiriman paket (detik), default 0.5s (sub-detik), bisa: python3 tx.py /dev/ttyAMA3 0.3
TX_INTERVAL  = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
NODE_ID      = 1          # ID Node Pengirim (Transmitter)

# Mode Payload: 'JSON', 'TEXT', atau 'BINARY_120'
PAYLOAD_MODE = 'JSON'
# =============================================================


def send_at_command(ser, cmd, timeout=1.5, expected=None):
    ser.reset_input_buffer()
    ser.write((cmd + '\r\n').encode('utf-8'))
    ser.flush()
    
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
    print("=" * 65)
    print("🚀 INISIALISASI TRANSMITTER (TX)")
    print(f"   Raspberry Pi 5 : UART3 ({port})")
    print(f"   Modul RAK3272S : UART1 (Header J5: Pin 5 RX1 & Pin 6 TX1)")
    print("=" * 65)
    
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1, rtscts=False)
    except Exception as e:
        print(f"❌ GAGAL membuka port serial {port}!")
        print(f"   Detail Error : {e}")
        print("   👉 Pastikan dtoverlay=uart3-pi5 sudah ada di /boot/firmware/config.txt")
        return None

    # 1. Tes Komunikasi AT pada UART1
    print("[1/5] Menguji komunikasi AT pada UART1 RAK3272S...")
    res = send_at_command(ser, "AT", timeout=1.0)
    if not any("OK" in line for line in res):
        time.sleep(0.5)
        res = send_at_command(ser, "AT", timeout=1.0)
        if not any("OK" in line for line in res):
            print("   ❌ ERROR: Modul RAK3272S tidak merespon perintah AT di UART1.")
            print("   👉 Periksa jalur PCB:")
            print("      - Pi 5 Pin 24 (TXD3) -> RAK Header J5 Pin 5 (RX1)")
            print("      - Pi 5 Pin 21 (RXD3) -> RAK Header J5 Pin 6 (TX1)")
            ser.close()
            return None
    print("   ✅ Respon AT OK! Modul terdeteksi pada UART1.")

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
        ser.flush()
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
    print(f"[3/5] Mengatur Parameter Radio P2P: {p2p_cmd}")
    res_p2p = send_at_command(ser, p2p_cmd, timeout=1.5)
    if not any("OK" in line for line in res_p2p):
        print(f"   ❌ Gagal mengatur parameter P2P! Respon: {res_p2p}")
        ser.close()
        return None
        
    print(f"   ✅ Parameter Radio Terkonfigurasi:")
    print(f"      - Frekuensi   : {P2P_FREQ / 1e6} MHz (AS923)")
    print(f"      - Spreading F : SF{P2P_SF}")
    print(f"      - Bandwidth   : 125 kHz")
    print(f"      - Coding Rate : 4/5")
    print(f"      - Tx Power    : {P2P_TXPOWER} dBm")

    print("[4/5] Memeriksa status parameter aktif...")
    p2p_status = send_at_command(ser, "AT+P2P=?", timeout=1.0)
    print(f"   ℹ️  Status Radio: {' '.join(p2p_status)}")

    print("=" * 65)
    print("📡 TRANSMITTER RAK3272S (UART1) SIAP MEMANCARKAN DATA!")
    print("=" * 65 + "\n")
    return ser


def generate_payload(mode, counter, node_id):
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
        # Simulasi 120 sensor parameter (masing-masing 16-bit uint16)
        scaled_params = [2550 + (i % 20) for i in range(120)]
        raw_bytes = struct.pack('<B120H', node_id, *scaled_params)
        human_readable = f"Binary Struct: Node {node_id}, 120 params ({len(raw_bytes)} Byte)"
    else:
        raw_bytes = f"Ping #{counter}".encode('utf-8')
        human_readable = f"Ping #{counter}"

    hex_string = raw_bytes.hex().upper()
    return hex_string, human_readable, len(raw_bytes)


def send_packet(ser, hex_payload, timeout=6.0):
    ser.reset_input_buffer()
    cmd = f"AT+PSEND={hex_payload}\r\n"
    ser.write(cmd.encode('utf-8'))
    ser.flush()
    
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
        return True, f"Terkirim via radio LoRa (+EVT:TXP2P DONE dalam {elapsed:.2f}s)"
    elif ok_received:
        return True, f"Diterima modul (OK dalam {elapsed:.2f}s)"
    else:
        return False, f"Timeout! Respon: {log_responses}"


def main():
    ser = init_rak3272s_tx(SERIAL_PORT, BAUDRATE)
    if not ser:
        sys.exit(1)

    counter = 1
    print(f"Mulai transmisi data berkala setiap {TX_INTERVAL} detik (Format: {PAYLOAD_MODE})...")
    print("Tekan Ctrl+C untuk menghentikan pengiriman.\n")

    try:
        while True:
            hex_payload, desc, byte_len = generate_payload(PAYLOAD_MODE, counter, NODE_ID)
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

            print(f"[{timestamp}] 📤 [Paket #{counter}]")
            print(f"   Payload  : {desc}")
            print(f"   Ukuran   : {byte_len} Byte ({len(hex_payload)} karakter Hex)")

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
        if ser and ser.is_open:
            ser.write(b"AT+PRECV=0\r\n")
            ser.flush()
            time.sleep(0.1)
            ser.close()
            print("Serial port berhasil ditutup dengan aman.")


if __name__ == '__main__':
    main()
