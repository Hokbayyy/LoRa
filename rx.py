#!/usr/bin/env python3
"""
=============================================================================
RAK3272S LoRa P2P Receiver (RX) Node
=============================================================================
Konfigurasi Perangkat Keras:
  - Modul RAK3272S  : UART2 (Header J4)
                      Pin 7 (TX2) & Pin 8 (RX2)
                      Tegangan 3.3V (Header J5 Pin 9) & GND (Header J4 Pin 4)
  - Raspberry Pi    : Port Serial Default /dev/ttyAMA0 (UART0) atau /dev/ttyAMA3
                      - Pi Pin 8  (GPIO 14 - TXD0) -> RAK Header J4 Pin 8 (RX2)
                      - Pi Pin 10 (GPIO 15 - RXD0) <- RAK Header J4 Pin 7 (TX2)
                      - Pi Pin 1  (3.3V)           -> RAK Header J5 Pin 9 (3V3)
                      - Pi Pin 6  (GND)            -> RAK Header J4 Pin 4 (GND)
=============================================================================
"""

import serial
import time
import struct
import json
import sys
import os

# ================= Konfigurasi Port & Radio =================
# Daftar kandidat port serial di Raspberry Pi Receiver
CANDIDATE_PORTS = [
    '/dev/ttyAMA0',
    '/dev/serial0',
    '/dev/ttyAMA3',
    '/dev/ttyUSB0'
]

# Tentukan port serial: gunakan argumen command-line jika ada (misal: python3 rx.py /dev/ttyAMA0)
if len(sys.argv) > 1:
    SERIAL_PORT = sys.argv[1]
else:
    # Cari port pertama yang tersedia dari daftar kandidat
    SERIAL_PORT = CANDIDATE_PORTS[0]
    for p in CANDIDATE_PORTS:
        if os.path.exists(p):
            SERIAL_PORT = p
            break

BAUDRATE = 115200

# Pengaturan Radio LoRa P2P (WAJIB PERSIS SAMA DENGAN TRANSMITTER)
# Frekuensi AS923 Indonesia: 923.2 MHz
P2P_FREQ     = 923200000  # 923.2 MHz
P2P_SF       = 7          # SF7 (Spread Factor 6-12)
P2P_BW       = 0          # 0: 125kHz, 1: 250kHz, 2: 500kHz
P2P_CR       = 0          # 0: 4/5, 1: 4/6, 2: 4/7, 3: 4/8
P2P_PREAMBLE = 8          # Preamble length (default 8)
P2P_TXPOWER  = 20         # Tx Power 20 dBm
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


def init_rak3272s_rx(port, baudrate):
    print("=" * 65)
    print("📥 INISIALISASI RECEIVER (RX)")
    print(f"   Raspberry Pi   : Port {port}")
    print("   Modul RAK3272S : UART2 (Header J4: Pin 7 TX2 & Pin 8 RX2)")
    print("=" * 65)
    
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1, rtscts=False)
    except Exception as e:
        print(f"❌ GAGAL membuka port serial {port}!")
        print(f"   Detail Error : {e}")
        print("   👉 Anda bisa menentukan port secara manual, contoh: python3 rx.py /dev/ttyAMA0")
        return None

    # 1. Tes Komunikasi AT pada UART2
    print("[1/5] Menguji komunikasi AT pada UART2 RAK3272S...")
    res = send_at_command(ser, "AT", timeout=1.0)
    if not any("OK" in line for line in res):
        time.sleep(0.5)
        res = send_at_command(ser, "AT", timeout=1.0)
        if not any("OK" in line for line in res):
            print("   ❌ ERROR: Modul RAK3272S tidak merespon perintah AT di UART2.")
            print("   👉 Periksa sambungan kabel:")
            print(f"      - Pi TX ({port}) -> Header J4 Pin 8 (RX2)")
            print(f"      - Pi RX ({port}) <- Header J4 Pin 7 (TX2)")
            ser.close()
            return None
    print("   ✅ Respon AT OK! Modul terdeteksi pada UART2.")

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

    # 3. Konfigurasi Parameter Radio P2P
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

    # 4. Aktifkan Mode Continuous RX (65534)
    print("[4/5] Mengaktifkan Mode Continuous RX (AT+PRECV=65534)...")
    res_precv = send_at_command(ser, "AT+PRECV=65534", timeout=1.5)
    if not any("OK" in line for line in res_precv):
        print(f"   ⚠️ Respon PRECV: {res_precv}")
    else:
        print("   ✅ Receiver aktif mendengarkan gelombang radio secara kontinu.")

    print("[5/5] Memeriksa status akhir...")
    p2p_status = send_at_command(ser, "AT+P2P=?", timeout=1.0)
    print(f"   ℹ️  Status Radio: {' '.join(p2p_status)}")

    print("=" * 65)
    print("📡 RECEIVER RAK3272S (UART2) STANDBY MENDENGARKAN DATA...")
    print("Menunggu paket dari Transmitter... (Tekan Ctrl+C untuk berhenti)")
    print("=" * 65 + "\n")
    return ser


def decode_hex_payload(hex_str):
    try:
        raw_bytes = bytes.fromhex(hex_str)
    except Exception:
        return f"[Hex Raw Error] {hex_str}", len(hex_str) // 2

    byte_len = len(raw_bytes)

    # Deteksi Format 1: Text UTF-8 / JSON
    try:
        decoded_text = raw_bytes.decode('utf-8')
        if decoded_text.startswith('{') and decoded_text.endswith('}'):
            try:
                parsed_json = json.loads(decoded_text)
                return f"[JSON Telemetri]: {parsed_json}", byte_len
            except json.JSONDecodeError:
                pass
        return f"[Plain Text]: \"{decoded_text}\"", byte_len
    except UnicodeDecodeError:
        pass

    # Deteksi Format 2: Binary Struct (1 Byte Node ID + 120 uint16)
    if byte_len == 241:
        try:
            node_id = raw_bytes[0]
            values = struct.unpack('<120H', raw_bytes[1:])
            preview = f"Sample: {values[:5]}... ({len(values)} parameter total)"
            return f"[Binary 120 Params] Node ID: {node_id} | {preview}", byte_len
        except Exception:
            pass

    return f"[Raw Hex]: {hex_str[:60]}{'...' if len(hex_str) > 60 else ''}", byte_len


def parse_rx_event(line):
    if "+EVT:RXP2P" not in line:
        return None

    rssi, snr, hex_payload = None, None, None

    if ":" in line:
        parts = line.split(":")
        if len(parts) >= 5:
            rssi = parts[2].strip()
            snr = parts[3].strip()
            hex_payload = parts[4].strip()
    
    if not hex_payload and " " in line:
        parts = line.split()
        if len(parts) >= 4:
            rssi = parts[-3]
            snr = parts[-2]
            hex_payload = parts[-1]

    if hex_payload:
        return {
            "rssi": rssi,
            "snr": snr,
            "hex": hex_payload
        }
    return None


def main():
    ser = init_rak3272s_rx(SERIAL_PORT, BAUDRATE)
    if not ser:
        sys.exit(1)

    packet_count = 0
    buffer = ""

    try:
        while True:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                buffer += chunk

                while '\r\n' in buffer:
                    line, buffer = buffer.split('\r\n', 1)
                    line = line.strip()

                    if not line:
                        continue

                    parsed = parse_rx_event(line)
                    if parsed:
                        packet_count += 1
                        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                        decoded_info, byte_len = decode_hex_payload(parsed['hex'])

                        print(f"🎉 [{timestamp}] 📥 PAKET MASUK #{packet_count}")
                        print(f"   Kualitas Sinyal : RSSI = {parsed['rssi']} dBm | SNR = {parsed['snr']} dB")
                        print(f"   Ukuran Data     : {byte_len} Byte")
                        print(f"   Isi Data        : {decoded_info}")
                        print(f"   Hex Raw         : {parsed['hex'][:50]}{'...' if len(parsed['hex']) > 50 else ''}")
                        print("-" * 65)
                    else:
                        if not line.startswith("OK"):
                            timestamp = time.strftime('%H:%M:%S')
                            print(f"[{timestamp}] [INFO/LOG RAK]: {line}")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[RECEIVER] Dihentikan oleh pengguna (Ctrl+C).")
    finally:
        if ser and ser.is_open:
            print("Menghentikan mode receive (AT+PRECV=0)...")
            send_at_command(ser, "AT+PRECV=0", timeout=0.5)
            ser.close()
            print("Serial port berhasil ditutup dengan aman.")


if __name__ == '__main__':
    main()
