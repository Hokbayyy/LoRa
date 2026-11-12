#!/usr/bin/env python3
"""
=============================================================================
RAK3272S LoRa P2P Ultra-Fast Receiver (RX) Node
=============================================================================
Fitur:
  - Bandwidth   : 500 kHz (Wajib SAMA dengan TX)
  - Frekuensi   : 923.2 MHz (AS923)
  - Spreading F : SF7
  - Port Default: /dev/ttyAMA0 (UART0 Header J4 Pin 7 & 8)
=============================================================================
"""

import serial
import time
import struct
import json
import sys
import os

# ================= Konfigurasi Port & Radio =================
CANDIDATE_PORTS = [
    '/dev/ttyAMA0',
    '/dev/serial0',
    '/dev/ttyAMA3',
    '/dev/ttyUSB0'
]

if len(sys.argv) > 1:
    SERIAL_PORT = sys.argv[1]
else:
    SERIAL_PORT = CANDIDATE_PORTS[0]
    for p in CANDIDATE_PORTS:
        if os.path.exists(p):
            SERIAL_PORT = p
            break

BAUDRATE = 115200

# Parameter Radio LoRa P2P (WAJIB PERSIS SAMA DENGAN TRANSMITTER)
P2P_FREQ     = 923200000  # 923.2 MHz
P2P_SF       = 7          # SF7
P2P_BW       = 2          # 2: 500kHz (Ultra Fast)
P2P_CR       = 0          # 0: 4/5
P2P_PREAMBLE = 8          # Preamble 8
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
        time.sleep(0.005)
        
    return responses


def init_rak3272s_rx(port, baudrate):
    print("=" * 65)
    print("⚡ INISIALISASI RECEIVER (RX)")
    print(f"   Raspberry Pi   : Port {port}")
    print("   Modul RAK3272S : UART2 (Header J4: Pin 7 TX2 & Pin 8 RX2)")
    print(f"   Frekuensi      : {P2P_FREQ / 1e6} MHz | BW: 500 kHz | SF: {P2P_SF}")
    print("=" * 65)
    
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1, rtscts=False)
    except Exception as e:
        print(f"❌ GAGAL membuka port serial {port}!")
        print(f"   Detail Error : {e}")
        print("   👉 Coba tentukan port manual, contoh: python3 rx.py /dev/ttyAMA0")
        return None

    # 1. Tes Komunikasi AT
    print("[1/4] Menguji komunikasi AT dasar pada UART2...")
    res = send_at_command(ser, "AT", timeout=1.0)
    if not any("OK" in line for line in res):
        time.sleep(0.3)
        res = send_at_command(ser, "AT", timeout=1.0)
        if not any("OK" in line for line in res):
            print("   ❌ ERROR: Modul RAK3272S tidak merespon perintah AT di UART2.")
            print(f"   👉 Periksa kabel di Header J4: Pi TX -> Pin 8 (RX2), Pi RX -> Pin 7 (TX2).")
            ser.close()
            return None
    print("   ✅ Respon AT OK! Modul terdeteksi di UART2.")

    # 2. Cek Mode Kerja (Harus LoRa P2P / NWM=0)
    res_nwm = send_at_command(ser, "AT+NWM=?", timeout=1.0)
    current_mode = None
    for line in res_nwm:
        if "AT+NWM=" in line:
            current_mode = line.split("=")[-1].strip()
            
    if current_mode != "0":
        print("   ⚠️ Mengubah ke Mode P2P (AT+NWM=0)...")
        ser.write(b"AT+NWM=0\r\n")
        ser.flush()
        time.sleep(2.5)
        ser.reset_input_buffer()
        res_check = send_at_command(ser, "AT", timeout=1.0)
        if not any("OK" in line for line in res_check):
            print("   ❌ Gagal reboot ke mode P2P!")
            ser.close()
            return None
    else:
        print("   ✅ Modul sudah berada dalam Mode P2P (NWM=0).")

    # Matikan mode RX sebelumnya agar konfigurasi parameter tidak 'busy'
    send_at_command(ser, "AT+PRECV=0", timeout=0.5)

    # 3. Konfigurasi Parameter Radio P2P Ultra-Fast (BW=500kHz)
    p2p_cmd = f"AT+P2P={P2P_FREQ}:{P2P_SF}:{P2P_BW}:{P2P_CR}:{P2P_PREAMBLE}:{P2P_TXPOWER}"
    print(f"[2/4] Mengatur Parameter Radio P2P: {p2p_cmd}")
    res_p2p = send_at_command(ser, p2p_cmd, timeout=1.5)
    if not any("OK" in line for line in res_p2p):
        print(f"   ❌ Gagal mengatur parameter P2P! Respon: {res_p2p}")
        ser.close()
        return None
        
    print(f"   ✅ Konfigurasi Radio Aktif:")
    print(f"      - Frekuensi   : {P2P_FREQ / 1e6} MHz")
    print(f"      - Bandwidth   : 500 kHz (BW=2)")
    print(f"      - Spreading F : SF{P2P_SF}")
    print(f"      - Coding Rate : 4/5")

    # 4. Aktifkan Continuous RX
    print("[3/4] Mengaktifkan Mode Continuous RX (AT+PRECV=65534)...")
    res_precv = send_at_command(ser, "AT+PRECV=65534", timeout=1.5)
    if not any("OK" in line for line in res_precv):
        print(f"   ⚠️ Respon PRECV: {res_precv}")
    else:
        print("   ✅ Receiver aktif mendengarkan gelombang radio secara kontinu.")

    # 5. Cek status radio final
    p2p_stat = send_at_command(ser, "AT+P2P=?", timeout=1.0)
    print(f"[4/4] Status Radio Final: {' '.join(p2p_stat)}")

    print("=" * 65)
    print("📡 RECEIVER STANDBY MENDENGARKAN SINYAL LORA...")
    print("Tekan Ctrl+C untuk berhenti.\n")
    return ser


def decode_hex_payload(hex_str):
    try:
        raw_bytes = bytes.fromhex(hex_str)
    except Exception:
        return f"[Hex Err] {hex_str}", len(hex_str) // 2

    byte_len = len(raw_bytes)

    # Deteksi Format 1: Text UTF-8 / JSON
    try:
        decoded_text = raw_bytes.decode('utf-8')
        if decoded_text.startswith('{') and decoded_text.endswith('}'):
            try:
                parsed_json = json.loads(decoded_text)
                return f"[JSON] {parsed_json}", byte_len
            except json.JSONDecodeError:
                pass
        return f"\"{decoded_text}\"", byte_len
    except UnicodeDecodeError:
        pass

    # Deteksi Format 2: Binary Struct 8 Byte
    if byte_len == 8:
        try:
            node_id, counter, temp_scaled, hum_int = struct.unpack('<BIHB', raw_bytes)
            return f"[Binary 8B] Node:{node_id} Cnt:{counter} Temp:{temp_scaled/10}C Hum:{hum_int}%", byte_len
        except Exception:
            pass

    return f"[Raw {byte_len}B]: {hex_str[:30]}", byte_len


def parse_rx_event(line):
    if "+EVT:RXP2P" not in line:
        return None

    # Format RUI3: +EVT:RXP2P:<RSSI>:<SNR>:<Payload>
    if ":" in line:
        parts = line.split(":")
        if len(parts) >= 5:
            return {
                "rssi": parts[2].strip(),
                "snr": parts[3].strip(),
                "hex": parts[4].strip()
            }
    
    # Format fallback jika pemisah spasi
    if " " in line:
        parts = line.split()
        if len(parts) >= 4:
            return {
                "rssi": parts[-3],
                "snr": parts[-2],
                "hex": parts[-1]
            }

    return None


def main():
    ser = init_rak3272s_rx(SERIAL_PORT, BAUDRATE)
    if not ser:
        sys.exit(1)

    packet_count = 0
    buffer = ""
    t_start_all = time.time()
    last_stat_time = time.time()
    stat_counter = 0

    try:
        while True:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                buffer += chunk

                # Tangani pemisah baris \n maupun \r\n secara aman
                if '\n' in buffer:
                    lines = buffer.split('\n')
                    buffer = lines[-1]

                    for raw_line in lines[:-1]:
                        line = raw_line.strip()
                        if not line:
                            continue

                        parsed = parse_rx_event(line)
                        if parsed:
                            packet_count += 1
                            stat_counter += 1
                            now = time.time()

                            if now - last_stat_time >= 1.0:
                                hz = stat_counter / (now - last_stat_time)
                                rate_str = f"| Speed: {hz:.1f} pkt/s ({hz:.1f} Hz)"
                                stat_counter = 0
                                last_stat_time = now
                            else:
                                rate_str = ""

                            decoded_info, byte_len = decode_hex_payload(parsed['hex'])
                            timestamp = time.strftime('%H:%M:%S')

                            print(f"[{timestamp}] ⚡ [#{packet_count:05d}] {decoded_info} ({byte_len}B) | RSSI:{parsed['rssi']}dBm SNR:{parsed['snr']}dB {rate_str}")
                        else:
                            # Tampilkan pesan atau peringatan dari modul
                            if "TIMEOUT" not in line and not line.startswith("OK"):
                                timestamp = time.strftime('%H:%M:%S')
                                print(f"[{timestamp}] [INFO DARI MODUL]: {line}")

            time.sleep(0.002)

    except KeyboardInterrupt:
        total_time = time.time() - t_start_all
        avg_rate = packet_count / total_time if total_time > 0 else 0
        print(f"\n[RECEIVER] Berhenti. Total {packet_count} paket diterima dalam {total_time:.2f} detik (Rata-rata: {avg_rate:.1f} pkt/s).")
    finally:
        if ser and ser.is_open:
            print("Menghentikan mode receive (AT+PRECV=0)...")
            send_at_command(ser, "AT+PRECV=0", timeout=0.3)
            ser.close()
            print("Serial port ditutup dengan aman.")


if __name__ == '__main__':
    main()
