#!/usr/bin/env python3
"""
=============================================================================
RAK3272S LoRa P2P Transmitter (TX) Node
=============================================================================
Konfigurasi:
  - Raspberry Pi 5  : UART3 (/dev/ttyAMA3)
  - Modul RAK3272S  : UART1 (Header J5: Pin 5 RX1 & Pin 6 TX1)
  - Frekuensi       : 923.2 MHz (AS923) | SF: 7 | BW: 125 kHz
=============================================================================
Penggunaan:
  python3 tx.py                 (Default: interval 0.5 detik)
  python3 tx.py 1               (Interval 1 detik)
  python3 tx.py 0.3             (Interval 0.3 detik / sub-detik)
  python3 tx.py /dev/ttyAMA3 0.5
=============================================================================
"""

import serial
import time
import struct
import json
import sys
import os

# ================= Konfigurasi Port & Interval =================
DEFAULT_PORT = '/dev/ttyAMA3'
SERIAL_PORT  = DEFAULT_PORT
TX_INTERVAL  = 0.5  # Default: 0.5 detik (sub-detik)

# Pintar membaca argumen terminal
if len(sys.argv) == 2:
    try:
        TX_INTERVAL = float(sys.argv[1])
    except ValueError:
        SERIAL_PORT = sys.argv[1]
elif len(sys.argv) >= 3:
    try:
        TX_INTERVAL = float(sys.argv[1])
        SERIAL_PORT = sys.argv[2]
    except ValueError:
        SERIAL_PORT = sys.argv[1]
        try:
            TX_INTERVAL = float(sys.argv[2])
        except ValueError:
            pass

BAUDRATE = 115200

# Pengaturan Radio LoRa P2P (WAJIB PERSIS SAMA DENGAN RECEIVER)
P2P_FREQ     = 923200000  # 923.2 MHz (AS923)
P2P_SF       = 7          # SF7 (Spread Factor 6-12)
P2P_BW       = 0          # 0: 125kHz (Standar AS923 terbukti stabil)
P2P_CR       = 0          # 0: 4/5
P2P_PREAMBLE = 8          # Preamble 8
P2P_TXPOWER  = 20         # Tx Power 20 dBm

NODE_ID      = 1          # ID Node Pengirim

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
        time.sleep(0.005)
        
    return responses


def init_rak3272s_tx(port, baudrate):
    print("=" * 65)
    print("🚀 INISIALISASI TRANSMITTER (TX)")
    print(f"   Raspberry Pi 5 : UART3 ({port})")
    print(f"   Modul RAK3272S : UART1 (Header J5: Pin 5 RX1 & Pin 6 TX1)")
    print(f"   Target Interval: {TX_INTERVAL} detik (~{1.0/TX_INTERVAL if TX_INTERVAL > 0 else 0:.1f} paket/detik)")
    print("=" * 65)
    
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1, rtscts=False)
    except Exception as e:
        print(f"❌ GAGAL membuka port serial {port}!")
        print(f"   Detail Error : {e}")
        return None

    # 1. Tes Komunikasi AT pada UART1
    print("[1/4] Menguji komunikasi AT pada UART1 RAK3272S...")
    res = send_at_command(ser, "AT", timeout=1.0)
    if not any("OK" in line for line in res):
        time.sleep(0.3)
        res = send_at_command(ser, "AT", timeout=1.0)
        if not any("OK" in line for line in res):
            print("   ❌ ERROR: Modul RAK3272S tidak merespon perintah AT di UART1.")
            ser.close()
            return None
    print("   ✅ Respon AT OK! Modul terdeteksi pada UART1.")

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
        print("   ✅ Sukses beralih ke Mode P2P.")
    else:
        print("   ✅ Modul sudah berada dalam Mode P2P (NWM=0).")

    # 3. Nonaktifkan mode listen jika sebelumnya aktif
    send_at_command(ser, "AT+PRECV=0", timeout=0.3)

    # 4. Konfigurasi Parameter Radio P2P
    p2p_cmd = f"AT+P2P={P2P_FREQ}:{P2P_SF}:{P2P_BW}:{P2P_CR}:{P2P_PREAMBLE}:{P2P_TXPOWER}"
    print(f"[2/4] Mengatur Parameter Radio P2P: {p2p_cmd}")
    res_p2p = send_at_command(ser, p2p_cmd, timeout=1.5)
    if not any("OK" in line for line in res_p2p):
        print(f"   ❌ Gagal mengatur parameter P2P! Respon: {res_p2p}")
        ser.close()
        return None
        
    print(f"   ✅ Parameter Radio Terkonfigurasi:")
    print(f"      - Frekuensi   : {P2P_FREQ / 1e6} MHz (AS923)")
    print(f"      - Bandwidth   : 125 kHz")
    print(f"      - Spreading F : SF{P2P_SF}")
    print(f"      - Tx Power    : {P2P_TXPOWER} dBm")

    # 5. Cek status radio final
    p2p_stat = send_at_command(ser, "AT+P2P=?", timeout=1.0)
    print(f"[3/4] Status Radio Final: {' '.join(p2p_stat)}")

    print("=" * 65)
    print(f"📡 TRANSMITTER SIAP MENGIRIM SETIAP {TX_INTERVAL} DETIK!")
    print("=" * 65 + "\n")
    return ser


def generate_payload(mode, counter, node_id):
    if mode == 'JSON':
        data = {
            "n": node_id,
            "c": counter,
            "t": round(25.0 + (counter % 10) * 0.5, 1),
            "h": round(60.0 + (counter % 5) * 1.0, 1)
        }
        raw_bytes = json.dumps(data, separators=(',', ':')).encode('utf-8')
        human_readable = json.dumps(data, separators=(',', ':'))
    elif mode == 'TEXT':
        text = f"P#{counter} Node{node_id}"
        raw_bytes = text.encode('utf-8')
        human_readable = text
    elif mode == 'BINARY_120':
        scaled_params = [2550 + (i % 20) for i in range(120)]
        raw_bytes = struct.pack('<B120H', node_id, *scaled_params)
        human_readable = f"Binary: Node {node_id}, 120 params ({len(raw_bytes)}B)"
    else:
        raw_bytes = f"Ping #{counter}".encode('utf-8')
        human_readable = f"Ping #{counter}"

    hex_string = raw_bytes.hex().upper()
    return hex_string, human_readable, len(raw_bytes)


def send_packet(ser, hex_payload, timeout=2.0):
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
                if "+EVT:TXP2P" in line:
                    done_received = True
                    break
                if "OK" in line:
                    ok_received = True
                if "AT_BUSY_ERROR" in line:
                    return False, f"Modul Sibuk ({line})"
                if "AT_PARAM_ERROR" in line or "AT_ERROR" in line:
                    return False, f"Error Parameter ({line})"
        else:
            # Begitu OK diterima dan waktu transmisi (~100ms) lewat, langsung break!
            if ok_received and (time.time() - start_time) >= 0.12:
                done_received = True
                break
        time.sleep(0.005)

    elapsed_ms = (time.time() - start_time) * 1000

    if done_received or ok_received:
        return True, f"DONE ({elapsed_ms:.0f} ms)"
    else:
        return False, f"Timeout ({elapsed_ms:.0f} ms)! Respon: {log_responses}"


def main():
    ser = init_rak3272s_tx(SERIAL_PORT, BAUDRATE)
    if not ser:
        sys.exit(1)

    counter = 1
    t_start_all = time.time()
    last_stat_time = time.time()
    stat_counter = 0

    print(f"Mulai transmisi aktif setiap {TX_INTERVAL} detik (~{1.0/TX_INTERVAL if TX_INTERVAL > 0 else 0:.1f} paket/detik)...")
    print("Tekan Ctrl+C untuk berhenti.\n")

    try:
        while True:
            t_pkt_start = time.time()
            hex_payload, desc, byte_len = generate_payload(PAYLOAD_MODE, counter, NODE_ID)

            success, msg = send_packet(ser, hex_payload, timeout=2.0)

            stat_counter += 1
            now = time.time()
            if now - last_stat_time >= 1.0:
                hz = stat_counter / (now - last_stat_time)
                rate_str = f"| Speed: {hz:.1f} pkt/s"
                stat_counter = 0
                last_stat_time = now
            else:
                rate_str = ""

            status_icon = "⚡" if success else "⚠️"
            timestamp = time.strftime('%H:%M:%S')
            print(f"[{timestamp}] {status_icon} [#{counter:05d}] {desc} ({byte_len}B) -> {msg} {rate_str}")

            counter += 1

            # Tunggu sisa waktu agar interval persis sesuai TX_INTERVAL
            elapsed_tx = time.time() - t_pkt_start
            sleep_time = max(0.0, TX_INTERVAL - elapsed_tx)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        total_time = time.time() - t_start_all
        avg_rate = (counter - 1) / total_time if total_time > 0 else 0
        print(f"\n[TRANSMITTER] Selesai. Total {counter - 1} paket terkirim dalam {total_time:.2f} detik (Rata-rata: {avg_rate:.1f} pkt/s).")
    finally:
        if ser and ser.is_open:
            ser.write(b"AT+PRECV=0\r\n")
            ser.flush()
            ser.close()
            print("Port serial ditutup.")


if __name__ == '__main__':
    main()
