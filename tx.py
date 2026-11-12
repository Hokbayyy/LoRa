#!/usr/bin/env python3
"""
=============================================================================
RAK3272S LoRa P2P Ultra-Fast Transmitter (TX) Node
=============================================================================
Fitur Kecepatan Tinggi (Sub-Detik):
  - Bandwidth   : 500 kHz (4x lebih cepat dari 125 kHz standar)
  - Time-on-Air : ~15 - 25 milidetik per paket
  - Interval    : Default 0.2s (5 Hz) atau 0.1s (10 Hz) hingga 0s (Streaming)
  - Port Default: /dev/ttyAMA3 (Pi 5 UART3 <--> RAK3272S UART1 Header J5)

Penggunaan:
  python3 tx.py                    (Pakai port /dev/ttyAMA3, interval 0.2 detik)
  python3 tx.py /dev/ttyAMA3 0.1   (Interval 0.1 detik / 10 paket per detik)
  python3 tx.py /dev/ttyAMA3 0.05  (Interval 50 ms / 20 paket per detik)
  python3 tx.py /dev/ttyAMA3 0     (Mode Streaming maksimum tanpa jeda)
=============================================================================
"""

import serial
import time
import struct
import json
import sys
import os

# ================= Konfigurasi Port & Interval =================
DEFAULT_PORT     = '/dev/ttyAMA3'
SERIAL_PORT      = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
TX_INTERVAL      = float(sys.argv[2]) if len(sys.argv) > 2 else 0.2  # Default: 0.2 detik (5 Hz)
BAUDRATE         = 115200

# ================= Parameter Radio Ultra-Fast =================
# Bandwidth 500 kHz + SF7 menghasilkan transmisi ultra cepat di udara (~20 ms)
P2P_FREQ         = 923200000  # 923.2 MHz (Alokasi AS923)
P2P_SF           = 7          # SF7 (Spread Factor cepat dan stabil)
P2P_BW           = 2          # 2: 500kHz (MAKSIMUM KECEPATAN!)
P2P_CR           = 0          # 0: 4/5 (Coding rate tercepat)
P2P_PREAMBLE     = 8          # Preamble standar
P2P_TXPOWER      = 20         # Tx Power 20 dBm

NODE_ID          = 1          # ID Node Pengirim

# Pilihan Mode Payload: 'COMPACT_JSON', 'FAST_TEXT', atau 'FAST_BINARY'
PAYLOAD_MODE     = 'COMPACT_JSON'
# ===============================================================


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
    print("⚡ INISIALISASI TRANSMITTER ULTRA-FAST (SUB-DETIK)")
    print(f"   Raspberry Pi 5 : UART3 ({port})")
    print(f"   Modul RAK3272S : UART1 (Header J5: Pin 5 RX1 & Pin 6 TX1)")
    print(f"   Target Interval: {TX_INTERVAL} detik ({1.0/TX_INTERVAL if TX_INTERVAL > 0 else 0:.1f} Hz)")
    print("=" * 65)
    
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1, rtscts=False)
    except Exception as e:
        print(f"❌ GAGAL membuka port serial {port}!")
        print(f"   Detail Error : {e}")
        return None

    # 1. Tes Komunikasi AT
    print("[1/4] Menguji komunikasi AT pada UART1...")
    res = send_at_command(ser, "AT", timeout=1.0)
    if not any("OK" in line for line in res):
        time.sleep(0.3)
        res = send_at_command(ser, "AT", timeout=1.0)
        if not any("OK" in line for line in res):
            print("   ❌ ERROR: Modul RAK3272S tidak merespon perintah AT.")
            ser.close()
            return None
    print("   ✅ Respon AT OK! Modul terdeteksi di UART1.")

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

    # 3. Nonaktifkan mode listen jika sebelumnya aktif
    send_at_command(ser, "AT+PRECV=0", timeout=0.3)

    # 4. Konfigurasi Parameter Radio P2P Ultra-Fast (BW=500kHz)
    p2p_cmd = f"AT+P2P={P2P_FREQ}:{P2P_SF}:{P2P_BW}:{P2P_CR}:{P2P_PREAMBLE}:{P2P_TXPOWER}"
    print(f"[2/4] Mengatur Radio Ultra-Fast: {p2p_cmd}")
    res_p2p = send_at_command(ser, p2p_cmd, timeout=1.5)
    if not any("OK" in line for line in res_p2p):
        print(f"   ❌ Gagal mengatur parameter P2P! Respon: {res_p2p}")
        ser.close()
        return None
        
    print(f"   ✅ Konfigurasi Radio Aktif:")
    print(f"      - Frekuensi   : {P2P_FREQ / 1e6} MHz")
    print(f"      - Bandwidth   : 500 kHz (Ultra Fast)")
    print(f"      - Spreading F : SF{P2P_SF}")
    print(f"      - Time-on-Air : ~15-25 milidetik!")

    print("=" * 65)
    print("🚀 TRANSMITTER SIAP MENGIRIM DENGAN KECEPATAN TINGGI!")
    print("=" * 65 + "\n")
    return ser


def generate_payload(mode, counter, node_id):
    if mode == 'COMPACT_JSON':
        # JSON ringkas agar ukuran paket kecil dan transmisi cepat
        data = {
            "n": node_id,
            "c": counter,
            "t": round(25.0 + (counter % 10) * 0.5, 1),
            "h": round(60.0 + (counter % 5) * 1.0, 1)
        }
        raw_bytes = json.dumps(data, separators=(',', ':')).encode('utf-8')
        human_readable = json.dumps(data, separators=(',', ':'))
    elif mode == 'FAST_TEXT':
        text = f"P#{counter} N{node_id}"
        raw_bytes = text.encode('utf-8')
        human_readable = text
    elif mode == 'FAST_BINARY':
        # 8-byte ultra lightweight struct: Node(1B) + Counter(4B) + Temp*10(2B) + Hum(1B)
        temp_scaled = int((25.0 + (counter % 10) * 0.5) * 10)
        hum_int = int(60 + (counter % 5))
        raw_bytes = struct.pack('<BIHB', node_id, counter, temp_scaled, hum_int)
        human_readable = f"Binary 8B: Node {node_id}, Cnt {counter}, T={temp_scaled/10}C, H={hum_int}%"
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
    done_received = False
    ok_received = False

    while (time.time() - start_time) < timeout:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                if "+EVT:TXP2P DONE" in line:
                    done_received = True
                    break
                if "OK" in line:
                    ok_received = True
                if "AT_BUSY_ERROR" in line:
                    return False, f"Modul Sibuk ({line})"
                if "AT_PARAM_ERROR" in line or "AT_ERROR" in line:
                    return False, f"Error Parameter ({line})"
        time.sleep(0.002)  # Polling cepat 2ms untuk latensi minimum

    elapsed_ms = (time.time() - start_time) * 1000

    if done_received:
        return True, f"DONE ({elapsed_ms:.1f} ms)"
    elif ok_received:
        return True, f"OK ({elapsed_ms:.1f} ms)"
    else:
        return False, f"Timeout ({elapsed_ms:.1f} ms)"


def main():
    ser = init_rak3272s_tx(SERIAL_PORT, BAUDRATE)
    if not ser:
        sys.exit(1)

    counter = 1
    t_start_all = time.time()
    last_stat_time = time.time()
    stat_counter = 0

    speed_desc = f"setiap {TX_INTERVAL} detik (~{1.0/TX_INTERVAL if TX_INTERVAL > 0 else 0:.1f} paket/detik)" if TX_INTERVAL > 0 else "MAKSIMUM STREAMING (0 jeda)"
    print(f"Transmisi aktif: {speed_desc}")
    print("Tekan Ctrl+C untuk berhenti.\n")

    try:
        while True:
            t0 = time.time()
            hex_payload, desc, byte_len = generate_payload(PAYLOAD_MODE, counter, NODE_ID)

            success, msg = send_packet(ser, hex_payload, timeout=2.0)
            t_tx = (time.time() - t0) * 1000

            stat_counter += 1
            now = time.time()
            if now - last_stat_time >= 1.0:
                hz = stat_counter / (now - last_stat_time)
                rate_str = f"| Speed: {hz:.1f} pkt/s ({hz:.1f} Hz)"
                stat_counter = 0
                last_stat_time = now
            else:
                rate_str = ""

            status_icon = "⚡" if success else "❌"
            timestamp = time.strftime('%H:%M:%S')
            print(f"[{timestamp}] {status_icon} [#{counter:05d}] {desc} ({byte_len}B) -> {msg} {rate_str}")

            counter += 1

            # Tunggu sisa waktu interval (jika ada)
            if TX_INTERVAL > 0:
                spent = time.time() - t0
                remainder = TX_INTERVAL - spent
                if remainder > 0:
                    time.sleep(remainder)

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
