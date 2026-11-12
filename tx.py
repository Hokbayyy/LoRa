#!/usr/bin/env python3
"""
=============================================================================
RAK3272S LoRa P2P Transmitter (TX) Node - 30 Parameter Sensor (>250 Byte)
=============================================================================
Konfigurasi:
  - Raspberry Pi 5  : UART3 (/dev/ttyAMA3)
  - Modul RAK3272S  : UART1 (Header J5: Pin 5 RX1 & Pin 6 TX1)
  - Frekuensi       : 923.2 MHz (AS923) | SF: 7 | BW: 125 kHz
  - Payload         : 30 Parameter Sensor Lengkap (508 Byte)
=============================================================================
Penggunaan:
  python3 tx.py                 (Default: 30 parameter, interval 1 detik)
  python3 tx.py 0.5             (Interval 0.5 detik)
  python3 tx.py 1 single        (Mode 1 Paket Tunggal 245 Byte)
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
TX_INTERVAL  = 1.0  # Default: 1.0 detik
PAYLOAD_MODE = '30_PARAMS_LONG'  # '30_PARAMS_LONG' (508B) atau '30_PARAMS_SINGLE_245B'

# Pintar membaca argumen terminal
args = sys.argv[1:]
for arg in args:
    if arg == 'single':
        PAYLOAD_MODE = '30_PARAMS_SINGLE_245B'
    elif arg.startswith('/dev/'):
        SERIAL_PORT = arg
    else:
        try:
            TX_INTERVAL = float(arg)
        except ValueError:
            pass

BAUDRATE = 115200

# Pengaturan Radio LoRa P2P (WAJIB PERSIS SAMA DENGAN RECEIVER)
P2P_FREQ     = 923200000  # 923.2 MHz (AS923)
P2P_SF       = 7          # SF7 (Spread Factor 6-12)
P2P_BW       = 0          # 0: 125kHz
P2P_CR       = 0          # 0: 4/5
P2P_PREAMBLE = 8          # Preamble 8
P2P_TXPOWER  = 20         # Tx Power 20 dBm

NODE_ID      = 1          # ID Node Pengirim
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
    print("🚀 INISIALISASI TRANSMITTER (TX) - 30 PARAMETER")
    print(f"   Raspberry Pi 5 : UART3 ({port})")
    print(f"   Modul RAK3272S : UART1 (Header J5: Pin 5 RX1 & Pin 6 TX1)")
    print(f"   Mode Payload   : {PAYLOAD_MODE}")
    print(f"   Target Interval: {TX_INTERVAL} detik")
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

    print("=" * 65)
    print(f"📡 TRANSMITTER SIAP MENGIRIM 30 PARAMETER DATA!")
    print("=" * 65 + "\n")
    return ser


def generate_30_params(counter, node_id):
    """Menghasilkan 30 parameter sensor telemetri realistis"""
    return {
        "node": node_id,
        "cnt": counter,
        # 1-5: Suhu (deg C)
        "temp_cpu": round(42.5 + (counter % 5) * 0.4, 2),
        "temp_env": round(28.3 + (counter % 7) * 0.3, 2),
        "temp_sensor": round(26.1 + (counter % 4) * 0.5, 2),
        "temp_batt": round(31.0 + (counter % 6) * 0.2, 2),
        "temp_motor": round(55.2 + (counter % 8) * 0.7, 2),
        # 6-10: Kelembapan & Tekanan
        "humidity": round(65.4 + (counter % 5) * 1.1, 2),
        "soil_moist": round(45.2 + (counter % 4) * 0.8, 2),
        "pressure_hpa": round(1013.25 + (counter % 3) * 0.5, 2),
        "altitude_m": round(152.4 + (counter % 5) * 0.2, 2),
        "dew_point": round(18.2 + (counter % 3) * 0.4, 2),
        # 11-15: Listrik / Daya
        "voltage_v": round(3.30 + (counter % 3) * 0.02, 2),
        "current_ma": round(450.5 + (counter % 10) * 5.2, 1),
        "power_mw": round(1485.0 + (counter % 10) * 12.0, 1),
        "batt_pct": max(10, 100 - (counter % 90)),
        "solar_v": round(5.12 + (counter % 4) * 0.1, 2),
        # 16-20: Kualitas Udara & Lingkungan
        "co2_ppm": 415 + (counter % 20) * 5,
        "tvoc_ppb": 120 + (counter % 15) * 4,
        "pm2_5": round(12.5 + (counter % 8) * 0.6, 1),
        "pm10": round(25.4 + (counter % 9) * 0.8, 1),
        "lux_light": 850 + (counter % 50) * 10,
        # 21-25: Dinamika / Getaran / Akselerometer
        "rpm": 1450 + (counter % 30) * 10,
        "accel_x": round(0.02 + (counter % 5) * 0.01, 3),
        "accel_y": round(-0.01 + (counter % 4) * 0.01, 3),
        "accel_z": round(0.98 + (counter % 3) * 0.01, 3),
        "vibration_g": round(0.12 + (counter % 6) * 0.02, 3),
        # 26-30: Sensor Fluida & Posisi
        "uv_index": round(3.2 + (counter % 4) * 0.2, 1),
        "ph_level": round(6.85 + (counter % 3) * 0.05, 2),
        "water_flow": round(15.4 + (counter % 5) * 0.3, 1),
        "tank_level_pct": round(85.0 - (counter % 10) * 0.5, 1),
        "gyro_z": round(0.15 + (counter % 5) * 0.05, 2)
    }


def send_single_raw_packet(ser, raw_bytes, timeout=2.0):
    """Mengirim 1 paket biner/teks via AT+PSEND (Maks 245 Byte per paket)"""
    hex_payload = raw_bytes.hex().upper()
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
            if ok_received and (time.time() - start_time) >= 0.12:
                done_received = True
                break
        time.sleep(0.005)

    elapsed_ms = (time.time() - start_time) * 1000

    if done_received or ok_received:
        return True, f"DONE ({elapsed_ms:.0f} ms)"
    else:
        return False, f"Timeout ({elapsed_ms:.0f} ms)! Respon: {log_responses}"


def send_large_data_fragmented(ser, raw_bytes, chunk_size=180):
    """
    Memecah data besar (>250 Byte) menjadi paket-paket terfragmen
    sehingga muat di batas buffer chip LoRa SX1262 (Maks 256B)
    """
    chunks = [raw_bytes[i:i + chunk_size] for i in range(0, len(raw_bytes), chunk_size)]
    total_parts = len(chunks)
    all_success = True
    total_ms = 0

    for idx, chunk in enumerate(chunks):
        tag = f"FRG:{idx + 1}/{total_parts}:".encode('utf-8')
        packet = tag + chunk
        
        t0 = time.time()
        success, msg = send_single_raw_packet(ser, packet, timeout=2.0)
        dt = (time.time() - t0) * 1000
        total_ms += dt

        if not success:
            all_success = False
            print(f"      ⚠️ Gagal fragmen [{idx+1}/{total_parts}]: {msg}")
            break
        
        time.sleep(0.03)  # Jeda mini 30ms antar fragmen

    return all_success, total_parts, total_ms


def main():
    ser = init_rak3272s_tx(SERIAL_PORT, BAUDRATE)
    if not ser:
        sys.exit(1)

    counter = 1
    t_start_all = time.time()

    print(f"Transmisi aktif setiap {TX_INTERVAL} detik (Mode: {PAYLOAD_MODE})...")
    print("Tekan Ctrl+C untuk berhenti.\n")

    try:
        while True:
            t_pkt_start = time.time()
            timestamp = time.strftime('%H:%M:%S')

            if PAYLOAD_MODE == '30_PARAMS_LONG':
                # Mode data besar (>250 Byte, sekitar 508 Byte)
                data_dict = generate_30_params(counter, NODE_ID)
                json_bytes = json.dumps(data_dict, separators=(',', ':')).encode('utf-8')
                total_bytes = len(json_bytes)

                print(f"[{timestamp}] 📤 [Paket #{counter:04d}] Mengirim 30 Parameter ({total_bytes} Byte > 250B)...")
                success, parts, ms = send_large_data_fragmented(ser, json_bytes, chunk_size=180)

                if success:
                    print(f"   STATUS : ⚡ SUKSES TERKIRIM! (Terbagi dalam {parts} Fragmen, Waktu Total: {ms:.0f} ms)")
                else:
                    print(f"   STATUS : ⚠️ Gagal sebagian fragmen")

            else:
                # Mode 1 paket tunggal pas 245 Byte
                data_dict = generate_30_params(counter, NODE_ID)
                p_values = [str(v) for k, v in data_dict.items() if k not in ('node', 'cnt')][:30]
                csv_payload = f"P30:{counter}:{','.join(p_values)}"
                # Pad ke tepat 245 byte (batas fisik LoRa 256B)
                pad_len = 245 - len(csv_payload)
                padded_bytes = (csv_payload + '#' * max(0, pad_len))[:245].encode('utf-8')

                success, msg = send_single_raw_packet(ser, padded_bytes, timeout=2.0)
                status_icon = "⚡" if success else "⚠️"
                print(f"[{timestamp}] {status_icon} [Paket #{counter:04d}] 30 Parameter (Single Packet: {len(padded_bytes)} Byte) -> {msg}")

            print("-" * 65)
            counter += 1

            # Tunggu sisa waktu interval
            elapsed = time.time() - t_pkt_start
            sleep_time = max(0.0, TX_INTERVAL - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        total_time = time.time() - t_start_all
        print(f"\n[TRANSMITTER] Selesai. Total {counter - 1} pengiriman dalam {total_time:.2f} detik.")
    finally:
        if ser and ser.is_open:
            ser.write(b"AT+PRECV=0\r\n")
            ser.flush()
            ser.close()
            print("Port serial ditutup.")


if __name__ == '__main__':
    main()
