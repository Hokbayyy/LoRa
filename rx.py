#!/usr/bin/env python3
"""
=============================================================================
RAK3272S LoRa P2P Receiver (RX) Node - Mendukung Data Besar & 30 Parameter
=============================================================================
Fitur:
  - Menerima dan me-reassemble data besar (>250 Byte / 508 Byte) secara otomatis
  - Menampilkan 30 parameter sensor telemetri lengkap
  - Mendukung paket tunggal 245 Byte dan JSON standar
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

# Pengaturan Radio LoRa P2P (WAJIB PERSIS SAMA DENGAN TRANSMITTER)
P2P_FREQ     = 923200000  # 923.2 MHz (AS923)
P2P_SF       = 7          # SF7 (Spread Factor 6-12)
P2P_BW       = 0          # 0: 125kHz
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
    print("📥 INISIALISASI RECEIVER (RX) - SIAP MENERIMA >250 BYTE")
    print(f"   Raspberry Pi   : Port {port}")
    print("   Modul RAK3272S : UART2 (Header J4: Pin 7 TX2 & Pin 8 RX2)")
    print(f"   Frekuensi      : {P2P_FREQ / 1e6} MHz | BW: 125 kHz | SF: {P2P_SF}")
    print("=" * 65)
    
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1, rtscts=False)
    except Exception as e:
        print(f"❌ GAGAL membuka port serial {port}!")
        print(f"   Detail Error : {e}")
        return None

    # 1. Tes Komunikasi AT
    print("[1/4] Menguji komunikasi AT pada UART2...")
    res = send_at_command(ser, "AT", timeout=1.0)
    if not any("OK" in line for line in res):
        time.sleep(0.3)
        res = send_at_command(ser, "AT", timeout=1.0)
        if not any("OK" in line for line in res):
            print("   ❌ ERROR: Modul RAK3272S tidak merespon perintah AT di UART2.")
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

    # Matikan mode RX sebelumnya agar konfigurasi parameter tidak 'busy'
    send_at_command(ser, "AT+PRECV=0", timeout=0.5)

    # 3. Konfigurasi Parameter Radio P2P
    p2p_cmd = f"AT+P2P={P2P_FREQ}:{P2P_SF}:{P2P_BW}:{P2P_CR}:{P2P_PREAMBLE}:{P2P_TXPOWER}"
    print(f"[2/4] Mengatur Parameter Radio P2P: {p2p_cmd}")
    res_p2p = send_at_command(ser, p2p_cmd, timeout=1.5)
    if not any("OK" in line for line in res_p2p):
        print(f"   ❌ Gagal mengatur parameter P2P! Respon: {res_p2p}")
        ser.close()
        return None
        
    print(f"   ✅ Konfigurasi Radio Aktif:")
    print(f"      - Frekuensi   : {P2P_FREQ / 1e6} MHz")
    print(f"      - Bandwidth   : 125 kHz")
    print(f"      - Spreading F : SF{P2P_SF}")

    # 4. Aktifkan Continuous RX
    print("[3/4] Mengaktifkan Mode Continuous RX (AT+PRECV=65534)...")
    res_precv = send_at_command(ser, "AT+PRECV=65534", timeout=1.5)
    if not any("OK" in line for line in res_precv):
        print(f"   ⚠️ Respon PRECV: {res_precv}")
    else:
        print("   ✅ Receiver aktif mendengarkan gelombang radio.")

    print("=" * 65)
    print("📡 RECEIVER STANDBY MENDENGARKAN DATA 30 PARAMETER...")
    print("Tekan Ctrl+C untuk berhenti.\n")
    return ser


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
    
    if " " in line:
        parts = line.split()
        if len(parts) >= 4:
            return {
                "rssi": parts[-3],
                "snr": parts[-2],
                "hex": parts[-1]
            }

    return None


def format_30_params_display(data_dict):
    """Menampilkan ringkasan 30 parameter sensor dengan rapi"""
    lines = []
    keys = [k for k in data_dict.keys() if k not in ('node', 'cnt')]
    
    # Kelompokkan per baris (3 parameter per baris)
    for i in range(0, len(keys), 3):
        chunk = keys[i:i+3]
        item_str = " | ".join([f"{k}: {data_dict[k]}" for k in chunk])
        lines.append(f"      🔹 {item_str}")
        
    return "\n".join(lines)


def main():
    ser = init_rak3272s_rx(SERIAL_PORT, BAUDRATE)
    if not ser:
        sys.exit(1)

    packet_count = 0
    buffer = ""
    frag_buffer = {}  # {part_num: chunk_bytes}
    expected_total_parts = 0
    last_rssi = None
    last_snr = None

    try:
        while True:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                buffer += chunk

                if '\n' in buffer:
                    lines = buffer.split('\n')
                    buffer = lines[-1]

                    for raw_line in lines[:-1]:
                        line = raw_line.strip()
                        if not line:
                            continue

                        parsed = parse_rx_event(line)
                        if parsed:
                            last_rssi = parsed['rssi']
                            last_snr = parsed['snr']
                            timestamp = time.strftime('%H:%M:%S')

                            try:
                                raw_bytes = bytes.fromhex(parsed['hex'])
                            except Exception:
                                continue

                            # 1. Cek apakah ini bagian dari data terfragmen (>250 Byte)
                            if raw_bytes.startswith(b"FRG:"):
                                try:
                                    # Format: FRG:1/3:data...
                                    meta_part, chunk_data = raw_bytes[4:].split(b":", 1)
                                    part_str, total_str = meta_part.decode('utf-8').split("/")
                                    part_num = int(part_str)
                                    total_parts = int(total_str)

                                    expected_total_parts = total_parts
                                    frag_buffer[part_num] = chunk_data

                                    print(f"[{timestamp}] 📦 Fragmen Diterima [{part_num}/{total_parts}] ({len(chunk_data)}B) | RSSI:{last_rssi}dBm SNR:{last_snr}dB")

                                    # Jika semua fragmen sudah lengkap, reassemble!
                                    if len(frag_buffer) == expected_total_parts:
                                        full_raw = b"".join([frag_buffer[p] for p in sorted(frag_buffer.keys())])
                                        total_size = len(full_raw)
                                        packet_count += 1

                                        try:
                                            full_json = json.loads(full_raw.decode('utf-8'))
                                            param_count = len(full_json) - 2  # kurangi node dan cnt

                                            print("=" * 65)
                                            print(f"🎉 [{timestamp}] 📥 SUKSES REASSEMBLE! DATA BESAR DITERIMA (#{packet_count})")
                                            print(f"   Ukuran Total  : {total_size} Byte (> 250 Byte!)")
                                            print(f"   Jumlah Param  : {param_count} Parameter Sensor Lengkap")
                                            print(f"   Sinyal Radio  : RSSI = {last_rssi} dBm | SNR = {last_snr} dB")
                                            print(f"   Node / Counter: Node {full_json.get('node', 1)} | Counter #{full_json.get('cnt', packet_count)}")
                                            print("   Rincian 30 Parameter:")
                                            print(format_30_params_display(full_json))
                                            print("=" * 65 + "\n")
                                        except Exception as e:
                                            print(f"   [Reassemble Error]: {e}")

                                        # Reset buffer fragmen
                                        frag_buffer = {}
                                        expected_total_parts = 0

                                except Exception as err:
                                    print(f"   [Fragment Parse Error]: {err}")

                            # 2. Cek apakah paket tunggal 245 Byte (Single Packet 30 Param)
                            elif raw_bytes.startswith(b"P30:"):
                                packet_count += 1
                                raw_str = raw_bytes.decode('utf-8', errors='ignore')
                                body = raw_str.split('#')[0]  # buang padding '#'
                                parts = body.split(':')
                                p_cnt = parts[1] if len(parts) > 1 else '?'
                                values = parts[2].split(',') if len(parts) > 2 else []

                                print("=" * 65)
                                print(f"⚡ [{timestamp}] 📥 PAKET TUNGGAL PENUH 245 BYTE (#{packet_count})")
                                print(f"   Ukuran Paket  : {len(raw_bytes)} Byte (Kapasitas Buffer Maksimal LoRa 256B)")
                                print(f"   Jumlah Param  : {len(values)} Parameter Sensor")
                                print(f"   Sinyal Radio  : RSSI = {last_rssi} dBm | SNR = {last_snr} dB")
                                print(f"   Nilai Sensor  : {values[:8]}... (+{len(values)-8} lagi)")
                                print("=" * 65 + "\n")

                            # 3. Payload standar lainnya (JSON kecil / teks biasa)
                            else:
                                packet_count += 1
                                try:
                                    text = raw_bytes.decode('utf-8')
                                    print(f"[{timestamp}] 📥 [Paket #{packet_count}] ({len(raw_bytes)}B): {text[:80]} | RSSI:{last_rssi}dBm")
                                except Exception:
                                    print(f"[{timestamp}] 📥 [Paket #{packet_count}] ({len(raw_bytes)}B Hex): {parsed['hex'][:60]} | RSSI:{last_rssi}dBm")

            time.sleep(0.002)

    except KeyboardInterrupt:
        print(f"\n[RECEIVER] Berhenti. Total {packet_count} paket/kumpulan data diterima.")
    finally:
        if ser and ser.is_open:
            print("Menghentikan mode receive (AT+PRECV=0)...")
            send_at_command(ser, "AT+PRECV=0", timeout=0.3)
            ser.close()
            print("Serial port ditutup.")


if __name__ == '__main__':
    main()
