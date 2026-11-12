#!/usr/bin/env python3
"""
=============================================================================
RAK3272S LoRa P2P Receiver (RX) Node - 35 Parameter Sensor (>250 Byte)
=============================================================================
Fitur:
  - Menerima dan me-reassemble data besar (>250 Byte / ~745 Byte) otomatis
  - Menampilkan 35 parameter sensor telemetri lengkap sesuai kategori:
      * ARDUINO UNO Q: Lidar, MQ, DFRobot, MPU (Raw & Kalman), ADXL (Raw & Kalman)
      * COSMIC LORA AURORA V3: LSM6DS3, BME280, SEN0232
  - Mendukung mode single packet CSV (CSV35:) maupun JSON utuh
=============================================================================
"""

import serial
import time
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
P2P_SF       = 7          # SF7
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
    print("=" * 68)
    print("📥 INISIALISASI RECEIVER (RX) - 35 PARAMETER SENSOR (>250 BYTE)")
    print(f"   Raspberry Pi   : Port {port}")
    print("   Modul RAK3272S : UART2 (Header J4: Pin 7 TX2 & Pin 8 RX2)")
    print(f"   Frekuensi      : {P2P_FREQ / 1e6} MHz | BW: 125 kHz | SF: {P2P_SF}")
    print("=" * 68)
    
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

    print("=" * 68)
    print("📡 RECEIVER STANDBY MENDENGARKAN 35 PARAMETER DATA...")
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


def format_35_sensor_display(d):
    """Menampilkan 35 parameter sensor dengan kategori rapi"""
    lines = []
    
    # 1. ARDUINO UNO Q
    lines.append("   📦 [ARDUINO UNO Q]")
    lines.append(f"      🔹 P. Lidar   : Raw = {d.get('lidar_raw', '-')}")
    lines.append(f"      🔹 P. MQ      : NH3 = {d.get('mq_nh3', '-')} ppm | CO = {d.get('mq_co', '-')} ppm | CO2 = {d.get('mq_co2', '-')} ppm")
    lines.append(f"      🔹 P. DFRobot : Smoke = {d.get('smoke', '-')} ppm")
    
    # MPU
    lines.append(f"      🔹 P. MPU Raw : Accel = ({d.get('mpu_acc_x_raw', '-')}, {d.get('mpu_acc_y_raw', '-')}, {d.get('mpu_acc_z_raw', '-')}) g")
    lines.append(f"                      Gyro  = ({d.get('mpu_gyro_x_raw', '-')}, {d.get('mpu_gyro_y_raw', '-')}, {d.get('mpu_gyro_z_raw', '-')}) dps")
    lines.append(f"      🔹 P. MPU Klmn: Accel = ({d.get('mpu_acc_x_kalman', '-')}, {d.get('mpu_acc_y_kalman', '-')}, {d.get('mpu_acc_z_kalman', '-')}) g")
    lines.append(f"                      Gyro  = ({d.get('mpu_gyro_x_kalman', '-')}, {d.get('mpu_gyro_y_kalman', '-')}, {d.get('mpu_gyro_z_kalman', '-')}) dps")
    
    # ADXL
    lines.append(f"      🔹 P. ADXL Raw: Accel = ({d.get('adxl_acc_x_raw', '-')}, {d.get('adxl_acc_y_raw', '-')}, {d.get('adxl_acc_z_raw', '-')}) g")
    lines.append(f"      🔹 P. ADXL Klm: Accel = ({d.get('adxl_acc_x_kalman', '-')}, {d.get('adxl_acc_y_kalman', '-')}, {d.get('adxl_acc_z_kalman', '-')}) g")
    
    # 2. COSMIC LORA AURORA V3
    lines.append("   🌌 [COSMIC LORA AURORA V3]")
    ax = d.get('accelx', d.get('lsm_accel_x', '-'))
    ay = d.get('accely', d.get('lsm_accel_y', '-'))
    az = d.get('accelz', d.get('lsm_accel_z', '-'))
    gx = d.get('gyrox', d.get('lsm_gyro_x', '-'))
    gy = d.get('gyroy', d.get('lsm_gyro_y', '-'))
    gz = d.get('gyroz', d.get('lsm_gyro_z', '-'))
    lines.append(f"      🔹 P. LSM6DS3 : Accel = ({ax}, {ay}, {az}) g | Gyro = ({gx}, {gy}, {gz}) dps")
    lines.append(f"      🔹 P. BME280  : Temp = {d.get('bme_temp', '-')}°C | Hum = {d.get('bme_hum', '-')}% | Pres = {d.get('bme_pres', '-')} hPa | Alt = {d.get('bme_alt', '-')} m")
    s_l = d.get('sound_dbLeft', d.get('sound_db_left', '-'))
    s_r = d.get('sound_dbRight', d.get('sound_db_right', '-'))
    lines.append(f"      🔹 P. SEN0232 : Sound Left = {s_l} dB | Sound Right = {s_r} dB")
    
    return "\n".join(lines)


SENSOR_35_KEYS = [
    "lidar_raw", "mq_nh3", "mq_co", "mq_co2", "smoke",
    "mpu_acc_x_raw", "mpu_acc_y_raw", "mpu_acc_z_raw", "mpu_gyro_x_raw", "mpu_gyro_y_raw", "mpu_gyro_z_raw",
    "mpu_acc_x_kalman", "mpu_acc_y_kalman", "mpu_acc_z_kalman", "mpu_gyro_x_kalman", "mpu_gyro_y_kalman", "mpu_gyro_z_kalman",
    "adxl_acc_x_raw", "adxl_acc_y_raw", "adxl_acc_z_raw",
    "adxl_acc_x_kalman", "adxl_acc_y_kalman", "adxl_acc_z_kalman",
    "accelx", "accely", "accelz", "gyrox", "gyroy", "gyroz",
    "bme_temp", "bme_hum", "bme_pres", "bme_alt",
    "sound_dbLeft", "sound_dbRight"
]


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
                                    # Format: FRG:1/4:payload
                                    meta_part, chunk_data = raw_bytes[4:].split(b":", 1)
                                    part_str, total_str = meta_part.decode('utf-8').split("/")
                                    part_num = int(part_str)
                                    total_parts = int(total_str)

                                    if part_num == 1:
                                        frag_buffer = {}

                                    expected_total_parts = total_parts
                                    frag_buffer[part_num] = chunk_data

                                    print(f"[{timestamp}] 📦 Fragmen Diterima [{part_num}/{total_parts}] ({len(chunk_data)}B) | RSSI: {last_rssi} dBm, SNR: {last_snr} dB")

                                    # Jika semua fragmen sudah lengkap, reassemble!
                                    if len(frag_buffer) == expected_total_parts:
                                        full_raw = b"".join([frag_buffer[p] for p in sorted(frag_buffer.keys())])
                                        total_size = len(full_raw)
                                        packet_count += 1

                                        try:
                                            full_json = json.loads(full_raw.decode('utf-8'))
                                            node = full_json.get('node', 1)
                                            cnt = full_json.get('cnt', packet_count)
                                            param_count = len(full_json) - 2

                                            print("=" * 68)
                                            print(f"🎉 [{timestamp}] 📥 SUKSES REASSEMBLE! DATA BESAR DITERIMA (#{packet_count})")
                                            print(f"   Ukuran Total  : {total_size} Byte (Jauh > 250 Byte!)")
                                            print(f"   Jumlah Param  : {param_count} Parameter Sensor")
                                            print(f"   Sinyal Radio  : RSSI = {last_rssi} dBm | SNR = {last_snr} dB")
                                            print(f"   Node / Counter: Node {node} | Counter #{cnt}")
                                            print("   Rincian 35 Parameter Sensor:")
                                            print(format_35_sensor_display(full_json))
                                            print("=" * 68 + "\n")
                                        except Exception as e:
                                            print(f"   [Reassemble JSON Error]: {e}")

                                        frag_buffer = {}
                                        expected_total_parts = 0

                                except Exception as err:
                                    print(f"   [Fragment Parse Error]: {err}")

                            # 2. Cek apakah format CSV 35 Parameter (Mode Single Packet)
                            elif raw_bytes.startswith(b"CSV35:"):
                                packet_count += 1
                                raw_str = raw_bytes.decode('utf-8', errors='ignore')
                                parts = raw_str.split(':')
                                p_cnt = parts[1] if len(parts) > 1 else '?'
                                values = parts[2].split(',') if len(parts) > 2 else []

                                # Susun dictionary dari CSV
                                d_csv = {"node": 1, "cnt": p_cnt}
                                for idx, val in enumerate(values):
                                    if idx < len(SENSOR_35_KEYS):
                                        d_csv[SENSOR_35_KEYS[idx]] = val

                                print("=" * 68)
                                print(f"⚡ [{timestamp}] 📥 DATA 35 PARAMETER DITERIMA (Single Packet #{packet_count})")
                                print(f"   Ukuran Paket  : {len(raw_bytes)} Byte (Format CSV)")
                                print(f"   Sinyal Radio  : RSSI = {last_rssi} dBm | SNR = {last_snr} dB")
                                print(f"   Counter       : #{p_cnt}")
                                print("   Rincian 35 Parameter Sensor:")
                                print(format_35_sensor_display(d_csv))
                                print("=" * 68 + "\n")

                            # 3. Payload JSON kecil / teks biasa (Kompatibilitas lama)
                            else:
                                packet_count += 1
                                try:
                                    text = raw_bytes.decode('utf-8')
                                    # Coba parse sebagai json biasa jika ada
                                    if text.startswith('{') and text.endswith('}'):
                                        js = json.loads(text)
                                        print("=" * 68)
                                        print(f"📥 [{timestamp}] Paket #{packet_count} (JSON Standar {len(raw_bytes)}B) | RSSI: {last_rssi} dBm")
                                        print(f"   Data: {js}")
                                        print("=" * 68 + "\n")
                                    else:
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
