#!/usr/bin/env python3
"""
=============================================================================
RAK3272S LoRa P2P Receiver (RX) Node - 35 Parameter Sensor (>250 Byte)
=============================================================================
Fitur:
  - Menerima dan me-reassemble data besar (>250 Byte / ~718 Byte) otomatis
  - Menampilkan seluruh 35 parameter sensor dalam 1 BARIS HORIZONTAL PENUH
  - Bebas emoji/emoticon (kompatibel penuh dengan terminal Linux/SSH)
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
    print("=" * 70)
    print("[INFO] INISIALISASI RECEIVER (RX) - 35 PARAMETER SENSOR")
    print(f"   Raspberry Pi   : Port {port}")
    print("   Modul RAK3272S : UART2 (Header J4: Pin 7 TX2 & Pin 8 RX2)")
    print(f"   Frekuensi      : {P2P_FREQ / 1e6} MHz | BW: 125 kHz | SF: {P2P_SF}")
    print("=" * 70)
    
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1, rtscts=False)
    except Exception as e:
        print(f"[ERROR] Gagal membuka port serial {port}!")
        print(f"   Detail Error : {e}")
        return None

    # 1. Tes Komunikasi AT
    print("[1/4] Menguji komunikasi AT pada UART2...")
    res = send_at_command(ser, "AT", timeout=1.0)
    if not any("OK" in line for line in res):
        time.sleep(0.3)
        res = send_at_command(ser, "AT", timeout=1.0)
        if not any("OK" in line for line in res):
            print("   [ERROR] Modul RAK3272S tidak merespon perintah AT di UART2.")
            ser.close()
            return None
    print("   [OK] Respon AT OK! Modul terdeteksi di UART2.")

    # 2. Cek Mode Kerja (Harus LoRa P2P / NWM=0)
    res_nwm = send_at_command(ser, "AT+NWM=?", timeout=1.0)
    current_mode = None
    for line in res_nwm:
        if "AT+NWM=" in line:
            current_mode = line.split("=")[-1].strip()
            
    if current_mode != "0":
        print("   [WARN] Mengubah ke Mode P2P (AT+NWM=0)...")
        ser.write(b"AT+NWM=0\r\n")
        ser.flush()
        time.sleep(2.5)
        ser.reset_input_buffer()
        res_check = send_at_command(ser, "AT", timeout=1.0)
        if not any("OK" in line for line in res_check):
            print("   [ERROR] Gagal reboot ke mode P2P!")
            ser.close()
            return None

    # Matikan mode RX sebelumnya agar konfigurasi parameter tidak 'busy'
    send_at_command(ser, "AT+PRECV=0", timeout=0.5)

    # 3. Konfigurasi Parameter Radio P2P
    p2p_cmd = f"AT+P2P={P2P_FREQ}:{P2P_SF}:{P2P_BW}:{P2P_CR}:{P2P_PREAMBLE}:{P2P_TXPOWER}"
    print(f"[2/4] Mengatur Parameter Radio P2P: {p2p_cmd}")
    res_p2p = send_at_command(ser, p2p_cmd, timeout=1.5)
    if not any("OK" in line for line in res_p2p):
        print(f"   [ERROR] Gagal mengatur parameter P2P! Respon: {res_p2p}")
        ser.close()
        return None
        
    print(f"   [OK] Konfigurasi Radio Aktif: {P2P_FREQ / 1e6} MHz | SF{P2P_SF} | BW 125kHz")

    # 4. Aktifkan Continuous RX
    print("[3/4] Mengaktifkan Mode Continuous RX (AT+PRECV=65534)...")
    res_precv = send_at_command(ser, "AT+PRECV=65534", timeout=1.5)
    if not any("OK" in line for line in res_precv):
        print(f"   [WARN] Respon PRECV: {res_precv}")
    else:
        print("   [OK] Receiver aktif mendengarkan gelombang radio.")

    print("=" * 70)
    print("[INFO] RECEIVER SIAP MENERIMA (Format Output: Horizontal 1 Baris)")
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


def format_35_horizontal(d):
    """Menyusun seluruh 35 parameter sensor menjadi 1 baris horizontal bersih"""
    p = []
    p.append(f"Lidar:{d.get('lidar_raw', '-')}")
    p.append(f"MQ(NH3:{d.get('mq_nh3', '-')},CO:{d.get('mq_co', '-')},CO2:{d.get('mq_co2', '-')})")
    p.append(f"Smoke:{d.get('smoke', '-')}")
    
    # MPU
    p.append(f"MPU_Raw[A:({d.get('mpu_acc_x_raw', '-')},{d.get('mpu_acc_y_raw', '-')},{d.get('mpu_acc_z_raw', '-')}) G:({d.get('mpu_gyro_x_raw', '-')},{d.get('mpu_gyro_y_raw', '-')},{d.get('mpu_gyro_z_raw', '-')})]")
    p.append(f"MPU_Klm[A:({d.get('mpu_acc_x_kalman', '-')},{d.get('mpu_acc_y_kalman', '-')},{d.get('mpu_acc_z_kalman', '-')}) G:({d.get('mpu_gyro_x_kalman', '-')},{d.get('mpu_gyro_y_kalman', '-')},{d.get('mpu_gyro_z_kalman', '-')})]")
    
    # ADXL
    p.append(f"ADXL_Raw:({d.get('adxl_acc_x_raw', '-')},{d.get('adxl_acc_y_raw', '-')},{d.get('adxl_acc_z_raw', '-')})")
    p.append(f"ADXL_Klm:({d.get('adxl_acc_x_kalman', '-')},{d.get('adxl_acc_y_kalman', '-')},{d.get('adxl_acc_z_kalman', '-')})")
    
    # LSM6DS3
    ax = d.get('accelx', d.get('lsm_accel_x', '-'))
    ay = d.get('accely', d.get('lsm_accel_y', '-'))
    az = d.get('accelz', d.get('lsm_accel_z', '-'))
    gx = d.get('gyrox', d.get('lsm_gyro_x', '-'))
    gy = d.get('gyroy', d.get('lsm_gyro_y', '-'))
    gz = d.get('gyroz', d.get('lsm_gyro_z', '-'))
    p.append(f"LSM6DS3[A:({ax},{ay},{az}) G:({gx},{gy},{gz})]")
    
    # BME280
    p.append(f"BME280({d.get('bme_temp', '-')}C,{d.get('bme_hum', '-') }%,{d.get('bme_pres', '-')}hPa,{d.get('bme_alt', '-')}m)")
    
    # SEN0232
    sl = d.get('sound_dbLeft', d.get('sound_db_left', '-'))
    sr = d.get('sound_dbRight', d.get('sound_db_right', '-'))
    p.append(f"Sound(L:{sl},R:{sr}dB)")
    
    return " | ".join(p)


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
    frag_buffer = {}
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

                            # 1. Bagian dari data terfragmen (>250 Byte / JSON 35 Param)
                            if raw_bytes.startswith(b"FRG:"):
                                try:
                                    meta_part, chunk_data = raw_bytes[4:].split(b":", 1)
                                    part_str, total_str = meta_part.decode('utf-8').split("/")
                                    part_num = int(part_str)
                                    total_parts = int(total_str)

                                    if part_num == 1:
                                        frag_buffer = {}

                                    expected_total_parts = total_parts
                                    frag_buffer[part_num] = chunk_data

                                    # Jika semua fragmen sudah lengkap, cetak 1 baris horizontal!
                                    if len(frag_buffer) == expected_total_parts:
                                        full_raw = b"".join([frag_buffer[p] for p in sorted(frag_buffer.keys())])
                                        total_size = len(full_raw)
                                        packet_count += 1

                                        try:
                                            full_json = json.loads(full_raw.decode('utf-8'))
                                            cnt = full_json.get('cnt', packet_count)
                                            h_str = format_35_horizontal(full_json)
                                            print(f"[{timestamp}] [RX #{cnt:04d}] [RSSI:{last_rssi}dBm SNR:{last_snr}dB] [{total_size}B] -> {h_str}")
                                        except Exception as e:
                                            print(f"[{timestamp}] [RX ERROR Reassemble JSON]: {e}")

                                        frag_buffer = {}
                                        expected_total_parts = 0

                                except Exception as err:
                                    print(f"[{timestamp}] [RX ERROR Fragment Parse]: {err}")

                            # 2. Format CSV 35 Parameter (Mode Single Packet)
                            elif raw_bytes.startswith(b"CSV35:"):
                                packet_count += 1
                                raw_str = raw_bytes.decode('utf-8', errors='ignore')
                                parts = raw_str.split(':')
                                p_cnt = parts[1] if len(parts) > 1 else '?'
                                values = parts[2].split(',') if len(parts) > 2 else []

                                d_csv = {"node": 1, "cnt": p_cnt}
                                for idx, val in enumerate(values):
                                    if idx < len(SENSOR_35_KEYS):
                                        d_csv[SENSOR_35_KEYS[idx]] = val

                                h_str = format_35_horizontal(d_csv)
                                print(f"[{timestamp}] [RX #{int(p_cnt):04d}] [RSSI:{last_rssi}dBm SNR:{last_snr}dB] [CSV {len(raw_bytes)}B] -> {h_str}")

                            # 3. Payload JSON kecil / teks biasa (Kompatibilitas lama)
                            else:
                                packet_count += 1
                                try:
                                    text = raw_bytes.decode('utf-8')
                                    print(f"[{timestamp}] [RX #{packet_count:04d}] [RSSI:{last_rssi}dBm] ({len(raw_bytes)}B): {text}")
                                except Exception:
                                    print(f"[{timestamp}] [RX #{packet_count:04d}] [RSSI:{last_rssi}dBm] ({len(raw_bytes)}B Hex): {parsed['hex']}")

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
