#!/usr/bin/env python3
"""
=============================================================================
RAK3272S LoRa P2P Receiver (RX) Node - 80 Parameter Sensor (1 Paket Utuh)
=============================================================================
Fitur:
  - Menerima 80 parameter sensor dalam 1 PAKET LANGSUNG SEKALI TEMBAK (166 Byte)
  - Tanpa fragmen, tanpa delay menunggu (instan diterima dan di-unpack)
  - Output: 1 BARIS HORIZONTAL PENUH per paket
  - Bebas emoji/emoticon (kompatibel penuh dengan terminal Linux/SSH)
=============================================================================
"""

import serial
import time
import struct
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

SENSOR_80_KEYS = [
    # 1. P. Lidar (1)
    "lidar_raw",
    # 2. P. MQ (3)
    "mq_nh3", "mq_co", "mq_co2",
    # 3. P. DFRobot (1)
    "smoke",
    # 4. P. MPU (12)
    "mpu_acc_x_raw", "mpu_acc_y_raw", "mpu_acc_z_raw",
    "mpu_gyro_x_raw", "mpu_gyro_y_raw", "mpu_gyro_z_raw",
    "mpu_acc_x_kalman", "mpu_acc_y_kalman", "mpu_acc_z_kalman",
    "mpu_gyro_x_kalman", "mpu_gyro_y_kalman", "mpu_gyro_z_kalman",
    # 5. P. ADXL (6)
    "adxl_acc_x_raw", "adxl_acc_y_raw", "adxl_acc_z_raw",
    "adxl_acc_x_kalman", "adxl_acc_y_kalman", "adxl_acc_z_kalman",
    # 6. P. LSM6DS3 (6)
    "accelx", "accely", "accelz", "gyrox", "gyroy", "gyroz",
    # 7. P. BME280 (4)
    "bme_temp", "bme_hum", "bme_pres", "bme_alt",
    # 8. P. SEN0232 (2)
    "sound_dbLeft", "sound_dbRight",
    # 9. Piezzo & FSR (14: C0 - C13)
    "pzo_r_arm_up", "pzo_r_arm_low", "pzo_l_arm_up", "pzo_l_arm_low",
    "pzo_r_back_up", "pzo_r_back_low", "pzo_l_back_up", "pzo_l_back_low",
    "pzo_r_thigh_up", "pzo_r_thigh_mid", "pzo_r_thigh_low",
    "pzo_l_thigh_up", "pzo_l_thigh_mid", "pzo_l_thigh_low",
    # 10. TCA9548A #1 (8: SD0 - SD7)
    "tca1_r_arm_up", "tca1_r_arm_low", "tca1_l_arm_up", "tca1_l_arm_low",
    "tca1_r_back_up", "tca1_r_back_low", "tca1_l_back_up", "tca1_l_back_low",
    # 11. TCA9548A #2 (6: SD0 - SD5)
    "tca2_r_thigh_up", "tca2_r_thigh_mid", "tca2_r_thigh_low",
    "tca2_l_thigh_up", "tca2_l_thigh_mid", "tca2_l_thigh_low",
    # 12. Thermal Sensor (1: SD6 on TCA #2)
    "thermal_temp",
    # 13. Flex Sensor (16: C0 - C15)
    "flex_r_shldr_f", "flex_r_shldr_b", "flex_l_shldr_f", "flex_l_shldr_b",
    "flex_r_elbw_f", "flex_r_elbw_b", "flex_l_elbw_f", "flex_l_elbw_b",
    "flex_r_waist_f", "flex_r_waist_b", "flex_l_waist_f", "flex_l_waist_b",
    "flex_r_knee_f", "flex_r_knee_b", "flex_l_knee_f", "flex_l_knee_b"
]


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
    print("[INFO] INISIALISASI RECEIVER (RX) - 80 PARAMETER SENSOR")
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
    print("[INFO] RECEIVER SIAP (80 Parameter, Format Output: Horizontal 1 Baris)")
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


def format_80_horizontal(d):
    """Menyusun seluruh 80 parameter sensor menjadi 1 baris horizontal bersih"""
    p = []
    p.append("Lidar:" + str(d.get("lidar_raw", "-")))
    p.append("MQ:" + str(d.get("mq_nh3", "-")) + "," + str(d.get("mq_co", "-")) + "," + str(d.get("mq_co2", "-")))
    p.append("Smoke:" + str(d.get("smoke", "-")))
    
    # MPU
    p.append("MPU_Raw:[A(" + str(d.get("mpu_acc_x_raw", "-")) + "," + str(d.get("mpu_acc_y_raw", "-")) + "," + str(d.get("mpu_acc_z_raw", "-")) + ") G(" + str(d.get("mpu_gyro_x_raw", "-")) + "," + str(d.get("mpu_gyro_y_raw", "-")) + "," + str(d.get("mpu_gyro_z_raw", "-")) + ")]")
    p.append("MPU_Klm:[A(" + str(d.get("mpu_acc_x_kalman", "-")) + "," + str(d.get("mpu_acc_y_kalman", "-")) + "," + str(d.get("mpu_acc_z_kalman", "-")) + ") G(" + str(d.get("mpu_gyro_x_kalman", "-")) + "," + str(d.get("mpu_gyro_y_kalman", "-")) + "," + str(d.get("mpu_gyro_z_kalman", "-")) + ")]")
    
    # ADXL
    p.append("ADXL_Raw:(" + str(d.get("adxl_acc_x_raw", "-")) + "," + str(d.get("adxl_acc_y_raw", "-")) + "," + str(d.get("adxl_acc_z_raw", "-")) + ")")
    p.append("ADXL_Klm:(" + str(d.get("adxl_acc_x_kalman", "-")) + "," + str(d.get("adxl_acc_y_kalman", "-")) + "," + str(d.get("adxl_acc_z_kalman", "-")) + ")")
    
    # LSM6DS3
    p.append("LSM6DS3:[A(" + str(d.get("accelx", "-")) + "," + str(d.get("accely", "-")) + "," + str(d.get("accelz", "-")) + ") G(" + str(d.get("gyrox", "-")) + "," + str(d.get("gyroy", "-")) + "," + str(d.get("gyroz", "-")) + ")]")
    
    # BME280 & SEN0232
    p.append("BME280:(" + str(d.get("bme_temp", "-")) + "C," + str(d.get("bme_hum", "-")) + "%," + str(d.get("bme_pres", "-")) + "hPa," + str(d.get("bme_alt", "-")) + "m)")
    p.append("Sound:(" + str(d.get("sound_dbLeft", "-")) + "," + str(d.get("sound_dbRight", "-")) + "dB)")
    
    # Piezzo & FSR (14 Ch: C0 - C13)
    pzo_keys = [
        "pzo_r_arm_up", "pzo_r_arm_low", "pzo_l_arm_up", "pzo_l_arm_low",
        "pzo_r_back_up", "pzo_r_back_low", "pzo_l_back_up", "pzo_l_back_low",
        "pzo_r_thigh_up", "pzo_r_thigh_mid", "pzo_r_thigh_low",
        "pzo_l_thigh_up", "pzo_l_thigh_mid", "pzo_l_thigh_low"
    ]
    p.append("PiezoFSR[C0-13]:(" + ",".join([str(d.get(k, "-")) for k in pzo_keys]) + ")")
    
    # TCA1 (8 Ch: SD0 - SD7)
    tca1_keys = [
        "tca1_r_arm_up", "tca1_r_arm_low", "tca1_l_arm_up", "tca1_l_arm_low",
        "tca1_r_back_up", "tca1_r_back_low", "tca1_l_back_up", "tca1_l_back_low"
    ]
    p.append("TCA1[SD0-7]:(" + ",".join([str(d.get(k, "-")) for k in tca1_keys]) + ")")
    
    # TCA2 (6 Ch: SD0 - SD5)
    tca2_keys = [
        "tca2_r_thigh_up", "tca2_r_thigh_mid", "tca2_r_thigh_low",
        "tca2_l_thigh_up", "tca2_l_thigh_mid", "tca2_l_thigh_low"
    ]
    p.append("TCA2[SD0-5]:(" + ",".join([str(d.get(k, "-")) for k in tca2_keys]) + ")")
    
    # Thermal (1 Ch)
    p.append("Thermal:" + str(d.get("thermal_temp", "-")) + "C")
    
    # Flex (16 Ch: C0 - C15)
    flex_keys = [
        "flex_r_shldr_f", "flex_r_shldr_b", "flex_l_shldr_f", "flex_l_shldr_b",
        "flex_r_elbw_f", "flex_r_elbw_b", "flex_l_elbw_f", "flex_l_elbw_b",
        "flex_r_waist_f", "flex_r_waist_b", "flex_l_waist_f", "flex_l_waist_b",
        "flex_r_knee_f", "flex_r_knee_b", "flex_l_knee_f", "flex_l_knee_b"
    ]
    p.append("Flex[C0-15]:(" + ",".join([str(d.get(k, "-")) for k in flex_keys]) + ")")
    
    return " | ".join(p)


def main():
    ser = init_rak3272s_rx(SERIAL_PORT, BAUDRATE)
    if not ser:
        sys.exit(1)

    packet_count = 0
    buffer = ""
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

                            # 1. Format 1 Paket Utama: P80: (80 Sensor Biner, 166 Byte Langsung)
                            if raw_bytes.startswith(b"P80:") and len(raw_bytes) >= 166:
                                packet_count += 1
                                try:
                                    cnt, = struct.unpack('<H', raw_bytes[4:6])
                                    vals = struct.unpack('<80e', raw_bytes[6:166])
                                    d_80 = {}
                                    for idx, val in enumerate(vals):
                                        if idx < len(SENSOR_80_KEYS):
                                            d_80[SENSOR_80_KEYS[idx]] = round(val, 3)

                                    h_str = format_80_horizontal(d_80)
                                    print(f"[{timestamp}] [RX #{cnt:04d}] [RSSI:{last_rssi}dBm SNR:{last_snr}dB] [1-Paket {len(raw_bytes)}B (80-Param)] -> {h_str}")
                                except Exception as err:
                                    print(f"[{timestamp}] [RX ERROR Unpack P80]: {err}")

                            # 2. Kompatibilitas Paket DAT35 / CSV35 Lama
                            elif raw_bytes.startswith(b"DAT35:") or raw_bytes.startswith(b"CSV35:"):
                                packet_count += 1
                                raw_str = raw_bytes.decode('utf-8', errors='ignore')
                                parts = raw_str.split(':')
                                p_cnt = parts[1] if len(parts) > 1 else str(packet_count)
                                values = parts[2].split(',') if len(parts) > 2 else []

                                d_dat = {"node": 1, "cnt": p_cnt}
                                for idx, val in enumerate(values):
                                    if idx < len(SENSOR_80_KEYS):
                                        d_dat[SENSOR_80_KEYS[idx]] = val

                                h_str = format_80_horizontal(d_dat)
                                print(f"[{timestamp}] [RX #{int(p_cnt):04d}] [RSSI:{last_rssi}dBm SNR:{last_snr}dB] [1-Paket {len(raw_bytes)}B] -> {h_str}")

                            # 3. Payload teks lainnya
                            else:
                                packet_count += 1
                                try:
                                    text = raw_bytes.decode('utf-8')
                                    print(f"[{timestamp}] [RX #{packet_count:04d}] [RSSI:{last_rssi}dBm] ({len(raw_bytes)}B): {text}")
                                except Exception:
                                    print(f"[{timestamp}] [RX #{packet_count:04d}] [RSSI:{last_rssi}dBm] ({len(raw_bytes)}B Hex): {parsed['hex']}")

            time.sleep(0.002)

    except KeyboardInterrupt:
        print(f"\n[RECEIVER] Berhenti. Total {packet_count} paket diterima.")
    finally:
        if ser and ser.is_open:
            print("Menghentikan mode receive (AT+PRECV=0)...")
            send_at_command(ser, "AT+PRECV=0", timeout=0.3)
            ser.close()
            print("Serial port ditutup.")


if __name__ == '__main__':
    main()
