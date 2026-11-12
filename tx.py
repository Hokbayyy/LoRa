#!/usr/bin/env python3
"""
=============================================================================
RAK3272S LoRa P2P Transmitter (TX) Node - 80 Parameter Sensor (1 Paket Utuh)
=============================================================================
Daftar Parameter Sensor Lengkap (Total 80 Parameter Tanpa Fragmen):
  [A] ARDUINO UNO Q (23 Parameter):
      1. P. Lidar   : Raw (1)
      2. P. MQ      : Amonia (NH3), Karbon Monoksida (CO), Karbon Dioksida (CO2) (3)
      3. P. DFRobot : SMOKE (1)
      4. P. MPU     : Accel X/Y/Z (Raw & Kalman), Gyro X/Y/Z (Raw & Kalman) (12)
      5. P. ADXL    : Accel X/Y/Z (Raw & Kalman) (6)

  [B] COSMIC LORA AURORA V3 (12 Parameter):
      6. P. LSM6DS3 : accelx, accely, accelz, gyrox, gyroy, gyroz (6)
      7. P. BME280  : bme_temp, bme_hum, bme_pres, bme_alt (4)
      8. P. SEN0232 : sound_dbLeft, sound_dbRight (2)

  [C] SENSOR PIEZZO & FSR (14 Parameter: C0 - C13):
      C0 : lengan kanan atas       C7 : punggung kiri bawah
      C1 : lengan kanan bawah      C8 : paha kanan atas
      C2 : lengan kiri atas        C9 : paha kanan tengah
      C3 : lengan kiri bawah       C10: paha kanan bawah
      C4 : punggung kanan atas     C11: paha kiri atas
      C5 : punggung kanan bawah    C12: paha kiri tengah
      C6 : punggung kiri atas      C13: paha kiri bawah

  [D] TCA9548A #1 (8 Parameter: SD0 - SD7):
      SD0: lengan kanan atas       SD4: punggung kanan atas
      SD1: lengan kanan bawah      SD5: punggung kanan bawah
      SD2: lengan kiri atas        SD6: punggung kiri atas
      SD3: lengan kiri bawah       SD7: punggung kiri bawah

  [E] TCA9548A #2 (6 Parameter: SD0 - SD5):
      SD0: paha kanan atas         SD3: paha kiri atas
      SD1: paha kanan tengah       SD4: paha kiri tengah
      SD2: paha kanan bawah        SD5: paha kiri bawah

  [F] SENSOR THERMAL (1 Parameter: TCA9548A #2 SD6):
      SD6: Sensor Thermal

  [G] SENSOR FLEX (16 Parameter: C0 - C15):
      C0 : bahu kanan depan        C8 : pinggang kanan depan
      C1 : bahu kanan belakang     C9 : pinggang kanan belakang
      C2 : bahu kiri depan         C10: pinggang kiri depan
      C3 : bahu kiri belakang      C11: pinggang kiri belakang
      C4 : sikut kanan depan       C12: lutut kanan depan
      C5 : sikut kanan belakang    C13: lutut kanan belakang
      C6 : sikut kiri depan        C14: lutut kiri depan
      C7 : sikut kiri belakang     C15: lutut kiri belakang

Total Parameter: 80 Sensor
Ukuran Paket Radio: Tepat 166 Byte (1 Paket Langsung Sekali Tembak, Tanpa Fragmen)
Batas Buffer LoRa SX1262: 256 Byte (Masih ada sisa 90 Byte bebas)
Output: 1 Baris Horizontal per Transmisi
=============================================================================
Penggunaan:
  python3 tx.py                  (Default: 80 param 1 paket langsung, interval 1s)
  python3 tx.py 0.5              (Transmisi super cepat tiap 0.5 detik)
  python3 tx.py 0.2              (Transmisi ultra cepat tiap 0.2 detik)
=============================================================================
"""

import serial
import time
import struct
import sys
import os

# ================= Konfigurasi Port & Radio =================
DEFAULT_PORT = '/dev/ttyAMA3'
SERIAL_PORT  = DEFAULT_PORT
TX_INTERVAL  = 1.0  # Default interval 1.0 detik

# Membaca argumen terminal
args = sys.argv[1:]
for arg in args:
    if arg.startswith('/dev/'):
        SERIAL_PORT = arg
    else:
        try:
            TX_INTERVAL = float(arg)
        except ValueError:
            pass

BAUDRATE = 115200
NODE_ID  = 1

# Pengaturan Radio LoRa P2P (WAJIB SAMA PERSIS DENGAN RECEIVER)
P2P_FREQ     = 923200000  # 923.2 MHz (AS923)
P2P_SF       = 7          # SF7 (Paling cepat & responsif di 125kHz)
P2P_BW       = 0          # 0: 125kHz (Standar stabil RUI3)
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


def init_rak3272s_tx(port, baudrate):
    print("=" * 70)
    print("[INFO] INISIALISASI TRANSMITTER (TX) - 80 PARAMETER SENSOR")
    print(f"   Raspberry Pi 5 : UART3 ({port})")
    print(f"   Modul RAK3272S : UART1 (Header J5: Pin 5 RX1 & Pin 6 TX1)")
    print("   Mode Transmisi : 1 Paket Tunggal Langsung (166 Byte, Tanpa Fragmen)")
    print(f"   Target Interval: {TX_INTERVAL} detik")
    print("=" * 70)
    
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1, rtscts=False)
    except Exception as e:
        print(f"[ERROR] Gagal membuka port serial {port}!")
        print(f"   Detail Error : {e}")
        return None

    # 1. Tes Komunikasi AT pada UART1
    print("[1/3] Menguji komunikasi AT pada UART1...")
    res = send_at_command(ser, "AT", timeout=1.0)
    if not any("OK" in line for line in res):
        time.sleep(0.3)
        res = send_at_command(ser, "AT", timeout=1.0)
        if not any("OK" in line for line in res):
            print("   [ERROR] Modul RAK3272S tidak merespon perintah AT di UART1.")
            ser.close()
            return None
    print("   [OK] Respon AT OK! Modul terdeteksi di UART1.")

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
        print("   [OK] Sukses beralih ke Mode P2P.")
    else:
        print("   [OK] Modul sudah berada dalam Mode P2P (NWM=0).")

    # 3. Konfigurasi Parameter Radio P2P
    p2p_cmd = f"AT+P2P={P2P_FREQ}:{P2P_SF}:{P2P_BW}:{P2P_CR}:{P2P_PREAMBLE}:{P2P_TXPOWER}"
    print(f"[2/3] Mengatur Parameter Radio P2P: {p2p_cmd}")
    res_p2p = send_at_command(ser, p2p_cmd, timeout=1.5)
    if not any("OK" in line for line in res_p2p):
        print(f"   [ERROR] Gagal mengatur parameter P2P! Respon: {res_p2p}")
        ser.close()
        return None
        
    print(f"   [OK] Konfigurasi Radio Aktif: {P2P_FREQ / 1e6} MHz | SF{P2P_SF} | BW 125kHz | Tx {P2P_TXPOWER}dBm")
    print("=" * 70)
    print("[INFO] TRANSMITTER SIAP (80 Parameter, 1 Paket Langsung Sekali Tembak)")
    print("=" * 70 + "\n")
    return ser


def get_80_sensor_data(counter, node_id=1):
    """Menghasilkan data lengkap 80 parameter sensor"""
    c = counter
    d = {
        "node": node_id,
        "cnt": c,
        
        # [A] Arduino Uno Q (23)
        "lidar_raw": round(145.0 + (c % 20) * 2.5, 1),
        "mq_nh3": round(12.4 + (c % 10) * 0.3, 2),
        "mq_co": round(4.2 + (c % 8) * 0.1, 2),
        "mq_co2": round(415.0 + (c % 25) * 4.0, 1),
        "smoke": round(28.5 + (c % 12) * 0.8, 1),
        "mpu_acc_x_raw": round(0.051 + (c % 5) * 0.01, 3),
        "mpu_acc_y_raw": round(-0.019 + (c % 4) * 0.01, 3),
        "mpu_acc_z_raw": round(0.982 + (c % 3) * 0.01, 3),
        "mpu_gyro_x_raw": round(1.24 + (c % 6) * 0.05, 2),
        "mpu_gyro_y_raw": round(-0.43 + (c % 5) * 0.04, 2),
        "mpu_gyro_z_raw": round(0.13 + (c % 4) * 0.03, 2),
        "mpu_acc_x_kalman": round(0.049 + (c % 5) * 0.009, 3),
        "mpu_acc_y_kalman": round(-0.020 + (c % 4) * 0.009, 3),
        "mpu_acc_z_kalman": round(0.980 + (c % 3) * 0.008, 3),
        "mpu_gyro_x_kalman": round(1.22 + (c % 6) * 0.048, 2),
        "mpu_gyro_y_kalman": round(-0.42 + (c % 5) * 0.039, 2),
        "mpu_gyro_z_kalman": round(0.12 + (c % 4) * 0.029, 2),
        "adxl_acc_x_raw": round(0.048 + (c % 5) * 0.011, 3),
        "adxl_acc_y_raw": round(-0.022 + (c % 4) * 0.011, 3),
        "adxl_acc_z_raw": round(0.979 + (c % 3) * 0.009, 3),
        "adxl_acc_x_kalman": round(0.050 + (c % 5) * 0.010, 3),
        "adxl_acc_y_kalman": round(-0.021 + (c % 4) * 0.010, 3),
        "adxl_acc_z_kalman": round(0.981 + (c % 3) * 0.010, 3),
        
        # [B] Cosmic LoRa Aurora V3 (12)
        "accelx": round(0.052 + (c % 5) * 0.01, 3),
        "accely": round(-0.017 + (c % 4) * 0.01, 3),
        "accelz": round(0.985 + (c % 3) * 0.01, 3),
        "gyrox": round(1.26 + (c % 6) * 0.05, 2),
        "gyroy": round(-0.41 + (c % 5) * 0.04, 2),
        "gyroz": round(0.14 + (c % 4) * 0.03, 2),
        "bme_temp": round(28.45 + (c % 10) * 0.2, 2),
        "bme_hum": round(62.3 + (c % 8) * 0.5, 1),
        "bme_pres": round(1013.25 + (c % 6) * 0.3, 2),
        "bme_alt": round(154.2 + (c % 5) * 0.4, 1),
        "sound_dbLeft": round(54.2 + (c % 15) * 0.8, 1),
        "sound_dbRight": round(56.1 + (c % 15) * 0.9, 1),
        
        # [C] Sensor Piezzo & FSR (14: C0 - C13)
        "pzo_r_arm_up": round(320.0 + (c % 10) * 12.0, 1),
        "pzo_r_arm_low": round(315.0 + (c % 8) * 11.5, 1),
        "pzo_l_arm_up": round(318.0 + (c % 9) * 10.0, 1),
        "pzo_l_arm_low": round(312.0 + (c % 7) * 9.5, 1),
        "pzo_r_back_up": round(410.0 + (c % 11) * 14.0, 1),
        "pzo_r_back_low": round(405.0 + (c % 10) * 13.0, 1),
        "pzo_l_back_up": round(412.0 + (c % 12) * 14.5, 1),
        "pzo_l_back_low": round(408.0 + (c % 9) * 12.5, 1),
        "pzo_r_thigh_up": round(520.0 + (c % 15) * 16.0, 1),
        "pzo_r_thigh_mid": round(515.0 + (c % 14) * 15.0, 1),
        "pzo_r_thigh_low": round(510.0 + (c % 13) * 14.0, 1),
        "pzo_l_thigh_up": round(522.0 + (c % 15) * 16.5, 1),
        "pzo_l_thigh_mid": round(518.0 + (c % 14) * 15.5, 1),
        "pzo_l_thigh_low": round(512.0 + (c % 13) * 14.5, 1),
        
        # [D] TCA9548A #1 (8: SD0 - SD7)
        "tca1_r_arm_up": round(510.0 + (c % 10) * 8.0, 1),
        "tca1_r_arm_low": round(508.0 + (c % 8) * 7.5, 1),
        "tca1_l_arm_up": round(512.0 + (c % 9) * 8.5, 1),
        "tca1_l_arm_low": round(505.0 + (c % 7) * 7.0, 1),
        "tca1_r_back_up": round(600.0 + (c % 12) * 10.0, 1),
        "tca1_r_back_low": round(595.0 + (c % 11) * 9.5, 1),
        "tca1_l_back_up": round(605.0 + (c % 12) * 10.5, 1),
        "tca1_l_back_low": round(598.0 + (c % 10) * 9.0, 1),
        
        # [E] TCA9548A #2 (6: SD0 - SD5)
        "tca2_r_thigh_up": round(450.0 + (c % 10) * 11.0, 1),
        "tca2_r_thigh_mid": round(445.0 + (c % 9) * 10.5, 1),
        "tca2_r_thigh_low": round(440.0 + (c % 8) * 10.0, 1),
        "tca2_l_thigh_up": round(452.0 + (c % 10) * 11.5, 1),
        "tca2_l_thigh_mid": round(448.0 + (c % 9) * 10.8, 1),
        "tca2_l_thigh_low": round(442.0 + (c % 8) * 10.2, 1),
        
        # [F] Sensor Thermal (1: TCA9548A #2 SD6)
        "thermal_temp": round(36.5 + (c % 10) * 0.15, 2),
        
        # [G] Sensor Flex (16: C0 - C15)
        "flex_r_shldr_f": round(180.0 + (c % 10) * 5.0, 1),
        "flex_r_shldr_b": round(182.0 + (c % 9) * 5.2, 1),
        "flex_l_shldr_f": round(179.0 + (c % 10) * 4.8, 1),
        "flex_l_shldr_b": round(181.0 + (c % 9) * 5.1, 1),
        "flex_r_elbw_f": round(210.0 + (c % 12) * 7.0, 1),
        "flex_r_elbw_b": round(212.0 + (c % 11) * 7.2, 1),
        "flex_l_elbw_f": round(208.0 + (c % 12) * 6.8, 1),
        "flex_l_elbw_b": round(211.0 + (c % 11) * 7.1, 1),
        "flex_r_waist_f": round(165.0 + (c % 8) * 4.0, 1),
        "flex_r_waist_b": round(168.0 + (c % 8) * 4.2, 1),
        "flex_l_waist_f": round(164.0 + (c % 8) * 3.9, 1),
        "flex_l_waist_b": round(167.0 + (c % 8) * 4.1, 1),
        "flex_r_knee_f": round(230.0 + (c % 15) * 8.0, 1),
        "flex_r_knee_b": round(235.0 + (c % 14) * 8.5, 1),
        "flex_l_knee_f": round(228.0 + (c % 15) * 7.8, 1),
        "flex_l_knee_b": round(233.0 + (c % 14) * 8.3, 1)
    }
    return d


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


def send_single_raw_packet(ser, raw_bytes, timeout=1.5):
    """Mengirim 1 paket biner via AT+PSEND (Maks 245 Byte per paket)"""
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
                if "OK" in line:
                    ok_received = True
                if "+EVT:TXP2P DONE" in line or "+EVT:TX_DONE" in line:
                    done_received = True
                    break
        else:
            if ok_received:
                time.sleep(0.005)
                if (time.time() - start_time) > 0.08:
                    done_received = True
                    break
            else:
                time.sleep(0.005)

    elapsed_ms = (time.time() - start_time) * 1000
    if done_received or ok_received:
        return True, f"{elapsed_ms:.0f}ms"
    else:
        return False, f"Timeout! {log_responses}"


def pack_80_sensors(counter, data_dict):
    """
    Mengemas 80 nilai float menjadi 1 paket biner (166 Byte).
    Format:
      - 4 Byte magic: b"P80:"
      - 2 Byte counter (uint16)
      - 160 Byte data (80 nilai x 2 Byte half-precision float)
    Total: 166 Byte (Jauh di bawah batas buffer LoRa 256B).
    """
    values = [float(data_dict[k]) for k in SENSOR_80_KEYS]
    payload = b"P80:" + struct.pack("<H80e", counter % 65536, *values)
    return payload


def main():
    ser = init_rak3272s_tx(SERIAL_PORT, BAUDRATE)
    if not ser:
        sys.exit(1)

    counter = 1

    print(f"[INFO] Transmisi 80 parameter sensor (1 Paket Langsung) setiap {TX_INTERVAL} detik.")
    print("Tekan Ctrl+C untuk berhenti.\n")

    try:
        while True:
            t_pkt_start = time.time()
            timestamp = time.strftime('%H:%M:%S')

            # Hasilkan seluruh 80 parameter data
            data_dict = get_80_sensor_data(counter, NODE_ID)
            h_str = format_80_horizontal(data_dict)

            # Kemas 80 sensor ke dalam 1 paket tunggal (166 Byte)
            payload = pack_80_sensors(counter, data_dict)

            success, msg = send_single_raw_packet(ser, payload, timeout=1.5)
            status_str = f"OK {msg}" if success else "WARN"

            print(f"[{timestamp}] [TX #{counter:04d}] [1-Paket {len(payload)}B (80-Param) {status_str}] -> {h_str}")

            counter += 1

            # Hitung sisa jeda agar interval tepat
            elapsed = time.time() - t_pkt_start
            sleep_time = max(0.01, TX_INTERVAL - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print(f"\n[TRANSMITTER] Dihentikan oleh pengguna. Total {counter - 1} data terkirim.")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial port UART1 ditutup.")


if __name__ == '__main__':
    main()
