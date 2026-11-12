#!/usr/bin/env python3
"""
=============================================================================
RAK3272S LoRa P2P Transmitter (TX) Node - 35 Parameter (1 Paket Sekali Kirim)
=============================================================================
Daftar Parameter Sensor (Total 35 Parameter Sesuai Permintaan):
  [A] ARDUINO UNO Q:
      1. P. Lidar   : Raw
      2. P. MQ      : Amonia (NH3), Karbon Monoksida (CO), Karbon Dioksida (CO2)
      3. P. DFRobot : SMOKE
      4. P. MPU     : Akselerasi Sumbu X/Y/Z (Raw & Kalman), 
                      Angular Sumbu X/Y/Z (Raw & Kalman) -> 12 Parameter
      5. P. ADXL    : Akselerasi Sumbu X/Y/Z (Raw & Kalman) -> 6 Parameter

  [B] COSMIC LORA AURORA V3:
      6. P. LSM6DS3 : accelx, accely, accelz, gyrox, gyroy, gyroz -> 6 Param
      7. P. BME280  : bme_temp, bme_hum, bme_pres, bme_alt -> 4 Param
      8. P. SEN0232 : sound_dbLeft, sound_dbRight -> 2 Param

Mode Transmisi:
  - Default: 1 Paket Langsung Sekali Kirim (~210 Byte). Tidak ada fragmen!
    Menggunakan seluruh 35 parameter secara instan (< 256B batas chip SX1262).
  - Output: 1 Baris Horizontal per Transmisi.
=============================================================================
Penggunaan:
  python3 tx.py                  (Default: 35 param 1 paket langsung, interval 1s)
  python3 tx.py 0.5              (Transmisi super cepat tiap 0.5 detik)
  python3 tx.py 0.2              (Transmisi ultra cepat tiap 0.2 detik)
=============================================================================
"""

import serial
import time
import json
import sys
import os

# ================= Konfigurasi Port & Radio =================
DEFAULT_PORT = '/dev/ttyAMA3'
SERIAL_PORT  = DEFAULT_PORT
TX_INTERVAL  = 1.0  # Default interval 1.0 detik
PAYLOAD_MODE = 'SINGLE_PACKET_35'  # Default: 1 Paket Langsung Sekali Kirim

# Membaca argumen terminal
args = sys.argv[1:]
for arg in args:
    if arg == 'json':
        PAYLOAD_MODE = 'JSON_35_LONG'
    elif arg.startswith('/dev/'):
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
    print("[INFO] INISIALISASI TRANSMITTER (TX) - 35 PARAMETER SENSOR")
    print(f"   Raspberry Pi 5 : UART3 ({port})")
    print(f"   Modul RAK3272S : UART1 (Header J5: Pin 5 RX1 & Pin 6 TX1)")
    print(f"   Mode Transmisi : 1 Paket Tunggal Langsung (~210 Byte, Tanpa Fragmen)")
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
        
    print(f"   [OK] Konfigurasi Radio Aktif:")
    print(f"      - Frekuensi   : {P2P_FREQ / 1e6} MHz")
    print(f"      - Bandwidth   : 125 kHz")
    print(f"      - Spreading F : SF{P2P_SF}")
    print(f"      - Tx Power    : {P2P_TXPOWER} dBm")

    print("=" * 70)
    print("[INFO] TRANSMITTER SIAP (1 Paket Sekali Kirim Langsung)")
    print("=" * 70 + "\n")
    return ser


def get_35_sensor_data(counter, node_id=1):
    """
    Menghasilkan data lengkap 35 parameter sensor:
    - Arduino Uno Q (23 Parameter)
    - Cosmic LoRa Aurora V3 (12 Parameter)
    """
    return {
        "node": node_id,
        "cnt": counter,
        
        # === [1] ARDUINO UNO Q ===
        # P. Lidar (1)
        "lidar_raw": round(145.0 + (counter % 20) * 2.5, 1),
        
        # P. MQ (3)
        "mq_nh3": round(12.4 + (counter % 10) * 0.3, 2),
        "mq_co": round(4.2 + (counter % 8) * 0.1, 2),
        "mq_co2": round(415.0 + (counter % 25) * 4.0, 1),
        
        # P. DFRobot (1)
        "smoke": round(28.5 + (counter % 12) * 0.8, 1),
        
        # P. MPU (12 Parameter: 6 Raw + 6 Kalman)
        "mpu_acc_x_raw": round(0.051 + (counter % 5) * 0.01, 3),
        "mpu_acc_y_raw": round(-0.019 + (counter % 4) * 0.01, 3),
        "mpu_acc_z_raw": round(0.982 + (counter % 3) * 0.01, 3),
        "mpu_gyro_x_raw": round(1.24 + (counter % 6) * 0.05, 2),
        "mpu_gyro_y_raw": round(-0.43 + (counter % 5) * 0.04, 2),
        "mpu_gyro_z_raw": round(0.13 + (counter % 4) * 0.03, 2),
        "mpu_acc_x_kalman": round(0.049 + (counter % 5) * 0.009, 3),
        "mpu_acc_y_kalman": round(-0.020 + (counter % 4) * 0.009, 3),
        "mpu_acc_z_kalman": round(0.980 + (counter % 3) * 0.008, 3),
        "mpu_gyro_x_kalman": round(1.22 + (counter % 6) * 0.048, 2),
        "mpu_gyro_y_kalman": round(-0.42 + (counter % 5) * 0.039, 2),
        "mpu_gyro_z_kalman": round(0.12 + (counter % 4) * 0.029, 2),
        
        # P. ADXL (6 Parameter: 3 Raw + 3 Kalman)
        "adxl_acc_x_raw": round(0.048 + (counter % 5) * 0.011, 3),
        "adxl_acc_y_raw": round(-0.022 + (counter % 4) * 0.011, 3),
        "adxl_acc_z_raw": round(0.979 + (counter % 3) * 0.009, 3),
        "adxl_acc_x_kalman": round(0.050 + (counter % 5) * 0.010, 3),
        "adxl_acc_y_kalman": round(-0.021 + (counter % 4) * 0.010, 3),
        "adxl_acc_z_kalman": round(0.981 + (counter % 3) * 0.010, 3),
        
        # === [2] COSMIC LORA AURORA V3 ===
        # P. LSM6DS3 (6 Parameter)
        "accelx": round(0.052 + (counter % 5) * 0.01, 3),
        "accely": round(-0.017 + (counter % 4) * 0.01, 3),
        "accelz": round(0.985 + (counter % 3) * 0.01, 3),
        "gyrox": round(1.26 + (counter % 6) * 0.05, 2),
        "gyroy": round(-0.41 + (counter % 5) * 0.04, 2),
        "gyroz": round(0.14 + (counter % 4) * 0.03, 2),
        
        # P. BME280 (4 Parameter)
        "bme_temp": round(28.45 + (counter % 10) * 0.2, 2),
        "bme_hum": round(62.3 + (counter % 8) * 0.5, 1),
        "bme_pres": round(1013.25 + (counter % 6) * 0.3, 2),
        "bme_alt": round(154.2 + (counter % 5) * 0.4, 1),
        
        # P. SEN0232 (2 Parameter)
        "sound_dbLeft": round(54.2 + (counter % 15) * 0.8, 1),
        "sound_dbRight": round(56.1 + (counter % 15) * 0.9, 1)
    }


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


def send_single_raw_packet(ser, raw_bytes, timeout=1.5):
    """Mengirim 1 paket langsung via AT+PSEND (Maks 245 Byte per paket)"""
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


def main():
    ser = init_rak3272s_tx(SERIAL_PORT, BAUDRATE)
    if not ser:
        sys.exit(1)

    counter = 1

    print(f"[INFO] Transmisi 35 parameter sensor (1 Paket Langsung) setiap {TX_INTERVAL} detik.")
    print("Tekan Ctrl+C untuk berhenti.\n")

    try:
        while True:
            t_pkt_start = time.time()
            timestamp = time.strftime('%H:%M:%S')

            # Hasilkan 35 parameter data
            data_dict = get_35_sensor_data(counter, NODE_ID)
            h_str = format_35_horizontal(data_dict)

            # Mode Default: 1 Paket Tunggal Langsung (~210 Byte)
            p_values = [str(v) for k, v in data_dict.items() if k not in ('node', 'cnt')]
            payload = f"DAT35:{counter}:{','.join(p_values)}".encode('utf-8')

            success, msg = send_single_raw_packet(ser, payload, timeout=1.5)
            status_str = f"OK {msg}" if success else "WARN"

            print(f"[{timestamp}] [TX #{counter:04d}] [1-Paket {len(payload)}B {status_str}] -> {h_str}")

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
