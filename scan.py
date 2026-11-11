#!/usr/bin/env python3
"""
=============================================================================
Pindai Port Serial & Deteksi Modul RAK3272S
=============================================================================
Fungsi:
Mencari di port serial mana modul RAK3272S terhubung, merespon perintah AT,
dan menampilkan informasi firmware serta mode kerja saat ini.
=============================================================================
"""

import serial
import time
import glob

# Daftar kandidat port serial pada Raspberry Pi 5 & USB-to-TTL adapter
candidate_ports = [
    '/dev/ttyAMA0',
    '/dev/serial0',
    '/dev/ttyUSB0',
    '/dev/ttyUSB1',
    '/dev/ttyAMA1',
    '/dev/ttyS0'
]

# Tambahkan port USB lainnya jika ada
candidate_ports.extend(glob.glob('/dev/ttyUSB*'))
candidate_ports.extend(glob.glob('/dev/ttyACM*'))
# Buang duplikasi dengan tetap menjaga urutan
candidate_ports = list(dict.fromkeys(candidate_ports))

baudrates = [115200, 9600]

print("=" * 65)
print("🔍 MEMINDAI PORT SERIAL UNTUK MENDETEKSI RAK3272S...")
print("=" * 65)

found = False

for port in candidate_ports:
    for baud in baudrates:
        print(f"👉 Menguji {port} pada {baud} bps ...", end=" ", flush=True)
        try:
            ser = serial.Serial(port, baudrate=baud, timeout=0.6)
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            # Kirim perintah AT
            ser.write(b"AT\r\n")
            time.sleep(0.2)

            response = ser.read_all().decode('utf-8', errors='ignore').strip()

            if "OK" in response:
                print(f"✅ TERHUBUNG!")
                print(f"\n🎉 DITEMUKAN RAK3272S:")
                print(f"   Port Serial : {port}")
                print(f"   Baud Rate   : {baud}")

                # Cek versi firmware
                ser.write(b"AT+VER=?\r\n")
                time.sleep(0.2)
                ver_res = ser.read_all().decode('utf-8', errors='ignore').strip()
                print(f"   Versi FW    : {ver_res.replace(chr(10), ' ')}")

                # Cek mode kerja
                ser.write(b"AT+NWM=?\r\n")
                time.sleep(0.2)
                nwm_res = ser.read_all().decode('utf-8', errors='ignore').strip()
                print(f"   Mode Kerja  : {nwm_res.replace(chr(10), ' ')}")

                print(f"\n--> Silakan set 'SERIAL_PORT = \"{port}\"' di file tx.py dan rx.py Anda!\n")
                found = True
                ser.close()
                break
            else:
                print("❌ (Tidak ada respon OK)")

            ser.close()
        except Exception as e:
            print(f"⚠️ ({e})")
    
    if found:
        break

if not found:
    print("\n" + "=" * 65)
    print("❌ RAK3272S BELUM MERESPON DI SEMUA PORT.")
    print("=" * 65)
    print("📋 Hal-hal penting yang perlu diperiksa:")
    print("  1. Posisi Pin TX dan RX:")
    print("     - Pin 8 Pi 5 (GPIO 14 TX) HARUS ke Pin RX2 (Header J4 Pin 8) RAK3272S.")
    print("     - Pin 10 Pi 5 (GPIO 15 RX) HARUS ke Pin TX2 (Header J4 Pin 7) RAK3272S.")
    print("  2. Daya Modul:")
    print("     - Pin 1 Pi 5 (3.3V) ke Pin 3V3 RAK3272S (JANGAN sambung ke 5V!).")
    print("     - Pin 6 Pi 5 (GND) ke Pin GND RAK3272S.")
    print("  3. Antena:")
    print("     - Pastikan antena LoRa terpasang pada konektor IPEX/SMA RAK3272S.")
    print("  4. Aktivasi UART Pi 5:")
    print("     - Tambahkan 'dtparam=uart0=on' di /boot/firmware/config.txt")
    print("     - Reboot Raspberry Pi 5.")
    print("=" * 65)
