#!/usr/bin/env python3
"""
=============================================================================
Pindai Port Serial & Baud Rate RAK3272S pada Raspberry Pi 5
=============================================================================
"""

import serial
import time
import glob

# Daftar kandidat port serial, prioritaskan /dev/ttyAMA3 sesuai permintaan
candidate_ports = [
    '/dev/ttyAMA3',
    '/dev/ttyAMA0',
    '/dev/serial0',
    '/dev/ttyUSB0',
    '/dev/ttyUSB1'
]

candidate_ports.extend(glob.glob('/dev/ttyUSB*'))
candidate_ports.extend(glob.glob('/dev/ttyACM*'))
candidate_ports = list(dict.fromkeys(candidate_ports))

baudrates = [115200, 9600, 57600, 38400, 19200]

print("=" * 65)
print("🔍 MEMINDAI PORT SERIAL UNTUK MENDETEKSI RAK3272S...")
print("=" * 65)

found = False

for port in candidate_ports:
    for baud in baudrates:
        print(f"👉 Menguji {port} ({baud} bps) ...", end=" ", flush=True)
        try:
            ser = serial.Serial(port, baudrate=baud, timeout=0.6, rtscts=False)
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            ser.write(b"AT\r\n")
            ser.flush()
            time.sleep(0.3)

            response = ser.read_all().decode('utf-8', errors='replace').strip()

            if "OK" in response:
                print(f"✅ TERHUBUNG!")
                print(f"\n🎉 DITEMUKAN MODUL RAK3272S:")
                print(f"   Port Serial : {port}")
                print(f"   Baud Rate   : {baud}")

                # Cek FW
                ser.write(b"AT+VER=?\r\n")
                ser.flush()
                time.sleep(0.2)
                ver_res = ser.read_all().decode('utf-8', errors='replace').strip()
                print(f"   Versi FW    : {ver_res.replace(chr(10), ' ')}")

                ser.close()
                found = True
                break
            else:
                if response:
                    print(f"⚠️ Balasan ({len(response)} byte): {repr(response)}")
                else:
                    print("❌ (Kosong)")

            ser.close()
        except Exception as e:
            print(f"⚠️ ({e})")
    
    if found:
        break

if not found:
    print("\n" + "=" * 65)
    print("Modul RAK3272S belum terdeteksi 'OK'.")
    print("Periksa kembali kabel TX/RX, GND, dan 3.3V.")
    print("=" * 65)
