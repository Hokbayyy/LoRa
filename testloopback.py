#!/usr/bin/env python3
"""
=============================================================================
Uji Loopback UART3 Raspberry Pi 5 (/dev/ttyAMA3)
=============================================================================
Koneksi Loopback:
Hubungkan Pin 24 (GPIO 8 - TXD3) langsung ke Pin 21 (GPIO 9 - RXD3) dengan
satu kabel jumper.
=============================================================================
"""

import serial
import time
import os
import sys

PORT = '/dev/ttyAMA3'
BAUDRATE = 115200

print("=" * 65)
print(f"🔍 UJI LOOPBACK UART3 PI 5 PADA PORT: {PORT}")
print("📌 PERSIAPAN:")
print("   Hubungkan Pin 24 (TXD3) langsung ke Pin 21 (RXD3) pada header 40-pin")
print("   menggunakan sebuah kabel jumper.")
print("=" * 65)

if not os.path.exists(PORT):
    print(f"❌ Port {PORT} belum aktif di sistem!")
    print("👉 Tambahkan 'dtoverlay=uart3-pi5' di /boot/firmware/config.txt")
    print("   lalu reboot Pi 5: sudo reboot")
    sys.exit(1)

try:
    ser = serial.Serial(PORT, baudrate=BAUDRATE, timeout=1.5, rtscts=False)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.2)

    test_msg = b"TEST_UART3_PI5_LOOPBACK_12345\n"
    print(f"1. Mengirim ({len(test_msg)} byte) : {test_msg.decode().strip()}")
    ser.write(test_msg)
    ser.flush()

    time.sleep(0.1)
    received = ser.read(len(test_msg))
    print(f"2. Diterima ({len(received)} byte) : {received.decode(errors='replace').strip()}")
    print("-" * 65)

    if test_msg == received or test_msg.strip() in received:
        print("🎉 HASIL: 100% SUKSES! LOOPBACK UART3 BERHASIL!")
        print("   Pin 24 (TXD3) dan Pin 21 (RXD3) bekerja dengan sempurna.")
        print("   Port /dev/ttyAMA3 siap dihubungkan ke modul RAK3272S!")
    else:
        print("⚠️ HASIL: Data tidak cocok atau belum diterima.")

    ser.close()

except Exception as e:
    print(f"❌ Error: {e}")

print("=" * 65)
