#!/usr/bin/env python3
"""
=============================================================================
Dengar Output Reset Modul RAK3272S pada Raspberry Pi 5 (/dev/ttyAMA3)
=============================================================================
Script ini mendengarkan data serial dari modul RAK3272S saat tombol RST ditekan.
Jika ada karakter yang masuk (banner RUI3 atau karakter acak), itu membuktikan:
1. Jalur trace PCB (TX RAK -> RX Pi 5) terhubung dengan benar.
2. Modul RAK menyala dan aktif.
=============================================================================
"""

import serial
import time
import sys

PORT = '/dev/ttyAMA3'
BAUDRATES = [115200, 9600]

print("=" * 65)
print(f"👂 MENDENGARKAN SERIAL PORT PI 5 ({PORT})")
print("=================================================================")

for baud in BAUDRATES:
    print(f"\n👉 Menguji pada Baud Rate: {baud} bps")
    try:
        ser = serial.Serial(PORT, baudrate=baud, timeout=0.5)
        ser.reset_input_buffer()
        print("👉 SEKARANG TEKAN & LEPAS TOMBOL RESET (RST) DI MODUL RAK3272S!")
        print("   Mendengarkan selama 5 detik...")
        
        start = time.time()
        captured = b""
        while time.time() - start < 5.0:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                captured += chunk
                print(f"   [DITERIMA]: {repr(chunk)}")
            time.sleep(0.05)
            
        if captured:
            print(f"✅ BERHASIL MENDETEKSI DATA DARI MODUL ({len(captured)} byte):")
            print(f"   Raw : {repr(captured)}")
            print(f"   Teks: {repr(captured.decode('utf-8', errors='replace'))}")
            ser.close()
            break
        else:
            print("   (Belum ada data masuk pada baud rate ini)")
        ser.close()
    except Exception as e:
        print(f"❌ Error membuka {PORT}: {e}")

print("\n" + "=" * 65)
