#!/usr/bin/env python3
"""
=============================================================================
Tes Komunikasi AT RAK3272S
=============================================================================
Dapat digunakan untuk menguji:
1. Node TX : python3 cek.py /dev/ttyAMA3  (UART1 Header J5)
2. Node RX : python3 cek.py /dev/ttyAMA0  (UART2 Header J4)
=============================================================================
"""

import serial
import time
import sys
import os

# Tentukan port dari argumen terminal, atau auto-detect default
if len(sys.argv) > 1:
    PORT = sys.argv[1]
else:
    PORT = '/dev/ttyAMA3' if os.path.exists('/dev/ttyAMA3') else '/dev/ttyAMA0'

BAUDRATE = 115200

print("=" * 65)
print(f"[INFO] MENGUJI RESPON AT MODUL RAK3272S")
print(f"   Port Serial : {PORT} ({BAUDRATE} bps)")
print("=" * 65)

try:
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        timeout=2.0,
        rtscts=False,
        dsrdtr=False,
        xonxoff=False
    )

    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.2)

    print("1. Mengirim: AT\\r\\n")
    ser.write(b"AT\r\n")
    ser.flush()

    print("2. Menunggu respon dari modul...")
    start = time.time()
    response = b""

    while (time.time() - start) < 2.0:
        if ser.in_waiting > 0:
            chunk = ser.read(ser.in_waiting)
            response += chunk
            if b"OK" in response:
                break
        time.sleep(0.02)

    print("-" * 65)
    print(f"Raw Data Diterima ({len(response)} byte): {repr(response)}")
    text = response.decode('utf-8', errors='replace').strip()
    print(f"Teks Diterima: {repr(text)}")
    print("-" * 65)

    if b"OK" in response or "OK" in text:
        print(f"[OK] BERHASIL! RAK3272S merespon 'OK' dengan sempurna pada {PORT}!")
        
        # Cek versi firmware
        ser.write(b"AT+VER=?\r\n")
        ser.flush()
        time.sleep(0.3)
        ver = ser.read_all().decode('utf-8', errors='replace').strip()
        print(f"[INFO] Firmware: {ver}")
    else:
        print("[WARN] Belum ada respon OK. Periksa kembali sambungan kabel TX/RX.")

    ser.close()

except Exception as e:
    print(f"[ERROR] Error membuka port {PORT}: {e}")

print("=" * 65)
