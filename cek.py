#!/usr/bin/env python3
"""
=============================================================================
Tes Komunikasi AT RAK3272S (UART1) via Raspberry Pi 5 (UART3: /dev/ttyAMA3)
=============================================================================
Koneksi:
  - Pi 5 Pin 24 (TXD3) -> RAK3272S Header J5 Pin 5 (RX1)
  - Pi 5 Pin 21 (RXD3) -> RAK3272S Header J5 Pin 6 (TX1)
  - Pi 5 Pin 1  (3.3V) -> RAK3272S Header J5 Pin 9 (3V3)
  - Pi 5 Pin 6  (GND)  -> RAK3272S Header J5 Pin 7 (GND)
=============================================================================
"""

import serial
import time
import sys

PORT = '/dev/ttyAMA3'
BAUDRATE = 115200

print("=" * 65)
print(f"📡 MENGUJI AT: RAK3272S (UART1) <--> PI 5 (UART3: {PORT})...")
print("📌 Susunan Kabel Semuanya di Header J5 RAK3272S:")
print("   - Pin 24 Pi 5 (TXD3) -> Pin 5 RAK (RX1)")
print("   - Pin 21 Pi 5 (RXD3) -> Pin 6 RAK (TX1)")
print("   - Pin 1  Pi 5 (3.3V) -> Pin 9 RAK (3V3)")
print("   - Pin 6  Pi 5 (GND)  -> Pin 7 RAK (GND)")
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

    print("2. Menunggu respon dari UART1 modul...")
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
        print("🎉 BERHASIL! RAK3272S (UART1) merespon 'OK' via UART3 (/dev/ttyAMA3)!")
    else:
        print("⚠️ Belum ada respon OK. Coba periksa urutan pin di Header J5.")

    ser.close()

except Exception as e:
    print(f"❌ Error membuka port {PORT}: {e}")

print("=" * 65)
