#!/usr/bin/env python3
"""
=============================================================================
Uji Loopback Hardware UART Raspberry Pi 5
=============================================================================
Fungsi:
Menghubungkan Pin 8 (GPIO 14 - TXD0) langsung ke Pin 10 (GPIO 15 - RXD0)
menggunakan satu kabel jumper pendek untuk memastikan bahwa peripheral UART
Raspberry Pi 5 aktif dan bekerja normal.
=============================================================================
"""

import serial
import time
import sys

# Pada Raspberry Pi 5, GPIO 14 & 15 adalah UART0 (/dev/ttyAMA0)
PORT = '/dev/ttyAMA0'
BAUDRATE = 115200

print("=" * 60)
print(f"🔍 UJI LOOPBACK RASPBERRY PI 5 PADA PORT: {PORT}")
print("=" * 60)
print("📌 PERSIAPAN:")
print("   Hubungkan langsung Pin 8 (TXD) ke Pin 10 (RXD) pada header 40-pin Pi 5")
print("   menggunakan sebuah kabel jumper (tanpa modul RAK3272S terlebih dahulu).\n")

try:
    ser = serial.Serial(PORT, baudrate=BAUDRATE, timeout=1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    test_msg = b"TEST_RASPI5_UART_LOOPBACK_12345\r\n"
    print(f"1. Mengirim data uji : {test_msg.strip().decode()}")
    ser.write(test_msg)
    time.sleep(0.2)

    received = ser.read_all()
    print(f"2. Data yang diterima: {received.strip().decode(errors='replace')}")

    print("-" * 60)
    if test_msg.strip() in received:
        print("✅ HASIL: LOOPBACK BERHASIL (100% SUKSES)!")
        print(f"   Peripheral UART0 ({PORT}) pada Pi 5 berfungsi sempurna.")
        print("   Pin 8 (TX) dan Pin 10 (RX) dapat mengirim dan menerima data dengan benar.")
        print("   --> Sekarang Anda dapat menghubungkannya ke modul RAK3272S.")
    else:
        print("❌ HASIL: LOOPBACK GAGAL!")
        print("   Penyebab kemungkinan:")
        print("   1. Kabel jumper tidak terpasang kencang di Pin 8 dan Pin 10.")
        print("   2. UART0 belum diaktifkan di /boot/firmware/config.txt (dtparam=uart0=on).")
        print("   3. Port sedang digunakan oleh proses lain (cek: lsof /dev/ttyAMA0).")

    ser.close()

except Exception as e:
    print(f"❌ Error membuka port {PORT}: {e}")
    print("\n👉 Tips: Pastikan Anda menjalankan perintah dengan user yang memiliki izin dialout:")
    print("   sudo usermod -a -G dialout $USER && sudo reboot")

print("=" * 60)
