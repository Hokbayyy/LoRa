#!/usr/bin/env python3
"""
=============================================================================
RAK3172 / RAK3272S YMODEM Firmware Flasher via UART
=============================================================================
Digunakan untuk meng-upload file binary (.bin) firmware ke modul RAK3272S
langsung dari Raspberry Pi 5 melalui port serial (misal: /dev/ttyAMA3).
=============================================================================
"""

import sys
import os
import time
import serial

SOH = 0x01  # 128 byte block
STX = 0x02  # 1024 byte block
EOT = 0x04  # End of Transmission
ACK = 0x06  # Acknowledge
NAK = 0x15  # Negative Acknowledge
CAN = 0x18  # Cancel
CRC16_C = 0x43  # 'C' request for CRC16


def calc_crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def send_ymodem_packet(ser, packet_num, data, block_size=128):
    if block_size == 128:
        header = bytes([SOH, packet_num & 0xFF, (~packet_num) & 0xFF])
    else:
        header = bytes([STX, packet_num & 0xFF, (~packet_num) & 0xFF])
    
    # Pad data to block size
    if len(data) < block_size:
        data = data + b'\x00' * (block_size - len(data))
    elif len(data) > block_size:
        data = data[:block_size]
        
    crc = calc_crc16(data)
    crc_bytes = bytes([(crc >> 8) & 0xFF, crc & 0xFF])
    
    packet = header + data + crc_bytes
    ser.write(packet)
    ser.flush()


def wait_for_char(ser, expected_chars, timeout=10.0):
    start = time.time()
    while time.time() - start < timeout:
        if ser.in_waiting > 0:
            ch = ser.read(1)
            if ch and ch[0] in expected_chars:
                return ch[0]
        time.sleep(0.01)
    return None


def flash_rak(port, bin_path, baudrate=115200):
    if not os.path.exists(bin_path):
        print(f"❌ File binary tidak ditemukan: {bin_path}")
        return False

    file_size = os.path.getsize(bin_path)
    file_name = os.path.basename(bin_path)
    print("=" * 65)
    print("🚀 MEMULAI PROSES FLASHING FIRMWARE RAK3272S")
    print(f"   Port Serial : {port} ({baudrate} bps)")
    print(f"   File Target : {file_name} ({file_size} bytes)")
    print("=" * 65)

    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception as e:
        print(f"❌ Gagal membuka serial port {port}: {e}")
        return False

    # 1. Pancing masuk ke Bootloader
    print("[1/5] Memeriksa komunikasi AT & beralih ke Bootloader...")
    ser.write(b"AT\r\n")
    time.sleep(0.3)
    ser.read_all()

    print("      Mengirim AT+BOOT...")
    ser.write(b"AT+BOOT\r\n")
    time.sleep(1.0)
    res_boot = ser.read_all().decode(errors='replace')
    print(f"      Respon boot: {repr(res_boot.strip())}")

    print("      Mengirim AT+UPDATE...")
    ser.write(b"AT+UPDATE\r\n")
    time.sleep(0.5)

    # 2. Tunggu sinyal 'C' dari RAK Bootloader
    print("[2/5] Menunggu sinyal 'C' (permintaan YMODEM) dari modul...")
    ch = wait_for_char(ser, [CRC16_C], timeout=6.0)
    if ch != CRC16_C:
        # Coba kirim enter atau AT+UPDATE sekali lagi
        ser.write(b"AT+UPDATE\r\n")
        ch = wait_for_char(ser, [CRC16_C], timeout=6.0)
        if ch != CRC16_C:
            print("❌ Tidak menerima sinyal 'C' dari bootloader modul!")
            print("👉 Pastikan port serial terhubung ke Header J4 (UART2) RAK3272S.")
            ser.close()
            return False

    print("      ✅ Sinyal 'C' diterima! Modul siap menerima firmware.")

    # 3. Kirim Packet 0 (Header informasi file)
    print(f"[3/5] Mengirim metadata file ({file_name}, {file_size} bytes)...")
    file_meta = f"{file_name}\x00{file_size} \x00".encode('utf-8')
    send_ymodem_packet(ser, 0, file_meta, block_size=128)

    ch = wait_for_char(ser, [ACK], timeout=3.0)
    if ch != ACK:
        print(f"❌ Gagal! Harusnya ACK, diterima: {hex(ch) if ch else 'None'}")
        ser.close()
        return False

    ch = wait_for_char(ser, [CRC16_C], timeout=3.0)
    if ch != CRC16_C:
        print(f"❌ Gagal! Menunggu 'C' kedua, diterima: {hex(ch) if ch else 'None'}")
        ser.close()
        return False

    # 4. Kirim Data Payload Firmware
    print("[4/5] Mentransfer data firmware ke flash modul...")
    with open(bin_path, "rb") as f:
        file_bytes = f.read()

    chunk_size = 1024
    total_packets = (len(file_bytes) + chunk_size - 1) // chunk_size
    pkt_num = 1

    for offset in range(0, len(file_bytes), chunk_size):
        chunk = file_bytes[offset:offset + chunk_size]
        send_ymodem_packet(ser, pkt_num, chunk, block_size=chunk_size)
        
        ch = wait_for_char(ser, [ACK], timeout=5.0)
        if ch != ACK:
            print(f"\n❌ Transfer gagal di paket ke-{pkt_num}! Diterima: {hex(ch) if ch else 'None'}")
            ser.close()
            return False
            
        progress = int((pkt_num / total_packets) * 100)
        sys.stdout.write(f"\r      Progress: [{pkt_num}/{total_packets}] {progress}% selesai")
        sys.stdout.flush()
        pkt_num += 1

    print("\n      ✅ Semua blok data firmware berhasil ditransfer!")

    # 5. Tutup Transmisi (EOT & Null Packet)
    print("[5/5] Menyelesaikan transmisi (EOT & Handshake Akhir)...")
    ser.write(bytes([EOT]))
    ch = wait_for_char(ser, [NAK], timeout=3.0)

    ser.write(bytes([EOT]))
    ch = wait_for_char(ser, [ACK], timeout=3.0)

    ch = wait_for_char(ser, [CRC16_C], timeout=3.0)

    # Kirim paket kosong penutup (Packet 0 kosong)
    send_ymodem_packet(ser, 0, b"", block_size=128)
    ch = wait_for_char(ser, [ACK], timeout=3.0)

    print("=" * 65)
    print("🎉 SUKSES BESAR! FIRMWARE BERHASIL DI-FLASH KE RAK3272S!")
    print("   Modul akan otomatis me-reboot ke firmware baru.")
    print("   UART1 sekarang sudah aktif dalam Mode AT!")
    print("=================================================================")
    ser.close()
    return True


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyAMA3"
    fw = sys.argv[2] if len(sys.argv) > 2 else "firmware_enable_uart1.bin"
    flash_rak(port, fw)
