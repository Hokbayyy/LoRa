import serial
import time

# Port fisik UART GPIO pada Raspberry Pi 5
PORT = '/dev/ttyAMA1'

print("🔍 UJI LOOPBACK PI 5 (Memastikan Pin 8 TX & Pin 10 RX Normal)...")

try:
    ser = serial.Serial(PORT, baudrate=115200, timeout=1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Kirim data uji coba
    test_msg = b"PING_LOOPBACK_TEST\r\n"
    ser.write(test_msg)
    time.sleep(0.2)

    received = ser.read_all()

    if test_msg.strip() in received:
        print("✅ SUCCESS LOOPBACK BERHASIL!")
        print(f"   Data terkirim lewat Pin 8 (TX) dan diterima kembali via Pin 10 (RX).")
        print(f"   Port {PORT} dan hardware UART Pi 5 berjalan 100% normal.\n")
    else:
        print("❌ LOOPBACK GAGAL!")
        print(f"   Data yang diterima: {received}")
        print("   Pastikan pin jumper menancap pas di Pin 8 dan Pin 10.\n")

    ser.close()
except Exception as e:
    print(f"❌ Error membuka port {PORT}: {e}")