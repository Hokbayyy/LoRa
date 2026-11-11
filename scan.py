import serial
import time

# Daftar port serial yang ada di Raspberry Pi 5
target_ports = ['/dev/ttyAMA1', '/dev/ttyAMA0', '/dev/ttyS0']

print("🔍 Memindai port serial untuk RAK3272S (Tanpa Reboot)...\n")

found = False
for port in target_ports:
    print(f"👉 Menguji {port} ...")
    try:
        ser = serial.Serial(port, baudrate=115200, timeout=0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Kirim perintah AT
        ser.write(b"AT\r\n")
        time.sleep(0.3)
        
        response = ser.read_all().decode('utf-8', errors='ignore').strip()
        
        if "OK" in response:
            print(f"  ✅ BERHASIL! RAK3272S merespon di: {port}")
            print(f"  --> Gunakan SERIAL_PORT = '{port}' pada script TX/RX Anda!\n")
            found = True
            ser.close()
            break
        else:
            print(f"  ❌ Port terbuka, tapi tidak ada respon OK. (Hasil: '{response}')")
            
        ser.close()
    except Exception as e:
        print(f"  ⚠️ Tidak dapat membuka {port}: {e}")

if not found:
    print("\n❌ RAK3272S belum merespon di semua port.")
    print("👉 Hal yang perlu dipastikan saat ini:")
    print("   1. Tukar kabel Pin 8 (TX) dan Pin 10 (RX) pada RAK3272S.")
    print("   2. Pastikan Pin GND RAK3272S tersambung ke Pin GND Pi 5.")