import serial
import time

SERIAL_PORT = '/dev/ttyAMA0'  # Gunakan /dev/ttyUSB0 jika memakai USB-to-TTL
BAUDRATE = 115200

def send_at(ser, cmd, delay=0.3):
    ser.write((cmd + '\r\n').encode('utf-8'))
    time.sleep(delay)
    return ser.read_all().decode('utf-8', errors='ignore').strip()

def check_and_setup_rx():
    try:
        ser = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=1)
        print(f"==================================================")
        print(f"🔍 MEMERIKSA STATUS RAK3272S RECEIVER ({SERIAL_PORT})...")
        print(f"==================================================")
        
        # 1. Tes Komunikasi AT Sederhana
        res_at = send_at(ser, "AT")
        if "OK" not in res_at:
            print("❌ ERROR: RAK3272S tidak merespon perintah AT!")
            print("👉 Periksa: Kabel TX/RX terbalik? Baudrate salah? Atau port serial belum diizinkan?")
            return None
        print("✅ 1. Komunikasi Serial OK (RAK3272S Terhubung)")

        # 2. Set & Cek Mode P2P
        send_at(ser, "AT+NWM=0") # Set Mode P2P
        res_nwm = send_at(ser, "AT+NWM=?")
        print(f"✅ 2. Mode Kerja Saat Ini : {res_nwm}")

        # 3. Set & Cek Parameter Radio P2P
        # Freq: 923.2MHz, SF: 7, BW: 125kHz, CR: 4/5, Preamble: 8, Power: 20dBm
        send_at(ser, "AT+P2P=923200000:7:0:0:8:20")
        res_p2p = send_at(ser, "AT+P2P=?")
        print(f"✅ 3. Parameter P2P Radio : {res_p2p}")

        # 4. Aktifkan Mode Receiver Continuous (PRECV)
        res_precv = send_at(ser, "AT+PRECV=65535")
        print(f"✅ 4. Status Listening RX  : {res_precv}")
        
        print(f"==================================================")
        print(f"📡 RAK3272S RX SIAP & MENDENGARKAN SINYAL RADIO...")
        print(f"Menunggu data masuk dari Node TX... (Tekan Ctrl+C untuk berhenti)\n")
        return ser

    except Exception as e:
        print(f"❌ Error muka serial: {e}")
        return None

def main():
    ser = check_and_setup_rx()
    if not ser:
        return

    buffer = ""
    while True:
        try:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                buffer += data
                
                while '\r\n' in buffer:
                    line, buffer = buffer.split('\r\n', 1)
                    line = line.strip()
                    
                    if line:
                        timestamp = time.strftime('%H:%M:%S')
                        # Indikator Jika Sinyal Radio Masuk
                        if "+EVT:RXP2P" in line:
                            print(f"\n🎉 [{timestamp}] SINYAL DITERIMA!")
                            print(f"   Raw Event Data : {line}")
                            parts = line.split(":")
                            if len(parts) >= 5:
                                print(f"   --> RSSI : {parts[2]} dBm")
                                print(f"   --> SNR  : {parts[3]} dB")
                                print(f"   --> Payload Hex ({len(parts[4])//2} Byte) : {parts[4][:30]}...")
                        else:
                            print(f"[{timestamp}] [RAK LOG]: {line}")
                            
            time.sleep(0.01)
        except KeyboardInterrupt:
            print("\n[RX TEST] Dihentikan oleh pengguna.")
            break

if __name__ == "__main__":
    main()