import serial
import time
import struct

SERIAL_PORT = '/dev/ttyAMA0'  # Gunakan /dev/ttyUSB0 jika memakai USB-to-TTL
BAUDRATE = 115200
NODE_ID = 1                   # ID Node TX

def send_at(ser, cmd, delay=0.3):
    ser.write((cmd + '\r\n').encode('utf-8'))
    time.sleep(delay)
    return ser.read_all().decode('utf-8', errors='ignore').strip()

def check_and_setup_tx():
    try:
        ser = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=1)
        print(f"==================================================")
        print(f"🔍 MEMERIKSA STATUS RAK3272S TRANSMITTER ({SERIAL_PORT})...")
        print(f"==================================================")
        
        # 1. Tes Komunikasi AT Sederhana
        res_at = send_at(ser, "AT")
        if "OK" not in res_at:
            print("❌ ERROR: RAK3272S TX tidak merespon perintah AT!")
            return None
        print("✅ 1. Komunikasi Serial OK (RAK3272S Terhubung)")

        # 2. Set & Cek Mode P2P
        send_at(ser, "AT+NWM=0")
        res_nwm = send_at(ser, "AT+NWM=?")
        print(f"✅ 2. Mode Kerja Saat Ini : {res_nwm}")

        # 3. Set & Cek Parameter Radio P2P (WAJIB SAMA DENGAN RX)
        send_at(ser, "AT+P2P=923200000:7:0:0:8:20")
        res_p2p = send_at(ser, "AT+P2P=?")
        print(f"✅ 3. Parameter P2P Radio : {res_p2p}")
        
        print(f"==================================================")
        print(f"🚀 RAK3272S TX SIAP MENGIRIM PAKET DUMMY...")
        print(f"==================================================\n")
        return ser

    except Exception as e:
        print(f"❌ Error membuka serial: {e}")
        return None

def make_dummy_payload(node_id):
    # Buat 120 parameter dummy (misal nilai 25.50)
    scaled_params = [2550 for _ in range(120)]
    # Format: 1 Byte Node ID + 120 x 2 Byte uint16 = 241 Byte
    raw_bytes = struct.pack('<B120H', node_id, *scaled_params)
    return raw_bytes.hex().upper()

def main():
    ser = check_and_setup_tx()
    if not ser:
        return

    counter = 1
    while True:
        try:
            timestamp = time.strftime('%H:%M:%S')
            hex_payload = make_dummy_payload(NODE_ID)
            
            print(f"[{timestamp}] Paket #{counter}: Mengirim 120 parameter (241 Byte) dari Node {NODE_ID}...")
            
            # Kirim Perintah AT+PSEND
            cmd = f"AT+PSEND={hex_payload}"
            ser.write((cmd + '\r\n').encode('utf-8'))
            
            # Tunggu dan baca respon dari RAK3272S
            time.sleep(0.5)
            response = ser.read_all().decode('utf-8', errors='ignore').strip()
            
            if "OK" in response or "+EVT:TXP2P DONE" in response:
                print(f"   STATUS : ✅ PAKET TERKIRIM KE UDARA! (Respon: {response.replace(chr(10), ' ')})")
            else:
                print(f"   STATUS : ⚠️ ADA KENDALA TRANSMISI! Respon Modul: {response}")
                
            print("-" * 60)
            counter += 1
            time.sleep(4) # Jeda pengiriman tiap 4 detik

        except KeyboardInterrupt:
            print("\n[TX TEST] Dihentikan oleh pengguna.")
            break

if __name__ == "__main__":
    main()