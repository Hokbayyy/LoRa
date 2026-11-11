import serial
import time
import struct
import random

# ==================== KONFIGURASI NODE ====================
NODE_ID = 1                 # Ubah menjadi 2 untuk Node TX Kedua
SERIAL_PORT = '/dev/ttyAMA0' # Gunakan '/dev/ttyUSB0' jika memakai USB-to-TTL
BAUDRATE = 115200
INTERVAL_SEND = 5           # Pengiriman tiap 5 detik

# Setup Serial Port UART
try:
    ser = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=1)
    print(f"[TX NODE {NODE_ID}] Port serial {SERIAL_PORT} berhasil dibuka.")
except Exception as e:
    print(f"[ERROR] Gagal membuka serial port: {e}")
    exit(1)

def send_at(cmd: str, delay: float = 0.3) -> str:
    """Mengirim perintah AT dan mengembalikan respon string."""
    ser.write((cmd + '\r\n').encode('utf-8'))
    time.sleep(delay)
    return ser.read_all().decode('utf-8', errors='ignore').strip()

def setup_rak3272s():
    print(f"[TX NODE {NODE_ID}] Konfigurasi RAK3272S (SF7, 923.2 MHz)...")
    send_at("AT+NWM=0")  # Set Mode P2P
    res = send_at("AT+P2P=923200000:7:0:0:8:20")
    print(f"[TX NODE {NODE_ID}] Respon P2P Config: {res}")

def generate_dummy_120_parameters() -> list:
    """
    Meng-generate 120 data dummy desimal float.
    Nilai bervariasi secara realistis tiap pengiriman.
    """
    parameters = []
    base_val = 24.0 if NODE_ID == 1 else 32.0
    for i in range(120):
        # Simulasi rentang nilai desimal, misal: 24.15 s.d. 45.80
        val = round(base_val + (i * 0.15) + random.uniform(-0.8, 0.8), 2)
        parameters.append(val)
    return parameters

def pack_payload(node_id: int, params: list) -> str:
    """
    Binary Bit-Packing Protocol:
    - 1 Byte Node ID (uint8)
    - 120 x 2 Byte uint16 (Fixed-point scaling x100)
    Total raw binary = 241 Byte (482 Hex Char)
    """
    scaled_params = [int(round(p * 100)) for p in params]
    fmt = '<B120H'
    raw_bytes = struct.pack(fmt, node_id, *scaled_params)
    return raw_bytes.hex().upper()

def main():
    setup_rak3272s()
    print(f"🚀 Node TX {NODE_ID} Siap Mengirim 120 Data Dummy...")
    
    while True:
        try:
            # 1. Generate 120 data dummy
            dummy_data = generate_dummy_120_parameters()
            
            # 2. Pack ke binary HEX payload
            hex_payload = pack_payload(NODE_ID, dummy_data)
            
            # 3. Transmisi P2P via RAK3272S
            cmd = f"AT+PSEND={hex_payload}"
            ser.write((cmd + '\r\n').encode('utf-8'))
            time.sleep(0.4)
            
            resp = ser.read_all().decode('utf-8', errors='ignore').strip()
            print(f"[{time.strftime('%H:%M:%S')}] TX Node {NODE_ID} ({len(hex_payload)//2} Byte) -> {resp}")
            
            time.sleep(INTERVAL_SEND)
            
        except KeyboardInterrupt:
            print("\n[TX] Pengiriman dihentikan.")
            break
        except Exception as e:
            print(f"[ERROR TX] {e}")
            time.sleep(2)

if __name__ == "__main__":
    main()