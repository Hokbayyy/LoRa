import asyncio
import sqlite3
import json
import struct
import time
import os
import serial
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import uvicorn

# ==================== KONFIGURASI SYSTEM ====================
DB_NAME = "telemetry_data.db"
SERIAL_PORT = '/dev/ttyAMA0' # Gunakan '/dev/ttyUSB0' jika menggunakan USB-to-TTL
BAUDRATE = 115200

app = FastAPI()

# ==================== DATABASE SQLITE HANDLER ====================
def init_db():
    conn = sqlite3.connect(DB_NAME)
    cursor = conn.cursor()
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS lora_telemetry (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
            node_id INTEGER NOT NULL,
            rssi REAL NOT NULL,
            snr REAL NOT NULL,
            parameters TEXT NOT NULL
        )
    ''')
    conn.commit()
    conn.close()
    print("✅ [DB] SQLite Database lokal terinisialisasi!")

def save_to_sqlite(node_id: int, rssi: float, snr: float, params: list):
    """Menyimpan data lengkap termasuk RSSI & SNR ke SQLite."""
    try:
        conn = sqlite3.connect(DB_NAME)
        cursor = conn.cursor()
        params_json = json.dumps(params)
        cursor.execute('''
            INSERT INTO lora_telemetry (node_id, rssi, snr, parameters)
            VALUES (?, ?, ?, ?)
        ''', (node_id, rssi, snr, params_json))
        conn.commit()
        conn.close()
    except Exception as e:
        print(f"❌ [DB ERROR] Gagal menyimpan ke SQLite: {e}")

# ==================== WEBSOCKET BROADCASTER ====================
class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        for connection in list(self.active_connections):
            try:
                await connection.send_json(message)
            except Exception:
                self.disconnect(connection)

manager = ConnectionManager()

# ==================== LORA RX WORKER TASK ====================
async def lora_rx_worker():
    loop = asyncio.get_event_loop()
    
    try:
        ser = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=0.1)
    except Exception as e:
        print(f"❌ [FATAL] Gagal membuka port serial RX: {e}")
        return

    # Set Up Mode RX Continuous
    ser.write(b"AT+NWM=0\r\n")
    await asyncio.sleep(0.2)
    ser.write(b"AT+P2P=923200000:7:0:0:8:20\r\n")
    await asyncio.sleep(0.2)
    ser.write(b"AT+PRECV=65535\r\n")
    await asyncio.sleep(0.2)
    print("📡 [RX GATEWAY] RAK3272S Listening Mode Active (SF7 P2P)...")

    buffer = ""
    fmt = '<B120H'
    expected_bytes = struct.calcsize(fmt) # 241 Byte

    while True:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            buffer += data
            
            while '\r\n' in buffer:
                line, buffer = buffer.split('\r\n', 1)
                line = line.strip()
                
                # Format: +EVT:RXP2P:<RSSI>:<SNR>:<HEX_PAYLOAD>
                if "+EVT:RXP2P" in line:
                    parts = line.split(":")
                    if len(parts) >= 5:
                        try:
                            rssi = float(parts[2])
                            snr = float(parts[3])
                            hex_payload = parts[4]
                            
                            raw_bytes = bytes.fromhex(hex_payload)
                            
                            if len(raw_bytes) == expected_bytes:
                                # Unpack 1 Byte Node ID + 120 uint16
                                unpacked = struct.unpack(fmt, raw_bytes)
                                node_id = unpacked[0]
                                params = [round(val / 100.0, 2) for val in unpacked[1:]]
                                timestamp_str = time.strftime('%H:%M:%S')
                                
                                print(f"[{timestamp_str}] RX Success | Node: {node_id} | RSSI: {rssi} dBm | SNR: {snr} dB (Saved to SQLite)")
                                
                                # 1. SIMPAN KE SQLITE (Termasuk RSSI & SNR)
                                await loop.run_in_executor(None, save_to_sqlite, node_id, rssi, snr, params)
                                
                                # 2. BROADCAST KE WEB (Fokus 120 Parameter tanpa RSSI/SNR)
                                ws_payload = {
                                    "timestamp": timestamp_str,
                                    "node_id": node_id,
                                    "parameters": params
                                }
                                await manager.broadcast(ws_payload)
                            else:
                                print(f"⚠️ Ukuran payload tidak cocok: {len(raw_bytes)} byte.")
                        except Exception as parse_err:
                            print(f"⚠️ Parsing Error: {parse_err}")

        await asyncio.sleep(0.01)

# ==================== FASTAPI ENDPOINTS ====================
@app.on_event("startup")
async def startup_event():
    init_db()
    asyncio.create_task(lora_rx_worker())

@app.get("/")
async def get_dashboard():
    if os.path.exists("templates/index.html"):
        with open("templates/index.html", "r", encoding="utf-8") as f:
            return HTMLResponse(content=f.read())
    return HTMLResponse(content="<h2>templates/index.html tidak ditemukan!</h2>", status_code=404)

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)