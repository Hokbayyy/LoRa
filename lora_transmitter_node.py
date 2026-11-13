#!/usr/bin/env python3
"""
LoRa P2P Transmitter Node (ROS2) - RAK3272S (Multi-Node 99 Parameter Terkompresi)

Mendukung banyak node pengirim (Node 1, Node 2, dst.) menggunakan 1 file yang sama.
Cara menjalankan:
  - Node 1: python3 lora_transmitter_node.py 1   (atau tanpa argumen / -p node_id:=1)
  - Node 2: python3 lora_transmitter_node.py 2   (atau -p node_id:=2)

Alur:
  1. Mengumpulkan data dari topic ROS2:
       - /node<id>/telemetri/arduino, /node<id>/telemetri/aurora, /node<id>/telemetri/suit
       - serta /telemetri/arduino, /telemetri/aurora, /telemetri/suit (jika global_topics=True)
  2. Seluruh 99 parameter dipadatkan (kompresi biner float16) ke dalam 1 paket tunggal:
       - Header magic : b"P99:" (4 byte)
       - Node ID      : uint8 (1 byte) -> 1 untuk Node 1, 2 untuk Node 2, dst.
       - Counter      : uint16 (2 byte)
       - Data 99 param: 99 x float16 (198 byte)
       - CRC16-CCITT  : uint16 (2 byte)
       Total ukuran    : TEPAT 207 Byte (414 karakter hex, di bawah batas 250 Byte RAK)
  3. Dikirim HANYA 1 KALI per siklus (sekali tembak, tanpa fragmen, tanpa pengiriman kedua)
     dengan offset waktu awal per Node ID + jitter kecil untuk menghindari tabrakan udara.
  4. Menampilkan log 1 baris horizontal bersih per pengiriman (tanpa emoji).
"""

import json
import math
import os
import random
import struct
import sys
import threading
import time
from collections import deque

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ============================================================
# KONFIGURASI RAK LORA
# ============================================================

SERIAL_PORT = "/dev/ttyAMA3"
BAUD_RATE = 115200
NODE_ID = 1

FREQUENCY = 923200000
SPREADING_FACTOR = 6
BANDWIDTH = 2
CODING_RATE = 0
PREAMBLE = 8
TX_POWER = 20

# Interval kirim dalam detik (minimal 0.2s agar modul radio tidak flooding)
TRANSMIT_INTERVAL = 1.0

# Batas hardware AT+PSEND RAK3272S adalah 250 byte
MAX_PAYLOAD = 250

HOLD_LAST_VALUE = True
ENABLE_DUMMY_DATA = True


# ============================================================
# URUTAN 99 PARAMETER (PERSIS SESUAI RECEIVER)
# JANGAN MENGUBAH URUTAN INI TANPA MENGUBAH RECEIVER
# ============================================================

EXPECTED_KEYS = [
    # Arduino UNO Q (23)
    "lidar_raw",
    "mq_nh3",
    "mq_co",
    "mq_co2",
    "smoke",

    "mpu_acc_x_raw",
    "mpu_acc_y_raw",
    "mpu_acc_z_raw",

    "mpu_gyro_x_raw",
    "mpu_gyro_y_raw",
    "mpu_gyro_z_raw",

    "mpu_acc_x_kalman",
    "mpu_acc_y_kalman",
    "mpu_acc_z_kalman",

    "mpu_gyro_x_kalman",
    "mpu_gyro_y_kalman",
    "mpu_gyro_z_kalman",

    "adxl_acc_x_raw",
    "adxl_acc_y_raw",
    "adxl_acc_z_raw",

    "adxl_acc_x_kalman",
    "adxl_acc_y_kalman",
    "adxl_acc_z_kalman",

    # Cosmic Aurora (12)
    "accelx",
    "accely",
    "accelz",

    "gyrox",
    "gyroy",
    "gyroz",

    "bme_temp",
    "bme_hum",
    "bme_pres",
    "bme_alt",

    "sound_dbLeft",
    "sound_dbRight",

    # Loadcells (5)
    "loadcell_paka",
    "loadcell_paki",
    "loadcell_leher",
    "loadcell_kaka",
    "loadcell_kaki",

    # Flex Sensors (16)
    "flex1",
    "flex2",
    "flex3",
    "flex4",
    "flex5",
    "flex6",
    "flex7",
    "flex8",
    "flex9",
    "flex10",
    "flex11",
    "flex12",
    "flex13",
    "flex14",
    "flex15",
    "flex16",

    # MCP Sensors (14)
    "mcp1",
    "mcp2",
    "mcp3",
    "mcp4",
    "mcp5",
    "mcp6",
    "mcp7",
    "mcp8",
    "mcp9",
    "mcp10",
    "mcp11",
    "mcp12",
    "mcp13",
    "mcp14",

    # FSR Sensors (14)
    "fsr1",
    "fsr2",
    "fsr3",
    "fsr4",
    "fsr5",
    "fsr6",
    "fsr7",
    "fsr8",
    "fsr9",
    "fsr10",
    "fsr11",
    "fsr12",
    "fsr13",
    "fsr14",

    # Piezo Sensors (14)
    "piezo1",
    "piezo2",
    "piezo3",
    "piezo4",
    "piezo5",
    "piezo6",
    "piezo7",
    "piezo8",
    "piezo9",
    "piezo10",
    "piezo11",
    "piezo12",
    "piezo13",
    "piezo14",

    # Thermal Sensor (1)
    "thermal",
]


# ============================================================
# DUMMY DATA
# ============================================================

DUMMY_VALUES = {
    "lidar_raw": 120.0,

    "mq_nh3": 12.5,
    "mq_co": 3.2,
    "mq_co2": 400.0,
    "smoke": 1.0,

    "mpu_acc_x_raw": 0.12,
    "mpu_acc_y_raw": 0.15,
    "mpu_acc_z_raw": 9.81,

    "mpu_gyro_x_raw": 0.01,
    "mpu_gyro_y_raw": 0.02,
    "mpu_gyro_z_raw": 0.03,

    "mpu_acc_x_kalman": 0.11,
    "mpu_acc_y_kalman": 0.14,
    "mpu_acc_z_kalman": 9.79,

    "mpu_gyro_x_kalman": 0.01,
    "mpu_gyro_y_kalman": 0.02,
    "mpu_gyro_z_kalman": 0.03,

    "adxl_acc_x_raw": 0.12,
    "adxl_acc_y_raw": 0.15,
    "adxl_acc_z_raw": 9.80,

    "adxl_acc_x_kalman": 0.11,
    "adxl_acc_y_kalman": 0.14,
    "adxl_acc_z_kalman": 9.78,

    "accelx": 0.25,
    "accely": 0.31,
    "accelz": 9.75,

    "gyrox": 0.01,
    "gyroy": 0.02,
    "gyroz": 0.03,

    "bme_temp": 28.5,
    "bme_hum": 65.0,
    "bme_pres": 1013.25,
    "bme_alt": 120.0,

    "sound_dbLeft": 43.0,
    "sound_dbRight": 43.0,

    "loadcell_paka": 12.5,
    "loadcell_paki": 20.0,
    "loadcell_leher": 23.5,
    "loadcell_kaka": 25.6,
    "loadcell_kaki": 25.6,

    "flex1": 25.6,
    "flex2": 25.6,
    "flex3": 25.6,
    "flex4": 25.6,
    "flex5": 25.6,
    "flex6": 25.6,
    "flex7": 25.6,
    "flex8": 25.6,
    "flex9": 25.6,
    "flex10": 25.6,
    "flex11": 25.6,
    "flex12": 25.6,
    "flex13": 25.6,
    "flex14": 25.6,
    "flex15": 25.6,
    "flex16": 25.6,

    "mcp1": 25.6,
    "mcp2": 25.6,
    "mcp3": 25.6,
    "mcp4": 25.6,
    "mcp5": 25.6,
    "mcp6": 25.6,
    "mcp7": 25.6,
    "mcp8": 25.6,
    "mcp9": 25.6,
    "mcp10": 25.6,
    "mcp11": 25.6,
    "mcp12": 25.6,
    "mcp13": 25.6,
    "mcp14": 25.6,

    "fsr1": 25.6,
    "fsr2": 25.6,
    "fsr3": 25.6,
    "fsr4": 25.6,
    "fsr5": 25.6,
    "fsr6": 25.6,
    "fsr7": 25.6,
    "fsr8": 25.6,
    "fsr9": 25.6,
    "fsr10": 25.6,
    "fsr11": 25.6,
    "fsr12": 25.6,
    "fsr13": 25.6,
    "fsr14": 25.6,

    "piezo1": 25.6,
    "piezo2": 25.6,
    "piezo3": 25.6,
    "piezo4": 25.6,
    "piezo5": 25.6,
    "piezo6": 25.6,
    "piezo7": 25.6,
    "piezo8": 25.6,
    "piezo9": 25.6,
    "piezo10": 25.6,
    "piezo11": 25.6,
    "piezo12": 25.6,
    "piezo13": 25.6,
    "piezo14": 25.6,

    "thermal": 25.6,
}


# ============================================================
# ALIAS COSMIC AURORA
# ============================================================

AURORA_ALIASES = {
    "accel_x": "accelx",
    "accel_y": "accely",
    "accel_z": "accelz",

    "gyro_x": "gyrox",
    "gyro_y": "gyroy",
    "gyro_z": "gyroz",

    "temperature": "bme_temp",
    "humidity": "bme_hum",
    "pressure": "bme_pres",
    "altitude": "bme_alt",

    "sound_left": "sound_dbLeft",
    "sound_right": "sound_dbRight",
}


# ============================================================
# UTILITAS & KOMPRESI
# ============================================================

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def valid_number(value):
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
        if not math.isfinite(number):
            return None
        return number
    except (ValueError, TypeError):
        return None


def to_float16_safe(val) -> float:
    """Memastikan angka aman masuk ke tipe half-precision float 16-bit."""
    num = valid_number(val)
    if num is None:
        return 0.0
    if num > 65504.0:
        return 65504.0
    if num < -65504.0:
        return -65504.0
    return num


def format_horizontal(d: dict) -> str:
    """Seluruh 99 parameter dalam 1 baris horizontal bersih."""
    g = lambda k: d.get(k, "-")
    p = [
        f"Lidar:{g('lidar_raw')}",
        f"MQ(NH3:{g('mq_nh3')},CO:{g('mq_co')},CO2:{g('mq_co2')})",
        f"Smoke:{g('smoke')}",
        f"MPU_Raw[A:({g('mpu_acc_x_raw')},{g('mpu_acc_y_raw')},{g('mpu_acc_z_raw')}) G:({g('mpu_gyro_x_raw')},{g('mpu_gyro_y_raw')},{g('mpu_gyro_z_raw')})]",
        f"MPU_Klm[A:({g('mpu_acc_x_kalman')},{g('mpu_acc_y_kalman')},{g('mpu_acc_z_kalman')}) G:({g('mpu_gyro_x_kalman')},{g('mpu_gyro_y_kalman')},{g('mpu_gyro_z_kalman')})]",
        f"ADXL_Raw:({g('adxl_acc_x_raw')},{g('adxl_acc_y_raw')},{g('adxl_acc_z_raw')})",
        f"ADXL_Klm:({g('adxl_acc_x_kalman')},{g('adxl_acc_y_kalman')},{g('adxl_acc_z_kalman')})",
        f"LSM6DS3[A:({g('accelx')},{g('accely')},{g('accelz')}) G:({g('gyrox')},{g('gyroy')},{g('gyroz')})]",
        f"BME280({g('bme_temp')}C,{g('bme_hum')}%,{g('bme_pres')}hPa,{g('bme_alt')}m)",
        f"Sound(L:{g('sound_dbLeft')},R:{g('sound_dbRight')}dB)",
        f"Loadcell({g('loadcell_paka')},{g('loadcell_paki')},{g('loadcell_leher')},{g('loadcell_kaka')},{g('loadcell_kaki')})",
        f"Flex[{','.join(str(g(f'flex{i}')) for i in range(1, 17))}]",
        f"MCP[{','.join(str(g(f'mcp{i}')) for i in range(1, 15))}]",
        f"FSR[{','.join(str(g(f'fsr{i}')) for i in range(1, 15))}]",
        f"Piezo[{','.join(str(g(f'piezo{i}')) for i in range(1, 15))}]",
        f"Thermal:{g('thermal')}",
    ]
    return " | ".join(p)


# ============================================================
# LORA TRANSMITTER NODE
# ============================================================

class LoRaTransmitterNode(Node):

    def __init__(self, default_node_id=NODE_ID, default_port=SERIAL_PORT):
        super().__init__(f"lora_transmitter_node_{default_node_id}")

        self.declare_parameter("node_id", default_node_id)
        self.declare_parameter("port", default_port)
        self.declare_parameter("baudrate", BAUD_RATE)
        self.declare_parameter("freq", FREQUENCY)
        self.declare_parameter("sf", SPREADING_FACTOR)
        self.declare_parameter("bw", BANDWIDTH)
        self.declare_parameter("cr", CODING_RATE)
        self.declare_parameter("preamble", PREAMBLE)
        self.declare_parameter("tx_power", TX_POWER)
        self.declare_parameter("interval", TRANSMIT_INTERVAL)
        self.declare_parameter("global_topics", True)

        gp = lambda n: self.get_parameter(n).value
        self.node_id = max(1, min(255, int(gp("node_id"))))
        self.port = str(gp("port"))
        self.baud = int(gp("baudrate"))
        self.freq = int(gp("freq"))
        self.sf = int(gp("sf"))
        self.bw = int(gp("bw"))
        self.cr = int(gp("cr"))
        self.preamble = int(gp("preamble"))
        self.tx_power = int(gp("tx_power"))
        self.interval = max(0.15, float(gp("interval")))
        self.global_topics = bool(gp("global_topics"))

        self.lock = threading.Lock()
        self.latest_values = {}
        self.last_values = {}
        self.tx_queue = deque(maxlen=1)
        self.serial_port = None
        self.tx_active = False
        self.packet_counter = 0
        self.last_reconnect_try = 0.0

        # ROS2 subscribers spesifik node (misal /node2/telemetri/arduino)
        self.node_ard_sub = self.create_subscription(
            String, f"/node{self.node_id}/telemetri/arduino", self.arduino_callback, 10
        )
        self.node_aur_sub = self.create_subscription(
            String, f"/node{self.node_id}/telemetri/aurora", self.aurora_callback, 10
        )
        self.node_suit_sub = self.create_subscription(
            String, f"/node{self.node_id}/telemetri/suit", self.arduino_callback, 10
        )

        # ROS2 subscribers global (/telemetri/arduino, dll.) jika diaktifkan
        if self.global_topics:
            self.arduino_sub = self.create_subscription(
                String, "/telemetri/arduino", self.arduino_callback, 10
            )
            self.aurora_sub = self.create_subscription(
                String, "/telemetri/aurora", self.aurora_callback, 10
            )
            self.suit_sub = self.create_subscription(
                String, "/telemetri/suit", self.arduino_callback, 10
            )

        self.initialize_serial()

        # Offset awal agar Node 1 dan Node 2 tidak memancar tepat bersamaan
        if self.node_id > 1:
            phase_delay = ((self.node_id - 1) * 0.45) % self.interval
            if phase_delay > 0.01:
                self.get_logger().info(
                    f"Node {self.node_id}: offset waktu awal {phase_delay:.2f}s untuk cegah tabrakan radio."
                )
                time.sleep(phase_delay)

        # Timer pengiriman paket tunggal
        self.transmit_timer = self.create_timer(
            self.interval,
            self.transmit_callback
        )

        # Timer antrean TX
        self.queue_timer = self.create_timer(
            0.02,
            self.process_queue
        )

        self.get_logger().info(
            f"LoRa transmitter NODE {self.node_id} started on {self.port} @ {self.baud} "
            f"(99 param compressed 207B, single packet every {self.interval}s)"
        )

    # ========================================================
    # SERIAL INITIALIZATION
    # ========================================================

    def initialize_serial(self):
        candidate_ports = [self.port, "/dev/ttyAMA3", "/dev/serial0", "/dev/ttyUSB0"]
        chosen_port = None

        for p in candidate_ports:
            if os.path.exists(p):
                chosen_port = p
                break

        if not chosen_port:
            chosen_port = self.port

        try:
            self.serial_port = serial.Serial(
                port=chosen_port,
                baudrate=self.baud,
                timeout=0.1,
                write_timeout=1.0,
                rtscts=False
            )
            self.port = chosen_port
            time.sleep(0.5)

            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()

            self.get_logger().info(
                f"RAK connected: {self.port}"
            )
            self.configure_rak()

        except Exception as error:
            self.get_logger().error(
                f"RAK connection failed on {chosen_port}: {error}"
            )
            self.serial_port = None

    def try_reconnect(self):
        now = time.monotonic()
        if now - self.last_reconnect_try < 3.0:
            return
        self.last_reconnect_try = now

        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None

        self.get_logger().warning("Mencoba reconnect ke port RAK3272S...")
        self.initialize_serial()

    # ========================================================
    # AT COMMAND
    # ========================================================

    def send_at(self, command, timeout=2.0):
        if self.serial_port is None:
            return False

        try:
            self.serial_port.reset_input_buffer()
            self.serial_port.write((command + "\r\n").encode())
            self.serial_port.flush()

            start_time = time.time()
            response = ""

            while time.time() - start_time < timeout:
                if self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    response += data.decode(errors="ignore")

                    if "ERROR" in response:
                        self.get_logger().error(
                            f"AT error: {command} | {response.strip()}"
                        )
                        return False

                    if "OK" in response:
                        self.get_logger().info(
                            f"AT OK: {command}"
                        )
                        return True

                time.sleep(0.02)

            self.get_logger().warning(
                f"AT timeout: {command} | {response.strip()}"
            )
            return False

        except Exception as error:
            self.get_logger().error(
                f"AT command failed: {error}"
            )
            return False

    # ========================================================
    # CONFIGURE RAK
    # ========================================================

    def configure_rak(self):
        if self.serial_port is None:
            return

        self.get_logger().info(f"Configuring RAK LoRa P2P Transmitter (Node {self.node_id})...")

        self.send_at("AT", timeout=1.0)

        self.serial_port.reset_input_buffer()
        self.serial_port.write(b"AT+NWM=?\r\n")
        self.serial_port.flush()
        time.sleep(0.3)
        res_nwm = self.serial_port.read_all().decode(errors="ignore")

        if "=0" not in res_nwm:
            self.get_logger().info("Mengubah RAK ke mode LoRa P2P (AT+NWM=0)...")
            self.send_at("AT+NWM=0", timeout=2.0)
            time.sleep(2.0)
            self.send_at("AT", timeout=1.0)

        self.send_at("AT+PRECV=0", timeout=1.0)

        p2p_cmd = (
            f"AT+P2P={self.freq}:"
            f"{self.sf}:"
            f"{self.bw}:"
            f"{self.cr}:"
            f"{self.preamble}:"
            f"{self.tx_power}"
        )
        ok_p2p = self.send_at(p2p_cmd, timeout=2.0)

        if not ok_p2p and self.bw == 2:
            p2p_fallback = (
                f"AT+P2P={self.freq}:"
                f"{self.sf}:"
                f"500:"
                f"{self.cr}:"
                f"{self.preamble}:"
                f"{self.tx_power}"
            )
            self.send_at(p2p_fallback, timeout=2.0)

        self.get_logger().info("RAK configuration sequence completed.")

    # ========================================================
    # PARSE JSON INPUT
    # ========================================================

    def parse_json(self, raw_data):
        try:
            data = json.loads(raw_data)
            if isinstance(data, dict):
                return data
            return None
        except (json.JSONDecodeError, TypeError):
            return None

    # ========================================================
    # UPDATE SENSOR VALUES
    # ========================================================

    def update_values(self, data, aliases=None):
        if not isinstance(data, dict):
            return
        aliases = aliases or {}

        with self.lock:
            for original_key, value in data.items():
                key = aliases.get(original_key, original_key)
                if key not in EXPECTED_KEYS:
                    continue
                number = valid_number(value)
                if number is None:
                    continue
                self.latest_values[key] = number
                self.last_values[key] = number

    # ========================================================
    # ROS2 CALLBACKS
    # ========================================================

    def arduino_callback(self, msg):
        data = self.parse_json(msg.data)
        if data is None:
            self.get_logger().warning("Arduino message is not valid JSON object.")
            return
        self.update_values(data)

    def aurora_callback(self, msg):
        data = self.parse_json(msg.data)
        if data is None:
            self.get_logger().warning("Aurora message is not valid JSON object.")
            return
        self.update_values(data, aliases=AURORA_ALIASES)

    # ========================================================
    # DUMMY DATA GENERATOR
    # ========================================================

    def generate_dummy_values(self):
        dummy = {}
        for key in EXPECTED_KEYS:
            base = DUMMY_VALUES[key]
            variation = abs(base) * 0.01
            if variation == 0:
                variation = 0.01
            dummy[key] = round(
                random.uniform(base - variation, base + variation),
                3
            )
        return dummy

    # ========================================================
    # BUILD COMPRESSED SINGLE PACKET (207 BYTES)
    # ========================================================

    def build_compressed_packet(self):
        dummy_values = self.generate_dummy_values()
        values = []
        sensor_snapshot = {}

        with self.lock:
            for key in EXPECTED_KEYS:
                value = None
                if key in self.latest_values:
                    value = self.latest_values[key]
                elif HOLD_LAST_VALUE and key in self.last_values:
                    value = self.last_values[key]
                elif ENABLE_DUMMY_DATA:
                    value = dummy_values[key]
                else:
                    value = 0.0

                safe_num = to_float16_safe(value)
                values.append(safe_num)
                sensor_snapshot[key] = round(safe_num, 3)

        self.packet_counter += 1
        cnt = self.packet_counter % 65536

        # Kemas biner terkompresi:
        # Header "P99:" (4B) + NodeID (1B) + Counter (2B) + 99 float16 (198B) + CRC16 (2B) = 207 Byte
        magic = b"P99:"
        body = magic + struct.pack(
            f"<BH{len(EXPECTED_KEYS)}e",
            self.node_id & 0xFF,
            cnt,
            *values
        )
        crc = crc16_ccitt(body)
        packet = body + struct.pack("<H", crc)

        if len(packet) > MAX_PAYLOAD:
            raise ValueError(
                f"Paket melebihi batas MAX_PAYLOAD ({len(packet)} > {MAX_PAYLOAD} bytes)"
            )

        return packet, self.packet_counter, sensor_snapshot

    # ========================================================
    # TRANSMIT CALLBACK (SINGLE PACKET, SEKALI TEMBAK)
    # ========================================================

    def transmit_callback(self):
        if self.serial_port is None or not self.serial_port.is_open:
            self.try_reconnect()
            return

        try:
            packet, cnt, snapshot = self.build_compressed_packet()
            with self.lock:
                self.tx_queue.append((packet, cnt, snapshot))
        except Exception as error:
            self.get_logger().error(f"Packet creation failed: {error}")

    # ========================================================
    # PROCESS TX QUEUE
    # ========================================================

    def process_queue(self):
        if self.serial_port is None or not self.serial_port.is_open:
            return
        if self.tx_active or not self.tx_queue:
            return

        with self.lock:
            if not self.tx_queue:
                return
            packet, cnt, snapshot = self.tx_queue.popleft()

        self.tx_active = True

        # Random micro-jitter (0 - 40 ms) agar multi-node tidak terkunci bentrok
        time.sleep(random.uniform(0.0, 0.04))
        t_start = time.time()

        try:
            hex_data = packet.hex().upper()
            command = f"AT+PSEND={hex_data}"

            self.serial_port.reset_input_buffer()
            self.serial_port.write((command + "\r\n").encode())
            self.serial_port.flush()

            success, err_msg = self.wait_tx_done(timeout=2.0)
            elapsed_ms = (time.time() - t_start) * 1000.0
            line = format_horizontal(snapshot)

            if success:
                self.get_logger().info(
                    f"[TX NODE {self.node_id} #{cnt:04d}] [{len(packet)}B 1Paket OK {elapsed_ms:.0f}ms] -> {line}"
                )
            else:
                self.get_logger().warning(
                    f"[TX NODE {self.node_id} #{cnt:04d}] [{len(packet)}B 1Paket GAGAL: {err_msg}]"
                )

        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f"Serial TX error: {exc}")
            self.try_reconnect()
        except Exception as error:
            self.get_logger().error(f"LoRa transmission error: {error}")
        finally:
            self.tx_active = False

    # ========================================================
    # WAIT TX DONE
    # ========================================================

    def wait_tx_done(self, timeout=2.0):
        start_time = time.time()
        response = ""
        ok_seen = False

        while time.time() - start_time < timeout:
            if self.serial_port.in_waiting:
                data = self.serial_port.read(self.serial_port.in_waiting)
                response += data.decode(errors="ignore")

                if "ERROR" in response:
                    return False, response.strip()

                if "OK" in response:
                    ok_seen = True

                if "+EVT:TXP2P DONE" in response or "+EVT:TX_DONE" in response:
                    return True, "DONE"

            time.sleep(0.01)

        if ok_seen:
            return True, "OK (no explicit TX_DONE)"

        return False, f"Timeout ({response.strip()})"

    # ========================================================
    # SHUTDOWN
    # ========================================================

    def destroy_node(self):
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
        super().destroy_node()


# ============================================================
# CLI PARSER & MAIN
# ============================================================

def parse_cli_args():
    cli_node_id = NODE_ID
    cli_port = SERIAL_PORT
    filtered_args = [sys.argv[0]]
    i = 1
    while i < len(sys.argv):
        arg = sys.argv[i]
        if arg == "--ros-args":
            filtered_args.extend(sys.argv[i:])
            break
        if arg.startswith("/dev/"):
            cli_port = arg
        elif arg.isdigit():
            val = int(arg)
            if 1 <= val <= 255:
                cli_node_id = val
            else:
                filtered_args.append(arg)
        else:
            filtered_args.append(arg)
        i += 1
    return cli_node_id, cli_port, filtered_args


def main(args=None):
    cli_node_id, cli_port, filtered_args = parse_cli_args()
    rclpy.init(args=args if args is not None else filtered_args)
    node = LoRaTransmitterNode(default_node_id=cli_node_id, default_port=cli_port)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
