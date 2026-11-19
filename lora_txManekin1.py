
#!/usr/bin/env python3

import json
import math
import time
import threading
from collections import deque

import serial
import rclpy

from rclpy.node import Node
from std_msgs.msg import String


# ============================================================
# RAK LORA CONFIGURATION
# ============================================================

SERIAL_PORT = "/dev/ttyAMA3"
BAUD_RATE = 115200

FREQUENCY = 923200000
SPREADING_FACTOR = 6
BANDWIDTH = 2
CODING_RATE = 0
PREAMBLE = 8
TX_POWER = 20

# Kirim satu paket setiap 1 detik
TRANSMIT_INTERVAL = 1.0

# Batas pemeriksaan ukuran paket di sisi gateway.
# Pastikan juga sesuai batas payload dan konfigurasi RAK.
MAX_PAYLOAD = 700

# Pertahankan nilai terakhir yang valid.
# Tidak pernah membuat nilai dummy.
HOLD_LAST_VALUE = True

# Maksimal antrean paket
MAX_QUEUE_SIZE = 3


# ============================================================
# FIELD ORDER - TOTAL 109 FIELD (DISAMAKAN DENGAN WEB)
# URUTAN INI TIDAK BOLEH DIUBAH TANPA MENYESUAIKAN RECEIVER
# ============================================================

EXPECTED_KEYS = [

    # --------------------------------------------------------
    # 1. ARDUINO UNO Q (24) -> Web readingId 1001..1024 (M1) / 2001..2024 (M2)
    # --------------------------------------------------------

    "lidar_raw",
    "lidar_median",

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

    # --------------------------------------------------------
    # 2. COSMIC AURORA (12)
    # --------------------------------------------------------

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

    # --------------------------------------------------------
    # 3. LOAD CELL (5)
    # --------------------------------------------------------

    "loadcell_paka",
    "loadcell_paki",
    "loadcell_leher",
    "loadcell_kaka",
    "loadcell_kaki",

    # --------------------------------------------------------
    # 4. FLEX (16)
    # --------------------------------------------------------

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

    # --------------------------------------------------------
    # 5. MCP9808 (14)
    # --------------------------------------------------------

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

    # --------------------------------------------------------
    # 6. FSR (14)
    # --------------------------------------------------------

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

    # --------------------------------------------------------
    # 7. PIEZO (14)
    # --------------------------------------------------------

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

    # --------------------------------------------------------
    # 8. AMG8833 (1)
    # --------------------------------------------------------

    "thermal",

    # --------------------------------------------------------
    # 9. WITMOTION WT901 (9)
    # --------------------------------------------------------

    "ax",
    "ay",
    "az",

    "gx",
    "gy",
    "gz",

    "roll",
    "pitch",
    "yaw",
]


# ============================================================
# ARDUINO FIELD MAPPING
# ============================================================

ARDUINO_MAP = {

    "lidar_distance_cm": "lidar_raw",
    "lidar_median_cm": "lidar_median",

    "mq135_nh3_ppm": "mq_nh3",
    "mq135_co_ppm": "mq_co",
    "mq135_co2_ppm": "mq_co2",
    "mq2_smoke_index": "smoke",

    "mpu_accel_x": "mpu_acc_x_raw",
    "mpu_accel_y": "mpu_acc_y_raw",
    "mpu_accel_z": "mpu_acc_z_raw",

    "mpu_gyro_x": "mpu_gyro_x_raw",
    "mpu_gyro_y": "mpu_gyro_y_raw",
    "mpu_gyro_z": "mpu_gyro_z_raw",

    "mpu_accel_kalman_x": "mpu_acc_x_kalman",
    "mpu_accel_kalman_y": "mpu_acc_y_kalman",
    "mpu_accel_kalman_z": "mpu_acc_z_kalman",

    "mpu_gyro_kalman_x": "mpu_gyro_x_kalman",
    "mpu_gyro_kalman_y": "mpu_gyro_y_kalman",
    "mpu_gyro_kalman_z": "mpu_gyro_z_kalman",

    "adxl_accel_x": "adxl_acc_x_raw",
    "adxl_accel_y": "adxl_acc_y_raw",
    "adxl_accel_z": "adxl_acc_z_raw",

    "adxl_accel_kalman_x": "adxl_acc_x_kalman",
    "adxl_accel_kalman_y": "adxl_acc_y_kalman",
    "adxl_accel_kalman_z": "adxl_acc_z_kalman",
}


# ============================================================
# COSMIC AURORA FIELD MAPPING
# ============================================================

AURORA_MAP = {

    "accel_x": "accelx",
    "accel_y": "accely",
    "accel_z": "accelz",

    "gyro_x": "gyrox",
    "gyro_y": "gyroy",
    "gyro_z": "gyroz",

    "bme_temperature": "bme_temp",
    "bme_humidity": "bme_hum",
    "bme_pressure": "bme_pres",
    "bme_altitude": "bme_alt",

    "sound_db_left": "sound_dbLeft",
    "sound_db_right": "sound_dbRight",
}


# ============================================================
# URUTAN LOADCELL DARI HX711_NODE (/sensors/loadcell)
# hx711_node mengirim {"loadcell": [[raw, gram, gram_kalman, newton, newton_kalman], ...]}
# dengan urutan: [0]=leher, [1]=paha_kanan, [2]=paha_kiri, [3]=kaki_kanan, [4]=kaki_kiri
# ============================================================

LOADCELL_INDEX_KEYS = [
    "loadcell_leher",
    "loadcell_paka",
    "loadcell_paki",
    "loadcell_kaka",
    "loadcell_kaki",
]


# ============================================================
# WEB COMPACT V2 READING ID MAPPING (Be-smart-manekin)
# Mannequin 1: 1001..1095 | Mannequin 2: otomatis +1000 (2001..2095)
# ============================================================

WEB_READING_ID_MAP_M1 = {
    # 1. Arduino Uno Q (1001..1024)
    "lidar_raw": 1001,
    "lidar_median": 1002,
    "mq_nh3": 1003,
    "mq_co": 1004,
    "mq_co2": 1005,
    "smoke": 1006,
    "mpu_acc_x_raw": 1007,
    "mpu_acc_y_raw": 1008,
    "mpu_acc_z_raw": 1009,
    "mpu_gyro_x_raw": 1010,
    "mpu_gyro_y_raw": 1011,
    "mpu_gyro_z_raw": 1012,
    "mpu_acc_x_kalman": 1013,
    "mpu_acc_y_kalman": 1014,
    "mpu_acc_z_kalman": 1015,
    "mpu_gyro_x_kalman": 1016,
    "mpu_gyro_y_kalman": 1017,
    "mpu_gyro_z_kalman": 1018,
    "adxl_acc_x_raw": 1019,
    "adxl_acc_y_raw": 1020,
    "adxl_acc_z_raw": 1021,
    "adxl_acc_x_kalman": 1022,
    "adxl_acc_y_kalman": 1023,
    "adxl_acc_z_kalman": 1024,

    # 2. Cosmic Aurora - LSM6DS3, BME280, SEN0232 (1025..1036)
    "accelx": 1025,
    "accely": 1026,
    "accelz": 1027,
    "gyrox": 1028,
    "gyroy": 1029,
    "gyroz": 1030,
    "bme_temp": 1031,
    "bme_hum": 1032,
    "bme_pres": 1033,
    "bme_alt": 1034,
    "sound_dbLeft": 1035,
    "sound_dbRight": 1036,

    # 3. SmartSkin Piezo 1..14 (1037..1050)
    **{f"piezo{i}": 1036 + i for i in range(1, 15)},

    # 4. SmartSkin FSR 1..14 (1051..1064)
    **{f"fsr{i}": 1050 + i for i in range(1, 15)},

    # 5. SmartSkin MCP9808 1..14 (1065..1078)
    **{f"mcp{i}": 1064 + i for i in range(1, 15)},

    # 6. SmartSkin Flex 1..16 (1079..1094)
    **{f"flex{i}": 1078 + i for i in range(1, 17)},

    # 7. Thermal (1095)
    "thermal": 1095,
}

GRAVITY_MS2 = 9.80665


# ============================================================
# CRC16-CCITT
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


# ============================================================
# NUMERIC VALIDATION
# ============================================================

def valid_number(value):

    if isinstance(value, bool):
        return None

    if value is None:
        return None

    try:

        number = float(value)

        if not math.isfinite(number):
            return None

        return number

    except (ValueError, TypeError):
        return None


# ============================================================
# LORA TRANSMITTER NODE
# ============================================================

class LoRaTransmitterNode(Node):

    def __init__(self):

        super().__init__("lora_transmitter_node")

        self.declare_parameter("mannequin_id", 1)
        self.declare_parameter("boot_count", 1)
        self.declare_parameter("payload_mode", "array")

        self.mannequin_id = int(self.get_parameter("mannequin_id").value)
        self.boot_count = int(self.get_parameter("boot_count").value)
        self.payload_mode = str(self.get_parameter("payload_mode").value).lower()
        self.packet_sequence = 0

        self.lock = threading.RLock()

        # Data terbaru yang sudah diterima
        self.latest_values = {}

        # Data valid terakhir
        self.last_values = {}

        self.tx_queue = deque()

        self.serial_port = None
        self.tx_active = False
        self.rak_ready = False
        self.rak_retry_interval = 10.0

        self.transmitted_count = 0
        self.failed_count = 0

        self.received_sources = set()

        # Alert queue has priority over regular telemetry packets.
        self.alert_sequence = 0
        self.alert_tx_queue = deque(maxlen=10)

        # Publisher JSON Compact v2 siap kirim langsung ke Web (Be-smart-manekin)
        self.web_v2_pub = self.create_publisher(
            String,
            "/telemetri/web/v2",
            10
        )

        # ====================================================
        # ROS SUBSCRIBERS
        # ====================================================

        # Arduino JSON
        self.arduino_sub = self.create_subscription(
            String,
            "/telemetri/arduino/data",
            self.arduino_callback,
            10
        )

        # Cosmic Aurora JSON
        self.aurora_sub = self.create_subscription(
            String,
            "/telemetri/aurora/data",
            self.aurora_callback,
            10
        )

        # Analog FSR, Piezo, Flex
        self.analog_sub = self.create_subscription(
            String,
            "/sensors/analog_data",
            self.analog_callback,
            10
        )

        # MCP9808
        self.mcp_sub = self.create_subscription(
            String,
            "/sensors/mcp9808",
            self.mcp_callback,
            10
        )

        # Load cell
        self.loadcell_sub = self.create_subscription(
            String,
            "/sensors/loadcell",
            self.loadcell_callback,
            10
        )

        # WitMotion WT901 JSON (/sensors/witmotion)
        self.witmotion_sub = self.create_subscription(
            String,
            "/sensors/witmotion",
            self.witmotion_callback,
            10
        )

        # Compact system alerts from Alert Manager
        self.alert_sub = self.create_subscription(
            String,
            "/system/alerts",
            self.alert_callback,
            10
        )

        # ====================================================
        # TIMERS
        # ====================================================

        self.transmit_timer = self.create_timer(
            TRANSMIT_INTERVAL,
            self.transmit_callback
        )

        self.queue_timer = self.create_timer(
            0.01,
            self.process_queue
        )

        # Retry RAK initialization while acquisition/ROS continues.
        self.rak_retry_timer = self.create_timer(
            self.rak_retry_interval,
            self.retry_rak_connection
        )

        # ====================================================
        # SERIAL INITIALIZATION
        # ====================================================

        self.initialize_serial()

        self.get_logger().info(
            f"LoRa node initialized | "
            f"Fields: {len(EXPECTED_KEYS)} | "
            f"Interval: {TRANSMIT_INTERVAL}s"
        )

    # ========================================================
    # SERIAL INITIALIZATION
    # ========================================================

    def initialize_serial(self):
        """Open UART3 and configure the RAK. Failure does not stop ROS."""
        try:
            if self.serial_port is None or not self.serial_port.is_open:
                self.serial_port = serial.Serial(
                    port=SERIAL_PORT,
                    baudrate=BAUD_RATE,
                    timeout=0.1,
                    write_timeout=1
                )
                # Give the UART/module time to settle after opening.
                time.sleep(1.0)
                self.get_logger().info(
                    f"UART opened: {SERIAL_PORT} @ {BAUD_RATE} baud"
                )

            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.rak_ready = self.configure_rak()
            return self.rak_ready

        except Exception as error:
            self.rak_ready = False
            self.get_logger().error(
                f"RAK UART initialization failed: {error}"
            )
            return False

    def retry_rak_connection(self):
        """Retry configuration periodically without blocking sensor nodes."""
        if self.rak_ready:
            return
        self.get_logger().warning(
            f"RAK not ready; retrying {SERIAL_PORT} initialization..."
        )
        self.initialize_serial()

    # ========================================================
    # SEND AT COMMAND
    # ========================================================

    def send_at(self, command, timeout=3.0):
        if self.serial_port is None or not self.serial_port.is_open:
            return False

        try:
            self.serial_port.reset_input_buffer()
            self.serial_port.write((command + "\r\n").encode("ascii"))
            self.serial_port.flush()

            start = time.monotonic()
            response = ""
            while time.monotonic() - start < timeout:
                waiting = self.serial_port.in_waiting
                if waiting:
                    response += self.serial_port.read(waiting).decode(
                        errors="ignore"
                    )
                    upper_response = response.upper()
                    if "ERROR" in upper_response:
                        self.get_logger().error(
                            f"AT error [{command}]: {response.strip()}"
                        )
                        return False
                    if "OK" in upper_response:
                        self.get_logger().info(f"AT OK: {command}")
                        return True
                time.sleep(0.02)

            self.get_logger().warning(
                f"AT timeout: {command} | response={response.strip()!r}"
            )
            return False

        except Exception as error:
            self.get_logger().error(f"AT command error [{command}]: {error}")
            return False

    # ========================================================
    # CONFIGURE RAK
    # ========================================================

    def configure_rak(self):
        if self.serial_port is None or not self.serial_port.is_open:
            self.rak_ready = False
            return False

        commands = [
            "AT",
            "AT+NWM=0",
            "AT+PRECV=0",
            (
                f"AT+P2P={FREQUENCY}:"
                f"{SPREADING_FACTOR}:"
                f"{BANDWIDTH}:"
                f"{CODING_RATE}:"
                f"{PREAMBLE}:"
                f"{TX_POWER}"
            ),
        ]

        self.rak_ready = False
        self.get_logger().info(
            f"Configuring RAK P2P on {SERIAL_PORT} "
            f"({FREQUENCY} Hz, SF{SPREADING_FACTOR})..."
        )

        for command in commands:
            success = False
            # A brief retry helps with module wake-up/boot timing.
            for attempt in range(1, 4):
                if self.send_at(command, timeout=3.0):
                    success = True
                    break
                if attempt < 3:
                    self.get_logger().warning(
                        f"Retry AT command {attempt + 1}/3: {command}"
                    )
                    time.sleep(0.5)

            if not success:
                self.get_logger().error(
                    f"RAK configuration failed at: {command}; "
                    f"TX is disabled until configuration succeeds"
                )
                self.rak_ready = False
                return False

            time.sleep(0.25)

        self.rak_ready = True
        self.get_logger().info(
            "RAK P2P READY. Telemetry and alert transmission enabled."
        )
        return True

    # ========================================================
    # PARSE JSON
    # ========================================================

    def parse_json(self, raw_data):

        try:

            result = json.loads(raw_data)

            if isinstance(result, dict):
                return result

            if isinstance(result, list):
                return result

            return None

        except (json.JSONDecodeError, TypeError):
            return None

    # ========================================================
    # UPDATE ONE FIELD
    # ========================================================

    def update_field(self, key, value):

        if key not in EXPECTED_KEYS:
            return False

        number = valid_number(value)

        if number is None:
            return False

        with self.lock:

            self.latest_values[key] = number
            self.last_values[key] = number

        return True

    # ========================================================
    # UPDATE MULTIPLE FIELDS USING MAP
    # ========================================================

    def update_mapped_values(self, data, mapping):

        if not isinstance(data, dict):
            return

        updated = 0

        for source_key, target_key in mapping.items():

            if source_key not in data:
                continue

            if self.update_field(
                target_key,
                data[source_key]
            ):
                updated += 1

        if updated > 0:
            return updated

        return 0

    # ========================================================
    # EXTRACT NORMALIZED JSON DATA
    # ========================================================

    def extract_sensor_data(self, payload):

        if not isinstance(payload, dict):
            return None

        # Struktur normal:
        # {
        #   "source": "...",
        #   "status": "...",
        #   "field_count": 25,
        #   "data": {...}
        # }

        nested = payload.get("data")

        if isinstance(nested, dict):
            return nested

        # Mendukung JSON flat bila suatu saat digunakan
        return payload

    # ========================================================
    # ARDUINO CALLBACK
    # ========================================================

    def arduino_callback(self, msg):

        payload = self.parse_json(msg.data)

        if payload is None:
            self.get_logger().warning(
                "Invalid Arduino JSON"
            )
            return

        data = self.extract_sensor_data(payload)

        if not isinstance(data, dict):
            return

        updated = self.update_mapped_values(
            data,
            ARDUINO_MAP
        )

        if updated:

            self.received_sources.add("arduino")

    # ========================================================
    # COSMIC AURORA CALLBACK
    # ========================================================

    def aurora_callback(self, msg):

        payload = self.parse_json(msg.data)

        if payload is None:
            self.get_logger().warning(
                "Invalid Cosmic Aurora JSON"
            )
            return

        data = self.extract_sensor_data(payload)

        if not isinstance(data, dict):
            return

        updated = self.update_mapped_values(
            data,
            AURORA_MAP
        )

        if updated:

            self.received_sources.add("aurora")

    # ========================================================
    # ANALOG CALLBACK
    # ========================================================

    def analog_callback(self, msg):

        payload = self.parse_json(msg.data)

        if not isinstance(payload, dict):
            self.get_logger().warning(
                "Invalid analog JSON"
            )
            return

        # Struktur:
        # {
        #   "fsr": [...14...],
        #   "piezo": [...14...],
        #   "flex": [...16...]
        # }

        updated = 0

        updated += self.update_array(
            payload.get("flex"),
            "flex"
        )

        updated += self.update_array(
            payload.get("fsr"),
            "fsr"
        )

        updated += self.update_array(
            payload.get("piezo"),
            "piezo"
        )

        if updated:
            self.received_sources.add("analog")

    # ========================================================
    # ARRAY MAPPING
    # ========================================================

    def update_array(self, values, prefix):

        if not isinstance(values, list):
            return 0

        updated = 0

        for index, value in enumerate(values):

            key = f"{prefix}{index + 1}"

            if key not in EXPECTED_KEYS:
                continue

            if self.update_field(key, value):
                updated += 1

        return updated

    # ========================================================
    # MCP9808 CALLBACK
    # ========================================================

    def mcp_callback(self, msg):

        payload = self.parse_json(msg.data)

        if not isinstance(payload, list):
            self.get_logger().warning(
                "MCP9808 data must be a JSON array"
            )
            return

        updated = self.update_array(
            payload,
            "mcp"
        )

        if updated:
            self.received_sources.add("mcp9808")

    # ========================================================
    # LOAD CELL CALLBACK
    # ========================================================

    def loadcell_callback(self, msg):

        payload = self.parse_json(msg.data)

        if not isinstance(payload, dict):
            return

        # 1. Format terbaru dari hx711_node.py:
        #    {"loadcell": [[raw, gram, gram_kalman, newton, newton_kalman], ...]}
        #    Urutan LOADCELLS di hx711_node.py:
        #    [0]=leher, [1]=paha_kanan, [2]=paha_kiri, [3]=kaki_kanan, [4]=kaki_kiri
        loadcell_list = payload.get("loadcell")

        if isinstance(loadcell_list, list):
            updated = 0

            for idx, target_key in enumerate(LOADCELL_INDEX_KEYS):
                if idx >= len(loadcell_list):
                    break

                entry = loadcell_list[idx]

                if isinstance(entry, list):
                    # Prioritaskan gram_kalman (index 2), fallback ke gram (index 1) atau raw (index 0)
                    if len(entry) >= 3:
                        val = entry[2]
                    elif len(entry) >= 2:
                        val = entry[1]
                    elif len(entry) >= 1:
                        val = entry[0]
                    else:
                        continue
                else:
                    val = entry

                if self.update_field(target_key, val):
                    updated += 1

            if updated:
                self.received_sources.add("loadcell")
            return

        # 2. Format dictionary (fallback):
        data = payload.get("data", payload)

        if not isinstance(data, dict):
            return

        mapping = {
            "loadcell_paka": "loadcell_paka",
            "loadcell_paki": "loadcell_paki",
            "loadcell_leher": "loadcell_leher",
            "loadcell_kaka": "loadcell_kaka",
            "loadcell_kaki": "loadcell_kaki",
            "leher": "loadcell_leher",
            "paha_kanan": "loadcell_paka",
            "paha_kiri": "loadcell_paki",
            "kaki_kanan": "loadcell_kaka",
            "kaki_kaki": "loadcell_kaki",
            "kaki_kiri": "loadcell_kaki",
        }

        updated = self.update_mapped_values(
            data,
            mapping
        )

        if updated:
            self.received_sources.add("loadcell")

    # ========================================================
    # WITMOTION JSON CALLBACK (/sensors/witmotion)
    # ========================================================

    def witmotion_callback(self, msg):

        payload = self.parse_json(msg.data)

        if not isinstance(payload, dict):
            self.get_logger().warning(
                "Invalid WitMotion JSON"
            )
            return

        # Saat masih auto-calibration (status == "calibrating"),
        # witmotion_node hanya mengirim progress kalibrasi tanpa field data.
        status = str(payload.get("status", "")).lower()
        if status == "calibrating":
            return

        data = self.extract_sensor_data(payload)

        if not isinstance(data, dict):
            return

        accel = data.get("accel")
        gyro = data.get("gyro")
        euler = data.get("euler")

        candidates = {}

        # Web Backend menggunakan satuan:
        # - Akselerasi: g (-16 .. 16 g). Jika witmotion_node mengirim m/s2, bagi 9.80665.
        # - Gyroscope : deg/s (-2000 .. 2000 °/s). Jika witmotion_node mengirim rad/s, konversi degrees().
        # - Euler     : degree (-180 .. 180 °).
        if isinstance(accel, dict):
            accel_unit = str(accel.get("unit", "m/s2")).lower()
            for axis, key in (("x", "ax"), ("y", "ay"), ("z", "az")):
                raw_val = valid_number(accel.get(axis))
                if raw_val is not None:
                    if "m/s" in accel_unit:
                        candidates[key] = round(raw_val / GRAVITY_MS2, 4)
                    else:
                        candidates[key] = round(raw_val, 4)
        else:
            candidates["ax"] = data.get("accel_x", data.get("ax"))
            candidates["ay"] = data.get("accel_y", data.get("ay"))
            candidates["az"] = data.get("accel_z", data.get("az"))

        if isinstance(gyro, dict):
            gyro_unit = str(gyro.get("unit", "rad/s")).lower()
            for axis, key in (("x", "gx"), ("y", "gy"), ("z", "gz")):
                raw_val = valid_number(gyro.get(axis))
                if raw_val is not None:
                    if "rad" in gyro_unit:
                        candidates[key] = round(math.degrees(raw_val), 4)
                    else:
                        candidates[key] = round(raw_val, 4)
        else:
            candidates["gx"] = data.get("gyro_x", data.get("gx"))
            candidates["gy"] = data.get("gyro_y", data.get("gy"))
            candidates["gz"] = data.get("gyro_z", data.get("gz"))

        if isinstance(euler, dict):
            for axis in ("roll", "pitch", "yaw"):
                raw_val = valid_number(euler.get(axis))
                if raw_val is not None:
                    candidates[axis] = round(raw_val, 3)
        else:
            candidates["roll"] = data.get("roll")
            candidates["pitch"] = data.get("pitch")
            candidates["yaw"] = data.get("yaw")

        updated = 0

        for key, val in candidates.items():
            if self.update_field(key, val):
                updated += 1

        if updated:
            self.received_sources.add("witmotion")

    # ========================================================
    # COMPACT ALERT ENCODER
    # ========================================================

    def alert_callback(self, msg):
        """Encode a JSON alert as a compact 12-byte ASCII payload.

        Layout: A1 + sequence(4 hex) + severity(1 hex) +
                event(2 hex) + status(1 hex) + offline-mask(2 hex).
        CRC16 and FRG header are added by fragment_packet().
        """
        try:
            alert = json.loads(msg.data)
            if not isinstance(alert, dict):
                raise ValueError("Alert JSON must be an object")

            severity_map = {"INFO": 0, "WARNING": 1, "ERROR": 2, "CRITICAL": 3}
            event_map = {
                "SYSTEM_STATUS_CHANGED": 0x01,
                "DEVICE_OFFLINE": 0x02,
                "DEVICE_RECOVERED": 0x03,
                "DEVICE_CALIBRATING": 0x04,
            }
            status_map = {
                "INITIALIZING": 0,
                "OPERATIONAL": 1,
                "DEGRADED": 2,
                "CALIBRATING": 3,
            }
            device_bits = {
                "arduino_uno_q": 0,
                "cosmic_aurora": 1,
                "analog_sensors": 2,
                "mcp9808": 3,
                "hx711_loadcells": 4,
                "witmotion_wt901": 5,
            }

            severity_name = str(alert.get("severity", "INFO")).upper()
            event_name = str(alert.get("code", "UNKNOWN")).upper()
            details = alert.get("details") or {}
            if not isinstance(details, dict):
                details = {}
            system = details.get("system_status") or {}
            if not isinstance(system, dict):
                system = {}

            status_name = system.get("status") or details.get("current_status") or "INITIALIZING"
            severity = severity_map.get(severity_name, 0)
            event_code = event_map.get(event_name, 0x7F)
            status = status_map.get(str(status_name).upper(), 0)

            offline_devices = system.get("offline_devices", [])
            if not isinstance(offline_devices, list):
                offline_devices = []
            # Device-specific events may identify one device directly.
            device_name = details.get("device") or details.get("device_name")
            if device_name:
                offline_devices = list(offline_devices) + [device_name]

            device_mask = 0
            for device in offline_devices:
                bit = device_bits.get(str(device))
                if bit is not None:
                    device_mask |= (1 << bit)

            self.alert_sequence = (self.alert_sequence + 1) & 0xFFFF
            compact = (
                f"A1{self.alert_sequence:04X}{severity:X}"
                f"{event_code:02X}{status:X}{device_mask:02X}"
            )
            if len(compact) != 12:
                raise ValueError(f"Invalid compact alert length: {len(compact)}")

            fragment = self.fragment_packet(compact)
            if len(self.alert_tx_queue) == self.alert_tx_queue.maxlen:
                self.get_logger().warning("Alert TX queue full; oldest alert will be discarded")
            self.alert_tx_queue.append(fragment)
            self.get_logger().info(
                f"Alert queued: {compact} ({len(compact)} payload bytes)"
            )

        except Exception as error:
            self.get_logger().error(f"Alert encoding failed: {error}")

    # ========================================================
    # BUILD FIXED JSON ARRAY
    # ========================================================

    def build_packet(self):

        values = []

        with self.lock:

            for key in EXPECTED_KEYS:

                value = self.latest_values.get(key)

                if value is None and HOLD_LAST_VALUE:
                    value = self.last_values.get(key)

                values.append(value)

        # None akan menjadi null di JSON.
        # Tidak ada dummy atau angka buatan.

        return json.dumps(
            values,
            separators=(",", ":"),
            allow_nan=False
        )

    # ========================================================
    # BUILD WEB COMPACT V2 PACKET (Be-smart-manekin)
    # {"v":2,"b":...,"s":...,"t":...,"r":[[readingId,value],...]}
    # ========================================================

    def build_web_v2_packet(self):

        id_offset = 1000 if self.mannequin_id == 2 else 0
        readings = []

        with self.lock:

            for key, base_id in WEB_READING_ID_MAP_M1.items():

                value = self.latest_values.get(key)

                if value is None and HOLD_LAST_VALUE:
                    value = self.last_values.get(key)

                readings.append([base_id + id_offset, value])

            self.packet_sequence = (self.packet_sequence + 1) & 0x7FFFFFFF
            seq = self.packet_sequence

        payload = {
            "v": 2,
            "b": self.boot_count,
            "s": seq,
            "t": int(time.time() * 1000),
            "r": readings,
        }

        return json.dumps(
            payload,
            separators=(",", ":"),
            allow_nan=False
        )

    # ========================================================
    # BUILD ONE LORA FRAGMENT
    # ========================================================

    def fragment_packet(self, payload):

        json_bytes = payload.encode("utf-8")

        crc = crc16_ccitt(json_bytes)

        complete_data = (
            json_bytes +
            f"*{crc:04X}".encode("ascii")
        )

        header = b"FRG:1/1:"

        fragment = header + complete_data

        if len(fragment) > MAX_PAYLOAD:

            raise ValueError(
                f"Payload terlalu besar: {len(fragment)} bytes"
            )

        return fragment

    # ========================================================
    # TRANSMIT TIMER
    # ========================================================

    def transmit_callback(self):

        # Jangan kirim paket kosong sebelum menerima data sensor.
        with self.lock:

            if not self.latest_values:
                return

        try:

            # Publish format Compact v2 Web ke topic ROS /telemetri/web/v2
            web_v2_json = self.build_web_v2_packet()
            web_msg = String()
            web_msg.data = web_v2_json
            self.web_v2_pub.publish(web_msg)

            if self.serial_port is None:
                return

            if self.payload_mode == "web_v2":
                payload = web_v2_json
            else:
                payload = self.build_packet()

            fragment = self.fragment_packet(payload)

            with self.lock:

                if len(self.tx_queue) >= MAX_QUEUE_SIZE:

                    self.tx_queue.popleft()

                    self.get_logger().warning(
                        "Oldest packet removed from TX queue"
                    )

                self.tx_queue.append(fragment)

        except Exception as error:

            self.failed_count += 1

            self.get_logger().error(
                f"Packet creation failed: {error}"
            )

    # ========================================================
    # PROCESS TX QUEUE
    # ========================================================

    def process_queue(self):

        if self.serial_port is None or not self.rak_ready:
            return

        if self.tx_active:
            return

        # Alert packets take priority; telemetry queue remains unchanged.
        if self.alert_tx_queue:
            fragment = self.alert_tx_queue.popleft()
        elif self.tx_queue:
            fragment = self.tx_queue.popleft()
        else:
            return

        self.tx_active = True

        try:

            hex_data = fragment.hex().upper()

            command = f"AT+PSEND={hex_data}"

            self.serial_port.reset_input_buffer()

            self.serial_port.write(
                (command + "\r\n").encode("ascii")
            )

            self.serial_port.flush()

            success = self.wait_tx_done(timeout=2.0)

            if success:

                self.transmitted_count += 1

                self.get_logger().info(
                    f"TX success: {self.transmitted_count} | "
                    f"Sources: {sorted(self.received_sources)}"
                )

            else:

                self.failed_count += 1

                self.get_logger().warning(
                    f"TX failed: {self.failed_count}"
                )

        except Exception as error:

            self.failed_count += 1

            self.get_logger().error(
                f"LoRa TX error: {error}"
            )

        finally:

            self.tx_active = False

    # ========================================================
    # WAIT TRANSMISSION CONFIRMATION
    # ========================================================

    def wait_tx_done(self, timeout=2.0):

        start = time.monotonic()

        response = ""

        while time.monotonic() - start < timeout:

            if self.serial_port.in_waiting:

                data = self.serial_port.read(
                    self.serial_port.in_waiting
                )

                response += data.decode(
                    errors="ignore"
                )

                if "+EVT:TXP2P DONE" in response:
                    return True

                if "ERROR" in response:
                    self.get_logger().error(
                        response.strip()
                    )
                    return False

            time.sleep(0.02)

        self.get_logger().warning(
            f"TX confirmation timeout: {response.strip()}"
        )

        return False

    # ========================================================
    # SHUTDOWN
    # ========================================================

    def destroy_node(self):

        self.rak_ready = False

        if self.serial_port is not None:

            try:
                self.serial_port.close()
            except Exception:
                pass

        super().destroy_node()


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = LoRaTransmitterNode()

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as error:
        print(f"LoRa node failed: {error}")

    finally:

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
