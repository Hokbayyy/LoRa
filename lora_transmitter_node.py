#!/usr/bin/env python3

import json
import math
import random
import time
import threading
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

FREQUENCY = 923200000
SPREADING_FACTOR = 6
BANDWIDTH = 2
CODING_RATE = 0
PREAMBLE = 8
TX_POWER = 20

TRANSMIT_INTERVAL = 1.0

# Maksimal data per fragment.
CHUNK_SIZE = 185

# Maksimal payload AT+PSEND.
MAX_PAYLOAD = 255

# Jika True, nilai sensor terakhir tetap digunakan ketika
# data sensor tidak diperbarui.
HOLD_LAST_VALUE = True

# Jika True, parameter yang belum diterima akan diisi dummy.
ENABLE_DUMMY_DATA = True


# ============================================================
# URUTAN 35 PARAMETER
# JANGAN MENGUBAH URUTAN INI TANPA MENGUBAH RECEIVER
# ============================================================

EXPECTED_KEYS = [
    # Arduino UNO Q (23 parameter)
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

    # Cosmic Aurora (12 parameter)
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
]


# ============================================================
# DUMMY DATA
# Digunakan ketika parameter belum diterima dari sensor.
# Nilai ini hanya data simulasi, bukan hasil pengukuran.
# ============================================================

DUMMY_VALUES = {
    # Arduino UNO Q
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

    # Cosmic Aurora
    "accelx": 0.25,
    "accely": 0.31,
    "accelz": 9.72,

    "gyrox": 0.01,
    "gyroy": 0.02,
    "gyroz": 0.03,

    "bme_temp": 28.5,
    "bme_hum": 65.2,
    "bme_pres": 1008.5,
    "bme_alt": 120.0,

    "sound_dbLeft": 42.5,
    "sound_dbRight": 43.1,
}


# ============================================================
# ALIAS COSMIC AURORA
# Menyesuaikan nama JSON ESP32 dengan nama parameter.
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
# VALIDASI NILAI NUMERIK
# ============================================================

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


# ============================================================
# NODE ROS2
# ============================================================

class LoRaTransmitterNode(Node):

    def __init__(self):
        super().__init__("lora_transmitter_node")

        self.lock = threading.Lock()

        self.latest_values = {}
        self.last_values = {}

        self.tx_queue = deque()

        self.serial_port = None
        self.tx_active = False

        self.last_transmit_time = 0.0

        # ----------------------------------------------------
        # ROS2 SUBSCRIBERS
        # ----------------------------------------------------

        self.arduino_sub = self.create_subscription(
            String,
            "/telemetri/arduino",
            self.arduino_callback,
            10
        )

        self.aurora_sub = self.create_subscription(
            String,
            "/telemetri/aurora",
            self.aurora_callback,
            10
        )

        # ----------------------------------------------------
        # TIMER
        # ----------------------------------------------------

        self.transmit_timer = self.create_timer(
            TRANSMIT_INTERVAL,
            self.transmit_callback
        )

        self.queue_timer = self.create_timer(
            0.01,
            self.process_queue
        )

        # ----------------------------------------------------
        # SERIAL INITIALIZATION
        # ----------------------------------------------------

        self.initialize_serial()

    # ========================================================
    # SERIAL INITIALIZATION
    # ========================================================

    def initialize_serial(self):

        try:
            self.serial_port = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                timeout=0.1,
                write_timeout=1
            )

            time.sleep(1)

            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()

            self.get_logger().info(
                f"RAK connected: {SERIAL_PORT}"
            )

            self.configure_rak()

        except Exception as error:

            self.get_logger().error(
                f"RAK connection failed: {error}"
            )

            self.serial_port = None

    # ========================================================
    # SEND AT COMMAND
    # ========================================================

    def send_at(self, command, timeout=2.0):

        if self.serial_port is None:
            return False

        try:
            self.serial_port.reset_input_buffer()

            self.serial_port.write(
                (command + "\r\n").encode()
            )

            self.serial_port.flush()

            start_time = time.time()
            response = ""

            while time.time() - start_time < timeout:

                if self.serial_port.in_waiting:

                    data = self.serial_port.read(
                        self.serial_port.in_waiting
                    )

                    response += data.decode(
                        errors="ignore"
                    )

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

                time.sleep(0.05)

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

        self.get_logger().info(
            "Configuring RAK LoRa P2P..."
        )

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

        for command in commands:

            result = self.send_at(command)

            if not result:
                self.get_logger().warning(
                    f"Configuration command not confirmed: {command}"
                )

            time.sleep(0.2)

        self.get_logger().info(
            "RAK configuration sequence completed."
        )

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
    # UPDATE SENSOR DATA
    # ========================================================

    def update_values(self, data, aliases=None):

        if not isinstance(data, dict):
            return

        aliases = aliases or {}

        with self.lock:

            for original_key, value in data.items():

                key = aliases.get(
                    original_key,
                    original_key
                )

                if key not in EXPECTED_KEYS:
                    continue

                number = valid_number(value)

                if number is None:
                    continue

                self.latest_values[key] = number
                self.last_values[key] = number

    # ========================================================
    # ARDUINO CALLBACK
    # ========================================================

    def arduino_callback(self, msg):

        data = self.parse_json(msg.data)

        if data is None:

            self.get_logger().warning(
                "Arduino message is not a valid JSON object."
            )

            return

        self.update_values(data)

    # ========================================================
    # COSMIC AURORA CALLBACK
    # ========================================================

    def aurora_callback(self, msg):

        data = self.parse_json(msg.data)

        if data is None:

            self.get_logger().warning(
                "Aurora message is not a valid JSON object."
            )

            return

        self.update_values(
            data,
            aliases=AURORA_ALIASES
        )

    # ========================================================
    # GENERATE DUMMY DATA
    # ========================================================

    def generate_dummy_values(self):

        dummy = {}

        for key in EXPECTED_KEYS:

            base_value = DUMMY_VALUES[key]

            # Berikan variasi kecil pada data simulasi.
            variation = abs(base_value) * 0.01

            if variation == 0:
                variation = 0.01

            dummy[key] = round(
                random.uniform(
                    base_value - variation,
                    base_value + variation
                ),
                3
            )

        return dummy

    # ========================================================
    # BUILD NUMERIC JSON ARRAY
    # ========================================================

    def build_packet(self):

        dummy_values = self.generate_dummy_values()

        values = []

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

                number = valid_number(value)

                if number is None:
                    number = dummy_values[key]

                values.append(number)

        # JSON hanya berisi angka dalam urutan tetap.
        payload = json.dumps(
            values,
            separators=(",", ":"),
            allow_nan=False
        )

        return payload

    # ========================================================
    # SPLIT JSON INTO LORA FRAGMENTS
    # ========================================================

    def fragment_packet(self, payload):

        json_bytes = payload.encode("utf-8")

        # CRC dihitung dari JSON asli.
        crc = crc16_ccitt(json_bytes)

        # CRC ditambahkan setelah JSON.
        # Receiver harus menghapus 5 byte terakhir (*XXXX)
        # sebelum melakukan json.loads().
        complete_data = (
            json_bytes + f"*{crc:04X}".encode()
        )

        chunks = [
            complete_data[i:i + CHUNK_SIZE]
            for i in range(
                0,
                len(complete_data),
                CHUNK_SIZE
            )
        ]

        total = len(chunks)

        fragments = []

        for index, chunk in enumerate(chunks, start=1):

            header = f"FRG:{index}/{total}:".encode()

            fragment = header + chunk

            if len(fragment) > MAX_PAYLOAD:
                raise ValueError(
                    "Fragment exceeds maximum LoRa payload."
                )

            fragments.append(fragment)

        return fragments

    # ========================================================
    # TRANSMIT CALLBACK
    # ========================================================

    def transmit_callback(self):

        if self.serial_port is None:
            return

        try:

            payload = self.build_packet()

            fragments = self.fragment_packet(payload)

            with self.lock:

                for fragment in fragments:
                    self.tx_queue.append(fragment)

            self.get_logger().info(
                f"Queued {len(fragments)} fragment(s), "
                f"{len(payload)} JSON characters."
            )

        except Exception as error:

            self.get_logger().error(
                f"Packet creation failed: {error}"
            )

    # ========================================================
    # PROCESS TRANSMISSION QUEUE
    # ========================================================

    def process_queue(self):

        if self.serial_port is None:
            return

        if self.tx_active:
            return

        if not self.tx_queue:
            return

        fragment = self.tx_queue.popleft()

        self.tx_active = True

        try:

            hex_data = fragment.hex().upper()

            command = f"AT+PSEND={hex_data}"

            self.serial_port.reset_input_buffer()

            self.serial_port.write(
                (command + "\r\n").encode()
            )

            self.serial_port.flush()

            success = self.wait_tx_done(timeout=2.0)

            if success:

                self.get_logger().info(
                    "LoRa fragment transmitted."
                )

            else:

                self.get_logger().warning(
                    "LoRa transmission not confirmed."
                )

        except Exception as error:

            self.get_logger().error(
                f"LoRa transmission error: {error}"
            )

        finally:

            self.tx_active = False

    # ========================================================
    # WAIT FOR TX DONE
    # ========================================================

    def wait_tx_done(self, timeout=2.0):

        start_time = time.time()
        response = ""

        while time.time() - start_time < timeout:

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
                        f"RAK response: {response.strip()}"
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

    node = LoRaTransmitterNode()

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
