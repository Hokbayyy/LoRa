#!/usr/bin/env python3
"""
LoRa P2P Receiver Node (ROS2) - RAK3272S (99 Parameter Sensor Terkompresi)
Pasangan persis dari lora_transmitter_node.py.

Alur:
  1. Radio LoRa P2P menerima event +EVT:RXP2P:<rssi>:<snr>:<hex_data>
  2. Konversi hex ke bytes:
       - Paket Terkompresi Biner ("P99:", tepat 206 Byte):
           * Verifikasi header b"P99:" dan panjang 206 Byte
           * Verifikasi CRC16-CCITT (2 byte terakhir)
           * Unpack counter + 99 nilai float16 (<99e)
       - Fallback paket JSON / "FRG:" (jika ada format lama)
  3. Memetakan 99 angka ke EXPECTED_KEYS:
       - 23 parameter  -> Arduino UNO Q
       - 12 parameter  -> Cosmic Aurora
       - 5 parameter   -> Loadcell
       - 16 parameter  -> Sensor Flex (flex1 - flex16)
       - 14 parameter  -> Sensor MCP (mcp1 - mcp14)
       - 14 parameter  -> Sensor FSR (fsr1 - fsr14)
       - 14 parameter  -> Sensor Piezo (piezo1 - piezo14)
       - 1 parameter   -> Sensor Thermal
  4. Menampilkan log 1 baris horizontal bersih tanpa jeda baris dan tanpa emoji.
  5. Mempublikasikan ke topic ROS2:
       - /telemetri/arduino       (JSON 23 parameter Arduino)
       - /telemetri/aurora        (JSON 12 parameter Aurora)
       - /telemetri/loadcell      (JSON 5 parameter Loadcell)
       - /telemetri/flex          (JSON 16 parameter Flex)
       - /telemetri/mcp           (JSON 14 parameter MCP)
       - /telemetri/fsr           (JSON 14 parameter FSR)
       - /telemetri/piezo         (JSON 14 parameter Piezo)
       - /telemetri/thermal       (JSON parameter Thermal)
       - /telemetri/received_raw  (JSON seluruh 99 parameter)
       - /telemetri/radio_status  (JSON status RSSI & SNR)
"""

import json
import math
import os
import struct
import time

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ============================================================
# KONFIGURASI RAK LORA (PERSIS SESUAI TRANSMITTER)
# ============================================================

SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200

FREQUENCY = 923200000
SPREADING_FACTOR = 6
BANDWIDTH = 2
CODING_RATE = 0
PREAMBLE = 8
TX_POWER = 20


# ============================================================
# URUTAN 99 PARAMETER (PERSIS SESUAI TRANSMITTER)
# JANGAN MENGUBAH URUTAN INI TANPA MENGUBAH TRANSMITTER
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

NUM_PARAMS = len(EXPECTED_KEYS)
COMPRESSED_PACKET_SIZE = 4 + 2 + (NUM_PARAMS * 2) + 2  # 206 Bytes


# ============================================================
# CRC16-CCITT (PERSIS TRANSMITTER)
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
# FORMAT LOG HORIZONTAL 1 BARIS BERSIH (99 PARAMETER)
# ============================================================

def format_horizontal(d: dict) -> str:
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
# NODE ROS2 RECEIVER
# ============================================================

class LoRaReceiverNode(Node):

    def __init__(self):
        super().__init__("lora_receiver_node")

        # ---- Parameters ----
        self.declare_parameter("port", SERIAL_PORT)
        self.declare_parameter("baudrate", BAUD_RATE)
        self.declare_parameter("freq", FREQUENCY)
        self.declare_parameter("sf", SPREADING_FACTOR)
        self.declare_parameter("bw", BANDWIDTH)
        self.declare_parameter("cr", CODING_RATE)
        self.declare_parameter("preamble", PREAMBLE)
        self.declare_parameter("tx_power", TX_POWER)
        self.declare_parameter("frag_timeout", 3.0)

        gp = lambda n: self.get_parameter(n).value
        self.port = str(gp("port"))
        self.baud = int(gp("baudrate"))
        self.freq = int(gp("freq"))
        self.sf = int(gp("sf"))
        self.bw = int(gp("bw"))
        self.cr = int(gp("cr"))
        self.preamble = int(gp("preamble"))
        self.tx_power = int(gp("tx_power"))
        self.frag_timeout = float(gp("frag_timeout"))

        # ---- Serial & State ----
        self.serial_port = None
        self.rx_buffer = ""
        self.last_reconnect_try = 0.0

        self.packet_count = 0
        self.frag_buffer = {}
        self.frag_total = 0
        self.frag_start_time = 0.0
        self.last_rssi = "-"
        self.last_snr = "-"

        # ---- Publishers ----
        self.pub_arduino = self.create_publisher(String, "/telemetri/arduino", 10)
        self.pub_aurora = self.create_publisher(String, "/telemetri/aurora", 10)
        self.pub_loadcell = self.create_publisher(String, "/telemetri/loadcell", 10)
        self.pub_flex = self.create_publisher(String, "/telemetri/flex", 10)
        self.pub_mcp = self.create_publisher(String, "/telemetri/mcp", 10)
        self.pub_fsr = self.create_publisher(String, "/telemetri/fsr", 10)
        self.pub_piezo = self.create_publisher(String, "/telemetri/piezo", 10)
        self.pub_thermal = self.create_publisher(String, "/telemetri/thermal", 10)
        self.pub_raw = self.create_publisher(String, "/telemetri/received_raw", 10)
        self.pub_status = self.create_publisher(String, "/telemetri/radio_status", 10)

        # ---- Initialize Serial & Radio ----
        self.initialize_serial()

        # ---- Timers ----
        self.create_timer(0.005, self.read_serial_data)
        self.create_timer(0.5, self.check_frag_timeout)

        self.get_logger().info(
            f"LoRa receiver node started on {self.port} @ {self.baud} "
            f"(Expecting compressed P99 {COMPRESSED_PACKET_SIZE}B / 99 parameters)"
        )

    # ========================================================
    # SERIAL INITIALIZATION
    # ========================================================

    def initialize_serial(self):
        candidate_ports = [self.port, "/dev/ttyAMA0", "/dev/serial0", "/dev/ttyAMA3", "/dev/ttyUSB0"]
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
    # SEND AT COMMAND
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
    # CONFIGURE RAK RECEIVER
    # ========================================================

    def configure_rak(self):
        if self.serial_port is None:
            return

        self.get_logger().info("Configuring RAK LoRa P2P Receiver...")

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

        res_recv = self.send_at("AT+PRECV=65534", timeout=2.0)
        if not res_recv:
            self.get_logger().info("Fallback AT+PRECV=65535...")
            self.send_at("AT+PRECV=65535", timeout=2.0)

        self.get_logger().info("RAK Receiver configuration sequence completed.")

    # ========================================================
    # BACA DATA DARI SERIAL
    # ========================================================

    def read_serial_data(self):
        if self.serial_port is None or not self.serial_port.is_open:
            self.try_reconnect()
            return

        try:
            n = self.serial_port.in_waiting
            if n <= 0:
                return

            data = self.serial_port.read(n)
            self.rx_buffer += data.decode("ascii", errors="replace")

            while "\n" in self.rx_buffer:
                line, self.rx_buffer = self.rx_buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue

                if "+EVT:RXP2P" in line or "RXP2P" in line:
                    rssi, snr, hex_str = self.parse_rx_line(line)
                    if hex_str:
                        try:
                            raw_bytes = bytes.fromhex(hex_str)
                            self.handle_received_bytes(raw_bytes, rssi, snr)
                        except ValueError:
                            self.get_logger().error(f"Gagal konversi hex: {hex_str[:30]}")

            if len(self.rx_buffer) > 16384:
                self.rx_buffer = ""

        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f"Serial read error: {exc}")
            self.try_reconnect()

    def parse_rx_line(self, line: str):
        """Mengekstrak RSSI, SNR, dan hex string dari event RXP2P secara tangguh."""
        if "+EVT:RXP2P" not in line and "RXP2P" not in line:
            return None, None, None

        idx = line.find("RXP2P")
        after = line[idx + len("RXP2P"):].strip()
        after = after.lstrip(": ")
        tokens = [t.strip() for t in after.split(":") if t.strip()]

        if len(tokens) >= 3:
            rssi = tokens[0]
            snr = tokens[1]
            hex_str = ":".join(tokens[2:]).strip()
            if hex_str.upper() in ["TIMEOUT", "ERROR", "RECEIVE TIMEOUT"]:
                return None, None, None
            return rssi, snr, hex_str

        parts = line.split()
        if len(parts) >= 3:
            hex_str = parts[-1].strip()
            if hex_str.upper() in ["TIMEOUT", "ERROR", "RECEIVE TIMEOUT"]:
                return None, None, None
            rssi = parts[-3].strip() if len(parts) >= 4 else "-"
            snr = parts[-2].strip() if len(parts) >= 4 else "-"
            return rssi, snr, hex_str

        return None, None, None

    # ========================================================
    # ROUTE PAKET MASUK (KOMPRESI P99 ATAU FALLBACK FRG/JSON)
    # ========================================================

    def handle_received_bytes(self, raw_bytes: bytes, rssi: str, snr: str):
        self.last_rssi = rssi or "-"
        self.last_snr = snr or "-"

        # 1. Jalur Utama: Paket Kompresi Biner "P99:" (206 Byte)
        if raw_bytes.startswith(b"P99:"):
            self.process_compressed_p99(raw_bytes)
            return

        # 2. Fallback: Paket Fragmen "FRG:i/n:<chunk>"
        if raw_bytes.startswith(b"FRG:"):
            now = time.monotonic()
            try:
                meta, chunk = raw_bytes[4:].split(b":", 1)
                idx_str, tot_str = meta.decode("ascii").split("/")
                idx, total = int(idx_str), int(tot_str)

                if idx == 1 or total != self.frag_total:
                    self.frag_buffer.clear()
                    self.frag_total = total
                    self.frag_start_time = now

                self.frag_buffer[idx] = chunk

                if len(self.frag_buffer) == self.frag_total:
                    complete_data = b"".join(
                        self.frag_buffer[i] for i in range(1, self.frag_total + 1)
                    )
                    elapsed_ms = (now - self.frag_start_time) * 1000
                    total_frags = self.frag_total

                    self.frag_buffer.clear()
                    self.frag_total = 0

                    self.process_json_payload(complete_data, total_frags, elapsed_ms)

            except Exception as err:
                self.get_logger().error(f"Gagal memproses fragmen: {err}")
            return

        # 3. Fallback: Raw JSON tanpa awalan
        self.process_json_payload(raw_bytes, total_frags=1, elapsed_ms=0.0)

    # ========================================================
    # DECODE PAKET KOMPRESI BINER P99 (206 BYTE)
    # ========================================================

    def process_compressed_p99(self, raw_bytes: bytes):
        self.packet_count += 1

        if len(raw_bytes) != COMPRESSED_PACKET_SIZE:
            self.get_logger().warning(
                f"[RX #{self.packet_count}] Panjang paket P99 tidak sesuai: "
                f"{len(raw_bytes)}B (seharusnya {COMPRESSED_PACKET_SIZE}B). Paket dibuang."
            )
            return

        body = raw_bytes[:-2]
        expected_crc = struct.unpack("<H", raw_bytes[-2:])[0]
        actual_crc = crc16_ccitt(body)

        if actual_crc != expected_crc:
            self.get_logger().warning(
                f"[RX #{self.packet_count}] CRC MISMATCH! "
                f"(Dihitung: {actual_crc:04X}, Diterima: {expected_crc:04X}). Paket dibuang."
            )
            return

        tx_counter = struct.unpack("<H", raw_bytes[4:6])[0]
        raw_values = struct.unpack(f"<{NUM_PARAMS}e", raw_bytes[6:COMPRESSED_PACKET_SIZE - 2])

        sensor_dict = {}
        for key, val in zip(EXPECTED_KEYS, raw_values):
            fval = float(val)
            if not math.isfinite(fval):
                fval = 0.0
            sensor_dict[key] = round(fval, 3)

        self.publish_and_log(sensor_dict, len(raw_bytes), 1, 0.0, tx_counter=tx_counter)

    # ========================================================
    # DECODE PAKET JSON FALLBACK
    # ========================================================

    def process_json_payload(self, complete_data: bytes, total_frags: int, elapsed_ms: float):
        self.packet_count += 1

        if b"*" in complete_data:
            json_bytes, crc_bytes = complete_data.rsplit(b"*", 1)
            try:
                expected_crc = int(crc_bytes.decode("ascii"), 16)
                actual_crc = crc16_ccitt(json_bytes)
                if actual_crc != expected_crc:
                    self.get_logger().warning(
                        f"[RX #{self.packet_count}] CRC MISMATCH JSON!"
                    )
                    return
            except Exception:
                json_bytes = complete_data
        else:
            json_bytes = complete_data

        try:
            values = json.loads(json_bytes.decode("utf-8"))
        except Exception as err:
            self.get_logger().warning(f"Gagal parse JSON ({err}): {json_bytes[:60]}")
            return

        if not isinstance(values, list):
            self.get_logger().warning(f"Format payload bukan list angka: {type(values)}")
            return

        sensor_dict = {key: val for key, val in zip(EXPECTED_KEYS, values)}
        self.publish_and_log(sensor_dict, len(complete_data), total_frags, elapsed_ms)

    # ========================================================
    # LOG 1 BARIS HORIZONTAL & PUBLISH ROS2 TOPICS
    # ========================================================

    def publish_and_log(self, sensor_dict: dict, byte_len: int, total_frags: int,
                        elapsed_ms: float, tx_counter: int = None):
        arduino_dict = {k: sensor_dict[k] for k in EXPECTED_KEYS[:23] if k in sensor_dict}
        aurora_dict = {k: sensor_dict[k] for k in EXPECTED_KEYS[23:35] if k in sensor_dict}
        loadcell_dict = {k: sensor_dict[k] for k in EXPECTED_KEYS[35:40] if k in sensor_dict}
        flex_dict = {k: sensor_dict[k] for k in EXPECTED_KEYS[40:56] if k in sensor_dict}
        mcp_dict = {k: sensor_dict[k] for k in EXPECTED_KEYS[56:70] if k in sensor_dict}
        fsr_dict = {k: sensor_dict[k] for k in EXPECTED_KEYS[70:84] if k in sensor_dict}
        piezo_dict = {k: sensor_dict[k] for k in EXPECTED_KEYS[84:98] if k in sensor_dict}
        thermal_dict = {"thermal": sensor_dict.get("thermal", 0.0)}

        line = format_horizontal(sensor_dict)
        cnt_info = f"Cnt:{tx_counter}" if tx_counter is not None else f"{elapsed_ms:.0f}ms"
        self.get_logger().info(
            f"[RX #{self.packet_count:04d}] [{byte_len}B {total_frags}Paket {cnt_info}] "
            f"[{self.last_rssi}dBm {self.last_snr}dB] "
            f"[{len(sensor_dict)}/{NUM_PARAMS} param] -> {line}"
        )

        def pub(publisher, payload_obj):
            m = String()
            m.data = json.dumps(payload_obj)
            publisher.publish(m)

        pub(self.pub_arduino, arduino_dict)
        pub(self.pub_aurora, aurora_dict)
        pub(self.pub_loadcell, loadcell_dict)
        pub(self.pub_flex, flex_dict)
        pub(self.pub_mcp, mcp_dict)
        pub(self.pub_fsr, fsr_dict)
        pub(self.pub_piezo, piezo_dict)
        pub(self.pub_thermal, thermal_dict)
        pub(self.pub_raw, sensor_dict)
        pub(self.pub_status, {
            "packet_count": self.packet_count,
            "tx_counter": tx_counter,
            "bytes": byte_len,
            "fragments": total_frags,
            "rssi": self.last_rssi,
            "snr": self.last_snr,
            "elapsed_ms": round(elapsed_ms, 1)
        })

    # ========================================================
    # TIMEOUT FRAGMEN MENGGANTUNG (FALLBACK MODE)
    # ========================================================

    def check_frag_timeout(self):
        if self.frag_buffer and (time.monotonic() - self.frag_start_time) > self.frag_timeout:
            self.get_logger().warning(
                f"Fragmen tidak lengkap dalam {self.frag_timeout}s "
                f"({len(self.frag_buffer)}/{self.frag_total}), dibatalkan."
            )
            self.frag_buffer.clear()
            self.frag_total = 0

    # ========================================================
    # SHUTDOWN
    # ========================================================

    def destroy_node(self):
        if self.serial_port is not None and self.serial_port.is_open:
            try:
                self.serial_port.write(b"AT+PRECV=0\r\n")
                time.sleep(0.1)
            except Exception:
                pass
            self.serial_port.close()
        super().destroy_node()


# ============================================================
# MAIN
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = LoRaReceiverNode()

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
