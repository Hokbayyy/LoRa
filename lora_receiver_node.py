#!/usr/bin/env python3
"""
LoRa P2P Receiver Node (ROS2) - RAK3272S
Pasangan persis dari lora_transmitter_node terbaru.

Alur:
  1. Radio LoRa P2P menerima event +EVT:RXP2P:<rssi>:<snr>:<hex_data>
  2. Reassemble fragmen "FRG:index/total:<chunk>"
  3. Memotong dan memvalidasi CRC16-CCITT (*XXXX di 5 byte terakhir)
  4. Parse JSON numeric array [val1, val2, ... val35]
  5. Memetakan 35 angka ke EXPECTED_KEYS:
       - 23 parameter pertama -> Arduino UNO Q
       - 12 parameter terakhir -> Cosmic Aurora
  6. Menampilkan log 1 baris horizontal bersih
  7. Mempublikasikan ke topic ROS2:
       - /telemetri/arduino       (JSON 23 parameter Arduino)
       - /telemetri/aurora        (JSON 12 parameter Aurora)
       - /telemetri/received_raw  (JSON seluruh 35 parameter)
       - /telemetri/radio_status  (JSON status RSSI & SNR)
"""

import json
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
# URUTAN 35 PARAMETER (PERSIS SESUAI TRANSMITTER)
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
# FORMAT LOG HORIZONTAL 1 BARIS
# ============================================================

def format_horizontal(d: dict) -> str:
    g = lambda k: d.get(k, "-")
    p = [
        f"Lidar:{g('lidar_raw')}",
        f"MQ(NH3:{g('mq_nh3')},CO:{g('mq_co')},CO2:{g('mq_co2')})",
        f"Smoke:{g('smoke')}",
        f"MPU_Raw[A:({g('mpu_acc_x_raw')},{g('mpu_acc_y_raw')},{g('mpu_acc_z_raw')}) "
        f"G:({g('mpu_gyro_x_raw')},{g('mpu_gyro_y_raw')},{g('mpu_gyro_z_raw')})]",
        f"MPU_Klm[A:({g('mpu_acc_x_kalman')},{g('mpu_acc_y_kalman')},{g('mpu_acc_z_kalman')}) "
        f"G:({g('mpu_gyro_x_kalman')},{g('mpu_gyro_y_kalman')},{g('mpu_gyro_z_kalman')})]",
        f"ADXL_Raw:({g('adxl_acc_x_raw')},{g('adxl_acc_y_raw')},{g('adxl_acc_z_raw')})",
        f"ADXL_Klm:({g('adxl_acc_x_kalman')},{g('adxl_acc_y_kalman')},{g('adxl_acc_z_kalman')})",
        f"LSM6DS3[A:({g('accelx')},{g('accely')},{g('accelz')}) "
        f"G:({g('gyrox')},{g('gyroy')},{g('gyroz')})]",
        f"BME280({g('bme_temp')}C,{g('bme_hum')}%,{g('bme_pres')}hPa,{g('bme_alt')}m)",
        f"Sound(L:{g('sound_dbLeft')},R:{g('sound_dbRight')}dB)",
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
        self.declare_parameter("frag_timeout", 2.5)

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
        self.pub_arduino = self.create_publisher(
            String,
            "/telemetri/arduino",
            10
        )

        self.pub_aurora = self.create_publisher(
            String,
            "/telemetri/aurora",
            10
        )

        self.pub_raw = self.create_publisher(
            String,
            "/telemetri/received_raw",
            10
        )

        self.pub_status = self.create_publisher(
            String,
            "/telemetri/radio_status",
            10
        )

        # ---- Initialize Serial & Radio ----
        self.initialize_serial()

        # ---- Timers ----
        self.create_timer(0.005, self.read_serial_data)
        self.create_timer(0.5, self.check_frag_timeout)

        self.get_logger().info(
            f"LoRa receiver node started on {self.port} @ {self.baud}"
        )

    # ========================================================
    # SERIAL INITIALIZATION
    # ========================================================

    def initialize_serial(self):
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.1,
                write_timeout=1
            )

            time.sleep(1)

            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()

            self.get_logger().info(
                f"RAK connected: {self.port}"
            )

            self.configure_rak()

        except Exception as error:
            self.get_logger().error(
                f"RAK connection failed: {error}"
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
    # CONFIGURE RAK RECEIVER
    # ========================================================

    def configure_rak(self):
        if self.serial_port is None:
            return

        self.get_logger().info("Configuring RAK LoRa P2P Receiver...")

        commands = [
            "AT",
            "AT+NWM=0",
            "AT+PRECV=0",
            (
                f"AT+P2P={self.freq}:"
                f"{self.sf}:"
                f"{self.bw}:"
                f"{self.cr}:"
                f"{self.preamble}:"
                f"{self.tx_power}"
            ),
            "AT+PRECV=65534",
        ]

        for command in commands:
            result = self.send_at(command)
            if not result:
                # Coba fallback 65535 jika 65534 tidak terkonfirmasi
                if "PRECV=65534" in command:
                    self.send_at("AT+PRECV=65535")
                else:
                    self.get_logger().warning(
                        f"Configuration command not confirmed: {command}"
                    )
            time.sleep(0.2)

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

                # Format event RAK P2P: +EVT:RXP2P:<rssi>:<snr>:<hex_data>
                if "+EVT:RXP2P" in line:
                    rssi, snr, hex_str = None, None, None

                    if ":" in line:
                        parts = line.split(":")
                        if len(parts) >= 5:
                            rssi = parts[2].strip()
                            snr = parts[3].strip()
                            hex_str = ":".join(parts[4:]).strip()

                    if not hex_str and " " in line:
                        parts = line.split()
                        if len(parts) >= 4:
                            rssi = parts[-3].strip()
                            snr = parts[-2].strip()
                            hex_str = parts[-1].strip()

                    if hex_str:
                        try:
                            raw_bytes = bytes.fromhex(hex_str)
                            self.handle_lora_fragment(raw_bytes, rssi, snr)
                        except ValueError:
                            self.get_logger().error(f"Gagal konversi hex: {hex_str[:30]}")

            if len(self.rx_buffer) > 8192:
                self.rx_buffer = ""

        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f"Serial read error: {exc}")
            self.try_reconnect()

    # ========================================================
    # REASSEMBLE FRAGMENT & VERIFIKASI CRC
    # ========================================================

    def handle_lora_fragment(self, raw_bytes: bytes, rssi: str, snr: str):
        now = time.monotonic()
        self.last_rssi = rssi or "-"
        self.last_snr = snr or "-"

        # Cek apakah paket berformat fragmen "FRG:index/total:<chunk>"
        if raw_bytes.startswith(b"FRG:"):
            try:
                meta, chunk = raw_bytes[4:].split(b":", 1)
                idx_str, tot_str = meta.decode("ascii").split("/")
                idx, total = int(idx_str), int(tot_str)

                # Jika paket baru (fragmen 1) atau jumlah total berbeda, reset buffer
                if idx == 1 or total != self.frag_total:
                    self.frag_buffer.clear()
                    self.frag_total = total
                    self.frag_start_time = now

                self.frag_buffer[idx] = chunk

                # Cek jika seluruh fragmen lengkap
                if len(self.frag_buffer) == self.frag_total:
                    complete_data = b"".join(
                        self.frag_buffer[i] for i in range(1, self.frag_total + 1)
                    )
                    elapsed_ms = (now - self.frag_start_time) * 1000
                    total_frags = self.frag_total

                    self.frag_buffer.clear()
                    self.frag_total = 0

                    self.process_packet_payload(complete_data, total_frags, elapsed_ms)

            except Exception as err:
                self.get_logger().error(f"Gagal memproses fragmen: {err}")

        # Jika paket datang utuh tanpa awalan FRG:
        else:
            self.process_packet_payload(raw_bytes, total_frags=1, elapsed_ms=0.0)

    # ========================================================
    # PROSES PAYLOAD LENGKAP
    # ========================================================

    def process_packet_payload(self, complete_data: bytes, total_frags: int, elapsed_ms: float):
        self.packet_count += 1

        # 1. Verifikasi dan buang 5 byte terakhir (*XXXX) CRC16
        if b"*" in complete_data:
            json_bytes, crc_bytes = complete_data.rsplit(b"*", 1)
            try:
                expected_crc = int(crc_bytes.decode("ascii"), 16)
                actual_crc = crc16_ccitt(json_bytes)

                if actual_crc != expected_crc:
                    self.get_logger().warning(
                        f"[RX #{self.packet_count}] CRC MISMATCH! "
                        f"(Dihitung: {actual_crc:04X}, Diterima: {expected_crc:04X}). Paket dibuang."
                    )
                    return

            except Exception as e:
                self.get_logger().warning(f"Format CRC tidak valid ({e}), memproses JSON mentah.")
                json_bytes = complete_data
        else:
            json_bytes = complete_data

        # 2. Parse JSON numeric array [val1, val2, ... val35]
        try:
            values = json.loads(json_bytes.decode("utf-8"))
        except Exception as err:
            self.get_logger().warning(f"Gagal parse JSON ({err}): {json_bytes[:60]}")
            return

        if not isinstance(values, list):
            self.get_logger().warning(f"Format payload bukan list angka: {type(values)}")
            return

        # 3. Petakan angka ke EXPECTED_KEYS (35 parameter)
        sensor_dict = {}
        for key, val in zip(EXPECTED_KEYS, values):
            sensor_dict[key] = val

        # Pisahkan menjadi data Arduino (23 parameter) dan Aurora (12 parameter)
        arduino_dict = {k: sensor_dict[k] for k in EXPECTED_KEYS[:23] if k in sensor_dict}
        aurora_dict = {k: sensor_dict[k] for k in EXPECTED_KEYS[23:] if k in sensor_dict}

        # 4. Tampilkan 1 baris horizontal bersih di terminal
        line = format_horizontal(sensor_dict)
        self.get_logger().info(
            f"[RX #{self.packet_count:04d}] [{len(complete_data)}B {total_frags}Frg {elapsed_ms:.0f}ms] "
            f"[{self.last_rssi}dBm {self.last_snr}dB] "
            f"[{len(sensor_dict)}/{len(EXPECTED_KEYS)} param] -> {line}"
        )

        # 5. Publikasikan ke Topic ROS2
        # A. Arduino UNO Q
        msg_ard = String()
        msg_ard.data = json.dumps(arduino_dict)
        self.pub_arduino.publish(msg_ard)

        # B. Cosmic Aurora
        msg_aur = String()
        msg_aur.data = json.dumps(aurora_dict)
        self.pub_aurora.publish(msg_aur)

        # C. Raw Whole 35 Parameters
        msg_raw = String()
        msg_raw.data = json.dumps(sensor_dict)
        self.pub_raw.publish(msg_raw)

        # D. Radio Status
        msg_stat = String()
        msg_stat.data = json.dumps({
            "packet_count": self.packet_count,
            "bytes": len(complete_data),
            "fragments": total_frags,
            "rssi": self.last_rssi,
            "snr": self.last_snr,
            "elapsed_ms": round(elapsed_ms, 1)
        })
        self.pub_status.publish(msg_stat)

    # ========================================================
    # TIMEOUT FRAGMEN MENGGANTUNG
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
