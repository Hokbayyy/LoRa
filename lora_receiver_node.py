#!/usr/bin/env python3
"""
LoRa P2P Receiver Node (ROS2) - RAK3272S, Reassembly Fragmen, CRC16

Node ini adalah pasangan persis (receiver) dari lora_transmitter_node.
Tidak ada protokol yang diubah. Seluruh parameter radio, format fragmen,
checksum CRC16 CCITT, dan struktur envelope JSON sama persis.

Alur:
  Radio LoRa P2P (+EVT:RXP2P)
           |
           v
  Reassemble fragmen "FRG:i/n:<chunk>"
           |
           v
  Verifikasi Checksum CRC16 (*XXXX)
           |
           v
  Parse JSON envelope & ekstrak data:
    - /telemetri/arduino  (JSON 23 parameter)
    - /telemetri/aurora   (JSON 12 parameter)
    - /telemetri/received_raw
    - /telemetri/radio_status
"""

import json
import time

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ============================================================
# 35 PARAMETER YANG DIHARAPKAN (PERSIS TRANSMITTER)
# ============================================================

EXPECTED_KEYS = [
    # Arduino Uno Q (23)
    "lidar_raw",
    "mq_nh3", "mq_co", "mq_co2",
    "smoke",
    "mpu_acc_x_raw", "mpu_acc_y_raw", "mpu_acc_z_raw",
    "mpu_gyro_x_raw", "mpu_gyro_y_raw", "mpu_gyro_z_raw",
    "mpu_acc_x_kalman", "mpu_acc_y_kalman", "mpu_acc_z_kalman",
    "mpu_gyro_x_kalman", "mpu_gyro_y_kalman", "mpu_gyro_z_kalman",
    "adxl_acc_x_raw", "adxl_acc_y_raw", "adxl_acc_z_raw",
    "adxl_acc_x_kalman", "adxl_acc_y_kalman", "adxl_acc_z_kalman",
    # Cosmic LoRa Aurora V3 (12)
    "accelx", "accely", "accelz", "gyrox", "gyroy", "gyroz",
    "bme_temp", "bme_hum", "bme_pres", "bme_alt",
    "sound_dbLeft", "sound_dbRight",
]


# ============================================================
# UTIL (PERSIS TRANSMITTER)
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


def format_horizontal(d: dict) -> str:
    """Seluruh parameter dalam 1 baris horizontal persis transmitter."""
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
# NODE RECEIVER
# ============================================================

class LoRaReceiverNode(Node):

    def __init__(self):
        super().__init__("lora_receiver_node")

        # ---- Umum ----
        self.declare_parameter("port", "/dev/ttyAMA0")
        self.declare_parameter("baudrate", 115200)

        # ---- Radio P2P (PERSIS SESUAI TRANSMITTER) ----
        self.declare_parameter("configure_radio", True)
        self.declare_parameter("freq", 923200000)
        self.declare_parameter("sf", 6)
        self.declare_parameter("bw", 2)                # 0:125k, 1:250k, 2:500k
        self.declare_parameter("cr", 0)
        self.declare_parameter("preamble", 8)
        self.declare_parameter("tx_power", 20)

        # ---- Verifikasi & Timeout ----
        self.declare_parameter("verify_crc", True)
        self.declare_parameter("frag_timeout", 2.5)    # timeout buang fragmen menggantung

        gp = lambda n: self.get_parameter(n).value
        self.port = gp("port")
        self.baud = int(gp("baudrate"))
        self.verify_crc = bool(gp("verify_crc"))
        self.frag_timeout = float(gp("frag_timeout"))

        # ---- State Serial & Reassembly ----
        self.ser = None
        self.rx_buffer = ""
        self.last_reconnect_try = 0.0

        self.packet_count = 0
        self.frag_buffer = {}
        self.frag_total = 0
        self.frag_start_time = 0.0
        self.last_rssi = "-"
        self.last_snr = "-"

        # ---- Publishers (Mengeluarkan kembali data yang dikirim TX) ----
        self.pub_arduino = self.create_publisher(String, "/telemetri/arduino", 10)
        self.pub_aurora = self.create_publisher(String, "/telemetri/aurora", 10)
        self.pub_raw = self.create_publisher(String, "/telemetri/received_raw", 10)
        self.pub_status = self.create_publisher(String, "/telemetri/radio_status", 10)

        # ---- Inisialisasi Koneksi Serial & Radio ----
        if self.connect_rak(self.port, self.baud):
            if bool(gp("configure_radio")):
                if not self.init_radio():
                    self.get_logger().error(
                        "Inisialisasi radio gagal, lanjut dengan konfigurasi modul saat ini")

        # ---- Timers ----
        self.create_timer(0.005, self.read_rak_data)
        self.create_timer(0.5, self.check_frag_timeout)

        self.get_logger().info(
            f"LoRa receiver started (port={self.port}, baud={self.baud})")

    # ========================================================
    # SERIAL
    # ========================================================

    def connect_rak(self, port, baud) -> bool:
        try:
            self.ser = serial.Serial(port=port, baudrate=baud,
                                     timeout=0, write_timeout=0.2)
            self.get_logger().info(f"RAK connected: {port} @ {baud}")
            return True
        except (serial.SerialException, OSError) as exc:
            self.ser = None
            self.get_logger().error(f"RAK connection failed: {exc}")
            return False

    def try_reconnect(self):
        now = time.monotonic()
        if now - self.last_reconnect_try < 3.0:
            return
        self.last_reconnect_try = now

        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

        self.get_logger().warning("Mencoba reconnect ke RAK...")
        if self.connect_rak(self.port, self.baud):
            self.rx_buffer = ""
            self.frag_buffer.clear()
            self.frag_total = 0
            if bool(self.get_parameter("configure_radio").value):
                self.init_radio()

    # ========================================================
    # AT HELPER (hanya saat init)
    # ========================================================

    def send_at(self, cmd, timeout=1.5):
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\r\n").encode("ascii"))
        self.ser.flush()

        start = time.monotonic()
        responses, buf = [], ""
        while time.monotonic() - start < timeout:
            n = self.ser.in_waiting
            if n > 0:
                buf += self.ser.read(n).decode("ascii", errors="ignore")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    responses.append(line)
                    if ("OK" in line or "AT_ERROR" in line
                            or "AT_PARAM_ERROR" in line):
                        return responses
            time.sleep(0.02)
        return responses

    @staticmethod
    def has_ok(lines):
        return any("OK" in l for l in lines)

    def init_radio(self) -> bool:
        log = self.get_logger()
        log.info("Inisialisasi RAK3272S Receiver...")

        if not self.has_ok(self.send_at("AT", 1.0)):
            time.sleep(0.5)
            if not self.has_ok(self.send_at("AT", 1.0)):
                log.error("Modul tidak merespon AT. Cek wiring dan port.")
                return False
        log.info("AT OK, modul terdeteksi")

        # Pastikan mode P2P (AT+NWM=0)
        res = self.send_at("AT+NWM=?", 1.0)
        mode = None
        for line in res:
            if "AT+NWM=" in line:
                mode = line.split("=")[-1].strip()

        if mode != "0":
            log.warning("Modul bukan mode P2P, mengubah ke AT+NWM=0...")
            self.ser.write(b"AT+NWM=0\r\n")
            self.ser.flush()
            time.sleep(2.5)
            self.ser.reset_input_buffer()
            if not self.has_ok(self.send_at("AT", 1.0)):
                log.error("Gagal reboot ke mode P2P")
                return False

        # Matikan receive sebelumnya agar bisa set parameter
        self.send_at("AT+PRECV=0", 0.5)

        gp = lambda n: int(self.get_parameter(n).value)
        cmd = (f"AT+P2P={gp('freq')}:{gp('sf')}:{gp('bw')}:"
               f"{gp('cr')}:{gp('preamble')}:{gp('tx_power')}")
        log.info(f"Set parameter: {cmd}")
        res = self.send_at(cmd, 1.5)
        if not self.has_ok(res):
            log.error(f"Gagal set parameter P2P: {res}")
            return False

        # Aktifkan Continuous Receive mode (65534)
        res_recv = self.send_at("AT+PRECV=65534", 1.5)
        if not self.has_ok(res_recv):
            res_recv = self.send_at("AT+PRECV=65535", 1.5)
            if not self.has_ok(res_recv):
                log.warning(f"Respon PRECV: {res_recv}")

        log.info(f"Status P2P: {' '.join(self.send_at('AT+P2P=?', 1.0))}")
        log.info("RAK3272S siap menerima (Continuous RX aktif)")
        return True

    # ========================================================
    # PEMROSESAN FRAGMEN & CRC
    # ========================================================

    def handle_raw_payload(self, raw_bytes: bytes, rssi: str, snr: str):
        now = time.monotonic()
        self.last_rssi = rssi
        self.last_snr = snr

        # Cek apakah fragmen "FRG:i/total:<chunk>"
        if raw_bytes.startswith(b"FRG:"):
            try:
                meta, chunk = raw_bytes[4:].split(b":", 1)
                idx_str, tot_str = meta.decode("ascii").split("/")
                idx, tot = int(idx_str), int(tot_str)

                # Jika paket baru (fragmen 1) atau jumlah total berbeda, reset buffer
                if idx == 1 or tot != self.frag_total:
                    self.frag_buffer.clear()
                    self.frag_total = tot
                    self.frag_start_time = now

                self.frag_buffer[idx] = chunk

                # Cek jika seluruh fragmen sudah lengkap
                if len(self.frag_buffer) == self.frag_total:
                    full_payload = b"".join(
                        self.frag_buffer[i] for i in range(1, self.frag_total + 1)
                    )
                    elapsed_ms = (now - self.frag_start_time) * 1000
                    frags_count = self.frag_total

                    self.frag_buffer.clear()
                    self.frag_total = 0

                    self.process_completed_message(full_payload, frags_count, elapsed_ms)

            except Exception as err:
                self.get_logger().error(f"Gagal memproses fragmen: {err}")

        # Jika paket tunggal utuh tanpa FRG
        else:
            self.process_completed_message(raw_bytes, frags_count=1, elapsed_ms=0.0)

    def process_completed_message(self, body: bytes, frags_count: int, elapsed_ms: float):
        self.packet_count += 1
        raw_text = None

        # 1. Verifikasi Checksum CRC16 CCITT (*XXXX di akhir payload)
        if self.verify_crc and b"*" in body:
            data_part, crc_part = body.rsplit(b"*", 1)
            try:
                expected_crc = int(crc_part.decode("ascii"), 16)
                actual_crc = crc16_ccitt(data_part)
                if actual_crc != expected_crc:
                    self.get_logger().warning(
                        f"[RX #{self.packet_count}] CRC MISMATCH (calc: {actual_crc:04X}, rx: {expected_crc:04X}), paket diabaikan")
                    return
                raw_text = data_part.decode("utf-8", errors="replace")
            except Exception as e:
                self.get_logger().warning(f"Format CRC tidak valid: {e}")
                raw_text = body.decode("utf-8", errors="replace")
        else:
            raw_text = body.decode("utf-8", errors="replace")

        # 2. Parse JSON Envelope (Sesuai packet transmitter)
        try:
            packet = json.loads(raw_text)
        except json.JSONDecodeError as err:
            self.get_logger().warning(f"Payload bukan JSON valid ({err}): {raw_text[:60]}")
            return

        seq = packet.get("seq", self.packet_count)
        sources = packet.get("sources", {})

        merged = {}
        arduino_raw = None
        aurora_raw = None

        # Ekstrak sumber data arduino
        if "arduino" in sources and sources["arduino"].get("data") is not None:
            ard_data = sources["arduino"]["data"]
            if isinstance(ard_data, dict):
                merged.update(ard_data)
                arduino_raw = json.dumps(ard_data)
            else:
                arduino_raw = str(ard_data)

        # Ekstrak sumber data aurora
        if "aurora" in sources and sources["aurora"].get("data") is not None:
            aur_data = sources["aurora"]["data"]
            if isinstance(aur_data, dict):
                merged.update(aur_data)
                aurora_raw = json.dumps(aur_data)
            else:
                aurora_raw = str(aur_data)

        # 3. Log 1 Baris Horizontal (Persis Format Transmitter)
        line = format_horizontal(merged)
        missing = [k for k in EXPECTED_KEYS if k not in merged]
        count = len(EXPECTED_KEYS) - len(missing)

        self.get_logger().info(
            f"[RX #{seq:04d}] [{len(body)}B {frags_count}Frg {elapsed_ms:.0f}ms] "
            f"[{self.last_rssi}dBm {self.last_snr}dB] "
            f"[{count}/{len(EXPECTED_KEYS)} param] -> {line}"
        )

        # 4. Publikasikan ke Topic ROS
        if arduino_raw is not None:
            m = String()
            m.data = arduino_raw
            self.pub_arduino.publish(m)

        if aurora_raw is not None:
            m = String()
            m.data = aurora_raw
            self.pub_aurora.publish(m)

        msg_raw = String()
        msg_raw.data = raw_text
        self.pub_raw.publish(msg_raw)

        msg_stat = String()
        msg_stat.data = json.dumps({
            "seq": seq,
            "bytes": len(body),
            "frags": frags_count,
            "rssi": self.last_rssi,
            "snr": self.last_snr,
            "elapsed_ms": round(elapsed_ms, 1)
        })
        self.pub_status.publish(msg_stat)

    def check_frag_timeout(self):
        """Buang fragmen yang tidak lengkap melebihi batas waktu."""
        if self.frag_buffer and (time.monotonic() - self.frag_start_time) > self.frag_timeout:
            self.get_logger().warning(
                f"Fragmen tidak lengkap dalam {self.frag_timeout}s ({len(self.frag_buffer)}/{self.frag_total}), dibatalkan")
            self.frag_buffer.clear()
            self.frag_total = 0

    # ========================================================
    # BACA EVENT DARI SERIAL RAK3272S
    # ========================================================

    def read_rak_data(self):
        if self.ser is None or not self.ser.is_open:
            self.try_reconnect()
            return

        try:
            n = self.ser.in_waiting
            if n <= 0:
                return

            self.rx_buffer += self.ser.read(n).decode("ascii", errors="replace")

            while "\n" in self.rx_buffer:
                line, self.rx_buffer = self.rx_buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue

                # Tangkap event penerimaan paket LoRa RUI3: +EVT:RXP2P:<rssi>:<snr>:<hex>
                if "+EVT:RXP2P" in line:
                    rssi, snr, hex_str = None, None, None

                    # Format dengan pemisah titik dua (standard RUI3)
                    if ":" in line:
                        parts = line.split(":")
                        if len(parts) >= 5:
                            rssi = parts[2].strip()
                            snr = parts[3].strip()
                            hex_str = ":".join(parts[4:]).strip()

                    # Format dengan pemisah spasi
                    if not hex_str and " " in line:
                        parts = line.split()
                        if len(parts) >= 4:
                            rssi = parts[-3].strip()
                            snr = parts[-2].strip()
                            hex_str = parts[-1].strip()

                    if hex_str:
                        try:
                            raw_bytes = bytes.fromhex(hex_str)
                            self.handle_raw_payload(raw_bytes, rssi, snr)
                        except ValueError:
                            self.get_logger().error(f"Gagal konversi hex: {hex_str[:30]}")

            if len(self.rx_buffer) > 8192:
                self.rx_buffer = ""

        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f"RAK read error: {exc}")
            self.try_reconnect()

    # ========================================================
    # CLEANUP
    # ========================================================

    def destroy_node(self):
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.write(b"AT+PRECV=0\r\n")
                time.sleep(0.1)
            except Exception:
                pass
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoRaReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
