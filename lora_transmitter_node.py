#!/usr/bin/env python3
"""
LoRa P2P Transmitter Node (ROS2) - RAK3272S, 35 parameter sensor, tiap detik

Alur:
  /telemetri/arduino  (JSON string, 23 parameter)  --\\
                                                       >-- envelope JSON (+CRC16)
  /telemetri/aurora   (JSON string, 12 parameter)  --/        |
                                                              v
                                   dipecah jadi fragmen "FRG:i/n:<data>" (maks chunk_size)
                                                              |
                                   dikirim berurutan via AT+PSEND, tiap fragmen
                                   menunggu +EVT:TXP2P DONE (tanpa AT_BUSY_ERROR)

Setiap siklus (interval, default 1 detik) SELURUH parameter dikirim ulang.
Jika sebuah sumber telat (stale), nilai terakhirnya tetap dikirim dengan
available=false dan age_ms, jadi parameter tidak pernah "hilang" dari paket.
"""

import json
import time
from collections import deque

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ============================================================
# 35 PARAMETER YANG DIHARAPKAN (untuk pengecekan kelengkapan)
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
# UTIL
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
    """Seluruh parameter dalam 1 baris horizontal."""
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
# NODE
# ============================================================

class LoRaTransmitterNode(Node):

    def __init__(self):
        super().__init__("lora_transmitter_node")

        # ---- Umum ----
        self.declare_parameter("port", "/dev/ttyAMA3")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("interval", 1.0)        # detik, 1 siklus = semua parameter
        self.declare_parameter("stale_timeout", 3.0)   # detik
        self.declare_parameter("hold_last", True)      # tetap kirim nilai terakhir saat stale

        # ---- Fragmentasi ----
        self.declare_parameter("chunk_size", 185)      # byte data per fragmen
        self.declare_parameter("append_crc", True)     # suffix *XXXX di akhir pesan utuh
        self.declare_parameter("tx_timeout", 1.5)      # tunggu TX DONE per fragmen
        self.declare_parameter("max_payload", 255)

        # ---- Radio P2P (HARUS sama dengan receiver) ----
        self.declare_parameter("configure_radio", True)
        self.declare_parameter("freq", 923200000)
        self.declare_parameter("sf", 6)
        self.declare_parameter("bw", 2)                # 0:125k 1:250k 2:500k
        self.declare_parameter("cr", 0)
        self.declare_parameter("preamble", 8)
        self.declare_parameter("tx_power", 20)

        gp = lambda n: self.get_parameter(n).value
        self.port = gp("port")
        self.baud = int(gp("baudrate"))
        self.interval = float(gp("interval"))
        self.stale_timeout = float(gp("stale_timeout"))
        self.hold_last = bool(gp("hold_last"))
        self.chunk_size = int(gp("chunk_size"))
        self.append_crc = bool(gp("append_crc"))
        self.tx_timeout = float(gp("tx_timeout"))
        self.max_payload = int(gp("max_payload"))

        # ---- State ----
        self.packet_seq = 0
        self.start_time = time.monotonic()
        self.sources = {
            "arduino": {"raw": None, "received_at": None},
            "aurora": {"raw": None, "received_at": None},
        }

        self.ser = None
        self.rx_buffer = ""
        self.last_reconnect_try = 0.0

        # State pengiriman fragmen
        self.tx_queue = deque()
        self.tx_busy = False
        self.tx_started_at = 0.0
        self.next_send_at = 0.0
        self.cur_msg = None   # info pesan yang sedang dikirim

        # ---- Serial & radio ----
        if self.connect_rak(self.port, self.baud):
            if bool(gp("configure_radio")):
                if not self.init_radio():
                    self.get_logger().error(
                        "Inisialisasi radio gagal, lanjut dengan konfigurasi modul saat ini")

        # ---- Subscriptions ----
        self.create_subscription(
            String, "/telemetri/arduino",
            lambda m: self.source_callback("arduino", m), 10)
        self.create_subscription(
            String, "/telemetri/aurora",
            lambda m: self.source_callback("aurora", m), 10)

        # ---- Timers ----
        self.create_timer(0.005, self.read_rak_response)
        self.create_timer(0.005, self.pump_queue)
        self.create_timer(self.interval, self.transmit)

        self.get_logger().info(
            f"LoRa transmitter started (interval={self.interval}s, chunk={self.chunk_size}B)")

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
            self.reset_tx_state()
            self.rx_buffer = ""
            if bool(self.get_parameter("configure_radio").value):
                self.init_radio()

    def reset_tx_state(self):
        self.tx_queue.clear()
        self.tx_busy = False
        self.cur_msg = None

    # ========================================================
    # AT HELPER (blocking, hanya saat init)
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
        log.info("Inisialisasi RAK3272S...")

        if not self.has_ok(self.send_at("AT", 1.0)):
            time.sleep(0.5)
            if not self.has_ok(self.send_at("AT", 1.0)):
                log.error("Modul tidak merespon AT. Cek wiring dan port.")
                return False
        log.info("AT OK, modul terdeteksi")

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

        self.send_at("AT+PRECV=0", 0.5)

        gp = lambda n: int(self.get_parameter(n).value)
        cmd = (f"AT+P2P={gp('freq')}:{gp('sf')}:{gp('bw')}:"
               f"{gp('cr')}:{gp('preamble')}:{gp('tx_power')}")
        log.info(f"Set parameter: {cmd}")
        res = self.send_at(cmd, 1.5)
        if not self.has_ok(res):
            log.error(f"Gagal set parameter P2P: {res}")
            return False

        log.info(f"Status P2P: {' '.join(self.send_at('AT+P2P=?', 1.0))}")
        log.info("RAK3272S siap mengirim")
        return True

    # ========================================================
    # ROS CALLBACK
    # ========================================================

    def source_callback(self, name, msg):
        self.sources[name]["raw"] = msg.data
        self.sources[name]["received_at"] = time.monotonic()

    # ========================================================
    # BUILD PAKET
    # ========================================================

    def build_packet(self):
        """Return (json_str, merged_params) atau (None, None)."""
        now = time.monotonic()
        sources_payload = {}
        merged = {}
        have_any = False

        for name, src in self.sources.items():
            raw, ts = src["raw"], src["received_at"]

            if raw is None or ts is None:
                sources_payload[name] = {"available": False,
                                         "age_ms": None, "data": None}
                continue

            fresh = (now - ts) <= self.stale_timeout
            age_ms = int((now - ts) * 1000)

            if not fresh and not self.hold_last:
                sources_payload[name] = {"available": False,
                                         "age_ms": age_ms, "data": None}
                continue

            have_any = True

            try:
                data = json.loads(raw)
                if isinstance(data, dict):
                    merged.update(data)
            except (ValueError, TypeError):
                data = raw

            sources_payload[name] = {"available": fresh,
                                     "age_ms": age_ms, "data": data}

        if not have_any:
            return None, None

        self.packet_seq += 1
        packet = {
            "version": 2,
            "seq": self.packet_seq,
            "uptime_ms": int((now - self.start_time) * 1000),
            "sources": sources_payload,
        }
        return json.dumps(packet, separators=(",", ":"),
                          ensure_ascii=True), merged

    # ========================================================
    # SIAPKAN FRAGMEN (dipanggil tiap interval)
    # ========================================================

    def transmit(self):
        if self.ser is None or not self.ser.is_open:
            self.try_reconnect()
            return

        if self.tx_queue or self.tx_busy:
            self.get_logger().warning(
                "Siklus sebelumnya belum selesai, siklus ini dilewati. "
                "Naikkan interval atau turunkan sf/naikkan bw.")
            return

        payload, merged = self.build_packet()
        if payload is None:
            self.get_logger().warning("Belum ada telemetry yang tersedia")
            return

        body = payload.encode("utf-8")
        if self.append_crc:
            body += f"*{crc16_ccitt(body):04X}".encode("ascii")

        chunks = [body[i:i + self.chunk_size]
                  for i in range(0, len(body), self.chunk_size)]
        total = len(chunks)

        for idx, chunk in enumerate(chunks, start=1):
            frag = f"FRG:{idx}/{total}:".encode("ascii") + chunk
            if len(frag) > self.max_payload:
                self.get_logger().error(
                    f"Fragmen {len(frag)}B > {self.max_payload}B, "
                    "kecilkan chunk_size")
                self.tx_queue.clear()
                return
            self.tx_queue.append(frag)

        missing = [k for k in EXPECTED_KEYS if k not in merged]
        self.cur_msg = {
            "seq": self.packet_seq,
            "bytes": len(body),
            "frags": total,
            "t_start": time.monotonic(),
            "line": format_horizontal(merged),
            "missing": missing,
            "count": len(EXPECTED_KEYS) - len(missing),
        }

    # ========================================================
    # KIRIM FRAGMEN SATU PER SATU (timer cepat)
    # ========================================================

    def pump_queue(self):
        if self.ser is None or not self.ser.is_open or not self.tx_queue:
            return

        now = time.monotonic()

        if self.tx_busy:
            if now - self.tx_started_at > self.tx_timeout:
                self.abort_message("timeout menunggu TX DONE")
            return

        if now < self.next_send_at:
            return

        frag = self.tx_queue[0]
        cmd = f"AT+PSEND={frag.hex().upper()}\r\n"

        try:
            self.ser.write(cmd.encode("ascii"))
            self.ser.flush()
            self.tx_busy = True
            self.tx_started_at = now
        except (serial.SerialException, serial.SerialTimeoutException,
                OSError) as exc:
            self.get_logger().error(f"LoRa TX error: {exc}")
            self.reset_tx_state()
            self.try_reconnect()

    def finish_message(self):
        m = self.cur_msg
        if m is None:
            return
        ms = (time.monotonic() - m["t_start"]) * 1000
        self.get_logger().info(
            f"[TX #{m['seq']:04d}] [{m['bytes']}B {m['frags']}Frg {ms:.0f}ms] "
            f"[{m['count']}/{len(EXPECTED_KEYS)} param] -> {m['line']}")
        if m["missing"]:
            self.get_logger().warning(
                f"Parameter belum diterima dari ROS: {', '.join(m['missing'])}")
        self.cur_msg = None

    def abort_message(self, reason):
        seq = self.cur_msg["seq"] if self.cur_msg else "?"
        self.get_logger().warning(
            f"[TX #{seq}] GAGAL sebagian fragmen ({reason}), pesan dibatalkan")
        self.reset_tx_state()

    # ========================================================
    # BACA RESPON RAK
    # ========================================================

    def read_rak_response(self):
        if self.ser is None or not self.ser.is_open:
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

                if "+EVT:TXP2P DONE" in line or "+EVT:TX_DONE" in line:
                    if self.tx_busy and self.tx_queue:
                        self.tx_queue.popleft()
                        self.tx_busy = False
                        if not self.tx_queue:
                            self.finish_message()
                elif "AT_BUSY_ERROR" in line:
                    self.tx_busy = False
                    self.next_send_at = time.monotonic() + 0.05
                elif "AT_PARAM_ERROR" in line or "AT_ERROR" in line:
                    self.get_logger().error(f"RAK RX: {line}")
                    self.abort_message(line)
                else:
                    self.get_logger().debug(f"RAK RX: {line}")

            if len(self.rx_buffer) > 4096:
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
    node = LoRaTransmitterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
