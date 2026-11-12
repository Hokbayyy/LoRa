#!/usr/bin/env python3
"""
LoRa P2P Receiver Node (ROS2) - RAK3272S
Pasangan persis dari lora_transmitter_node terbaru (99 parameter sensor).

Alur:
  1. Radio LoRa P2P menerima event +EVT:RXP2P:<rssi>:<snr>:<hex_data>
  2. Reassemble fragmen "FRG:index/total:<chunk>" (termasuk single fragment "FRG:1/1:<chunk>")
  3. Memotong dan memvalidasi CRC16-CCITT (*XXXX di akhir payload)
  4. Parse JSON numeric array [val1, val2, ... val99]
  5. Memetakan 99 angka ke EXPECTED_KEYS:
       - 23 parameter pertama  -> Arduino UNO Q
       - 12 parameter          -> Cosmic Aurora
       - 5 parameter           -> Loadcell
       - 16 parameter          -> Sensor Flex (flex1 - flex16)
       - 14 parameter          -> Sensor MCP (mcp1 - mcp14)
       - 14 parameter          -> Sensor FSR (fsr1 - fsr14)
       - 14 parameter          -> Sensor Piezo (piezo1 - piezo14)
       - 1 parameter           -> Sensor Thermal
  6. Menampilkan log 1 baris horizontal bersih tanpa jeda baris/emoji
  7. Mempublikasikan ke topic ROS2:
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
import os
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
            f"LoRa receiver node started on {self.port} @ {self.baud} (Expecting 99 parameters)"
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

        # 1. Pastikan komunikasi AT dasar
        self.send_at("AT", timeout=1.0)

        # 2. Pastikan mode kerja P2P (AT+NWM=0)
        self.serial_port.reset_input_buffer()
        self.serial_port.write(b"AT+NWM=?\r\n")
        self.serial_port.flush()
        time.sleep(0.3)
        res_nwm = self.serial_port.read_all().decode(errors="ignore")

        if "=0" not in res_nwm:
            self.get_logger().info("Mengubah RAK ke mode LoRa P2P (AT+NWM=0)...")
            self.send_at("AT+NWM=0", timeout=2.0)
            time.sleep(1.5)
            self.send_at("AT", timeout=1.0)

        # 3. Matikan receive sementara agar setting P2P tidak bentrok
        self.send_at("AT+PRECV=0", timeout=1.0)

        # 4. Set parameter radio P2P
        p2p_cmd = (
            f"AT+P2P={self.freq}:"
            f"{self.sf}:"
            f"{self.bw}:"
            f"{self.cr}:"
            f"{self.preamble}:"
            f"{self.tx_power}"
        )
        self.send_at(p2p_cmd, timeout=2.0)

        # 5. Aktifkan continuous RX (65534 / fallback 65535)
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

                # Format event RAK P2P: +EVT:RXP2P:<rssi>:<snr>:<hex_data>
                if "+EVT:RXP2P" in line or "RXP2P" in line:
                    rssi, snr, hex_str = self.parse_rx_line(line)
                    if hex_str:
                        try:
                            raw_bytes = bytes.fromhex(hex_str)
                            self.handle_lora_fragment(raw_bytes, rssi, snr)
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

        # Fallback pemisahan spasi
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

                # Cek jika seluruh fragmen lengkap (termasuk kasus total == 1)
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

        # 2. Parse JSON numeric array [val1, val2, ... val99]
        try:
            values = json.loads(json_bytes.decode("utf-8"))
        except Exception as err:
            self.get_logger().warning(f"Gagal parse JSON ({err}): {json_bytes[:60]}")
            return

        if not isinstance(values, list):
            self.get_logger().warning(f"Format payload bukan list angka: {type(values)}")
            return

        # 3. Petakan angka ke EXPECTED_KEYS (99 parameter)
        sensor_dict = {}
        for key, val in zip(EXPECTED_KEYS, values):
            sensor_dict[key] = val

        # Pisahkan menjadi sub-kelompok parameter
        arduino_dict = {k: sensor_dict[k] for k in EXPECTED_KEYS[:23] if k in sensor_dict}
        aurora_dict = {k: sensor_dict[k] for k in EXPECTED_KEYS[23:35] if k in sensor_dict}
        loadcell_dict = {k: sensor_dict[k] for k in EXPECTED_KEYS[35:40] if k in sensor_dict}
        flex_dict = {k: sensor_dict[k] for k in EXPECTED_KEYS[40:56] if k in sensor_dict}
        mcp_dict = {k: sensor_dict[k] for k in EXPECTED_KEYS[56:70] if k in sensor_dict}
        fsr_dict = {k: sensor_dict[k] for k in EXPECTED_KEYS[70:84] if k in sensor_dict}
        piezo_dict = {k: sensor_dict[k] for k in EXPECTED_KEYS[84:98] if k in sensor_dict}
        thermal_dict = {"thermal": sensor_dict.get("thermal", 0.0)}

        # 4. Tampilkan 1 baris horizontal bersih di terminal
        line = format_horizontal(sensor_dict)
        self.get_logger().info(
            f"[RX #{self.packet_count:04d}] [{len(complete_data)}B {total_frags}Frg {elapsed_ms:.0f}ms] "
            f"[{self.last_rssi}dBm {self.last_snr}dB] "
            f"[{len(sensor_dict)}/{len(EXPECTED_KEYS)} param] -> {line}"
        )

        # 5. Publikasikan ke Topic ROS2
        # A. Arduino UNO Q (23 params)
        msg_ard = String()
        msg_ard.data = json.dumps(arduino_dict)
        self.pub_arduino.publish(msg_ard)

        # B. Cosmic Aurora (12 params)
        msg_aur = String()
        msg_aur.data = json.dumps(aurora_dict)
        self.pub_aurora.publish(msg_aur)

        # C. Loadcells (5 params)
        msg_lc = String()
        msg_lc.data = json.dumps(loadcell_dict)
        self.pub_loadcell.publish(msg_lc)

        # D. Flex Sensors (16 params)
        msg_flx = String()
        msg_flx.data = json.dumps(flex_dict)
        self.pub_flex.publish(msg_flx)

        # E. MCP Sensors (14 params)
        msg_mcp = String()
        msg_mcp.data = json.dumps(mcp_dict)
        self.pub_mcp.publish(msg_mcp)

        # F. FSR Sensors (14 params)
        msg_fsr = String()
        msg_fsr.data = json.dumps(fsr_dict)
        self.pub_fsr.publish(msg_fsr)

        # G. Piezo Sensors (14 params)
        msg_pzo = String()
        msg_pzo.data = json.dumps(piezo_dict)
        self.pub_piezo.publish(msg_pzo)

        # H. Thermal Sensor (1 param)
        msg_thm = String()
        msg_thm.data = json.dumps(thermal_dict)
        self.pub_thermal.publish(msg_thm)

        # I. Raw Whole 99 Parameters
        msg_raw = String()
        msg_raw.data = json.dumps(sensor_dict)
        self.pub_raw.publish(msg_raw)

        # J. Radio Status
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
