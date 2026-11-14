#!/usr/bin/env python3
"""
Modul bersama untuk node LoRa P2P RAK3272S (Transmitter & Receiver ROS2).

Berisi definisi skema 99 parameter sensor, utilitas CRC16 dan kompresi paket
biner P99, formatter log 1 baris horizontal, parser event serial RAK RUI3,
serta driver serial RAK3272S agar tidak terjadi duplikasi kode antara TX dan RX.
"""

from dataclasses import dataclass
import math
import os
import struct
import time
from typing import Dict, List, Optional, Tuple

import serial


# ============================================================
# KONFIGURASI DEFAULT RADIO LORA P2P
# ============================================================

DEFAULT_BAUDRATE = 115200
DEFAULT_FREQUENCY = 923200000
DEFAULT_SPREADING_FACTOR = 6
DEFAULT_BANDWIDTH = 2
DEFAULT_CODING_RATE = 0
DEFAULT_PREAMBLE = 8
DEFAULT_TX_POWER = 20
MAX_AT_PSEND_BYTES = 250


@dataclass(frozen=True)
class RadioConfig:
    """Parameter konfigurasi modul RAK3272S mode LoRa P2P."""

    port: str
    baudrate: int = DEFAULT_BAUDRATE
    freq: int = DEFAULT_FREQUENCY
    sf: int = DEFAULT_SPREADING_FACTOR
    bw: int = DEFAULT_BANDWIDTH
    cr: int = DEFAULT_CODING_RATE
    preamble: int = DEFAULT_PREAMBLE
    tx_power: int = DEFAULT_TX_POWER

    def p2p_command(self, bw_override: Optional[int] = None) -> str:
        """Membentuk perintah AT+P2P sesuai spesifikasi RAK RUI3."""
        bw = self.bw if bw_override is None else bw_override
        return (
            f"AT+P2P={self.freq}:{self.sf}:{bw}:"
            f"{self.cr}:{self.preamble}:{self.tx_power}"
        )


def declare_radio_config(node, default_port: str) -> RadioConfig:
    """Mendeklarasikan parameter ROS2 radio LoRa dan mengembalikan RadioConfig."""
    node.declare_parameter("port", default_port)
    node.declare_parameter("baudrate", DEFAULT_BAUDRATE)
    node.declare_parameter("freq", DEFAULT_FREQUENCY)
    node.declare_parameter("sf", DEFAULT_SPREADING_FACTOR)
    node.declare_parameter("bw", DEFAULT_BANDWIDTH)
    node.declare_parameter("cr", DEFAULT_CODING_RATE)
    node.declare_parameter("preamble", DEFAULT_PREAMBLE)
    node.declare_parameter("tx_power", DEFAULT_TX_POWER)

    return RadioConfig(
        port=str(node.get_parameter("port").value),
        baudrate=int(node.get_parameter("baudrate").value),
        freq=int(node.get_parameter("freq").value),
        sf=int(node.get_parameter("sf").value),
        bw=int(node.get_parameter("bw").value),
        cr=int(node.get_parameter("cr").value),
        preamble=int(node.get_parameter("preamble").value),
        tx_power=int(node.get_parameter("tx_power").value),
    )


# ============================================================
# SKEMA 99 PARAMETER SENSOR (URUTAN WAJIB SAMA ANTARA TX & RX)
# ============================================================

ARDUINO_KEYS = (
    "lidar_raw",
    "mq_nh3", "mq_co", "mq_co2",
    "smoke",
    "mpu_acc_x_raw", "mpu_acc_y_raw", "mpu_acc_z_raw",
    "mpu_gyro_x_raw", "mpu_gyro_y_raw", "mpu_gyro_z_raw",
    "mpu_acc_x_kalman", "mpu_acc_y_kalman", "mpu_acc_z_kalman",
    "mpu_gyro_x_kalman", "mpu_gyro_y_kalman", "mpu_gyro_z_kalman",
    "adxl_acc_x_raw", "adxl_acc_y_raw", "adxl_acc_z_raw",
    "adxl_acc_x_kalman", "adxl_acc_y_kalman", "adxl_acc_z_kalman",
)

AURORA_KEYS = (
    "accelx", "accely", "accelz",
    "gyrox", "gyroy", "gyroz",
    "bme_temp", "bme_hum", "bme_pres", "bme_alt",
    "sound_dbLeft", "sound_dbRight",
)

LOADCELL_KEYS = (
    "loadcell_paka",
    "loadcell_paki",
    "loadcell_leher",
    "loadcell_kaka",
    "loadcell_kaki",
)

FLEX_KEYS = tuple(f"flex{i}" for i in range(1, 17))
MCP_KEYS = tuple(f"mcp{i}" for i in range(1, 15))
FSR_KEYS = tuple(f"fsr{i}" for i in range(1, 15))
PIEZO_KEYS = tuple(f"piezo{i}" for i in range(1, 15))
THERMAL_KEYS = ("thermal",)

SENSOR_GROUPS: Dict[str, Tuple[str, ...]] = {
    "arduino": ARDUINO_KEYS,
    "aurora": AURORA_KEYS,
    "loadcell": LOADCELL_KEYS,
    "flex": FLEX_KEYS,
    "mcp": MCP_KEYS,
    "fsr": FSR_KEYS,
    "piezo": PIEZO_KEYS,
    "thermal": THERMAL_KEYS,
}

EXPECTED_KEYS: List[str] = [
    key for group_keys in SENSOR_GROUPS.values() for key in group_keys
]
NUM_PARAMS = len(EXPECTED_KEYS)  # 99 parameter

_BASE_DUMMY_VALUES: Dict[str, float] = {
    "lidar_raw": 120.0,
    "mq_nh3": 12.5, "mq_co": 3.2, "mq_co2": 400.0,
    "smoke": 1.0,
    "mpu_acc_x_raw": 0.12, "mpu_acc_y_raw": 0.15, "mpu_acc_z_raw": 9.81,
    "mpu_gyro_x_raw": 0.01, "mpu_gyro_y_raw": 0.02, "mpu_gyro_z_raw": 0.03,
    "mpu_acc_x_kalman": 0.11, "mpu_acc_y_kalman": 0.14, "mpu_acc_z_kalman": 9.79,
    "mpu_gyro_x_kalman": 0.01, "mpu_gyro_y_kalman": 0.02, "mpu_gyro_z_kalman": 0.03,
    "adxl_acc_x_raw": 0.12, "adxl_acc_y_raw": 0.15, "adxl_acc_z_raw": 9.80,
    "adxl_acc_x_kalman": 0.11, "adxl_acc_y_kalman": 0.14, "adxl_acc_z_kalman": 9.78,
    "accelx": 0.25, "accely": 0.31, "accelz": 9.75,
    "gyrox": 0.01, "gyroy": 0.02, "gyroz": 0.03,
    "bme_temp": 28.5, "bme_hum": 65.0, "bme_pres": 1013.25, "bme_alt": 120.0,
    "sound_dbLeft": 43.0, "sound_dbRight": 43.0,
    "loadcell_paka": 12.5, "loadcell_paki": 20.0, "loadcell_leher": 23.5,
    "loadcell_kaka": 25.6, "loadcell_kaki": 25.6,
    "thermal": 25.6,
}

# Kunci flex/mcp/fsr/piezo memakai nilai default simulasi 25.6
DUMMY_VALUES: Dict[str, float] = {
    key: _BASE_DUMMY_VALUES.get(key, 25.6) for key in EXPECTED_KEYS
}

AURORA_ALIASES: Dict[str, str] = {
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
# PROTOKOL KOMPRESI BINER P99 (207 BYTE MULTI-NODE)
# ============================================================

PACKET_MAGIC = b"P99:"
PACKET_SIZE_MULTINODE = 4 + 1 + 2 + (NUM_PARAMS * 2) + 2  # 207 Byte
PACKET_SIZE_LEGACY = 4 + 2 + (NUM_PARAMS * 2) + 2        # 206 Byte
_FLOAT16_MAX = 65504.0


def crc16_ccitt(data: bytes) -> int:
    """Menghitung checksum CRC16-CCITT (polynomial 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def valid_number(value) -> Optional[float]:
    """Mengonversi nilai input menjadi float hingga; mengembalikan None jika tidak valid."""
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (ValueError, TypeError):
        return None
    return number if math.isfinite(number) else None


def to_float16_safe(value) -> float:
    """Membatasi angka agar aman dikemas ke tipe IEEE 754 half-precision float16."""
    num = valid_number(value)
    if num is None:
        return 0.0
    return max(-_FLOAT16_MAX, min(_FLOAT16_MAX, num))


def encode_p99_packet(node_id: int, counter: int, values: List[float]) -> bytes:
    """Mengemas node_id, counter, dan 99 nilai float16 menjadi paket biner 207 Byte."""
    if len(values) != NUM_PARAMS:
        raise ValueError(f"Jumlah nilai harus {NUM_PARAMS}, diterima {len(values)}")

    body = PACKET_MAGIC + struct.pack(
        f"<BH{NUM_PARAMS}e",
        node_id & 0xFF,
        counter % 65536,
        *values,
    )
    crc = crc16_ccitt(body)
    packet = body + struct.pack("<H", crc)

    if len(packet) > MAX_AT_PSEND_BYTES:
        raise ValueError(
            f"Ukuran paket {len(packet)}B melebihi batas RAK {MAX_AT_PSEND_BYTES}B"
        )
    return packet


def decode_p99_packet(raw_bytes: bytes) -> Tuple[int, int, Dict[str, float]]:
    """
    Memvalidasi dan membuka paket biner P99 (207B multi-node atau 206B legacy).

    Mengembalikan tuple (node_id, tx_counter, sensor_dict).
    Melempar ValueError bila panjang paket atau CRC16 tidak valid.
    """
    if len(raw_bytes) not in (PACKET_SIZE_MULTINODE, PACKET_SIZE_LEGACY):
        raise ValueError(
            f"Panjang paket P99 tidak sesuai: {len(raw_bytes)}B "
            f"(seharusnya {PACKET_SIZE_MULTINODE}B)"
        )

    body = raw_bytes[:-2]
    expected_crc = struct.unpack("<H", raw_bytes[-2:])[0]
    actual_crc = crc16_ccitt(body)
    if actual_crc != expected_crc:
        raise ValueError(
            f"CRC mismatch (dihitung {actual_crc:04X}, diterima {expected_crc:04X})"
        )

    if len(raw_bytes) == PACKET_SIZE_MULTINODE:
        node_id, tx_counter, *raw_values = struct.unpack(
            f"<BH{NUM_PARAMS}e", raw_bytes[4:PACKET_SIZE_MULTINODE - 2]
        )
    else:
        node_id = 1
        tx_counter, *raw_values = struct.unpack(
            f"<H{NUM_PARAMS}e", raw_bytes[4:PACKET_SIZE_LEGACY - 2]
        )

    sensor_dict: Dict[str, float] = {}
    for key, val in zip(EXPECTED_KEYS, raw_values):
        fval = float(val)
        sensor_dict[key] = round(fval, 3) if math.isfinite(fval) else 0.0

    return node_id, tx_counter, sensor_dict


def parse_rx_event(line: str) -> Optional[Tuple[str, str, str]]:
    """Mengekstrak (rssi, snr, hex_str) dari baris event +EVT:RXP2P RAK RUI3."""
    if "RXP2P" not in line:
        return None

    idx = line.find("RXP2P")
    after = line[idx + len("RXP2P"):].strip().lstrip(": ")
    tokens = [t.strip() for t in after.split(":") if t.strip()]

    if len(tokens) >= 3:
        rssi, snr = tokens[0], tokens[1]
        hex_str = ":".join(tokens[2:]).strip()
    else:
        parts = line.split()
        if len(parts) < 3:
            return None
        rssi = parts[-3].strip() if len(parts) >= 4 else "-"
        snr = parts[-2].strip() if len(parts) >= 4 else "-"
        hex_str = parts[-1].strip()

    if hex_str.upper() in {"TIMEOUT", "ERROR", "RECEIVE TIMEOUT"}:
        return None
    return rssi, snr, hex_str


def format_horizontal(d: Dict[str, object]) -> str:
    """Memformat seluruh 99 parameter sensor menjadi 1 baris horizontal bersih."""
    def val(k: str) -> object:
        return d.get(k, "-")

    parts = [
        f"Lidar:{val('lidar_raw')}",
        f"MQ(NH3:{val('mq_nh3')},CO:{val('mq_co')},CO2:{val('mq_co2')})",
        f"Smoke:{val('smoke')}",
        (
            f"MPU_Raw[A:({val('mpu_acc_x_raw')},{val('mpu_acc_y_raw')},{val('mpu_acc_z_raw')}) "
            f"G:({val('mpu_gyro_x_raw')},{val('mpu_gyro_y_raw')},{val('mpu_gyro_z_raw')})]"
        ),
        (
            f"MPU_Klm[A:({val('mpu_acc_x_kalman')},{val('mpu_acc_y_kalman')},{val('mpu_acc_z_kalman')}) "
            f"G:({val('mpu_gyro_x_kalman')},{val('mpu_gyro_y_kalman')},{val('mpu_gyro_z_kalman')})]"
        ),
        f"ADXL_Raw:({val('adxl_acc_x_raw')},{val('adxl_acc_y_raw')},{val('adxl_acc_z_raw')})",
        f"ADXL_Klm:({val('adxl_acc_x_kalman')},{val('adxl_acc_y_kalman')},{val('adxl_acc_z_kalman')})",
        (
            f"LSM6DS3[A:({val('accelx')},{val('accely')},{val('accelz')}) "
            f"G:({val('gyrox')},{val('gyroy')},{val('gyroz')})]"
        ),
        f"BME280({val('bme_temp')}C,{val('bme_hum')}%,{val('bme_pres')}hPa,{val('bme_alt')}m)",
        f"Sound(L:{val('sound_dbLeft')},R:{val('sound_dbRight')}dB)",
        f"Loadcell({','.join(str(val(k)) for k in LOADCELL_KEYS)})",
        f"Flex[{','.join(str(val(k)) for k in FLEX_KEYS)}]",
        f"MCP[{','.join(str(val(k)) for k in MCP_KEYS)}]",
        f"FSR[{','.join(str(val(k)) for k in FSR_KEYS)}]",
        f"Piezo[{','.join(str(val(k)) for k in PIEZO_KEYS)}]",
        f"Thermal:{val('thermal')}",
    ]
    return " | ".join(parts)


# ============================================================
# DRIVER SERIAL RAK3272S (DIGUNAKAN OLEH TX & RX)
# ============================================================

class RakP2PRadio:
    """Mengelola koneksi serial, konfigurasi AT, TX, dan pembacaan RX modul RAK3272S."""

    def __init__(
        self,
        config: RadioConfig,
        fallback_ports: Tuple[str, ...],
        logger,
        rx_mode: bool = False,
    ):
        self.config = config
        self.fallback_ports = fallback_ports
        self.logger = logger
        self.rx_mode = rx_mode
        self.serial_port: Optional[serial.Serial] = None
        self.active_port: str = config.port
        self._rx_buffer: str = ""
        self._last_reconnect_try: float = 0.0

    @property
    def is_connected(self) -> bool:
        return self.serial_port is not None and self.serial_port.is_open

    def _select_port(self) -> str:
        candidates = (self.config.port,) + tuple(
            p for p in self.fallback_ports if p != self.config.port
        )
        for candidate in candidates:
            if os.path.exists(candidate):
                return candidate
        return self.config.port

    def connect(self) -> bool:
        """Membuka port serial dan menjalankan urutan konfigurasi LoRa P2P RAK3272S."""
        chosen_port = self._select_port()
        try:
            self.serial_port = serial.Serial(
                port=chosen_port,
                baudrate=self.config.baudrate,
                timeout=0.1,
                write_timeout=1.0,
                rtscts=False,
            )
            self.active_port = chosen_port
            time.sleep(0.5)
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.logger.info(f"RAK connected: {self.active_port}")
            self.configure()
            return True
        except Exception as exc:
            self.logger.error(f"RAK connection failed on {chosen_port}: {exc}")
            self.serial_port = None
            return False

    def reconnect_if_due(self, cooldown: float = 3.0) -> bool:
        """Mencoba menyambung ulang port serial jika jeda cooldown sudah tercapai."""
        now = time.monotonic()
        if now - self._last_reconnect_try < cooldown:
            return False
        self._last_reconnect_try = now
        self.close(stop_rx=False)
        self.logger.warning("Mencoba reconnect ke port RAK3272S...")
        return self.connect()

    def send_at(self, command: str, timeout: float = 2.0) -> bool:
        """Mengirim perintah AT dan menunggu respon OK atau ERROR."""
        if not self.is_connected:
            return False

        try:
            self.serial_port.reset_input_buffer()
            self.serial_port.write((command + "\r\n").encode())
            self.serial_port.flush()

            deadline = time.monotonic() + timeout
            response = ""
            while time.monotonic() < deadline:
                waiting = self.serial_port.in_waiting
                if waiting:
                    response += self.serial_port.read(waiting).decode(errors="ignore")
                    if "ERROR" in response:
                        self.logger.error(f"AT error: {command} | {response.strip()}")
                        return False
                    if "OK" in response:
                        self.logger.info(f"AT OK: {command}")
                        return True
                time.sleep(0.02)

            self.logger.warning(f"AT timeout: {command} | {response.strip()}")
            return False
        except Exception as exc:
            self.logger.error(f"AT command failed ({command}): {exc}")
            return False

    def configure(self) -> None:
        """Mengatur mode kerja P2P (NWM=0), parameter radio AT+P2P, dan mode RX bila aktif."""
        if not self.is_connected:
            return

        role = "Receiver" if self.rx_mode else "Transmitter"
        self.logger.info(f"Configuring RAK LoRa P2P {role}...")

        self.send_at("AT", timeout=1.0)
        self._ensure_p2p_mode()
        self.send_at("AT+PRECV=0", timeout=1.0)

        if not self.send_at(self.config.p2p_command(), timeout=2.0) and self.config.bw == 2:
            # Beberapa versi firmware RUI3 memakai nilai kHz (500) alih-alih indeks 2
            self.send_at(self.config.p2p_command(bw_override=500), timeout=2.0)

        if self.rx_mode:
            if not self.send_at("AT+PRECV=65534", timeout=2.0):
                self.logger.info("Fallback AT+PRECV=65535...")
                self.send_at("AT+PRECV=65535", timeout=2.0)

        self.logger.info(f"RAK {role} configuration sequence completed.")

    def _ensure_p2p_mode(self) -> None:
        self.serial_port.reset_input_buffer()
        self.serial_port.write(b"AT+NWM=?\r\n")
        self.serial_port.flush()
        time.sleep(0.3)
        res_nwm = self.serial_port.read_all().decode(errors="ignore")

        if "=0" not in res_nwm:
            # Perpindahan mode LoRaWAN -> P2P membuat RAK3272S reboot ~1.5-2 detik
            self.logger.info("Mengubah RAK ke mode LoRa P2P (AT+NWM=0)...")
            self.send_at("AT+NWM=0", timeout=2.0)
            time.sleep(2.0)
            self.send_at("AT", timeout=1.0)

    def transmit_hex(self, hex_data: str, timeout: float = 2.0) -> Tuple[bool, str]:
        """Mengirim payload heksadesimal melalui AT+PSEND dan menunggu konfirmasi TX."""
        if not self.is_connected:
            return False, "Serial not connected"

        self.serial_port.reset_input_buffer()
        self.serial_port.write(f"AT+PSEND={hex_data}\r\n".encode())
        self.serial_port.flush()

        deadline = time.monotonic() + timeout
        response = ""
        ok_seen = False

        while time.monotonic() < deadline:
            waiting = self.serial_port.in_waiting
            if waiting:
                response += self.serial_port.read(waiting).decode(errors="ignore")
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

    def read_lines(self) -> List[str]:
        """Membaca data serial yang tersedia dan mengembalikan daftar baris utuh."""
        if not self.is_connected:
            return []

        waiting = self.serial_port.in_waiting
        if waiting <= 0:
            return []

        data = self.serial_port.read(waiting)
        self._rx_buffer += data.decode("ascii", errors="replace")

        lines: List[str] = []
        while "\n" in self._rx_buffer:
            line, self._rx_buffer = self._rx_buffer.split("\n", 1)
            stripped = line.strip()
            if stripped:
                lines.append(stripped)

        if len(self._rx_buffer) > 16384:
            self._rx_buffer = ""

        return lines

    def close(self, stop_rx: bool = True) -> None:
        """Menutup koneksi serial secara aman."""
        if self.serial_port is None:
            return
        if self.serial_port.is_open:
            if stop_rx and self.rx_mode:
                try:
                    self.serial_port.write(b"AT+PRECV=0\r\n")
                    time.sleep(0.1)
                except Exception:
                    pass
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None
