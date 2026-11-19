#!/usr/bin/env python3
"""
LoRa P2P Receiver Node (ROS2) - RAK3272S + Direct Web Forwarder + Local Communication Logger
(1 File Standalone)

Fitur:
  - Mandiri 1 file utuh tanpa perlu file modul tambahan (cukup jalankan lora_rxManekin1.py).
  - Sinkron dengan parameter radio lora_txManekin1.py (923.2 MHz, SF6, BW 500 kHz, CR 4/5, Preamble 8).
  - Menerima frame FRG:1/1:...*CRC16 maupun multi-fragmen FRG:i/N:...*CRC16 + validasi CRC16-CCITT.
  - Mendekode 3 jenis payload dari lora_txManekin1.py:
      1. Array JSON 109 parameter sensor (payload_mode == "array", mendukung nilai null)
      2. Web Compact V2 JSON (payload_mode == "web_v2", readingId 1001..1095 / 2001..2095)
      3. Alert kompak 12-byte ASCII ("A1" + seq + severity + event + status + offline_mask)
  - MENYIMPAN DATA KOMUNIKASI LORA & TELEMETRI SECARA LOKAL DI RASPBERRY PI:
      Folder default: ~/blackbox2_ws/logs/communication/ (parameter ROS2 `log_dir`):
      1. lora_rx_communication.csv : RSSI (dBm), SNR (dB), ukuran byte, fragmen, waktu reassembly (ms),
                                     interval antar-paket / jitter (ms), packet rate (Hz), status CRC16,
                                     total CRC error, frekuensi, SF, BW, CR, jumlah sensor valid, & status ACK Web.
      2. lora_rx_telemetry.csv     : Timestamp, RSSI, SNR, ukuran byte + seluruh 109 kolom nilai sensor.
      3. lora_rx_alerts.csv        : Timestamp, RSSI, SNR + detail alert kompak 12-byte yang diterima.
  - LANGSUNG MENGIRIM KE WEB BACKEND (Be-smart-manekin) pada background thread non-blocking:
      1. Jalur Utama MQTT Compact V2:
         Publish ke topic `smart-mannequin/{device_id}/telemetry` (QoS 1, retain=False)
         & subscribe ACK `smart-mannequin/{device_id}/ack`.
      2. Jalur Opsional HTTP REST Ingest (`urllib.request`):
         POST ke `/sensor-reading/batch` & `/sensor/*` dengan header `x-ingest-key`.
  - Mempublikasikan ke topic ROS2:
      /telemetri/web/v2, /system/alerts, /telemetri/received_raw, /telemetri/radio_status,
      serta /telemetri/{arduino,aurora,loadcell,flex,mcp,fsr,piezo,thermal,witmotion}.
"""

from collections import deque
import csv
from dataclasses import dataclass
from datetime import datetime
import json
import math
import os
import socket
import struct
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple
import urllib.error
import urllib.request

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import paho.mqtt.client as paho_mqtt  # type: ignore
except ImportError:
    paho_mqtt = None


# ============================================================
# KONFIGURASI RADIO RAK3272S (IDENTIK lora_txManekin1.py)
# ============================================================

DEFAULT_SERIAL_PORT = "/dev/ttyAMA0"
FALLBACK_PORTS = ("/dev/ttyAMA0", "/dev/serial0", "/dev/ttyAMA3", "/dev/ttyUSB0")

DEFAULT_BAUDRATE = 115200
DEFAULT_FREQUENCY = 923200000
DEFAULT_SPREADING_FACTOR = 6
DEFAULT_BANDWIDTH = 2
DEFAULT_CODING_RATE = 0
DEFAULT_PREAMBLE = 8
DEFAULT_TX_POWER = 20

# ============================================================
# KONFIGURASI LOG LOKAL KOMUNIKASI LORA DI RASPBERRY PI
# ============================================================

DEFAULT_LOG_ENABLED = True
DEFAULT_LOG_DIR = os.path.expanduser("~/blackbox2_ws/logs/communication")

# ============================================================
# KONFIGURASI DEFAULT PENGIRIMAN KE WEB (Be-smart-manekin)
# ============================================================

DEFAULT_MQTT_ENABLED = True
DEFAULT_MQTT_HOST = os.environ.get("WEB_MQTT_HOST", "127.0.0.1")
DEFAULT_MQTT_PORT = int(os.environ.get("WEB_MQTT_PORT", "1883"))
DEFAULT_MQTT_USERNAME = os.environ.get("WEB_MQTT_USER", "")
DEFAULT_MQTT_PASSWORD = os.environ.get("WEB_MQTT_PASS", "")

DEFAULT_DEVICE_ID_M1 = "AD8121DAD12451E2"
DEFAULT_DEVICE_ID_M2 = "AD8121DAD12451E1"

DEFAULT_HTTP_ENABLED = bool(os.environ.get("WEB_HTTP_URL", ""))
DEFAULT_HTTP_BASE_URL = os.environ.get("WEB_HTTP_URL", "http://127.0.0.1:4013")
DEFAULT_INGEST_API_KEY = os.environ.get("WEB_INGEST_KEY", "")


# ============================================================
# SKEMA 109 PARAMETER SENSOR (URUTAN IDENTIK lora_txManekin1.py)
# ============================================================

SENSOR_GROUPS: Dict[str, Tuple[str, ...]] = {
    "arduino": (
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
    ),
    "aurora": (
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
    ),
    "loadcell": (
        "loadcell_paka",
        "loadcell_paki",
        "loadcell_leher",
        "loadcell_kaka",
        "loadcell_kaki",
    ),
    "flex": tuple(f"flex{i}" for i in range(1, 17)),
    "mcp": tuple(f"mcp{i}" for i in range(1, 15)),
    "fsr": tuple(f"fsr{i}" for i in range(1, 15)),
    "piezo": tuple(f"piezo{i}" for i in range(1, 15)),
    "thermal": ("thermal",),
    "witmotion": (
        "ax",
        "ay",
        "az",
        "gx",
        "gy",
        "gz",
        "roll",
        "pitch",
        "yaw",
    ),
}

EXPECTED_KEYS: List[str] = [
    key for group_keys in SENSOR_GROUPS.values() for key in group_keys
]
NUM_PARAMS = len(EXPECTED_KEYS)  # 109 field


# ============================================================
# MAPPING WEB COMPACT V2 & POSISI ANATOMIS SMARTSKIN
# ============================================================

WEB_READING_ID_MAP_M1: Dict[str, int] = {
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
    **{f"piezo{i}": 1036 + i for i in range(1, 15)},
    **{f"fsr{i}": 1050 + i for i in range(1, 15)},
    **{f"mcp{i}": 1064 + i for i in range(1, 15)},
    **{f"flex{i}": 1078 + i for i in range(1, 17)},
    "thermal": 1095,
}
REVERSE_WEB_ID_MAP: Dict[int, str] = {
    rid: key for key, rid in WEB_READING_ID_MAP_M1.items()
}

SMARTSKIN_14_LOCATIONS: Tuple[Tuple[str, int], ...] = (
    ("right arm", 1),
    ("right arm", 2),
    ("left arm", 1),
    ("left arm", 2),
    ("back", 1),
    ("back", 2),
    ("back", 3),
    ("back", 4),
    ("right leg", 1),
    ("right leg", 2),
    ("right leg", 3),
    ("left leg", 1),
    ("left leg", 2),
    ("left leg", 3),
)

SMARTSKIN_16_FLEX_LOCATIONS: Tuple[Tuple[str, int], ...] = (
    ("right shoulder", 1),
    ("right shoulder", 2),
    ("left shoulder", 1),
    ("left shoulder", 2),
    ("right elbow", 1),
    ("right elbow", 2),
    ("left elbow", 1),
    ("left elbow", 2),
    ("right waist", 1),
    ("right waist", 2),
    ("left waist", 1),
    ("left waist", 2),
    ("right knee", 1),
    ("right knee", 2),
    ("left knee", 1),
    ("left knee", 2),
)

LOADCELL_WEB_IDS: Tuple[Tuple[str, int], ...] = (
    ("loadcell_leher", 801),
    ("loadcell_paki", 802),
    ("loadcell_paka", 803),
    ("loadcell_kaki", 804),
    ("loadcell_kaka", 805),
)

ALERT_SEVERITY_NAMES: Dict[int, str] = {
    0: "INFO",
    1: "WARNING",
    2: "ERROR",
    3: "CRITICAL",
}

ALERT_EVENT_NAMES: Dict[int, str] = {
    0x01: "SYSTEM_STATUS_CHANGED",
    0x02: "DEVICE_OFFLINE",
    0x03: "DEVICE_RECOVERED",
    0x04: "DEVICE_CALIBRATING",
    0x7F: "UNKNOWN",
}

ALERT_STATUS_NAMES: Dict[int, str] = {
    0: "INITIALIZING",
    1: "OPERATIONAL",
    2: "DEGRADED",
    3: "CALIBRATING",
}

ALERT_DEVICE_BITS: Tuple[Tuple[int, str], ...] = (
    (0, "arduino_uno_q"),
    (1, "cosmic_aurora"),
    (2, "analog_sensors"),
    (3, "mcp9808"),
    (4, "hx711_loadcells"),
    (5, "witmotion_wt901"),
)


# ============================================================
# LOGGER CSV LOKAL KOMUNIKASI LORA (~/blackbox2_ws/logs/communication)
# ============================================================

COMMUNICATION_CSV_HEADER: List[str] = [
    "timestamp",
    "packet_no",
    "mannequin_id",
    "packet_type",
    "payload_mode",
    "rssi_dbm",
    "snr_db",
    "freq_hz",
    "sf",
    "bw_code",
    "cr_code",
    "preamble",
    "bytes",
    "fragments",
    "reassembly_ms",
    "inter_packet_ms",
    "packet_rate_hz",
    "crc_status",
    "crc_hex",
    "crc_errors_total",
    "frag_timeouts_total",
    "valid_fields",
    "total_fields",
    "web_mqtt_sent",
    "web_ack_status",
    "web_http_sent",
]

TELEMETRY_CSV_HEADER: List[str] = [
    "timestamp",
    "packet_no",
    "mannequin_id",
    "payload_mode",
    "rssi_dbm",
    "snr_db",
    "bytes",
    "fragments",
    "reassembly_ms",
    "inter_packet_ms",
    "valid_fields",
    *EXPECTED_KEYS,
]

ALERT_CSV_HEADER: List[str] = [
    "timestamp",
    "packet_no",
    "alert_no",
    "mannequin_id",
    "rssi_dbm",
    "snr_db",
    "bytes",
    "sequence",
    "severity",
    "code",
    "system_status",
    "offline_mask",
    "offline_devices",
    "raw_compact",
]


class LocalCommunicationLogger:
    """
    Menyimpan log komunikasi LoRa (RSSI, SNR, CRC, ukuran byte, fragmen, jitter,
    packet rate) serta data 109 sensor ke file CSV lokal di Raspberry Pi.
    """

    def __init__(self, enabled: bool, log_dir: str, logger):
        self.enabled = enabled
        self.log_dir = os.path.expanduser(log_dir)
        self.logger = logger
        self._lock = threading.Lock()

        self.comm_csv_path = os.path.join(self.log_dir, "lora_rx_communication.csv")
        self.telemetry_csv_path = os.path.join(self.log_dir, "lora_rx_telemetry.csv")
        self.alert_csv_path = os.path.join(self.log_dir, "lora_rx_alerts.csv")

        self._comm_file = None
        self._telemetry_file = None
        self._alert_file = None

        self._comm_writer = None
        self._telemetry_writer = None
        self._alert_writer = None

        if self.enabled:
            self._open_files()

    def _open_files(self) -> None:
        try:
            os.makedirs(self.log_dir, exist_ok=True)

            self._comm_file, self._comm_writer = self._init_csv(
                self.comm_csv_path, COMMUNICATION_CSV_HEADER
            )
            self._telemetry_file, self._telemetry_writer = self._init_csv(
                self.telemetry_csv_path, TELEMETRY_CSV_HEADER
            )
            self._alert_file, self._alert_writer = self._init_csv(
                self.alert_csv_path, ALERT_CSV_HEADER
            )
            self.logger.info(
                f"Local LoRa Communication CSV Logger aktif di: {self.log_dir}"
            )
        except Exception as exc:
            self.enabled = False
            self.logger.error(f"Gagal membuka file log komunikasi lokal: {exc}")

    @staticmethod
    def _init_csv(path: str, header: List[str]):
        file_obj = open(path, "a", newline="", encoding="utf-8", buffering=1)
        writer = csv.writer(file_obj)
        if os.path.getsize(path) == 0:
            writer.writerow(header)
            file_obj.flush()
        return file_obj, writer

    @staticmethod
    def _now_str() -> str:
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    def log_communication(
        self,
        packet_no: int,
        mannequin_id: int,
        packet_type: str,
        payload_mode: str,
        rssi: str,
        snr: str,
        radio_cfg: "RadioConfig",
        byte_len: int,
        fragments: int,
        reassembly_ms: float,
        inter_packet_ms: Optional[float],
        packet_rate_hz: Optional[float],
        crc_status: str,
        crc_hex: str,
        crc_errors_total: int,
        frag_timeouts_total: int,
        valid_fields: int,
        total_fields: int,
        web_mqtt_sent: int,
        web_ack_status: str,
        web_http_sent: int,
    ) -> None:
        if not self.enabled or self._comm_writer is None:
            return

        row = [
            self._now_str(),
            packet_no,
            mannequin_id,
            packet_type,
            payload_mode,
            rssi,
            snr,
            radio_cfg.freq,
            radio_cfg.sf,
            radio_cfg.bw,
            radio_cfg.cr,
            radio_cfg.preamble,
            byte_len,
            fragments,
            round(reassembly_ms, 2),
            "" if inter_packet_ms is None else round(inter_packet_ms, 2),
            "" if packet_rate_hz is None else round(packet_rate_hz, 3),
            crc_status,
            crc_hex,
            crc_errors_total,
            frag_timeouts_total,
            valid_fields,
            total_fields,
            web_mqtt_sent,
            web_ack_status,
            web_http_sent,
        ]
        with self._lock:
            try:
                self._comm_writer.writerow(row)
            except Exception as exc:
                self.logger.warning(f"Gagal menulis lora_rx_communication.csv: {exc}")

    def log_telemetry(
        self,
        packet_no: int,
        mannequin_id: int,
        payload_mode: str,
        rssi: str,
        snr: str,
        byte_len: int,
        fragments: int,
        reassembly_ms: float,
        inter_packet_ms: Optional[float],
        valid_fields: int,
        sensor_dict: Dict[str, Optional[float]],
    ) -> None:
        if not self.enabled or self._telemetry_writer is None:
            return

        row = [
            self._now_str(),
            packet_no,
            mannequin_id,
            payload_mode,
            rssi,
            snr,
            byte_len,
            fragments,
            round(reassembly_ms, 2),
            "" if inter_packet_ms is None else round(inter_packet_ms, 2),
            valid_fields,
            *[
                "" if sensor_dict.get(k) is None else sensor_dict.get(k)
                for k in EXPECTED_KEYS
            ],
        ]
        with self._lock:
            try:
                self._telemetry_writer.writerow(row)
            except Exception as exc:
                self.logger.warning(f"Gagal menulis lora_rx_telemetry.csv: {exc}")

    def log_alert(
        self,
        packet_no: int,
        alert_no: int,
        mannequin_id: int,
        rssi: str,
        snr: str,
        byte_len: int,
        decoded_alert: Dict[str, object],
    ) -> None:
        if not self.enabled or self._alert_writer is None:
            return

        offline_devs = decoded_alert.get("offline_devices", [])
        offline_str = (
            ";".join(str(x) for x in offline_devs)
            if isinstance(offline_devs, list)
            else str(offline_devs)
        )
        row = [
            self._now_str(),
            packet_no,
            alert_no,
            mannequin_id,
            rssi,
            snr,
            byte_len,
            decoded_alert.get("sequence", ""),
            decoded_alert.get("severity", ""),
            decoded_alert.get("code", ""),
            decoded_alert.get("status", ""),
            decoded_alert.get("device_mask", ""),
            offline_str,
            decoded_alert.get("raw_compact", ""),
        ]
        with self._lock:
            try:
                self._alert_writer.writerow(row)
            except Exception as exc:
                self.logger.warning(f"Gagal menulis lora_rx_alerts.csv: {exc}")

    def close(self) -> None:
        with self._lock:
            for f in (self._comm_file, self._telemetry_file, self._alert_file):
                if f is not None:
                    try:
                        f.flush()
                        f.close()
                    except Exception:
                        pass
            self._comm_file = None
            self._telemetry_file = None
            self._alert_file = None


# ============================================================
# BUILT-IN PURE PYTHON MQTT v3.1.1 CLIENT (FALLBACK TANPA PIP)
# ============================================================

def _encode_mqtt_remaining_length(length: int) -> bytes:
    out = bytearray()
    while True:
        digit = length % 128
        length //= 128
        if length > 0:
            digit |= 0x80
        out.append(digit)
        if length <= 0:
            break
    return bytes(out)


def _encode_mqtt_utf8(text: str) -> bytes:
    raw = text.encode("utf-8")
    return struct.pack("!H", len(raw)) + raw


class SimpleSocketMqttClient:
    """Client MQTT v3.1.1 murni berbasis socket standar Python (tanpa instalasi pip)."""

    def __init__(
        self,
        host: str,
        port: int,
        username: str = "",
        password: str = "",
        client_id: str = "lora_rx_manekin",
        keepalive: int = 30,
    ):
        self.host = host
        self.port = port
        self.username = username
        self.password = password
        self.client_id = client_id
        self.keepalive = keepalive
        self._sock: Optional[socket.socket] = None
        self._pkt_id: int = 1
        self._last_io: float = 0.0

    @property
    def is_connected(self) -> bool:
        return self._sock is not None

    def connect(self) -> bool:
        self.close()
        sock = socket.create_connection((self.host, self.port), timeout=3.0)
        sock.settimeout(2.0)

        connect_flags = 0x02  # Clean Session
        payload = _encode_mqtt_utf8(self.client_id)
        if self.username:
            connect_flags |= 0x80
            payload += _encode_mqtt_utf8(self.username)
            if self.password:
                connect_flags |= 0x40
                payload += _encode_mqtt_utf8(self.password)

        var_header = _encode_mqtt_utf8("MQTT") + struct.pack(
            "!BBH", 4, connect_flags, self.keepalive
        )
        body = var_header + payload
        packet = b"\x10" + _encode_mqtt_remaining_length(len(body)) + body
        sock.sendall(packet)

        resp = sock.recv(4)
        if len(resp) < 4 or resp[0] != 0x20 or resp[3] != 0x00:
            sock.close()
            raise ConnectionError(f"MQTT CONNACK ditolak: {resp!r}")

        self._sock = sock
        self._last_io = time.monotonic()
        return True

    def publish(self, topic: str, payload_str: str, qos: int = 1) -> bool:
        if self._sock is None:
            self.connect()
        assert self._sock is not None

        topic_bytes = _encode_mqtt_utf8(topic)
        payload_bytes = payload_str.encode("utf-8")

        if qos == 1:
            self._pkt_id = (self._pkt_id % 65535) + 1
            header_byte = 0x32  # PUBLISH, QoS 1, retain=0
            body = topic_bytes + struct.pack("!H", self._pkt_id) + payload_bytes
        else:
            header_byte = 0x30  # PUBLISH, QoS 0, retain=0
            body = topic_bytes + payload_bytes

        packet = bytes([header_byte]) + _encode_mqtt_remaining_length(len(body)) + body
        self._sock.sendall(packet)
        self._last_io = time.monotonic()

        if qos == 1:
            try:
                ack = self._sock.recv(4)
                if not ack:
                    raise ConnectionError("MQTT socket closed by broker")
            except socket.timeout:
                pass
        return True

    def ping_if_needed(self) -> None:
        if self._sock is None:
            return
        if time.monotonic() - self._last_io >= (self.keepalive * 0.7):
            try:
                self._sock.sendall(b"\xC0\x00")
                self._sock.recv(2)
                self._last_io = time.monotonic()
            except Exception:
                self.close()

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.sendall(b"\xE0\x00")
            except Exception:
                pass
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None


# ============================================================
# WEB TELEMETRY FORWARDER (NON-BLOCKING MQTT V2 + HTTP REST)
# ============================================================

@dataclass(frozen=True)
class WebForwarderConfig:
    """Konfigurasi pengiriman telemetry dari Receiver ke Web Backend (Be-smart-manekin)."""

    mqtt_enabled: bool = DEFAULT_MQTT_ENABLED
    mqtt_host: str = DEFAULT_MQTT_HOST
    mqtt_port: int = DEFAULT_MQTT_PORT
    mqtt_username: str = DEFAULT_MQTT_USERNAME
    mqtt_password: str = DEFAULT_MQTT_PASSWORD
    device_id: str = ""
    http_enabled: bool = DEFAULT_HTTP_ENABLED
    http_base_url: str = DEFAULT_HTTP_BASE_URL
    ingest_api_key: str = DEFAULT_INGEST_API_KEY
    http_timeout_sec: float = 2.5

    def resolve_device_id(self, mannequin_id: int) -> str:
        if self.device_id.strip():
            return self.device_id.strip()
        return DEFAULT_DEVICE_ID_M2 if mannequin_id == 2 else DEFAULT_DEVICE_ID_M1


class WebTelemetryForwarder:
    """
    Pengirim data dari Receiver LoRa ke Web Backend (Be-smart-manekin) di background thread:
      1. MQTT Compact V2 -> topic `smart-mannequin/{deviceIdentifier}/telemetry`
         + subscribe Application ACK `smart-mannequin/{deviceIdentifier}/ack`
      2. HTTP REST Ingest -> `POST /sensor-reading/batch` & `POST /sensor/*`
    """

    def __init__(self, config: WebForwarderConfig, logger):
        self.config = config
        self.logger = logger
        self._queue: deque = deque(maxlen=64)
        self._cond = threading.Condition()
        self._running = True

        self.mqtt_sent_count = 0
        self.mqtt_ack_count = 0
        self.mqtt_fail_count = 0
        self.http_sent_count = 0
        self.http_fail_count = 0
        self.last_ack_status = "-"

        self._paho_client = None
        self._paho_connected = False
        self._simple_mqtt: Optional[SimpleSocketMqttClient] = None

        if self.config.mqtt_enabled:
            self._init_mqtt()

        self._worker = threading.Thread(
            target=self._worker_loop, name="web_telemetry_forwarder", daemon=True
        )
        self._worker.start()

    def _init_mqtt(self) -> None:
        client_id = f"lora_rx_manekin_{os.getpid()}"
        if paho_mqtt is not None:
            try:
                try:
                    client = paho_mqtt.Client(
                        paho_mqtt.CallbackAPIVersion.VERSION2, client_id=client_id
                    )
                except AttributeError:
                    client = paho_mqtt.Client(client_id=client_id)

                if self.config.mqtt_username:
                    client.username_pw_set(
                        self.config.mqtt_username,
                        self.config.mqtt_password or None,
                    )

                client.on_connect = self._on_paho_connect
                client.on_disconnect = self._on_paho_disconnect
                client.on_message = self._on_paho_message
                client.connect_async(self.config.mqtt_host, self.config.mqtt_port, 30)
                client.loop_start()
                self._paho_client = client
                self.logger.info(
                    f"Web MQTT client (paho-mqtt) aktif -> "
                    f"mqtt://{self.config.mqtt_host}:{self.config.mqtt_port}"
                )
                return
            except Exception as exc:
                self.logger.warning(
                    f"Gagal inisialisasi paho-mqtt ({exc}), beralih ke socket MQTT bawaan."
                )

        self._simple_mqtt = SimpleSocketMqttClient(
            host=self.config.mqtt_host,
            port=self.config.mqtt_port,
            username=self.config.mqtt_username,
            password=self.config.mqtt_password,
            client_id=client_id,
        )
        self.logger.info(
            f"Web MQTT client (built-in socket) siap -> "
            f"mqtt://{self.config.mqtt_host}:{self.config.mqtt_port}"
        )

    def _on_paho_connect(self, client, userdata, flags, rc, *args) -> None:
        rc_val = getattr(rc, "value", rc)
        if rc_val == 0:
            self._paho_connected = True
            client.subscribe("smart-mannequin/+/ack", qos=1)
            self.logger.info(
                f"Terhubung ke Web MQTT Broker ({self.config.mqtt_host}:{self.config.mqtt_port}) "
                f"& subscribe smart-mannequin/+/ack"
            )
        else:
            self._paho_connected = False
            self.logger.warning(f"Koneksi Web MQTT gagal dengan kode: {rc}")

    def _on_paho_disconnect(self, client, userdata, *args) -> None:
        self._paho_connected = False

    def _on_paho_message(self, client, userdata, msg) -> None:
        try:
            ack = json.loads(msg.payload.decode("utf-8", errors="ignore"))
            if isinstance(ack, dict):
                self.mqtt_ack_count += 1
                status = str(ack.get("status", "UNKNOWN"))
                self.last_ack_status = status
                dev_id = ack.get("deviceIdentifier", "-")
                seq = ack.get("s", "-")
                readings_cnt = ack.get("readingsCount", 0)
                if status == "REJECTED":
                    err_code = ack.get("errorCode", "UNKNOWN_ERROR")
                    self.logger.error(
                        f"[WEB-ACK REJECTED] dev={dev_id} s={seq} errorCode={err_code}"
                    )
                else:
                    self.logger.info(
                        f"[WEB-ACK #{self.mqtt_ack_count:04d}] dev={dev_id} s={seq} "
                        f"status={status} readings={readings_cnt}"
                    )
        except Exception:
            pass

    def enqueue_telemetry(
        self,
        mannequin_id: int,
        web_v2_obj: Dict[str, object],
        sensor_dict: Dict[str, Optional[float]],
    ) -> None:
        if not self.config.mqtt_enabled and not self.config.http_enabled:
            return
        with self._cond:
            self._queue.append(("telemetry", mannequin_id, web_v2_obj, sensor_dict))
            self._cond.notify()

    def enqueue_alert(self, mannequin_id: int, alert_payload: Dict[str, object]) -> None:
        if not self.config.mqtt_enabled and not self.config.http_enabled:
            return
        with self._cond:
            self._queue.append(("alert", mannequin_id, alert_payload, None))
            self._cond.notify()

    def _worker_loop(self) -> None:
        while self._running:
            item = None
            with self._cond:
                if not self._queue:
                    self._cond.wait(timeout=1.0)
                if self._queue:
                    item = self._queue.popleft()

            if item is None:
                if self._simple_mqtt is not None:
                    self._simple_mqtt.ping_if_needed()
                continue

            kind, mannequin_id, payload_obj, sensor_dict = item
            device_id = self.config.resolve_device_id(mannequin_id)

            if kind == "telemetry":
                if self.config.mqtt_enabled:
                    self._send_mqtt_telemetry(device_id, payload_obj)
                if self.config.http_enabled and isinstance(sensor_dict, dict):
                    self._send_http_telemetry(mannequin_id, sensor_dict)
            elif kind == "alert":
                if self.config.mqtt_enabled:
                    self._send_mqtt_alert(device_id, payload_obj)

    def _send_mqtt_telemetry(
        self, device_id: str, web_v2_obj: Dict[str, object]
    ) -> None:
        raw_readings = web_v2_obj.get("r", [])
        if isinstance(raw_readings, list):
            non_null = [
                pair for pair in raw_readings
                if isinstance(pair, (list, tuple)) and len(pair) >= 2 and pair[1] is not None
            ]
            outbound_r = non_null if non_null else raw_readings
        else:
            outbound_r = []

        if not outbound_r:
            return

        wire_obj = {
            "v": 2,
            "b": int(web_v2_obj.get("b", 1)),
            "s": int(web_v2_obj.get("s", 1)),
            "t": int(web_v2_obj.get("t", int(time.time() * 1000))),
            "r": outbound_r,
        }
        topic = f"smart-mannequin/{device_id}/telemetry"
        payload_str = json.dumps(wire_obj, separators=(",", ":"), allow_nan=False)

        if self._publish_mqtt_raw(topic, payload_str):
            self.mqtt_sent_count += 1
        else:
            self.mqtt_fail_count += 1

    def _send_mqtt_alert(
        self, device_id: str, alert_payload: Dict[str, object]
    ) -> None:
        topic = f"smart-mannequin/{device_id}/alerts"
        payload_str = json.dumps(alert_payload, separators=(",", ":"), allow_nan=False)
        self._publish_mqtt_raw(topic, payload_str)

    def _publish_mqtt_raw(self, topic: str, payload_str: str) -> bool:
        if self._paho_client is not None:
            try:
                info = self._paho_client.publish(topic, payload_str, qos=1, retain=False)
                return info.rc == 0
            except Exception as exc:
                self.logger.warning(f"Web MQTT publish gagal ({topic}): {exc}")
                return False

        if self._simple_mqtt is not None:
            try:
                return self._simple_mqtt.publish(topic, payload_str, qos=1)
            except Exception as exc:
                self._simple_mqtt.close()
                self.logger.warning(
                    f"Web MQTT socket gagal ({self.config.mqtt_host}:{self.config.mqtt_port}): {exc}"
                )
                return False
        return False

    def _post_http_json(self, path: str, body_obj: object) -> bool:
        base = self.config.http_base_url.rstrip("/")
        url = f"{base}{path}" if path.startswith("/") else f"{base}/{path}"
        data = json.dumps(body_obj, separators=(",", ":"), allow_nan=False).encode("utf-8")

        headers = {
            "Content-Type": "application/json",
            "Accept": "application/json",
        }
        if self.config.ingest_api_key:
            headers["x-ingest-key"] = self.config.ingest_api_key

        req = urllib.request.Request(url, data=data, headers=headers, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.config.http_timeout_sec) as resp:
                return 200 <= resp.status < 300
        except urllib.error.HTTPError as exc:
            self.logger.warning(f"HTTP POST {url} ditolak ({exc.code})")
            return False
        except Exception as exc:
            self.logger.warning(f"HTTP POST {url} gagal: {exc}")
            return False

    def _send_http_telemetry(
        self, mannequin_id: int, d: Dict[str, Optional[float]]
    ) -> None:
        ok_any = False

        readings: List[Dict[str, object]] = []
        for idx, (loc, pt) in enumerate(SMARTSKIN_14_LOCATIONS, start=1):
            v_piezo = d.get(f"piezo{idx}")
            if v_piezo is not None:
                readings.append({
                    "sensorType": "vibration",
                    "location": loc,
                    "sensorNumber": pt,
                    "processingType": "RAW",
                    "value": v_piezo,
                })
            v_fsr = d.get(f"fsr{idx}")
            if v_fsr is not None:
                readings.append({
                    "sensorType": "pressure",
                    "location": loc,
                    "sensorNumber": pt,
                    "processingType": "CALIBRATED_LINEAR",
                    "value": v_fsr,
                })
            v_mcp = d.get(f"mcp{idx}")
            if v_mcp is not None:
                readings.append({
                    "sensorType": "temperature",
                    "location": loc,
                    "sensorNumber": pt,
                    "processingType": "RAW",
                    "value": v_mcp,
                })

        for idx, (loc, pt) in enumerate(SMARTSKIN_16_FLEX_LOCATIONS, start=1):
            v_flex = d.get(f"flex{idx}")
            if v_flex is not None:
                readings.append({
                    "sensorType": "flex",
                    "location": loc,
                    "sensorNumber": pt,
                    "processingType": "RAW",
                    "value": v_flex,
                })

        if d.get("thermal") is not None:
            readings.append({
                "sensorType": "thermal_temperature",
                "location": "chest",
                "sensorNumber": 1,
                "processingType": "RAW",
                "value": d["thermal"],
            })

        if readings:
            if self._post_http_json(
                "/sensor-reading/batch",
                {"mannequinId": mannequin_id, "readings": readings},
            ):
                ok_any = True

        if d.get("lidar_raw") is not None or d.get("lidar_median") is not None:
            lidar_body = {"sensor_id": 901}
            if d.get("lidar_raw") is not None:
                lidar_body["value"] = d["lidar_raw"]
            if d.get("lidar_median") is not None:
                lidar_body["kalmanvalue"] = d["lidar_median"]
            ok_any = self._post_http_json("/sensor/lidar", lidar_body) or ok_any

        if any(d.get(k) is not None for k in ("mq_co", "mq_co2", "smoke", "mq_nh3")):
            mq_body = {"sensor_id": 101}
            if d.get("mq_nh3") is not None:
                mq_body["value"] = d["mq_nh3"]
            if d.get("mq_co") is not None:
                mq_body["co"] = d["mq_co"]
            if d.get("mq_co2") is not None:
                mq_body["co2"] = d["mq_co2"]
            if d.get("smoke") is not None:
                mq_body["smoke"] = d["smoke"]
            ok_any = self._post_http_json("/sensor/mq", mq_body) or ok_any

        if any(
            d.get(k) is not None
            for k in (
                "adxl_acc_x_raw",
                "adxl_acc_y_raw",
                "adxl_acc_z_raw",
                "adxl_acc_x_kalman",
                "adxl_acc_y_kalman",
                "adxl_acc_z_kalman",
            )
        ):
            adxl_body: Dict[str, object] = {"sensor_id": 201}
            for src_k, dst_k in (
                ("adxl_acc_x_raw", "x"),
                ("adxl_acc_y_raw", "y"),
                ("adxl_acc_z_raw", "z"),
                ("adxl_acc_x_kalman", "x_kalman"),
                ("adxl_acc_y_kalman", "y_kalman"),
                ("adxl_acc_z_kalman", "z_kalman"),
            ):
                if d.get(src_k) is not None:
                    adxl_body[dst_k] = d[src_k]
            ok_any = self._post_http_json("/sensor/adxl", adxl_body) or ok_any

        if any(d.get(k) is not None for k in ("bme_temp", "bme_pres", "bme_hum")):
            bme_body: Dict[str, object] = {}
            if d.get("bme_temp") is not None:
                bme_body["temperature"] = d["bme_temp"]
            if d.get("bme_pres") is not None:
                bme_body["pressure"] = d["bme_pres"]
            if d.get("bme_hum") is not None:
                bme_body["humidity"] = d["bme_hum"]
            ok_any = self._post_http_json("/sensor/bme", bme_body) or ok_any

        for lc_key, lc_id in LOADCELL_WEB_IDS:
            if d.get(lc_key) is not None:
                ok_any = (
                    self._post_http_json(
                        "/sensor/loadcell", {"sensor_id": lc_id, "value": d[lc_key]}
                    )
                    or ok_any
                )

        if any(d.get(k) is not None for k in ("roll", "pitch", "yaw")):
            wit_body: Dict[str, object] = {}
            if d.get("roll") is not None:
                wit_body["x_angle"] = d["roll"]
            if d.get("pitch") is not None:
                wit_body["y_angle"] = d["pitch"]
            if d.get("yaw") is not None:
                wit_body["z_angle"] = d["yaw"]
            ok_any = self._post_http_json("/sensor/witsensor", wit_body) or ok_any

        if d.get("sound_dbRight") is not None:
            ok_any = (
                self._post_http_json(
                    "/sensor/ky",
                    {
                        "sensor_id": 601,
                        "value": d["sound_dbRight"],
                        "mannequin_id": mannequin_id,
                    },
                )
                or ok_any
            )
        if d.get("sound_dbLeft") is not None:
            ok_any = (
                self._post_http_json(
                    "/sensor/ky",
                    {
                        "sensor_id": 602,
                        "value": d["sound_dbLeft"],
                        "mannequin_id": mannequin_id,
                    },
                )
                or ok_any
            )

        if ok_any:
            self.http_sent_count += 1
        else:
            self.http_fail_count += 1

    def close(self) -> None:
        self._running = False
        with self._cond:
            self._cond.notify_all()
        if self._paho_client is not None:
            try:
                self._paho_client.loop_stop()
                self._paho_client.disconnect()
            except Exception:
                pass
        if self._simple_mqtt is not None:
            self._simple_mqtt.close()


# ============================================================
# KONFIGURASI & DRIVER SERIAL RAK3272S
# ============================================================

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
        bw = self.bw if bw_override is None else bw_override
        return (
            f"AT+P2P={self.freq}:{self.sf}:{bw}:"
            f"{self.cr}:{self.preamble}:{self.tx_power}"
        )


def declare_radio_config(node: Node, default_port: str) -> RadioConfig:
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


def declare_web_config(node: Node) -> WebForwarderConfig:
    node.declare_parameter("mqtt_enabled", DEFAULT_MQTT_ENABLED)
    node.declare_parameter("mqtt_host", DEFAULT_MQTT_HOST)
    node.declare_parameter("mqtt_port", DEFAULT_MQTT_PORT)
    node.declare_parameter("mqtt_username", DEFAULT_MQTT_USERNAME)
    node.declare_parameter("mqtt_password", DEFAULT_MQTT_PASSWORD)
    node.declare_parameter("device_id", "")
    node.declare_parameter("http_enabled", DEFAULT_HTTP_ENABLED)
    node.declare_parameter("http_base_url", DEFAULT_HTTP_BASE_URL)
    node.declare_parameter("ingest_api_key", DEFAULT_INGEST_API_KEY)
    node.declare_parameter("http_timeout_sec", 2.5)

    return WebForwarderConfig(
        mqtt_enabled=bool(node.get_parameter("mqtt_enabled").value),
        mqtt_host=str(node.get_parameter("mqtt_host").value),
        mqtt_port=int(node.get_parameter("mqtt_port").value),
        mqtt_username=str(node.get_parameter("mqtt_username").value),
        mqtt_password=str(node.get_parameter("mqtt_password").value),
        device_id=str(node.get_parameter("device_id").value),
        http_enabled=bool(node.get_parameter("http_enabled").value),
        http_base_url=str(node.get_parameter("http_base_url").value),
        ingest_api_key=str(node.get_parameter("ingest_api_key").value),
        http_timeout_sec=float(node.get_parameter("http_timeout_sec").value),
    )


class RakP2PReceiver:
    """Mengelola koneksi serial, konfigurasi AT P2P, dan pembacaan RX modul RAK3272S."""

    def __init__(self, config: RadioConfig, fallback_ports: Tuple[str, ...], logger):
        self.config = config
        self.fallback_ports = fallback_ports
        self.logger = logger
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
        now = time.monotonic()
        if now - self._last_reconnect_try < cooldown:
            return False
        self._last_reconnect_try = now
        self.close(stop_rx=False)
        self.logger.warning("Mencoba reconnect ke port RAK3272S...")
        return self.connect()

    def send_at(self, command: str, timeout: float = 2.0) -> bool:
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
        if not self.is_connected:
            return

        self.logger.info("Configuring RAK LoRa P2P Receiver...")
        self.send_at("AT", timeout=1.0)
        self._ensure_p2p_mode()
        self.send_at("AT+PRECV=0", timeout=1.0)

        if not self.send_at(self.config.p2p_command(), timeout=2.0) and self.config.bw == 2:
            self.send_at(self.config.p2p_command(bw_override=500), timeout=2.0)

        if not self.send_at("AT+PRECV=65534", timeout=2.0):
            self.logger.info("Fallback AT+PRECV=65535...")
            self.send_at("AT+PRECV=65535", timeout=2.0)

        self.logger.info("RAK Receiver configuration sequence completed.")

    def _ensure_p2p_mode(self) -> None:
        self.serial_port.reset_input_buffer()
        self.serial_port.write(b"AT+NWM=?\r\n")
        self.serial_port.flush()
        time.sleep(0.3)
        res_nwm = self.serial_port.read_all().decode(errors="ignore")

        if "=0" not in res_nwm:
            self.logger.info("Mengubah RAK ke mode LoRa P2P (AT+NWM=0)...")
            self.send_at("AT+NWM=0", timeout=2.0)
            time.sleep(2.0)
            self.send_at("AT", timeout=1.0)

    def read_lines(self) -> List[str]:
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
        if self.serial_port is None:
            return
        if self.serial_port.is_open:
            if stop_rx:
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


# ============================================================
# HELPER PARSING, CRC16, FRAGMENT, DECODER & FORMATTING
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


def valid_number(value) -> Optional[float]:
    if isinstance(value, bool) or value is None:
        return None
    try:
        number = float(value)
    except (ValueError, TypeError):
        return None
    return number if math.isfinite(number) else None


def publish_json(publisher, payload: object, compact: bool = False) -> None:
    msg = String()
    if compact:
        msg.data = json.dumps(payload, separators=(",", ":"), allow_nan=False)
    else:
        msg.data = json.dumps(payload)
    publisher.publish(msg)


def parse_rx_event(line: str) -> Optional[Tuple[str, str, str]]:
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


class FragmentAssembler:
    """Menangani reassembly paket berheader FRG:index/total secara aman."""

    def __init__(self, timeout_sec: float):
        self.timeout_sec = timeout_sec
        self._chunks: Dict[int, bytes] = {}
        self._total: int = 0
        self._start_time: float = 0.0

    def add_fragment(self, raw_bytes: bytes) -> Optional[Tuple[bytes, int, float]]:
        meta, chunk = raw_bytes[4:].split(b":", 1)
        idx_str, tot_str = meta.decode("ascii").split("/")
        idx, total = int(idx_str), int(tot_str)
        now = time.monotonic()

        if idx == 1 or total != self._total:
            self._chunks.clear()
            self._total = total
            self._start_time = now

        self._chunks[idx] = chunk
        if len(self._chunks) < self._total:
            return None

        complete_data = b"".join(self._chunks[i] for i in range(1, self._total + 1))
        elapsed_ms = (now - self._start_time) * 1000.0
        completed_total = self._total
        self.reset()
        return complete_data, completed_total, elapsed_ms

    def expire_if_stale(self) -> Optional[Tuple[int, int]]:
        if not self._chunks:
            return None
        if (time.monotonic() - self._start_time) <= self.timeout_sec:
            return None
        dropped = (len(self._chunks), self._total)
        self.reset()
        return dropped

    def reset(self) -> None:
        self._chunks.clear()
        self._total = 0


def split_and_verify_crc(complete_data: bytes) -> Tuple[bytes, str]:
    """
    Memeriksa akhiran CRC16-CCITT (*XXXX) bila tersedia dan mengembalikan (payload_bytes, crc_hex).

    Melempar ValueError jika checksum *XXXX ada tetapi tidak cocok.
    """
    if b"*" not in complete_data:
        return complete_data, "NONE"

    candidate_body, crc_bytes = complete_data.rsplit(b"*", 1)
    crc_text = crc_bytes.strip()
    if len(crc_text) != 4:
        return complete_data, "NONE"

    try:
        crc_hex = crc_text.decode("ascii").upper()
        expected_crc = int(crc_hex, 16)
    except (ValueError, UnicodeDecodeError):
        return complete_data, "NONE"

    actual_crc = crc16_ccitt(candidate_body)
    if actual_crc != expected_crc:
        raise ValueError(
            f"CRC16 mismatch (dihitung {actual_crc:04X}, diterima {expected_crc:04X})"
        )
    return candidate_body, crc_hex


def decode_compact_alert(text: str) -> Dict[str, object]:
    if len(text) != 12 or not text.startswith("A1"):
        raise ValueError(f"Format alert kompak tidak valid: {text!r}")

    sequence = int(text[2:6], 16)
    severity_code = int(text[6:7], 16)
    event_code = int(text[7:9], 16)
    status_code = int(text[9:10], 16)
    device_mask = int(text[10:12], 16)

    severity = ALERT_SEVERITY_NAMES.get(severity_code, f"SEVERITY_{severity_code}")
    code = ALERT_EVENT_NAMES.get(event_code, f"EVENT_{event_code:02X}")
    status = ALERT_STATUS_NAMES.get(status_code, f"STATUS_{status_code}")
    offline_devices = [
        name for bit, name in ALERT_DEVICE_BITS if device_mask & (1 << bit)
    ]

    return {
        "sequence": sequence,
        "severity": severity,
        "code": code,
        "status": status,
        "device_mask": device_mask,
        "offline_devices": offline_devices,
        "raw_compact": text,
    }


def decode_array_telemetry(
    values: list,
) -> Tuple[Dict[str, Optional[float]], Optional[str]]:
    warning_msg: Optional[str] = None
    if len(values) != NUM_PARAMS:
        warning_msg = (
            f"Jumlah field array tidak standar: diterima {len(values)}, "
            f"seharusnya {NUM_PARAMS}"
        )

    sensor_dict: Dict[str, Optional[float]] = {key: None for key in EXPECTED_KEYS}
    for key, raw_val in zip(EXPECTED_KEYS, values):
        sensor_dict[key] = valid_number(raw_val)

    return sensor_dict, warning_msg


def decode_web_v2_telemetry(
    payload_obj: dict, default_mannequin_id: int
) -> Tuple[int, Dict[str, Optional[float]]]:
    sensor_dict: Dict[str, Optional[float]] = {key: None for key in EXPECTED_KEYS}
    readings = payload_obj.get("r", [])

    inferred_id = default_mannequin_id
    for entry in readings:
        if not isinstance(entry, (list, tuple)) or len(entry) < 2:
            continue
        raw_id, raw_val = entry[0], entry[1]
        if isinstance(raw_id, bool):
            continue
        try:
            rid = int(raw_id)
        except (ValueError, TypeError):
            continue

        if 2000 <= rid < 3000:
            inferred_id = 2
            base_id = rid - 1000
        elif 1000 <= rid < 2000:
            inferred_id = 1
            base_id = rid
        else:
            base_id = rid

        key = REVERSE_WEB_ID_MAP.get(base_id)
        if key is not None:
            sensor_dict[key] = valid_number(raw_val)

    return inferred_id, sensor_dict


def build_web_v2_payload(
    sensor_dict: Dict[str, Optional[float]],
    mannequin_id: int,
    boot_count: int,
    seq: int,
) -> Dict[str, object]:
    id_offset = 1000 if mannequin_id == 2 else 0
    readings = [
        [base_id + id_offset, sensor_dict.get(key)]
        for key, base_id in WEB_READING_ID_MAP_M1.items()
    ]
    return {
        "v": 2,
        "b": boot_count,
        "s": seq,
        "t": int(time.time() * 1000),
        "r": readings,
    }


def format_manekin_horizontal(d: Dict[str, Optional[float]]) -> str:
    def val(k: str) -> object:
        v = d.get(k)
        return "-" if v is None else v

    parts = [
        f"Lidar:(raw:{val('lidar_raw')},med:{val('lidar_median')})",
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
        f"Loadcell({','.join(str(val(k)) for k in SENSOR_GROUPS['loadcell'])})",
        f"Flex[{','.join(str(val(k)) for k in SENSOR_GROUPS['flex'])}]",
        f"MCP[{','.join(str(val(k)) for k in SENSOR_GROUPS['mcp'])}]",
        f"FSR[{','.join(str(val(k)) for k in SENSOR_GROUPS['fsr'])}]",
        f"Piezo[{','.join(str(val(k)) for k in SENSOR_GROUPS['piezo'])}]",
        f"Thermal:{val('thermal')}",
        (
            f"WT901[A:({val('ax')},{val('ay')},{val('az')}) "
            f"G:({val('gx')},{val('gy')},{val('gz')}) "
            f"RPY:({val('roll')},{val('pitch')},{val('yaw')})]"
        ),
    ]
    return " | ".join(parts)


# ============================================================
# NODE ROS2 RECEIVER MANEKIN 1 (+ WEB FORWARDER + LOCAL LOGGER)
# ============================================================

class LoRaManekin1ReceiverNode(Node):
    """Node ROS2 receiver LoRa P2P standalone + Web Forwarder + Local CSV Communication Logger."""

    def __init__(self, default_port: str = DEFAULT_SERIAL_PORT):
        super().__init__("lora_rx_manekin1_node")

        self.radio_config = declare_radio_config(self, default_port)
        web_config = declare_web_config(self)

        self.declare_parameter("frag_timeout", 3.0)
        self.declare_parameter("mannequin_id", 1)
        self.declare_parameter("boot_count", 1)
        self.declare_parameter("log_enabled", DEFAULT_LOG_ENABLED)
        self.declare_parameter("log_dir", DEFAULT_LOG_DIR)

        frag_timeout = float(self.get_parameter("frag_timeout").value)
        self.default_mannequin_id = int(self.get_parameter("mannequin_id").value)
        self.boot_count = int(self.get_parameter("boot_count").value)
        log_enabled = bool(self.get_parameter("log_enabled").value)
        log_dir = str(self.get_parameter("log_dir").value)

        self.packet_count = 0
        self.alert_count = 0
        self.crc_error_count = 0
        self.frag_timeout_count = 0
        self.web_seq = 0
        self.last_rssi = "-"
        self.last_snr = "-"
        self.last_packet_monotonic: Optional[float] = None
        self.assembler = FragmentAssembler(frag_timeout)

        # Logger CSV Lokal di ~/blackbox2_ws/logs/communication/
        self.comm_logger = LocalCommunicationLogger(
            enabled=log_enabled,
            log_dir=log_dir,
            logger=self.get_logger(),
        )

        # Publisher utama ROS 2 (Web Compact V2, Alerts, Status, Raw, dan 9 group sensor)
        self.web_v2_pub = self.create_publisher(String, "/telemetri/web/v2", 10)
        self.alert_pub = self.create_publisher(String, "/system/alerts", 10)
        self.raw_pub = self.create_publisher(String, "/telemetri/received_raw", 10)
        self.status_pub = self.create_publisher(String, "/telemetri/radio_status", 10)
        self.group_pubs = {
            group_name: self.create_publisher(String, f"/telemetri/{group_name}", 10)
            for group_name in SENSOR_GROUPS
        }

        # Pengirim langsung ke Web Backend (MQTT Compact V2 & HTTP REST)
        self.web_forwarder = WebTelemetryForwarder(
            config=web_config,
            logger=self.get_logger(),
        )

        self.radio = RakP2PReceiver(
            config=self.radio_config,
            fallback_ports=FALLBACK_PORTS,
            logger=self.get_logger(),
        )
        self.radio.connect()

        self.create_timer(0.005, self.read_serial_data)
        self.create_timer(0.5, self.check_frag_timeout)

        active_dev_id = web_config.resolve_device_id(self.default_mannequin_id)
        self.get_logger().info(
            f"LoRa Manekin Receiver started on {self.radio.active_port} "
            f"@ {self.radio_config.baudrate} | Freq:{self.radio_config.freq} SF{self.radio_config.sf} "
            f"BW{self.radio_config.bw} CR{self.radio_config.cr} | {NUM_PARAMS} params | "
            f"LocalLog:{self.comm_logger.log_dir} | "
            f"WebMQTT:{web_config.mqtt_enabled} (topic=smart-mannequin/{active_dev_id}/telemetry)"
        )

    def _compute_inter_packet_timing(self) -> Tuple[Optional[float], Optional[float]]:
        now = time.monotonic()
        if self.last_packet_monotonic is None:
            self.last_packet_monotonic = now
            return None, None
        delta_sec = now - self.last_packet_monotonic
        self.last_packet_monotonic = now
        inter_ms = delta_sec * 1000.0
        rate_hz = (1.0 / delta_sec) if delta_sec > 0 else None
        return inter_ms, rate_hz

    def read_serial_data(self) -> None:
        if not self.radio.is_connected:
            self.radio.reconnect_if_due()
            return

        try:
            lines = self.radio.read_lines()
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f"Serial read error: {exc}")
            self.radio.reconnect_if_due()
            return

        for line in lines:
            parsed = parse_rx_event(line)
            if parsed is None:
                continue
            rssi, snr, hex_str = parsed
            try:
                raw_bytes = bytes.fromhex(hex_str)
            except ValueError:
                self.get_logger().error(f"Gagal konversi hex: {hex_str[:30]}")
                continue
            self.handle_received_bytes(raw_bytes, rssi, snr)

    def handle_received_bytes(self, raw_bytes: bytes, rssi: str, snr: str) -> None:
        self.last_rssi = rssi or "-"
        self.last_snr = snr or "-"

        if raw_bytes.startswith(b"FRG:"):
            try:
                assembled = self.assembler.add_fragment(raw_bytes)
            except Exception as exc:
                self.get_logger().error(f"Gagal memproses fragmen FRG: {exc}")
                return
            if assembled is not None:
                complete_data, total_frags, elapsed_ms = assembled
                self.process_complete_data(complete_data, total_frags, elapsed_ms)
            return

        self.process_complete_data(raw_bytes, total_frags=1, elapsed_ms=0.0)

    def process_complete_data(
        self, complete_data: bytes, total_frags: int, elapsed_ms: float
    ) -> None:
        self.packet_count += 1
        inter_ms, rate_hz = self._compute_inter_packet_timing()

        try:
            payload_bytes, crc_hex = split_and_verify_crc(complete_data)
        except ValueError as exc:
            self.crc_error_count += 1
            self.comm_logger.log_communication(
                packet_no=self.packet_count,
                mannequin_id=self.default_mannequin_id,
                packet_type="crc_error",
                payload_mode="invalid_crc",
                rssi=self.last_rssi,
                snr=self.last_snr,
                radio_cfg=self.radio_config,
                byte_len=len(complete_data),
                fragments=total_frags,
                reassembly_ms=elapsed_ms,
                inter_packet_ms=inter_ms,
                packet_rate_hz=rate_hz,
                crc_status="CRC_ERROR",
                crc_hex="MISMATCH",
                crc_errors_total=self.crc_error_count,
                frag_timeouts_total=self.frag_timeout_count,
                valid_fields=0,
                total_fields=NUM_PARAMS,
                web_mqtt_sent=self.web_forwarder.mqtt_sent_count,
                web_ack_status=self.web_forwarder.last_ack_status,
                web_http_sent=self.web_forwarder.http_sent_count,
            )
            self.get_logger().warning(
                f"[RX #{self.packet_count:04d}] {exc}. Paket dibuang."
            )
            return

        try:
            text = payload_bytes.decode("utf-8").strip()
        except UnicodeDecodeError as exc:
            self.get_logger().warning(
                f"[RX #{self.packet_count:04d}] Payload bukan UTF-8 valid ({exc})."
            )
            return

        if text.startswith("A1") and len(text) == 12:
            self._handle_alert(
                text, len(complete_data), total_frags, elapsed_ms, inter_ms, rate_hz, crc_hex
            )
            return

        self._handle_json_payload(
            text, len(complete_data), total_frags, elapsed_ms, inter_ms, rate_hz, crc_hex
        )

    def _handle_alert(
        self,
        text: str,
        byte_len: int,
        total_frags: int,
        elapsed_ms: float,
        inter_ms: Optional[float],
        rate_hz: Optional[float],
        crc_hex: str,
    ) -> None:
        try:
            decoded = decode_compact_alert(text)
        except ValueError as exc:
            self.get_logger().warning(f"[RX #{self.packet_count:04d}] {exc}")
            return

        self.alert_count += 1
        alert_payload = {
            "source": "lora_rx_manekin",
            "mannequin_id": self.default_mannequin_id,
            "sequence": decoded["sequence"],
            "severity": decoded["severity"],
            "code": decoded["code"],
            "details": {
                "current_status": decoded["status"],
                "offline_mask": decoded["device_mask"],
                "system_status": {
                    "status": decoded["status"],
                    "offline_devices": decoded["offline_devices"],
                },
            },
            "raw_compact": decoded["raw_compact"],
            "rssi": self.last_rssi,
            "snr": self.last_snr,
            "received_at_ms": int(time.time() * 1000),
        }
        publish_json(self.alert_pub, alert_payload)
        self.web_forwarder.enqueue_alert(self.default_mannequin_id, alert_payload)

        # Simpan ke CSV komunikasi & CSV alert lokal
        self.comm_logger.log_communication(
            packet_no=self.packet_count,
            mannequin_id=self.default_mannequin_id,
            packet_type="alert",
            payload_mode="alert",
            rssi=self.last_rssi,
            snr=self.last_snr,
            radio_cfg=self.radio_config,
            byte_len=byte_len,
            fragments=total_frags,
            reassembly_ms=elapsed_ms,
            inter_packet_ms=inter_ms,
            packet_rate_hz=rate_hz,
            crc_status="OK",
            crc_hex=crc_hex,
            crc_errors_total=self.crc_error_count,
            frag_timeouts_total=self.frag_timeout_count,
            valid_fields=0,
            total_fields=NUM_PARAMS,
            web_mqtt_sent=self.web_forwarder.mqtt_sent_count,
            web_ack_status=self.web_forwarder.last_ack_status,
            web_http_sent=self.web_forwarder.http_sent_count,
        )
        self.comm_logger.log_alert(
            packet_no=self.packet_count,
            alert_no=self.alert_count,
            mannequin_id=self.default_mannequin_id,
            rssi=self.last_rssi,
            snr=self.last_snr,
            byte_len=byte_len,
            decoded_alert=decoded,
        )

        self._publish_radio_status(
            mannequin_id=self.default_mannequin_id,
            valid_fields=0,
            byte_len=byte_len,
            total_frags=total_frags,
            elapsed_ms=elapsed_ms,
            inter_ms=inter_ms,
            rate_hz=rate_hz,
            payload_mode="alert",
        )

        pkt_label = "1Paket" if total_frags == 1 else f"{total_frags}Frg"
        self.get_logger().warning(
            f"[RX #{self.packet_count:04d}] [M{self.default_mannequin_id} ALERT #{self.alert_count:04d}] "
            f"[{byte_len}B {pkt_label}] [{self.last_rssi}dBm {self.last_snr}dB] "
            f"seq={decoded['sequence']} sev={decoded['severity']} code={decoded['code']} "
            f"status={decoded['status']} offline={decoded['offline_devices']}"
        )

    def _handle_json_payload(
        self,
        text: str,
        byte_len: int,
        total_frags: int,
        elapsed_ms: float,
        inter_ms: Optional[float],
        rate_hz: Optional[float],
        crc_hex: str,
    ) -> None:
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(
                f"[RX #{self.packet_count:04d}] Gagal parse JSON ({exc}): {text[:80]}"
            )
            return

        mannequin_id = self.default_mannequin_id
        web_v2_obj: Optional[Dict[str, object]] = None

        if isinstance(parsed, list):
            payload_mode = "array"
            sensor_dict, len_warn = decode_array_telemetry(parsed)
            if len_warn:
                self.get_logger().warning(f"[RX #{self.packet_count:04d}] {len_warn}")
        elif isinstance(parsed, dict) and parsed.get("v") == 2 and isinstance(parsed.get("r"), list):
            payload_mode = "web_v2"
            mannequin_id, sensor_dict = decode_web_v2_telemetry(
                parsed, self.default_mannequin_id
            )
            web_v2_obj = parsed
        elif isinstance(parsed, dict):
            payload_mode = "dict"
            source_obj = parsed.get("data") if isinstance(parsed.get("data"), dict) else parsed
            sensor_dict = {
                key: valid_number(source_obj.get(key)) for key in EXPECTED_KEYS
            }
        else:
            self.get_logger().warning(
                f"[RX #{self.packet_count:04d}] Tipe JSON tidak didukung: {type(parsed).__name__}"
            )
            return

        self._publish_telemetry(
            sensor_dict=sensor_dict,
            mannequin_id=mannequin_id,
            payload_mode=payload_mode,
            byte_len=byte_len,
            total_frags=total_frags,
            elapsed_ms=elapsed_ms,
            inter_ms=inter_ms,
            rate_hz=rate_hz,
            crc_hex=crc_hex,
            web_v2_obj=web_v2_obj,
        )

    def _publish_radio_status(
        self,
        mannequin_id: int,
        valid_fields: int,
        byte_len: int,
        total_frags: int,
        elapsed_ms: float,
        inter_ms: Optional[float],
        rate_hz: Optional[float],
        payload_mode: str,
    ) -> None:
        status_obj = {
            "mannequin_id": mannequin_id,
            "packet_count": self.packet_count,
            "valid_fields": valid_fields,
            "total_fields": NUM_PARAMS,
            "bytes": byte_len,
            "fragments": total_frags,
            "rssi": self.last_rssi,
            "snr": self.last_snr,
            "freq_hz": self.radio_config.freq,
            "sf": self.radio_config.sf,
            "bw": self.radio_config.bw,
            "cr": self.radio_config.cr,
            "elapsed_ms": round(elapsed_ms, 1),
            "inter_packet_ms": None if inter_ms is None else round(inter_ms, 1),
            "packet_rate_hz": None if rate_hz is None else round(rate_hz, 3),
            "payload_mode": payload_mode,
            "crc_errors": self.crc_error_count,
            "frag_timeouts": self.frag_timeout_count,
            "web_mqtt_sent": self.web_forwarder.mqtt_sent_count,
            "web_mqtt_acks": self.web_forwarder.mqtt_ack_count,
            "web_mqtt_last_ack": self.web_forwarder.last_ack_status,
            "web_http_sent": self.web_forwarder.http_sent_count,
        }
        publish_json(self.status_pub, status_obj)

    def _publish_telemetry(
        self,
        sensor_dict: Dict[str, Optional[float]],
        mannequin_id: int,
        payload_mode: str,
        byte_len: int,
        total_frags: int,
        elapsed_ms: float,
        inter_ms: Optional[float],
        rate_hz: Optional[float],
        crc_hex: str,
        web_v2_obj: Optional[Dict[str, object]] = None,
    ) -> None:
        valid_fields = sum(1 for v in sensor_dict.values() if v is not None)
        self.web_seq = (self.web_seq + 1) & 0x7FFFFFFF

        if web_v2_obj is None:
            web_v2_obj = build_web_v2_payload(
                sensor_dict=sensor_dict,
                mannequin_id=mannequin_id,
                boot_count=self.boot_count,
                seq=self.web_seq,
            )

        # 1. Topic ROS 2 Web Compact V2 (/telemetri/web/v2)
        publish_json(self.web_v2_pub, web_v2_obj, compact=True)

        # 2. KIRIM LANGSUNG KE WEB BACKEND (MQTT Compact V2 & HTTP REST)
        self.web_forwarder.enqueue_telemetry(
            mannequin_id=mannequin_id,
            web_v2_obj=web_v2_obj,
            sensor_dict=sensor_dict,
        )

        # 3. SIMPAN LOKAL DI RASPBERRY PI (~/blackbox2_ws/logs/communication/)
        self.comm_logger.log_communication(
            packet_no=self.packet_count,
            mannequin_id=mannequin_id,
            packet_type="telemetry",
            payload_mode=payload_mode,
            rssi=self.last_rssi,
            snr=self.last_snr,
            radio_cfg=self.radio_config,
            byte_len=byte_len,
            fragments=total_frags,
            reassembly_ms=elapsed_ms,
            inter_packet_ms=inter_ms,
            packet_rate_hz=rate_hz,
            crc_status="OK",
            crc_hex=crc_hex,
            crc_errors_total=self.crc_error_count,
            frag_timeouts_total=self.frag_timeout_count,
            valid_fields=valid_fields,
            total_fields=NUM_PARAMS,
            web_mqtt_sent=self.web_forwarder.mqtt_sent_count,
            web_ack_status=self.web_forwarder.last_ack_status,
            web_http_sent=self.web_forwarder.http_sent_count,
        )
        self.comm_logger.log_telemetry(
            packet_no=self.packet_count,
            mannequin_id=mannequin_id,
            payload_mode=payload_mode,
            rssi=self.last_rssi,
            snr=self.last_snr,
            byte_len=byte_len,
            fragments=total_frags,
            reassembly_ms=elapsed_ms,
            inter_packet_ms=inter_ms,
            valid_fields=valid_fields,
            sensor_dict=sensor_dict,
        )

        # 4. Topic ROS 2 per kelompok sensor (/telemetri/arduino .. /telemetri/witmotion)
        for group_name, group_keys in SENSOR_GROUPS.items():
            group_payload = {key: sensor_dict[key] for key in group_keys}
            publish_json(self.group_pubs[group_name], group_payload)

        # 5. Topic gabungan & status radio
        raw_payload = {
            "mannequin_id": mannequin_id,
            "payload_mode": payload_mode,
            "valid_fields": valid_fields,
            "total_fields": NUM_PARAMS,
            **sensor_dict,
        }
        publish_json(self.raw_pub, raw_payload)
        self._publish_radio_status(
            mannequin_id=mannequin_id,
            valid_fields=valid_fields,
            byte_len=byte_len,
            total_frags=total_frags,
            elapsed_ms=elapsed_ms,
            inter_ms=inter_ms,
            rate_hz=rate_hz,
            payload_mode=payload_mode,
        )

        # 6. Log 1 baris horizontal bersih
        pkt_label = "1Paket" if total_frags == 1 else f"{total_frags}Frg"
        web_info = (
            f"MQTT:{self.web_forwarder.mqtt_sent_count}"
            f"(ACK:{self.web_forwarder.last_ack_status})"
        )
        if self.web_forwarder.config.http_enabled:
            web_info += f" HTTP:{self.web_forwarder.http_sent_count}"

        self.get_logger().info(
            f"[RX #{self.packet_count:04d}] [M{mannequin_id} {payload_mode}] "
            f"[{byte_len}B {pkt_label} {elapsed_ms:.0f}ms] "
            f"[{self.last_rssi}dBm {self.last_snr}dB] [{valid_fields}/{NUM_PARAMS} valid] "
            f"[{web_info}] -> {format_manekin_horizontal(sensor_dict)}"
        )

    def check_frag_timeout(self) -> None:
        dropped = self.assembler.expire_if_stale()
        if dropped is not None:
            received, total = dropped
            self.frag_timeout_count += 1
            self.comm_logger.log_communication(
                packet_no=self.packet_count,
                mannequin_id=self.default_mannequin_id,
                packet_type="frag_timeout",
                payload_mode=f"incomplete_{received}_of_{total}",
                rssi=self.last_rssi,
                snr=self.last_snr,
                radio_cfg=self.radio_config,
                byte_len=0,
                fragments=received,
                reassembly_ms=self.assembler.timeout_sec * 1000.0,
                inter_packet_ms=None,
                packet_rate_hz=None,
                crc_status="FRAG_TIMEOUT",
                crc_hex="NONE",
                crc_errors_total=self.crc_error_count,
                frag_timeouts_total=self.frag_timeout_count,
                valid_fields=0,
                total_fields=NUM_PARAMS,
                web_mqtt_sent=self.web_forwarder.mqtt_sent_count,
                web_ack_status=self.web_forwarder.last_ack_status,
                web_http_sent=self.web_forwarder.http_sent_count,
            )
            self.get_logger().warning(
                f"Fragmen tidak lengkap dalam {self.assembler.timeout_sec}s "
                f"({received}/{total}), dibatalkan."
            )

    def destroy_node(self) -> None:
        self.web_forwarder.close()
        self.comm_logger.close()
        self.radio.close(stop_rx=True)
        super().destroy_node()


def parse_cli_args(argv: Optional[List[str]] = None) -> Tuple[str, List[str]]:
    args = sys.argv if argv is None else argv
    cli_port = DEFAULT_SERIAL_PORT
    filtered = [args[0]]

    for idx, arg in enumerate(args[1:], start=1):
        if arg == "--ros-args":
            filtered.extend(args[idx:])
            break
        if arg.startswith("/dev/") or arg.upper().startswith("COM"):
            cli_port = arg
        else:
            filtered.append(arg)

    return cli_port, filtered


def main(args=None) -> None:
    cli_port, filtered_args = parse_cli_args()
    rclpy.init(args=filtered_args if args is None else args)
    node = LoRaManekin1ReceiverNode(default_port=cli_port)

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