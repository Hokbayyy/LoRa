#!/usr/bin/env python3
"""
LoRa P2P Receiver Node (ROS2) - RAK3272S (Pasangan Persis lora_txManekin1.py)

Fitur:
  - Sinkron dengan parameter radio lora_txManekin1.py (923.2 MHz, SF6, BW 500 kHz, CR 4/5, Preamble 8).
  - Menerima frame FRG:1/1:...*CRC16 maupun multi-fragmen FRG:i/N:...*CRC16 + validasi CRC16-CCITT.
  - Mendekode 3 jenis payload dari lora_txManekin1.py:
      1. Array JSON 109 parameter sensor (payload_mode == "array", mendukung nilai null)
      2. Web Compact V2 JSON (payload_mode == "web_v2", readingId 1001..1095 / 2001..2095)
      3. Alert kompak 12-byte ASCII ("A1" + seq + severity + event + status + offline_mask)
  - Mempublikasikan ke topic ROS2:
      /telemetri/web/v2, /system/alerts, /telemetri/received_raw, /telemetri/radio_status,
      serta /telemetri/{arduino,aurora,loadcell,flex,mcp,fsr,piezo,thermal,witmotion}.
  - Menampilkan log terminal 1 baris horizontal bersih per paket tanpa emoji.
"""

import json
import sys
import time
from typing import Dict, List, Optional, Tuple

import serial

import rclpy
from rclpy.node import Node

from lora_common import (
    FragmentAssembler,
    RakP2PRadio,
    crc16_ccitt,
    declare_radio_config,
    parse_rx_event,
    publish_json,
    valid_number,
)


DEFAULT_SERIAL_PORT = "/dev/ttyAMA0"
FALLBACK_PORTS = ("/dev/ttyAMA0", "/dev/serial0", "/dev/ttyAMA3", "/dev/ttyUSB0")


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
# MAPPING WEB COMPACT V2 & COMPACT ALERT (IDENTIK TX MANEKIN 1)
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
# HELPER DECODING & FORMATTING
# ============================================================

def split_and_verify_crc(complete_data: bytes) -> bytes:
    """
    Memeriksa akhiran CRC16-CCITT (*XXXX) bila tersedia dan mengembalikan payload bersih.

    Melempar ValueError jika checksum *XXXX ada tetapi tidak cocok.
    """
    if b"*" not in complete_data:
        return complete_data

    candidate_body, crc_bytes = complete_data.rsplit(b"*", 1)
    crc_text = crc_bytes.strip()
    if len(crc_text) != 4:
        return complete_data

    try:
        expected_crc = int(crc_text.decode("ascii"), 16)
    except (ValueError, UnicodeDecodeError):
        return complete_data

    actual_crc = crc16_ccitt(candidate_body)
    if actual_crc != expected_crc:
        raise ValueError(
            f"CRC16 mismatch (dihitung {actual_crc:04X}, diterima {expected_crc:04X})"
        )
    return candidate_body


def decode_compact_alert(text: str) -> Dict[str, object]:
    """Mendekode alert kompak 12-byte A1<seq:4X><sev:1X><event:2X><status:1X><mask:2X>."""
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
    """Memetakan list angka/null dari lora_txManekin1.py ke dictionary 109 parameter."""
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
    """Memetakan payload JSON Web Compact V2 {"v":2,"r":[[id,val],...]} ke 109 key."""
    sensor_dict: Dict[str, Optional[float]] = {key: None for key in EXPECTED_KEYS}
    readings = payload_obj.get("r", [])

    inferred_id = default_mannequin_id
    for entry in readings:
        if not isinstance(entry, (list, tuple)) or len(entry) != 2:
            continue
        raw_id, raw_val = entry
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
    """Membangun JSON Web Compact V2 yang identik dengan build_web_v2_packet() TX."""
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
    """Memformat seluruh 109 parameter telemetry lora_txManekin1.py menjadi 1 baris."""
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
# NODE ROS2 RECEIVER MANEKIN 1
# ============================================================

class LoRaManekin1ReceiverNode(Node):
    """Node ROS2 receiver LoRa P2P yang sesuai penuh dengan lora_txManekin1.py."""

    def __init__(self, default_port: str = DEFAULT_SERIAL_PORT):
        super().__init__("lora_rx_manekin1_node")

        radio_config = declare_radio_config(self, default_port)
        self.declare_parameter("frag_timeout", 3.0)
        self.declare_parameter("mannequin_id", 1)
        self.declare_parameter("boot_count", 1)

        frag_timeout = float(self.get_parameter("frag_timeout").value)
        self.default_mannequin_id = int(self.get_parameter("mannequin_id").value)
        self.boot_count = int(self.get_parameter("boot_count").value)

        self.packet_count = 0
        self.alert_count = 0
        self.crc_error_count = 0
        self.web_seq = 0
        self.last_rssi = "-"
        self.last_snr = "-"
        self.assembler = FragmentAssembler(frag_timeout)

        # Publisher utama (Web Compact V2, Alerts, Status, Raw, dan 9 group sensor)
        self.web_v2_pub = self.create_publisher(
            self._string_msg_cls(), "/telemetri/web/v2", 10
        )
        self.alert_pub = self.create_publisher(
            self._string_msg_cls(), "/system/alerts", 10
        )
        self.raw_pub = self.create_publisher(
            self._string_msg_cls(), "/telemetri/received_raw", 10
        )
        self.status_pub = self.create_publisher(
            self._string_msg_cls(), "/telemetri/radio_status", 10
        )
        self.group_pubs = {
            group_name: self.create_publisher(
                self._string_msg_cls(), f"/telemetri/{group_name}", 10
            )
            for group_name in SENSOR_GROUPS
        }

        self.radio = RakP2PRadio(
            config=radio_config,
            fallback_ports=FALLBACK_PORTS,
            logger=self.get_logger(),
            rx_mode=True,
        )
        self.radio.connect()

        self.create_timer(0.005, self.read_serial_data)
        self.create_timer(0.5, self.check_frag_timeout)

        self.get_logger().info(
            f"LoRa Manekin Receiver started on {self.radio.active_port} "
            f"@ {radio_config.baudrate} | Freq:{radio_config.freq} SF{radio_config.sf} "
            f"BW{radio_config.bw} CR{radio_config.cr} | {NUM_PARAMS} params "
            f"(array / web_v2 / alert)"
        )

    @staticmethod
    def _string_msg_cls():
        from std_msgs.msg import String
        return String

    def read_serial_data(self) -> None:
        """Membaca event +EVT:RXP2P dari RAK3272S melalui serial."""
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
        """Membuka framing FRG (bila ada) dan meneruskan payload utuh ke decoder."""
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
        """Memverifikasi CRC16-CCITT dan merutekan Alert, Array JSON, atau Web V2 JSON."""
        self.packet_count += 1

        try:
            payload_bytes = split_and_verify_crc(complete_data)
        except ValueError as exc:
            self.crc_error_count += 1
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
            self._handle_alert(text, len(complete_data), total_frags, elapsed_ms)
            return

        self._handle_json_payload(text, len(complete_data), total_frags, elapsed_ms)

    def _handle_alert(
        self, text: str, byte_len: int, total_frags: int, elapsed_ms: float
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
        self._publish_radio_status(
            mannequin_id=self.default_mannequin_id,
            valid_fields=0,
            byte_len=byte_len,
            total_frags=total_frags,
            elapsed_ms=elapsed_ms,
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
        self, text: str, byte_len: int, total_frags: int, elapsed_ms: float
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
            web_v2_obj=web_v2_obj,
        )

    def _publish_radio_status(
        self,
        mannequin_id: int,
        valid_fields: int,
        byte_len: int,
        total_frags: int,
        elapsed_ms: float,
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
            "elapsed_ms": round(elapsed_ms, 1),
            "payload_mode": payload_mode,
            "crc_errors": self.crc_error_count,
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

        # 1. Topic Web Compact V2 (/telemetri/web/v2) untuk backend web
        publish_json(self.web_v2_pub, web_v2_obj, compact=True)

        # 2. Topic per kelompok sensor (/telemetri/arduino .. /telemetri/witmotion)
        for group_name, group_keys in SENSOR_GROUPS.items():
            group_payload = {key: sensor_dict[key] for key in group_keys}
            publish_json(self.group_pubs[group_name], group_payload)

        # 3. Topic gabungan & status radio
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
            payload_mode=payload_mode,
        )

        # 4. Log 1 baris horizontal bersih
        pkt_label = "1Paket" if total_frags == 1 else f"{total_frags}Frg"
        self.get_logger().info(
            f"[RX #{self.packet_count:04d}] [M{mannequin_id} {payload_mode}] "
            f"[{byte_len}B {pkt_label} {elapsed_ms:.0f}ms] "
            f"[{self.last_rssi}dBm {self.last_snr}dB] [{valid_fields}/{NUM_PARAMS} valid] -> "
            f"{format_manekin_horizontal(sensor_dict)}"
        )

    def check_frag_timeout(self) -> None:
        dropped = self.assembler.expire_if_stale()
        if dropped is not None:
            received, total = dropped
            self.get_logger().warning(
                f"Fragmen tidak lengkap dalam {self.assembler.timeout_sec}s "
                f"({received}/{total}), dibatalkan."
            )

    def destroy_node(self) -> None:
        self.radio.close(stop_rx=True)
        super().destroy_node()


def parse_cli_args(argv: Optional[List[str]] = None) -> Tuple[str, List[str]]:
    """Memisahkan argumen serial port (/dev/...) sebelum diteruskan ke rclpy."""
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
