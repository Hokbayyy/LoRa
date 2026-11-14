#!/usr/bin/env python3
"""
LoRa P2P Transmitter Node (ROS2) - RAK3272S (Multi-Node 99 Parameter Terkompresi)

Mendukung banyak node pengirim (Node 1, Node 2, dst.) menggunakan 1 file yang sama.
Cara menjalankan:
  - Node 1: python3 lora_transmitter_node.py 1   (atau tanpa argumen / -p node_id:=1)
  - Node 2: python3 lora_transmitter_node.py 2   (atau -p node_id:=2)

Seluruh 99 parameter dipadatkan menjadi 1 paket biner P99 tepat 207 Byte (414 hex)
dan dikirim 1 kali per siklus tanpa fragmen.
"""

from collections import deque
import json
import random
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from lora_common import (
    AURORA_ALIASES,
    DUMMY_VALUES,
    EXPECTED_KEYS,
    RakP2PRadio,
    declare_radio_config,
    encode_p99_packet,
    format_horizontal,
    to_float16_safe,
    valid_number,
)


DEFAULT_SERIAL_PORT = "/dev/ttyAMA3"
FALLBACK_PORTS = ("/dev/ttyAMA3", "/dev/serial0", "/dev/ttyUSB0")
DEFAULT_NODE_ID = 1
DEFAULT_TRANSMIT_INTERVAL = 1.0
HOLD_LAST_VALUE = True
ENABLE_DUMMY_DATA = True


def generate_dummy_values() -> Dict[str, float]:
    """Menghasilkan nilai simulasi bervariasi +-1% di sekitar nilai dasar sensor."""
    dummy: Dict[str, float] = {}
    for key in EXPECTED_KEYS:
        base = DUMMY_VALUES[key]
        variation = abs(base) * 0.01 or 0.01
        dummy[key] = round(random.uniform(base - variation, base + variation), 3)
    return dummy


def parse_json_dict(raw_data: str) -> Optional[Dict[str, object]]:
    """Mem-parse string JSON menjadi dictionary; mengembalikan None jika gagal."""
    try:
        data = json.loads(raw_data)
    except (json.JSONDecodeError, TypeError):
        return None
    return data if isinstance(data, dict) else None


class LoRaTransmitterNode(Node):
    """Node ROS2 yang membaca telemetri sensor dan memancarkannya via LoRa P2P."""

    def __init__(
        self,
        default_node_id: int = DEFAULT_NODE_ID,
        default_port: str = DEFAULT_SERIAL_PORT,
    ):
        super().__init__(f"lora_transmitter_node_{default_node_id}")

        self.declare_parameter("node_id", default_node_id)
        self.declare_parameter("interval", DEFAULT_TRANSMIT_INTERVAL)
        self.declare_parameter("global_topics", True)

        self.node_id = max(1, min(255, int(self.get_parameter("node_id").value)))
        self.interval = max(0.15, float(self.get_parameter("interval").value))
        self.global_topics = bool(self.get_parameter("global_topics").value)
        radio_config = declare_radio_config(self, default_port)

        self.lock = threading.Lock()
        self.latest_values: Dict[str, float] = {}
        self.last_values: Dict[str, float] = {}
        self.tx_queue: deque = deque(maxlen=1)
        self.tx_active = False
        self.packet_counter = 0

        self.radio = RakP2PRadio(
            config=radio_config,
            fallback_ports=FALLBACK_PORTS,
            logger=self.get_logger(),
            rx_mode=False,
        )

        self._subscriptions_list = self._init_subscriptions()
        self.radio.connect()
        self._apply_initial_phase_offset()

        self.transmit_timer = self.create_timer(self.interval, self.transmit_callback)
        self.queue_timer = self.create_timer(0.02, self.process_queue)

        self.get_logger().info(
            f"LoRa transmitter NODE {self.node_id} started on {self.radio.active_port} "
            f"@ {radio_config.baudrate} (99 param compressed 207B, interval {self.interval}s)"
        )

    def _init_subscriptions(self) -> list:
        topic_specs = (
            ("arduino", self.arduino_callback),
            ("aurora", self.aurora_callback),
            ("suit", self.arduino_callback),
        )
        prefixes = [f"/node{self.node_id}/telemetri"]
        if self.global_topics:
            prefixes.append("/telemetri")

        subs = []
        for prefix in prefixes:
            for suffix, callback in topic_specs:
                subs.append(
                    self.create_subscription(String, f"{prefix}/{suffix}", callback, 10)
                )
        return subs

    def _apply_initial_phase_offset(self) -> None:
        """Menggeser fase awal Node > 1 agar dua node tidak memancar serempak."""
        if self.node_id <= 1:
            return
        phase_delay = ((self.node_id - 1) * 0.45) % self.interval
        if phase_delay > 0.01:
            self.get_logger().info(
                f"Node {self.node_id}: offset waktu awal {phase_delay:.2f}s untuk cegah tabrakan radio."
            )
            time.sleep(phase_delay)

    def _handle_sensor_message(
        self,
        msg: String,
        source_name: str,
        aliases: Optional[Dict[str, str]] = None,
    ) -> None:
        data = parse_json_dict(msg.data)
        if data is None:
            self.get_logger().warning(f"{source_name} message is not valid JSON object.")
            return
        self.update_values(data, aliases=aliases)

    def arduino_callback(self, msg: String) -> None:
        self._handle_sensor_message(msg, "Arduino")

    def aurora_callback(self, msg: String) -> None:
        self._handle_sensor_message(msg, "Aurora", aliases=AURORA_ALIASES)

    def update_values(
        self,
        data: Dict[str, object],
        aliases: Optional[Dict[str, str]] = None,
    ) -> None:
        """Memperbarui nilai terbaru dan nilai terakhir yang valid untuk setiap sensor."""
        if not isinstance(data, dict):
            return
        aliases = aliases or {}

        with self.lock:
            for original_key, raw_value in data.items():
                key = aliases.get(original_key, original_key)
                if key not in EXPECTED_KEYS:
                    continue
                number = valid_number(raw_value)
                if number is None:
                    continue
                self.latest_values[key] = number
                self.last_values[key] = number

    def _resolve_value(self, key: str, dummy_values: Dict[str, float]) -> float:
        """Memilih nilai sensor aktif: terbaru -> tahan terakhir -> dummy -> 0.0."""
        if key in self.latest_values:
            raw = self.latest_values[key]
        elif HOLD_LAST_VALUE and key in self.last_values:
            raw = self.last_values[key]
        elif ENABLE_DUMMY_DATA:
            raw = dummy_values[key]
        else:
            return 0.0
        return to_float16_safe(raw)

    def build_compressed_packet(self) -> Tuple[bytes, int, Dict[str, float]]:
        """Membangun 1 paket biner P99 (207 Byte) berisi seluruh 99 parameter sensor."""
        dummy_values = generate_dummy_values()
        values: List[float] = []
        snapshot: Dict[str, float] = {}

        with self.lock:
            for key in EXPECTED_KEYS:
                safe_num = self._resolve_value(key, dummy_values)
                values.append(safe_num)
                snapshot[key] = round(safe_num, 3)

            self.packet_counter += 1
            cnt = self.packet_counter

        packet = encode_p99_packet(self.node_id, cnt, values)
        return packet, cnt, snapshot

    def transmit_callback(self) -> None:
        """Menyiapkan paket terbaru ke antrean TX sesuai interval timer."""
        if not self.radio.is_connected:
            self.radio.reconnect_if_due()
            return

        try:
            packet, cnt, snapshot = self.build_compressed_packet()
            with self.lock:
                self.tx_queue.append((packet, cnt, snapshot))
        except Exception as error:
            self.get_logger().error(f"Packet creation failed: {error}")

    def process_queue(self) -> None:
        """Mengirim satu paket dari antrean via AT+PSEND dan mencatat log 1 baris."""
        if not self.radio.is_connected or self.tx_active or not self.tx_queue:
            return

        with self.lock:
            if not self.tx_queue:
                return
            packet, cnt, snapshot = self.tx_queue.popleft()

        self.tx_active = True
        # Micro-jitter 0-40 ms agar node pengirim tidak terus bertabrakan bila clock bergeser
        time.sleep(random.uniform(0.0, 0.04))
        t_start = time.monotonic()

        try:
            success, err_msg = self.radio.transmit_hex(packet.hex().upper(), timeout=2.0)
            elapsed_ms = (time.monotonic() - t_start) * 1000.0
            if success:
                self.get_logger().info(
                    f"[TX NODE {self.node_id} #{cnt:04d}] [{len(packet)}B 1Paket "
                    f"OK {elapsed_ms:.0f}ms] -> {format_horizontal(snapshot)}"
                )
            else:
                self.get_logger().warning(
                    f"[TX NODE {self.node_id} #{cnt:04d}] [{len(packet)}B 1Paket GAGAL: {err_msg}]"
                )
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f"Serial TX error: {exc}")
            self.radio.reconnect_if_due()
        except Exception as error:
            self.get_logger().error(f"LoRa transmission error: {error}")
        finally:
            self.tx_active = False

    def destroy_node(self) -> None:
        self.radio.close(stop_rx=False)
        super().destroy_node()


def parse_cli_args(argv: Optional[List[str]] = None) -> Tuple[int, str, List[str]]:
    """Memisahkan argumen sederhana CLI (node_id dan /dev/tty*) sebelum diteruskan ke rclpy."""
    args = sys.argv if argv is None else argv
    cli_node_id = DEFAULT_NODE_ID
    cli_port = DEFAULT_SERIAL_PORT
    filtered = [args[0]]

    for idx, arg in enumerate(args[1:], start=1):
        if arg == "--ros-args":
            filtered.extend(args[idx:])
            break
        if arg.startswith("/dev/"):
            cli_port = arg
        elif arg.isdigit() and 1 <= int(arg) <= 255:
            cli_node_id = int(arg)
        else:
            filtered.append(arg)

    return cli_node_id, cli_port, filtered


def main(args=None) -> None:
    cli_node_id, cli_port, filtered_args = parse_cli_args()
    rclpy.init(args=filtered_args if args is None else args)
    node = LoRaTransmitterNode(default_node_id=cli_node_id, default_port=cli_port)

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
