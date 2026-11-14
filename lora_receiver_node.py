#!/usr/bin/env python3
"""
LoRa P2P Multi-Node Receiver Node (ROS2) - RAK3272S (1 File untuk Banyak Transmitter)

Pasangan persis dari lora_transmitter_node.py:
  - Menerima paket terkompresi biner P99 (207B multi-node / 206B legacy) dan fallback FRG/JSON.
  - Memisahkan topic ROS2 otomatis per node (/node1/telemetri/..., /node2/telemetri/...).
  - Menyediakan topic gabungan (/telemetri/received_raw dan /telemetri/radio_status).
  - Menampilkan log terminal 1 baris horizontal bersih per paket tanpa emoji.
"""

import json
import time
from typing import Dict, Optional, Tuple

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from lora_common import (
    EXPECTED_KEYS,
    NUM_PARAMS,
    PACKET_MAGIC,
    PACKET_SIZE_MULTINODE,
    SENSOR_GROUPS,
    RakP2PRadio,
    crc16_ccitt,
    declare_radio_config,
    decode_p99_packet,
    format_horizontal,
    parse_rx_event,
)


DEFAULT_SERIAL_PORT = "/dev/ttyAMA0"
FALLBACK_PORTS = ("/dev/ttyAMA0", "/dev/serial0", "/dev/ttyAMA3", "/dev/ttyUSB0")


def publish_json(publisher, payload: object) -> None:
    """Mengemas object Python ke std_msgs/String JSON dan mempublikasikannya."""
    msg = String()
    msg.data = json.dumps(payload)
    publisher.publish(msg)


class FragmentAssembler:
    """Menangani reassembly paket legacy berheader FRG:index/total secara aman."""

    def __init__(self, timeout_sec: float):
        self.timeout_sec = timeout_sec
        self._chunks: Dict[int, bytes] = {}
        self._total: int = 0
        self._start_time: float = 0.0

    def add_fragment(self, raw_bytes: bytes) -> Optional[Tuple[bytes, int, float]]:
        """Menambahkan fragmen FRG; mengembalikan (complete_data, total, elapsed_ms) jika utuh."""
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
        """Membersihkan fragmen menggantung bila melewati batas timeout."""
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


class LoRaReceiverNode(Node):
    """Node ROS2 receiver LoRa P2P yang melayani multi-transmitter dalam 1 file."""

    def __init__(self, default_port: str = DEFAULT_SERIAL_PORT):
        super().__init__("lora_receiver_node")

        radio_config = declare_radio_config(self, default_port)
        self.declare_parameter("frag_timeout", 3.0)
        self.declare_parameter("legacy_global_node_id", 1)

        frag_timeout = float(self.get_parameter("frag_timeout").value)
        self.legacy_global_node_id = int(
            self.get_parameter("legacy_global_node_id").value
        )

        self.packet_count = 0
        self.node_packet_counts: Dict[int, int] = {}
        self.last_rssi = "-"
        self.last_snr = "-"
        self.assembler = FragmentAssembler(frag_timeout)

        # Publisher global & per-node menggunakan helper yang seragam
        self.global_pubs = self._create_topic_bundle("/telemetri")
        self.node_publishers: Dict[int, Dict[str, object]] = {}
        self.get_or_create_node_publishers(1)
        self.get_or_create_node_publishers(2)

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
            f"LoRa multi-node receiver started on {self.radio.active_port} "
            f"@ {radio_config.baudrate} (Node 1, Node 2, etc. | {NUM_PARAMS} params / "
            f"{PACKET_SIZE_MULTINODE}B)"
        )

    def _create_topic_bundle(self, prefix: str) -> Dict[str, object]:
        topic_names = tuple(SENSOR_GROUPS.keys()) + ("received_raw", "radio_status")
        return {
            name: self.create_publisher(String, f"{prefix}/{name}", 10)
            for name in topic_names
        }

    def get_or_create_node_publishers(self, node_id: int) -> Dict[str, object]:
        if node_id not in self.node_publishers:
            self.node_publishers[node_id] = self._create_topic_bundle(
                f"/node{node_id}/telemetri"
            )
        return self.node_publishers[node_id]

    def read_serial_data(self) -> None:
        """Membaca event RXP2P dari port serial modul RAK3272S."""
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
        """Merutekan payload biner P99, fragmen FRG, atau JSON mentah."""
        self.last_rssi = rssi or "-"
        self.last_snr = snr or "-"

        if raw_bytes.startswith(PACKET_MAGIC):
            self.process_compressed_p99(raw_bytes)
            return

        if raw_bytes.startswith(b"FRG:"):
            try:
                assembled = self.assembler.add_fragment(raw_bytes)
            except Exception as exc:
                self.get_logger().error(f"Gagal memproses fragmen: {exc}")
                return
            if assembled is not None:
                complete_data, total_frags, elapsed_ms = assembled
                self.process_json_payload(complete_data, total_frags, elapsed_ms)
            return

        self.process_json_payload(raw_bytes, total_frags=1, elapsed_ms=0.0)

    def process_compressed_p99(self, raw_bytes: bytes) -> None:
        """Memverifikasi dan mendekode paket biner terkompresi P99."""
        self.packet_count += 1
        try:
            node_id, tx_counter, sensor_dict = decode_p99_packet(raw_bytes)
        except ValueError as exc:
            self.get_logger().warning(f"[RX #{self.packet_count}] {exc}. Paket dibuang.")
            return

        self.publish_and_log(
            sensor_dict=sensor_dict,
            byte_len=len(raw_bytes),
            total_frags=1,
            elapsed_ms=0.0,
            node_id=node_id,
            tx_counter=tx_counter,
        )

    def process_json_payload(
        self, complete_data: bytes, total_frags: int, elapsed_ms: float
    ) -> None:
        """Memverifikasi opsional CRC16 dan mem-parse fallback JSON list."""
        self.packet_count += 1
        json_bytes = complete_data

        if b"*" in complete_data:
            candidate_json, crc_bytes = complete_data.rsplit(b"*", 1)
            try:
                expected_crc = int(crc_bytes.decode("ascii"), 16)
            except ValueError:
                expected_crc = None

            if expected_crc is not None:
                if crc16_ccitt(candidate_json) != expected_crc:
                    self.get_logger().warning(
                        f"[RX #{self.packet_count}] CRC MISMATCH JSON!"
                    )
                    return
                json_bytes = candidate_json

        try:
            values = json.loads(json_bytes.decode("utf-8"))
        except Exception as err:
            self.get_logger().warning(f"Gagal parse JSON ({err}): {json_bytes[:60]}")
            return

        if not isinstance(values, list):
            self.get_logger().warning(f"Format payload bukan list angka: {type(values)}")
            return

        sensor_dict = dict(zip(EXPECTED_KEYS, values))
        self.publish_and_log(
            sensor_dict=sensor_dict,
            byte_len=len(complete_data),
            total_frags=total_frags,
            elapsed_ms=elapsed_ms,
            node_id=1,
        )

    def publish_and_log(
        self,
        sensor_dict: Dict[str, float],
        byte_len: int,
        total_frags: int,
        elapsed_ms: float,
        node_id: int = 1,
        tx_counter: Optional[int] = None,
    ) -> None:
        """Mencatat 1 baris log horizontal dan menerbitkan data ke topic ROS2 per node."""
        node_pkt_cnt = self.node_packet_counts.get(node_id, 0) + 1
        self.node_packet_counts[node_id] = node_pkt_cnt

        grouped_payloads = {
            group_name: {
                k: sensor_dict.get(k, 0.0) if group_name == "thermal" else sensor_dict[k]
                for k in group_keys
                if group_name == "thermal" or k in sensor_dict
            }
            for group_name, group_keys in SENSOR_GROUPS.items()
        }

        raw_with_node = {"node_id": node_id, **sensor_dict}
        status_obj = {
            "node_id": node_id,
            "packet_count": self.packet_count,
            "node_packet_count": node_pkt_cnt,
            "tx_counter": tx_counter,
            "bytes": byte_len,
            "fragments": total_frags,
            "rssi": self.last_rssi,
            "snr": self.last_snr,
            "elapsed_ms": round(elapsed_ms, 1),
        }

        pkt_label = "1Paket" if total_frags == 1 else f"{total_frags}Frg"
        cnt_info = f"Cnt:{tx_counter}" if tx_counter is not None else f"{elapsed_ms:.0f}ms"
        self.get_logger().info(
            f"[RX #{self.packet_count:04d}] [NODE {node_id} #{node_pkt_cnt:04d}] "
            f"[{byte_len}B {pkt_label} {cnt_info}] [{self.last_rssi}dBm {self.last_snr}dB] "
            f"[{len(sensor_dict)}/{NUM_PARAMS} param] -> {format_horizontal(sensor_dict)}"
        )

        # 1. Topic terpisah per node (/node1/telemetri/..., /node2/telemetri/...)
        node_pubs = self.get_or_create_node_publishers(node_id)
        for group_name, payload in grouped_payloads.items():
            publish_json(node_pubs[group_name], payload)
        publish_json(node_pubs["received_raw"], raw_with_node)
        publish_json(node_pubs["radio_status"], status_obj)

        # 2. Topic gabungan seluruh node
        publish_json(self.global_pubs["received_raw"], raw_with_node)
        publish_json(self.global_pubs["radio_status"], status_obj)

        # 3. Topic legacy (/telemetri/arduino, dll.) khusus node default
        if node_id == self.legacy_global_node_id:
            for group_name, payload in grouped_payloads.items():
                publish_json(self.global_pubs[group_name], payload)

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


def main(args=None) -> None:
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
