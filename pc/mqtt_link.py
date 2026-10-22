"""MQTT 链路：CustomControl（30 B）上行 / CustomByteBlock（≤300 B）下行。

2026 通信协议里的关键约定：
    服务器 192.168.12.1:3333，客户端固定 192.168.12.2，topic = 指令名，
    载荷是 Protobuf 二进制；CustomControl 会被图传链路转成 0x0311。
本文件只实现用到的两个消息，字段都是 `bytes data = 1`，所以手写 protobuf 即可，
不引 protobuf 运行时。
"""

from __future__ import annotations

import threading
import time
from typing import Callable

import paho.mqtt.client as mqtt

TOPIC_CONTROL = "CustomControl"
TOPIC_BYTE_BLOCK = "CustomByteBlock"


def _varint(n: int) -> bytes:
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        out.append(b | (0x80 if n else 0))
        if not n:
            return bytes(out)


def pb_bytes_field(field: int, data: bytes) -> bytes:
    return bytes([(field << 3) | 2]) + _varint(len(data)) + bytes(data)


def pb_parse_bytes(buf: bytes, field: int = 1) -> bytes | None:
    i = 0
    while i < len(buf):
        tag = 0
        shift = 0
        while i < len(buf):
            b = buf[i]
            i += 1
            tag |= (b & 0x7F) << shift
            if not (b & 0x80):
                break
            shift += 7
        if (tag >> 3) != field or (tag & 7) != 2:
            return None
        ln = 0
        shift = 0
        while i < len(buf):
            b = buf[i]
            i += 1
            ln |= (b & 0x7F) << shift
            if not (b & 0x80):
                break
            shift += 7
        return bytes(buf[i : i + ln])
    return None


class MqttLink:
    def __init__(self, host: str, port: int, client_id: str,
                 on_byte_block: Callable[[bytes], None] | None = None):
        self.host = host
        self.port = port
        self.handlers: dict[str, Callable[[bytes], None]] = {}
        if on_byte_block:
            self.handlers[TOPIC_BYTE_BLOCK] = on_byte_block
        self.connected = threading.Event()
        try:
            from paho.mqtt.enums import CallbackAPIVersion

            self.client = mqtt.Client(CallbackAPIVersion.VERSION2, client_id=client_id)
        except ImportError:  # paho < 2.0
            self.client = mqtt.Client(client_id=client_id)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.published = 0
        self.received = 0
        self.last_error: str | None = None

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        self.connected.set()
        for topic in self.handlers:
            client.subscribe(topic, qos=0)

    def _on_message(self, client, userdata, msg):
        cb = self.handlers.get(msg.topic)
        if cb is None:
            return
        data = pb_parse_bytes(msg.payload, 1)
        if data is None:
            return
        self.received += 1
        cb(data)

    def subscribe(self, topic: str, cb: Callable[[bytes], None]) -> None:
        self.handlers[topic] = cb
        if self.connected.is_set():
            self.client.subscribe(topic, qos=0)

    def start(self, timeout: float = 3.0) -> bool:
        try:
            self.client.connect(self.host, self.port, keepalive=30)
        except OSError as exc:
            self.last_error = str(exc)
            return False
        self.client.loop_start()
        return self.connected.wait(timeout)

    def publish_control(self, data30: bytes) -> bool:
        if len(data30) != 30:
            raise ValueError("CustomControl 必须是 30 字节")
        return self.publish_raw(TOPIC_CONTROL, pb_bytes_field(1, data30), qos=1)

    def publish_telemetry(self, data300: bytes) -> bool:
        return self.publish_raw(TOPIC_BYTE_BLOCK, pb_bytes_field(1, data300), qos=0)

    def publish_raw(self, topic: str, payload: bytes, qos: int = 0) -> bool:
        info = self.client.publish(topic, payload, qos=qos)
        ok = info.rc == mqtt.MQTT_ERR_SUCCESS
        if ok:
            self.published += 1
        return ok

    def stop(self) -> None:
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass


def default_config() -> dict:
    # 2026 协议：服务器固定 IP / 端口；客户端固定 IP 由系统网络配置保证
    return {"host": "192.168.12.1", "port": 3333}


if __name__ == "__main__":  # 手动冒烟：连服务器发 30 字节，等回包
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=default_config()["host"])
    ap.add_argument("--port", type=int, default=default_config()["port"])
    ap.add_argument("--id", default="1")
    a = ap.parse_args()
    link = MqttLink(a.host, a.port, a.id,
                    on_byte_block=lambda d: print("downlink", len(d), d[:8].hex()))
    if not link.start():
        raise SystemExit(f"connect failed: {link.last_error}")
    for i in range(10):
        link.publish_control(bytes([0x11, i] + [0] * 28))
        time.sleep(0.05)
    print("published", link.published, "received", link.received)
    link.stop()
