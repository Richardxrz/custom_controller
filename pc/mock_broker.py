"""最小 MQTT 3.1.1 broker，只用于本机回环测试。

支持 CONNECT / SUBSCRIBE / PUBLISH(QoS0/1) / PINGREQ / DISCONNECT，
订阅按完全匹配或 '#' 通配；不做保留消息、遗嘱、认证。
"""

from __future__ import annotations

import socket
import threading


def _varint(n: int) -> bytes:
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        out.append(b | (0x80 if n else 0))
        if not n:
            return bytes(out)


class MockBroker:
    def __init__(self, host: str = "127.0.0.1", port: int = 0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.listen(8)
        self.host, self.port = self.sock.getsockname()[:2]
        self.clients: list[tuple[socket.socket, set[str]]] = []
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._accept_loop, daemon=True)
        self.thread.start()

    # ---- 生命周期 ----
    def close(self) -> None:
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass
        with self.lock:
            for c, _ in self.clients:
                try:
                    c.close()
                except OSError:
                    pass
            self.clients.clear()

    def _accept_loop(self) -> None:
        while self.running:
            try:
                conn, _ = self.sock.accept()
            except OSError:
                return
            threading.Thread(target=self._client_loop, args=(conn,), daemon=True).start()

    # ---- 收发工具 ----
    @staticmethod
    def _recv_exact(conn: socket.socket, n: int) -> bytes | None:
        buf = b""
        while len(buf) < n:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    @staticmethod
    def _send(conn: socket.socket, data: bytes) -> None:
        try:
            conn.sendall(data)
        except OSError:
            pass

    def _client_loop(self, conn: socket.socket) -> None:
        subs: set[str] = set()
        with self.lock:
            self.clients.append((conn, subs))
        try:
            while self.running:
                hdr = self._recv_exact(conn, 1)
                if hdr is None:
                    break
                b1 = hdr[0]
                mult, rem = 1, 0
                while True:
                    b = self._recv_exact(conn, 1)
                    if b is None:
                        return
                    rem += (b[0] & 0x7F) * mult
                    if not (b[0] & 0x80):
                        break
                    mult *= 128
                body = self._recv_exact(conn, rem) if rem else b""
                if body is None:
                    break
                ptype = b1 >> 4
                if ptype == 1:  # CONNECT
                    self._send(conn, bytes([0x20, 0x02, 0x00, 0x00]))
                elif ptype == 8:  # SUBSCRIBE
                    pid = body[0:2]
                    i = 2
                    granted = b""
                    while i + 2 <= len(body):
                        ln = int.from_bytes(body[i : i + 2], "big")
                        topic = body[i + 2 : i + 2 + ln].decode()
                        i += 2 + ln + 1
                        subs.add(topic)
                        granted += b"\x00"
                    self._send(conn, bytes([0x90, 2 + len(granted)]) + pid + granted)
                elif ptype == 3:  # PUBLISH
                    qos = (b1 >> 1) & 0x03
                    ln = int.from_bytes(body[0:2], "big")
                    topic = body[2 : 2 + ln].decode()
                    i = 2 + ln
                    pid = b""
                    if qos:
                        pid = body[i : i + 2]
                        i += 2
                    data = body[i:]
                    if qos == 1:
                        self._send(conn, bytes([0x40, 0x02]) + pid)
                    self._forward(topic, data)
                elif ptype == 12:  # PINGREQ
                    self._send(conn, bytes([0xD0, 0x00]))
                elif ptype == 14:  # DISCONNECT
                    break
        finally:
            with self.lock:
                self.clients = [(c, s) for (c, s) in self.clients if c is not conn]
            try:
                conn.close()
            except OSError:
                pass

    def _forward(self, topic: str, data: bytes) -> None:
        t = topic.encode()
        rem = 2 + len(t) + len(data)
        pkt = bytes([0x30]) + _varint(rem) + len(t).to_bytes(2, "big") + t + data
        with self.lock:
            targets = [c for (c, s) in self.clients if topic in s or "#" in s]
        for c in targets:
            self._send(c, pkt)
