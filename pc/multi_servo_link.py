"""两条（或更多）ZL 总线并行轮询，合并成一份 30 B 上行；3+3 约 55 Hz。"""

from __future__ import annotations

import threading
import time

import payload
import rmproto


class _Worker(threading.Thread):
    def __init__(self, link, base, joints, stamps, lock, updated):
        super().__init__(daemon=True)
        self.link, self.base = link, base
        self.joints, self.stamps, self.lock = joints, stamps, lock
        self.updated = updated
        self.ids = list(link.ids)
        self._stop = False

    def run(self):
        while not self._stop:
            for i, sid in enumerate(self.ids):
                v = self.link.read_angle(sid)
                if v is None:
                    continue
                with self.lock:
                    self.joints[self.base + i] = v
                    self.stamps[self.base + i] = time.monotonic()
                    self.updated[0] += 1

    def stop(self):
        self._stop = True


class MultiServoLink:
    def __init__(self, specs=None, links=None, publish_hz=75.0, stale_s=0.25):
        if links is None:
            from servo_link import ServoLink

            links = [ServoLink(p, ids) for p, ids in (specs or [])]
        self.links = links
        self.publish_hz = publish_hz
        self.stale_s = stale_s
        self.joints = [1500] * payload.JOINTS
        self.stamps = [0.0] * payload.JOINTS
        self.lock = threading.Lock()
        self.updated = [0]
        self.workers = []
        base = 0
        for link in links:
            self.workers.append(_Worker(link, base, self.joints, self.stamps,
                                        self.lock, self.updated))
            base += len(link.ids)
        self.seq = 0
        self._last_pub = 0.0

    def start(self):
        for w in self.workers:
            w.start()

    def poll(self):
        now = time.monotonic()
        if now - self._last_pub < 1.0 / self.publish_hz:
            return []
        with self.lock:
            if self.updated[0] == 0:
                return []
            self.updated[0] = 0
            joints = list(self.joints)
            valid = all(now - t <= self.stale_s for t in self.stamps)
        self._last_pub = now
        up = payload.Uplink(seq=self.seq, joints=joints, flags=0x01 if valid else 0x00)
        frame = rmproto.Frame(cmd_id=0x0302, seq=self.seq,
                              data=payload.encode_uplink(up))
        self.seq = (self.seq + 1) & 0xFF
        return [frame]

    def close(self):
        for w in self.workers:
            w.stop()
        for w in self.workers:
            w.join(timeout=0.5)
        for link in self.links:
            link.close()
