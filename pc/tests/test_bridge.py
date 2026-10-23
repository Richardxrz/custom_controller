import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import mqtt_link  # noqa: E402
import payload  # noqa: E402
import rmproto  # noqa: E402
import uart_0311_bridge as bridge_mod  # noqa: E402
from mock_broker import MockBroker  # noqa: E402


class TestBridge(unittest.TestCase):
    def test_control_to_0311_and_telemetry_back(self):
        broker = MockBroker()
        fake = bridge_mod.FakeSer()
        bridge = bridge_mod.Bridge(broker.host, broker.port, "1", ser=fake)
        down_got = []
        sub = mqtt_link.MqttLink(broker.host, broker.port, "obs")
        sub.subscribe(mqtt_link.TOPIC_BYTE_BLOCK, lambda d: down_got.append(d))
        try:
            self.assertTrue(bridge.start())
            self.assertTrue(sub.start())

            up = payload.Uplink(seq=3, joints=[1000, 1100, 1200, 1300, 1400, 1500], flags=1)
            data = payload.encode_uplink(up)
            pub = mqtt_link.MqttLink(broker.host, broker.port, "pub")
            self.assertTrue(pub.start())
            pub.publish_control(data)

            t0 = time.time()
            while time.time() - t0 < 2.0 and bridge.sent == 0:
                bridge.step()
                time.sleep(0.005)
            self.assertEqual(bridge.sent, 1)
            parser = rmproto.Parser()
            frames = [f for f in (parser.feed(b) for b in fake.tx) if f is not None]
            self.assertEqual(len(frames), 1)
            self.assertEqual(frames[0].cmd_id, 0x0311)
            self.assertEqual(frames[0].data, data)

            # 臂控回 0x0310 → 桥应转成 CustomByteBlock
            d = payload.Downlink(ver=1, flags=1, vbus_mv=24000)
            frame = rmproto.encode(0x0310, 7, payload.encode_downlink(d))
            fake.rx += frame
            t0 = time.time()
            while time.time() - t0 < 2.0 and not down_got:
                bridge.step()
                time.sleep(0.005)
            self.assertEqual(len(down_got), 1)
            self.assertEqual(payload.decode_downlink(down_got[0]).vbus_mv, 24000)
            pub.stop()
        finally:
            sub.stop()
            bridge.close()
            broker.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
