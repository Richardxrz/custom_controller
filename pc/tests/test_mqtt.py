import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import mqtt_link  # noqa: E402
from mock_broker import MockBroker  # noqa: E402


class TestMqtt(unittest.TestCase):
    def setUp(self):
        self.broker = MockBroker()

    def tearDown(self):
        self.broker.close()

    def test_pb_roundtrip(self):
        data = bytes(range(30))
        enc = mqtt_link.pb_bytes_field(1, data)
        self.assertEqual(mqtt_link.pb_parse_bytes(enc, 1), data)

    def test_publish_and_receive(self):
        got = []
        sub = mqtt_link.MqttLink(self.broker.host, self.broker.port, "sub")
        sub.subscribe(mqtt_link.TOPIC_CONTROL, lambda d: got.append(d))
        pub = mqtt_link.MqttLink(self.broker.host, self.broker.port, "pub")
        self.assertTrue(sub.start())
        self.assertTrue(pub.start())

        data = bytes(range(30))
        self.assertTrue(pub.publish_control(data))

        t0 = time.time()
        while time.time() - t0 < 2.0 and not got:
            time.sleep(0.01)
        self.assertEqual(len(got), 1)
        self.assertEqual(got[0], data)

        sub.stop()
        pub.stop()

    def test_qos1_puback(self):
        """QoS1 发布后 broker 要回 PUBACK，否则 paho 会重发/卡住。"""
        pub = mqtt_link.MqttLink(self.broker.host, self.broker.port, "pub2")
        self.assertTrue(pub.start())
        for _ in range(10):
            self.assertTrue(pub.publish_control(bytes(30)))
        time.sleep(0.3)
        self.assertGreaterEqual(pub.published, 10)
        pub.stop()


if __name__ == "__main__":
    unittest.main(verbosity=2)
