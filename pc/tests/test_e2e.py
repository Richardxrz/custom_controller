import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import mqtt_link  # noqa: E402
import robot_mock  # noqa: E402
import terminal_app  # noqa: E402
from mock_broker import MockBroker  # noqa: E402


class TestEndToEnd(unittest.TestCase):
    """终端（模拟主端）→ broker → 机器人替身 → 回传 → 终端。"""

    def test_loop(self):
        broker = MockBroker()
        robot = robot_mock.RobotMock(broker.host, broker.port, "robot-1")
        app = terminal_app.TerminalApp(
            terminal_app.SimLink(hz=100.0),
            mqtt_link.MqttLink(broker.host, broker.port, "1"),
            hz=50.0,
        )
        app.mq.handlers[mqtt_link.TOPIC_BYTE_BLOCK] = app._on_telemetry
        try:
            self.assertTrue(robot.start())
            self.assertTrue(app.mq.start())
            t0 = time.time()
            while time.time() - t0 < 1.5:
                app.step()
                time.sleep(0.002)
            self.assertGreater(robot.controls, 10)
            self.assertGreater(app.serial_frames, 10)
            self.assertIsNotNone(app.downlink)
            self.assertGreater(app.downlink.vbus_mv, 0)
        finally:
            app.mq.stop()
            robot.stop()
            broker.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
