import threading
import unittest

import yaml
import zmq

from pyxbot.zmq_client import XbotZmqClient


class FakeZmqIoServer:
    def __init__(self, replies):
        self._ctx = zmq.Context.instance()
        self._socket = self._ctx.socket(zmq.REP)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._port = self._socket.bind_to_random_port("tcp://127.0.0.1")
        self._replies = list(replies)
        self.requests = []
        self._thread = threading.Thread(target=self._run, daemon=True)

    @property
    def port(self):
        return self._port

    def start(self):
        self._thread.start()
        return self

    def join(self):
        self._thread.join(timeout=2.0)
        self._socket.close()

    def _run(self):
        for reply in self._replies:
            request = yaml.safe_load(self._socket.recv_string())
            self.requests.append(request)
            self._socket.send_string(yaml.safe_dump(reply))


class TestXbotZmqClientServices(unittest.TestCase):
    def _client(self, server):
        return XbotZmqClient(
            protocol="tcp",
            remote_ip="127.0.0.1",
            tcp_service_port=server.port,
        )

    def test_plugin_status_and_health_requests(self):
        server = FakeZmqIoServer([
            {"success": True, "data": {"state": "Running"}},
            {"success": True, "data": {
                "zmq_io_state_ok": True,
                "zmq_io_state": "Running",
                "safety_triggered": False,
                "state_last_publish_age_s": 0.01,
            }},
        ]).start()
        client = self._client(server)

        self.assertEqual(client.get_plugin_status("zmq_io"), "Running")
        health = client.get_health()
        self.assertEqual(health["zmq_io_state"], "Running")
        self.assertFalse(health["safety_triggered"])

        server.join()
        self.assertEqual(server.requests[0], {"type": "plugin_status", "plugin": "zmq_io"})
        self.assertEqual(server.requests[1], {"type": "health"})

    def test_plugin_command_uses_explicit_command_names(self):
        server = FakeZmqIoServer([{"success": True}]).start()
        client = self._client(server)

        client.plugin_command("homing", "start")

        server.join()
        self.assertEqual(server.requests[0], {
            "type": "plugin_command",
            "plugin": "homing",
            "command": "start",
        })

    def test_failed_service_reply_raises(self):
        server = FakeZmqIoServer([{"success": False, "message": "not running"}]).start()
        client = self._client(server)

        with self.assertRaisesRegex(RuntimeError, "not running"):
            client.get_safety_status()

        server.join()


if __name__ == "__main__":
    unittest.main()
