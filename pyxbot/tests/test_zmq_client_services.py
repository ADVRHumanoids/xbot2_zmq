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

    def test_health_request(self):
        # 'health' is the single liveness+safety service (safety_status/state_stats/cmd_stats and
        # plugin_status were folded away / removed).
        server = FakeZmqIoServer([
            {"success": True, "data": {
                "zmq_io_state_ok": True,
                "zmq_io_state": "Running",
                "safety_enabled": True,
                "filter_enabled": False,
                "filter_cutoff_hz": 0.0,
                "safety_triggered": False,
                "state_last_publish_age_s": 0.01,
            }},
        ]).start()
        client = self._client(server)

        health = client.get_health()
        self.assertEqual(health["zmq_io_state"], "Running")
        self.assertFalse(health["safety_triggered"])
        self.assertIn("state_last_publish_age_s", health)

        server.join()
        self.assertEqual(server.requests[0], {"type": "health"})

    def test_removed_client_services_are_gone(self):
        # These services were removed from both the plugin and the client; guard against a
        # re-introduction that would resurrect the redundant/unsafe surface.
        for name in ("get_safety_status", "get_state_stats", "get_cmd_stats",
                     "get_plugin_status", "plugin_command", "restore_safety"):
            self.assertFalse(hasattr(XbotZmqClient, name), f"{name} should have been removed")

    def test_failed_service_reply_raises(self):
        server = FakeZmqIoServer([{"success": False, "message": "not running"}]).start()
        client = self._client(server)

        with self.assertRaisesRegex(RuntimeError, "not running"):
            client.get_health()

        server.join()


if __name__ == "__main__":
    unittest.main()
