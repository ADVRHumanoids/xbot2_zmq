"""The C++ client against pyxbot and against a stand-in for zmq_io."""

import struct
import tempfile
import threading
import time
import unittest
from pathlib import Path

import numpy as np
import yaml
import zmq

from pyxbot.zmq_client import JointsCommand, XbotZmqClient

try:
    import xbot2_zmq_client as cxx
except ImportError as error:
    raise unittest.SkipTest("C++ client bindings are not built") from error

JOINTS = ["hip", "knee", "ankle"]
IMUS = ["base_imu"]


def state_message(seq, stamp, joints, imus):
    """Bytes in the layout zmq_io publishes."""
    return (struct.pack("<idii", seq, stamp, len(imus), len(joints))
            + np.asarray(joints, dtype=np.float64).tobytes()
            + np.asarray(imus, dtype=np.float64).tobytes())


def state_arrays(seed):
    rng = np.random.default_rng(seed)
    return rng.normal(size=(len(JOINTS), 12)), rng.normal(size=(len(IMUS), 10))


def python_client():
    client = XbotZmqClient(verbose=False)
    client._joint_names = list(JOINTS)
    client._joints_num = len(JOINTS)
    client._joint_names_to_idx = {n: i for i, n in enumerate(JOINTS)}
    client._imu_names = list(IMUS)
    client._imus_num = len(IMUS)
    return client


def cpp_command(ids, pvesd, ctrl):
    command = cxx.JointsCommand()
    command.joint_ids = list(ids)
    command.pvesd = np.asarray(pvesd).flatten().tolist()
    command.ctrl_mode = list(ctrl)
    return command


class FakeZmqIo:
    """Binds the three sockets of zmq_io and answers name requests."""

    def __init__(self, root):
        self.paths = [str(Path(root) / name) for name in ("rep", "pub", "cmd")]
        self.context = zmq.Context()
        self.service = self.context.socket(zmq.REP)
        self.service.bind("ipc://" + self.paths[0])
        self.publisher = self.context.socket(zmq.PUB)
        self.publisher.bind("ipc://" + self.paths[1])
        self.commands = self.context.socket(zmq.SUB)
        self.commands.setsockopt(zmq.SUBSCRIBE, b"")
        self.commands.bind("ipc://" + self.paths[2])
        self.stop = threading.Event()
        self.replies = {
            "joint_names": {"success": True, "data": JOINTS},
            "imu_names": {"success": True, "data": IMUS},
            "urdf": {"success": True, "data": "<robot/>"},
            "refused": {"success": False, "message": "nope"},
        }
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def options(self):
        options = cxx.ClientOptions()
        options.ipc_service_path = self.paths[0]
        options.ipc_state_path = self.paths[1]
        options.ipc_command_path = self.paths[2]
        return options

    def _serve(self):
        while not self.stop.is_set():
            if self.service.poll(10):
                request = yaml.safe_load(self.service.recv_string())
                self.service.send_string(yaml.dump(self.replies[request["type"]]))

    def close(self):
        self.stop.set()
        self.thread.join()
        for socket in (self.service, self.publisher, self.commands):
            socket.close(0)
        self.context.term()


class TestWireFormat(unittest.TestCase):
    def test_command_bytes_match_pyxbot(self):
        client = python_client()
        ids = [2, 0]
        pvesd = np.arange(10.0).reshape(2, 5) + 0.5
        ctrl = np.array([[31], [1]], dtype=np.int32)
        client.set_command(JointsCommand(
            pvesd=pvesd, ctrl_mode=ctrl, joint_names=[JOINTS[i] for i in ids]))
        client._cmd_seq = 7
        client._cmd_stamp_ns = 123456789012345
        client._client_session_id = np.array([987654321], dtype=np.uint64)
        expected = client._build_command_raw()
        encoded = cxx.encode_command(
            7, 123456789012345, 987654321,
            cpp_command(ids, pvesd, ctrl.flatten().tolist()))
        self.assertEqual(encoded, expected)

    def test_state_parses_like_pyxbot(self):
        joints, imus = state_arrays(1)
        message = state_message(11, 4.25, joints, imus)
        seq, stamp, py_joints, py_imus = python_client()._extract_arrs_raw(message)
        state = cxx.decode_state(message, len(JOINTS), len(IMUS))
        self.assertEqual((state.seq, state.stamp), (seq, stamp))
        np.testing.assert_array_equal(
            np.reshape(state.joints, (len(JOINTS), 12)), py_joints)
        np.testing.assert_array_equal(
            np.reshape(state.imus, (len(IMUS), 10)), py_imus)

    def test_malformed_state_is_rejected(self):
        joints, imus = state_arrays(2)
        message = state_message(1, 0.0, joints, imus)
        for bad, joints_num, imus_num in (
                (message[:-1], 3, 1), (message[:6], 3, 1),
                (message, 2, 1), (message, 3, 0)):
            with self.assertRaises(ValueError):
                cxx.decode_state(bad, joints_num, imus_num)

    def test_inconsistent_command_is_rejected(self):
        with self.assertRaises(ValueError):
            cxx.encode_command(0, 0, 0, cpp_command([0, 1], np.zeros(5), [1, 1]))


class TestAgainstServer(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.TemporaryDirectory()
        self.server = FakeZmqIo(self.root.name)
        self.client = cxx.Client(self.server.options())
        self.client.start(2.0)
        time.sleep(0.2)

    def tearDown(self):
        del self.client
        self.server.close()
        self.root.cleanup()

    def _publish(self, seq):
        joints, imus = state_arrays(seq)
        self.server.publisher.send(state_message(seq, seq * 0.5, joints, imus))
        return joints, imus

    def test_names_and_joint_ids(self):
        self.assertEqual(self.client.joint_names, JOINTS)
        self.assertEqual(self.client.imu_names, IMUS)
        self.assertEqual(self.client.joint_id("ankle"), 2)
        with self.assertRaises(IndexError):
            self.client.joint_id("wing")

    def test_sense_returns_false_without_a_new_state(self):
        self.assertFalse(self.client.sense(0.0))

    def test_sense_receives_the_published_state(self):
        joints, imus = self._publish(3)
        self.assertTrue(self.client.sense(1.0))
        state = self.client.state
        self.assertEqual(state.seq, 3)
        self.assertEqual(state.stamp, 1.5)
        np.testing.assert_array_equal(np.reshape(state.joints, (3, 12)), joints)
        np.testing.assert_array_equal(np.reshape(state.imus, (1, 10)), imus)
        self.assertFalse(self.client.sense(0.0))

    def test_sense_keeps_only_the_newest_state(self):
        for seq in range(1, 21):
            self._publish(seq)
        time.sleep(0.2)
        self.assertTrue(self.client.sense(1.0))
        self.assertEqual(self.client.state.seq, 20)

    def test_command_reaches_the_server_in_the_wire_format(self):
        pvesd = np.arange(15.0).reshape(3, 5)
        command = cpp_command([0, 1, 2], pvesd, [31, 31, 31])
        for _ in range(100):
            self.client.send(command)
            if self.server.commands.poll(20):
                break
        raw = self.server.commands.recv()
        seq, stamp_ns, joints_num, session = struct.unpack_from("<IQIQ", raw, 0)
        self.assertEqual(joints_num, 3)
        self.assertGreater(stamp_ns, 0)
        self.assertEqual(len(raw), 24 + 3 * 4 + 15 * 8 + 3 * 4)
        np.testing.assert_array_equal(
            np.frombuffer(raw, np.int32, 3, 24), [0, 1, 2])
        np.testing.assert_array_equal(
            np.frombuffer(raw, np.float64, 15, 36), pvesd.flatten())
        np.testing.assert_array_equal(
            np.frombuffer(raw, np.int32, 3, 156), [31, 31, 31])

    def test_sequence_increments_and_session_stays(self):
        command = cpp_command([0], np.zeros(5), [31])
        seen = []
        for _ in range(200):
            self.client.send(command)
            while self.server.commands.poll(2):
                seen.append(struct.unpack_from("<IQIQ", self.server.commands.recv(), 0))
            if len(seen) >= 2:
                break
        self.assertGreaterEqual(len(seen), 2)
        self.assertEqual(seen[1][0], seen[0][0] + 1)
        self.assertEqual(seen[1][3], seen[0][3])

    def test_refused_request_reports_the_server_message(self):
        with self.assertRaisesRegex(RuntimeError, "nope"):
            self.client.request("refused", 1.0)

    def test_pyxbot_reads_the_same_server(self):
        client = XbotZmqClient(
            protocol="ipc", ipc_service_path=self.server.paths[0],
            ipc_pub_path=self.server.paths[1],
            ipc_cmd_path=self.server.paths[2], verbose=False).start()
        self.assertEqual(client.get_joint_names(), self.client.joint_names)


class TestLaterClient(unittest.TestCase):
    def test_request_timeout_reports_failure(self):
        with tempfile.TemporaryDirectory() as root:
            options = cxx.ClientOptions()
            options.ipc_service_path = str(Path(root) / "absent")
            client = cxx.Client(options)
            with self.assertRaises(RuntimeError):
                client.start(0.2)


if __name__ == "__main__":
    unittest.main()
