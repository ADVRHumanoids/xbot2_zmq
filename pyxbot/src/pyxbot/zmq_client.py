import zmq
import yaml
import time
import numpy as np
from typing import List, Sequence

from dataclasses import dataclass
import gc

@dataclass
class JointsCommand():
    """Command structure holding position/velocity/effort/stiffness/damping references for a set of joints.

    Typical usage: construct a ``JointsCommand`` and pass it to :meth:`XbotZmqClient.send_command`.

    Attributes
    ----------
    pvesd : np.ndarray
        Array of shape (N, 5). Each row is [pos_ref, vel_ref, tor_ref, stiffness, damping] for one joint.
    ctrl_mode : np.ndarray
        Integer array of shape (N, 1). Each element is the control mode for one joint.
    joint_names : list of str
        Ordered list of N joint names corresponding to rows in ``pvesd`` and ``ctrl_mode``.
    """
    pvesd : np.ndarray
    """ An array of shape (N, 5) where N is the number of joints. Each row contains [pos_ref, vel_ref, tor_ref, K, D] for a joint"""
    ctrl_mode : np.ndarray
    """ An array of shape (N,) where N is the number of joints. Each element is the control mode for the corresponding joint """
    joint_names : List[str]
    """ A list of N joint names corresponding to the rows in pvesd """

class JointState():
    """Read-only view of the last received state for a subset of robot joints.

    Returned by :meth:`XbotZmqClient.get_joints_state`. All properties return
    views into the underlying NumPy array.

    Properties
    ----------
    pos_joint, pos_motor : np.ndarray
        Joint-side and motor-side position (rad or meters).
    vel_joint, vel_motor : np.ndarray
        Joint-side and motor-side velocity (rad/s or m/s).
    eff : np.ndarray
        Measured effort (Nm or N).
    temperature_motor, temperature_driver : np.ndarray
        Motor and driver temperatures (°C).
    pos_ref : np.ndarray
        Currently active position references.
    vel_ref : np.ndarray
        Currently active velocity references.
    eff_ref : np.ndarray
        Currently active effort references.
    stiff : np.ndarray
        Currently active stiffness gains.
    damp : np.ndarray
        Currently active damping gains.
    """
    def __init__(self, ppvvettpvekd) -> None:
        """
        Parameters
        ----------
        ppvvettpvekd : np.ndarray
            Raw state array of shape (N, 12). Column layout: pos_joint, pos_motor,
            vel_joint, vel_motor, effort, temp_motor, temp_driver, pos_ref, vel_ref,
            eff_ref, stiff, damp.
        """
        self._joint_states_ppvvettpvekd = ppvvettpvekd
        self._joint_states_ppvvettpvekd.flags.writeable = False
        self._pve_idx = np.array([0,3,4]) # we get motor-side joint velocity (higher resolution, less noise on real robot)
        self._pvesd_refs_idx = np.array([7,8,9,10,11])

    def data(self) -> np.ndarray:
        """Return the full raw state array of shape (N, 12)."""
        return self._joint_states_ppvvettpvekd

    def pve(self) -> np.ndarray:
        """Return columns [pos_joint, motor-side vel_joint, eff] as shape (N, 3)."""
        
        return self._joint_states_ppvvettpvekd[:, self._pve_idx]

    def pvesd_refs(self) -> np.ndarray:
        """Return reference columns [pos_ref, vel_ref, eff_ref, stiff, damp] as shape (N, 5)."""
        return self._joint_states_ppvvettpvekd[:, self._pvesd_refs_idx]

    @property
    def pos_joint(self) -> np.ndarray:
        # print(f"getting pos from buffer of size {self._joint_states_ppvvettpvekd.shape}")
        return self._joint_states_ppvvettpvekd[:,0]
    @property
    def pos_motor(self) -> np.ndarray:
        return self._joint_states_ppvvettpvekd[:,1]
    @property
    def vel_joint(self) -> np.ndarray:
        return self._joint_states_ppvvettpvekd[:,2]
    @property
    def vel_motor(self) -> np.ndarray:
        return self._joint_states_ppvvettpvekd[:,3]
    @property
    def eff(self) -> np.ndarray:
        return self._joint_states_ppvvettpvekd[:,4]
    @property
    def temperature_motor(self) -> np.ndarray:
        return self._joint_states_ppvvettpvekd[:,5]
    @property
    def temperature_driver(self) -> np.ndarray:
        return self._joint_states_ppvvettpvekd[:,6]
    @property
    def pos_ref(self) -> np.ndarray:
        return self._joint_states_ppvvettpvekd[:,7]
    @property
    def vel_ref(self) -> np.ndarray:
        return self._joint_states_ppvvettpvekd[:,8]
    @property
    def eff_ref(self) -> np.ndarray:
        return self._joint_states_ppvvettpvekd[:,9]
    @property
    def stiff(self) -> np.ndarray:
        return self._joint_states_ppvvettpvekd[:,10]
    @property
    def damp(self) -> np.ndarray:
        return self._joint_states_ppvvettpvekd[:,11]

    def __repr__(self) -> str:
        return (f"JointState(pos={self.pos_joint},\n"
                f"           vel={self.vel_motor},\n"
                f"           eff={self.eff},\n"
                f"           stiff={self.stiff},\n"
                f"           damp={self.damp},\n"
                f"           pos_ref={self.pos_ref},\n"
                f"           vel_ref={self.vel_ref},\n"
                f"           eff_ref={self.eff_ref})")

class ClockDeltaEstimator:
    """Sliding-window minimum estimator for (t_recv - t_send).

    Approximates the time delta between server and client, obtaining a lower bound on 
    true_clock_offset + one_way_latency over a rolling window. The clock delta between
    our clock the server's and is always delta >= true_clock_offset + one_way_latency.
    We cannot distinguish between offset and latency with this method, but we can at 
    least approximate a lower bound on the sum of the two, which gives us a lower bound on
    the delta.
    This allows to check for state message ages.

    """

    def __init__(self, window: int = 200):
        self._buf = np.full(window, np.inf, dtype=np.float64)
        self._head = 0

    def update(self, server_stamp: float) -> None:
        delta = time.monotonic() - server_stamp
        self._buf[self._head] = delta
        self._head = (self._head + 1) % len(self._buf)

    @property
    def min_delta(self) -> float:
        """ The real delta is approximately at least this small"""
        return float(np.min(self._buf))

    def age(self, server_stamp: float) -> float:
        """This is an approximate lower bound on the age. The age could be higher than this,
          but not much lower."""
        return time.monotonic() - (server_stamp + self.min_delta)


class XbotZmqClient:
    """ZMQ-based client for communicating with an xbot2 robot.

    Two transports are supported:

    - ``'ipc'``: Unix domain sockets for same-machine communication (default).
    - ``'tcp'``: TCP sockets for remote communication.

    The typical workflow is:

    1. Instantiate the client.
    2. Call :meth:`start` to connect and fetch joint/IMU metadata from the server.
    3. In a loop: call :meth:`sense` to update state, read state via
       :meth:`get_joints_state` / ``getImu*``, build a :class:`JointsCommand`,
       and send it with :meth:`send_command`.

    Example::

        client = XbotZmqClient(protocol='ipc').start()
        joint_names = client.get_joint_names()
        while True:
            client.sense()
            state = client.get_joints_state()
            pvesd = np.zeros((len(joint_names), 5))
            pvesd[:, 0] = state.pos_joint   # hold current position
            pvesd[:, 3] = 500               # stiffness
            pvesd[:, 4] = 10                # damping
            client.send_command(JointsCommand(
                pvesd=pvesd,
                joint_names=joint_names,
                ctrl_mode=np.full((len(joint_names), 1), 63, dtype=np.uint32),
            ))
    """
    def __init__(self,  protocol : str = 'ipc',
                        remote_ip : str = 'localhost',
                        tcp_service_port : int = 5557,
                        tcp_pub_port : int = 5559,
                        tcp_cmd_port : int = 5558,
                        ipc_pub_path : str = '/tmp/xbot2_zmq_pub.ipc',
                        ipc_cmd_path : str = '/tmp/xbot2_zmq_cmd.ipc',
                        ipc_service_path : str = '/tmp/xbot2_zmq_rep.ipc',
                        verbose : bool = False):
        """ Initialize the client, call start() before using it.

        Parameters
        ----------
        protocol : str
            Transport to use: ``'ipc'`` (same machine) or ``'tcp'`` (remote).
        remote_ip : str
            IP address of the xbot2 server. Used only with ``protocol='tcp'``.
        tcp_service_port : int
            Port for the request-reply service socket (tcp only).
        tcp_pub_port : int
            Port for the joint-state publisher socket (tcp only).
        tcp_cmd_port : int
            Port for the command socket (tcp only).
        ipc_pub_path : str
            IPC socket path for joint-state publishing (ipc only).
        ipc_cmd_path : str
            IPC socket path for commands (ipc only).
        ipc_service_path : str
            IPC socket path for the request-reply service (ipc only).
        verbose : bool
            Print connection and discovery info to stdout.
        """
        if protocol not in ('tcp', 'ipc'):
            raise ValueError(f"Unknown protocol '{protocol}', expected 'tcp' or 'ipc'")
        self._protocol = protocol
        self._remote_ip = remote_ip
        self._verbose = verbose
        self._tcp_service_port = tcp_service_port
        self._tcp_pub_port = tcp_pub_port
        self._tcp_cmd_port = tcp_cmd_port
        self._ipc_pub_path = ipc_pub_path
        self._ipc_cmd_path = ipc_cmd_path
        self._ipc_rep_path = ipc_service_path
        self._request_reply_url = None
        self._jointstates_url = None
        self._out_cmd_url = None
        self._context = zmq.Context.instance()
        self._next_joint_cmd = JointsCommand(joint_names=[], pvesd=np.zeros((0,5)), ctrl_mode=np.zeros((0,1), dtype=np.int32))
        self._last_msg_seq = 0
        self._last_msg_stamp = 0.0
        self._cmd_seq = 0
        self._max_state_age_s = 0.1 # warn if received state messages are estimated to be older than this threshold
        self._sense_call_count = 0
        self._last_msg_rec_time = float("-inf")
        self._delta_estimator = ClockDeltaEstimator()
        self._last_joints_state_arr : np.ndarray = None
        self._last_imu_state_arr : np.ndarray = None
        self._imu_states : dict[str,np.ndarray] = {}
        self._client_session_id = np.array([np.random.randint(0, np.iinfo(np.uint64).max, dtype=np.uint64)], dtype=np.uint64)

    def _resolve_urls(self):
        if self._protocol == 'tcp':
            self._request_reply_url = f"tcp://{self._remote_ip}:{self._tcp_service_port}"
            self._jointstates_url   = f"tcp://{self._remote_ip}:{self._tcp_pub_port}"
            self._out_cmd_url       = f"tcp://{self._remote_ip}:{self._tcp_cmd_port}"
        else:
            self._request_reply_url = f"ipc://{self._ipc_rep_path}"
            self._jointstates_url   = f"ipc://{self._ipc_pub_path}"
            self._out_cmd_url       = f"ipc://{self._ipc_cmd_path}"

    def start(self) -> "XbotZmqClient":
        """Connect to the xbot2 server and fetch joint/IMU metadata.

        Opens the ZMQ sockets and queries the server for joint names, IMU names,
        and the robot URDF. Must be called once before :meth:`sense`,
        :meth:`get_joints_state`, or any command method.

        Returns
        -------
        XbotZmqClient
            ``self``, to allow chaining: ``client = XbotZmqClient().start()``.
        """
        self._resolve_urls()
        if self._verbose:
            print(f"Connected to request-reply socket at {self._request_reply_url}")

        self._joint_names : List[str] = self._get_joint_names_remote()
        self._joints_num = len(self._joint_names)
        self._joint_names_to_idx = {name: idx for idx, name in enumerate(self._joint_names)}
        self._raw_joints_state_shape = (len(self._joint_names), 12) # fix this here, does not change anymore
        if self._verbose:
            print(f"Got joint names: {self._joint_names}")

        self._imu_names : List[str] = self._get_imu_names_remote()
        self._imus_num = len(self._imu_names)
        self._imu_names_to_idx = {name: idx for idx, name in enumerate(self._imu_names)}
        self._raw_imu_state_shape = (len(self._imu_names), 10) # fix this here, does not change anymore
        if self._verbose:
            print(f"Got IMU names: {self._imu_names}")

        self._jointstates_socket = self._context.socket(zmq.SUB)
        self._jointstates_socket.setsockopt(zmq.CONFLATE, 1)  # last msg only. IMPORTANT! this must be before connect!
        self._jointstates_socket.subscribe("")  # Subscribe to all topics
        self._jointstates_socket.connect(self._jointstates_url)
        if self._verbose:
            print(f"Connected to joint states socket at {self._jointstates_url}")

        self._out_cmd_socket = self._context.socket(zmq.PUB)
        self._out_cmd_socket.connect(self._out_cmd_url)
        if self._verbose:
            print(f"Connected to command socket at {self._out_cmd_url}")

        self._urdf = self._get_urdf_remote()
        return self

    def _send_request(self, request : dict, timeout_s: float = 5.0) -> dict:
        """Send a YAML-encoded request dict and return the parsed response.

        A fresh REQ socket is used for each request so timeouts do not poison a
        long-lived request socket.
        """
        if self._request_reply_url is None:
            self._resolve_urls()
        socket = self._context.socket(zmq.REQ)
        timeout_ms = max(1, int(timeout_s * 1000))
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        try:
            socket.connect(self._request_reply_url)
            socket.send_string(yaml.dump(request))
            response_str = socket.recv_string()
            response = yaml.safe_load(response_str)
            return response or {}
        finally:
            socket.close()

    def _request_data(self, request: dict, timeout_s: float = 5.0):
        response = self._send_request(request, timeout_s=timeout_s)
        if not response.get("success", False):
            raise RuntimeError(response.get("message", f"request failed: {request}"))
        return response.get("data")

    def _get_joint_names_remote(self):
        """Fetch the ordered list of joint names from the server."""
        names = self._request_data({"type": "joint_names"})
        if self._verbose:
            print(f"Received joint names from server: {names}")
        return names

    def _get_imu_names_remote(self):
        """Fetch the ordered list of IMU names from the server."""
        return self._request_data({"type": "imu_names"})

    def set_filter_frequency_hz(self, cutoff_freq, enabled=True):
        """Set the server-side low-pass filter cutoff frequency for joint states."""
        self._request_data({"type": "set_filter_frequency_hz", "enabled": enabled, "cutoff_hz": cutoff_freq})

    def get_health(self, timeout_s: float = 1.0) -> dict:
        """Single liveness + safety report from the server.

        Fields: zmq_io_state_ok, zmq_io_state, safety_enabled, filter_enabled, filter_cutoff_hz,
        safety_triggered, state_last_publish_age_s. This is the one place safety and liveness are
        reported; the former separate 'safety_status'/'state_stats'/'cmd_stats' services were folded
        away. 'plugin_status'/'plugin_command' (client plugin control) and 'restore_safety' were
        removed / neutered server-side (a client must not have that authority).
        """
        return self._request_data({"type": "health"}, timeout_s=timeout_s)

    def _get_urdf_remote(self) -> str:
        """Fetch the robot URDF string from the server."""
        return self._request_data({"type": "urdf"})

    def get_urdf(self) -> str:
        """Return the robot URDF string retrieved at startup."""
        return self._urdf

    def get_joint_names(self):
        """Return the ordered list of joint names as known to the server."""
        return self._joint_names.copy()

    def _extract_arrs_raw(self, msg : bytes):
        """ Extracts state data from a raw bytes message, which should follow the following format:
            - All data is in 64-bit double precision for floating-point values and 32-bit integers for integer values.
            - First integer is the sequence number (seq).
            - Second double is the timestamp (stamp).
            - Next comes the number of IMUs (imus_num) as an integer.
            - Next comes the number of joints (joints_num) as an integer.
            - Then follows the joint states matrix (joints_state) serialized in row-major order, with dimensions (joints_num x 12).
            - Finally, the IMU states matrix (imus_state) serialized in row-major order, with dimensions (imus_num x 10).
        """
        # print(f"Buffer size: {len(msg)} bytes")
        dsize = 8
        isize = 4
        header_size = isize + dsize + isize + isize # seq (int32) + stamp (float64) + imus_num (int32) + joints_num (int32)
        joints_elem_num = self._joints_num * 12 # number of elements in the joints state matrix
        joints_state_size = joints_elem_num * dsize # size of the joints state matrix in bytes
        imus_elem_num = self._imus_num * 10 # number of elements in the IMUs state matrix
        imus_state_size = imus_elem_num * dsize # size of the IMUs state matrix in bytes
        expected_size = header_size + joints_state_size + imus_state_size
        if len(msg) != expected_size:
            raise ValueError(f"Unexpected message size: {len(msg)} bytes, expected: {expected_size} bytes. expected joints count: {self._joints_num}, expected imus count: {self._imus_num}")

        offset = 0
        seq       = np.frombuffer(msg, dtype=np.int32,   count=1, offset=offset)[0]
        offset += isize
        stamp     = np.frombuffer(msg, dtype=np.float64, count=1, offset=offset)[0]
        offset += dsize
        imus_num  = np.frombuffer(msg, dtype=np.int32,   count=1, offset=offset)[0]
        offset += isize
        joints_num = np.frombuffer(msg, dtype=np.int32,  count=1, offset=offset)[0]
        offset += isize

        if joints_num != self._joints_num:
            raise ValueError(f"Unexpected joints count in message: {joints_num}, expected: {self._joints_num}")
        if imus_num != self._imus_num:
            raise ValueError(f"Unexpected IMUs count in message: {imus_num}, expected: {self._imus_num}")

        joints_state_arr = np.frombuffer(msg, dtype=np.float64, count=joints_elem_num, offset=offset).reshape((joints_num, 12), order='C')
        offset += joints_num * 12 * dsize

        imu_state_arr = np.frombuffer(msg, dtype=np.float64, count=imus_elem_num, offset=offset).reshape((imus_num, 10), order='C')

        seq = int(seq)
        stamp = float(stamp)
        return seq, stamp, joints_state_arr, imu_state_arr

    def _busy_sense(self, timeout_s : float = float("+inf")) -> bool:
        msg = None
        t0 = time.monotonic()
        while msg is None:
            while True:
                try:
                    msg = self._jointstates_socket.recv(flags=zmq.NOBLOCK)
                    # self._last_msg_seq, self._last_msg_stamp, self._last_joints_state_arr, self._last_imu_state_arr = self._extract_arrs_proto(msg)
                    self._last_msg_seq, self._last_msg_stamp, self._last_joints_state_arr, self._last_imu_state_arr = self._extract_arrs_raw(msg)
                except zmq.Again:
                    # print("No joint state message available yet...")
                    if time.monotonic() - t0 > timeout_s:
                        raise TimeoutError(f"Timeout while waiting for joint state message after {timeout_s} seconds")
                    break # no data available
        return True # we got a message
    
    def sense(self, timeout_s : float = float("+inf"), blocking : bool = True) -> bool:
        """Read the latest joint and IMU state from the robot.

        Drains any queued messages and stores the result internally. Call this
        at the top of every control-loop iteration before reading state via
        :meth:`get_joints_state` or ``getImu*`` methods.

        Parameters
        ----------
        timeout_s : float
            How long to wait for a message before raising ``TimeoutError``.
            Defaults to infinity (blocks until a message arrives).

        Raises
        ------
        TimeoutError
            If no message arrives within ``timeout_s`` seconds.
        """
        self._sense_call_count += 1
        # return self._busy_sense(timeout_s=timeout_s)
                
        msg = None
        t0 = time.monotonic()
        while msg is None:
                try:
                    msg = self._jointstates_socket.recv(flags=zmq.NOBLOCK)
                    self._last_msg_rec_time = time.monotonic()
                    self._last_msg_seq, self._last_msg_stamp, self._last_joints_state_arr, self._last_imu_state_arr = self._extract_arrs_raw(msg)
                    self._delta_estimator.update(self._last_msg_stamp)
                    # print(f"[{self._sense_call_count}] {time.monotonic()*1000:.3f} Received joint state message {self._last_msg_seq} of size {len(msg)} bytes after waiting {self._last_msg_rec_time - t0:.3f} seconds")
                    age = self._delta_estimator.age(self._last_msg_stamp)
                    if age > self._max_state_age_s:
                        print(f"[{self._sense_call_count}] {time.monotonic()*1000:.3f} Warning: received a message with estimated age {age:.3f} seconds, which is above the configured maximum of {self._max_state_age_s} seconds")
                    break
                    # self._last_msg_seq, self._last_msg_stamp, self._last_joints_state_arr, self._last_imu_state_arr = self._extract_arrs_proto(msg)
                except zmq.Again:
                    # print(f"[{self._sense_call_count}] {time.monotonic()*1000:.3f} No joint state message available yet after {time.monotonic() - t0:.3f} seconds...")
                    remainingtime = timeout_s - (time.monotonic() - t0)
                    if remainingtime <=0:
                        if blocking:
                            raise TimeoutError(f"Timeout while waiting for joint state message after {timeout_s} seconds")
                        else:
                            # print(f"[{self._sense_call_count}] {time.monotonic()*1000:.3f} returning false")
                            return False
                    polling_max_dur_ms = int(min(10, remainingtime*1000)) if timeout_s != float("+inf") else None
                    self._jointstates_socket.poll(timeout=polling_max_dur_ms) # wait max 10ms for new messages to arrive, then manually check again.
        return True # we got a message

    def get_last_state_rec_time(self):
        """Get the timestamp of when the last robot state message was received, in time.monotonic time.
            Returns None if no message has been received yet."""
        return self._last_msg_rec_time

    def get_state_age_estimate(self):
        """Get the age of the last received robot state message in seconds. Returns None if no message has been received yet.
        This is an estimate, only use it a lower bound (age is at least this much)"""
        return self._delta_estimator.age(self._last_msg_stamp) if self._last_msg_rec_time != float("-inf") else None

    def wait_for_state_stream(self, timeout_s: float, min_seq_delta: int = 1):
        initial_seq = self._last_msg_seq
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(f"Timeout while waiting for xbot2_zmq state stream after {timeout_s} seconds")
            self.sense(timeout_s=min(remaining, 0.1))
            if self._last_msg_seq - initial_seq >= min_seq_delta:
                return

    def _build_command_raw(self) -> bytes:
        """Build a raw bytes command using the current client state and following the recv_cmd_v3 format:
          - seq                 : 1 x uint32
          - stamp_ns            : 1 x uint64  (nanoseconds since epoch)
          - joints_num          : 1 x uint32
          - client_session_id   : 1 x uint64
          - joint_ids           : joints_num x int32  (indices into the server's joint list)
          - pvesd               : joints_num x 5 x float64, row-major
          - ctrl_mode           : joints_num x int32
        """

        cmd = self._next_joint_cmd
        stamp_ns = self._cmd_stamp_ns
        jnames = cmd.joint_names if cmd.joint_names is not None else self._joint_names
        joints_num = len(jnames)

        if cmd.pvesd.shape != (joints_num, 5):
            raise ValueError(f"Invalid pvesd shape: {cmd.pvesd.shape}, expected: {(joints_num, 5)}")
        if cmd.ctrl_mode.shape != (joints_num, 1):
            raise ValueError(f"Invalid ctrl_mode shape: {cmd.ctrl_mode.shape}, expected: {(joints_num, 1)}")

        seq_arr = np.array([self._cmd_seq], dtype=np.uint32, order='C')
        stamp_arr = np.array([stamp_ns], dtype=np.uint64, order='C') # convert seconds to nanoseconds
        joints_num_arr = np.array([joints_num], dtype=np.uint32, order='C')
        client_session_id_arr = self._client_session_id
        joint_ids = np.array([self._joint_names_to_idx[n] for n in jnames], dtype=np.uint32, order='C')
        pvesd = cmd.pvesd.astype(np.float64, order='C')
        ctrl  = cmd.ctrl_mode.flatten().astype(np.int32, order='C')
        return (  seq_arr.tobytes()
                + stamp_arr.tobytes()
                + joints_num_arr.tobytes()
                + client_session_id_arr.tobytes()
                + joint_ids.tobytes()
                + pvesd.tobytes()
                + ctrl.tobytes())

    # def _build_command_proto(self, cmd: JointsCommand):
    #     if cmd.pvesd.shape != (self._joints_num, 5):
    #         raise ValueError(f"Invalid pvesd shape: {cmd.pvesd.shape}, expected: {(self._joints_num, 5)}")
    #     if cmd.ctrl_mode.shape != (self._joints_num, 1):
    #         raise ValueError(f"Invalid ctrl_mode shape: {cmd.ctrl_mode.shape}, expected: {(self._joints_num, 1)}")

    #     joint_cmd = proto_msgs.JointCommand()
    #     jnames = cmd.joint_names if cmd.joint_names is not None else self._joint_names
    #     joint_cmd.name.extend(jnames)
    #     # Let's be picky here with the inputs
    #     joint_cmd.joints_pvesd = cmd.pvesd.astype(np.float64, order="C").tobytes()
    #     joint_cmd.joints_ctrl = cmd.ctrl_mode.astype(np.int32, order="C").tobytes()
    #     return joint_cmd

    def move(self):
        """Send the next joint command to the robot. The command is set by calling :meth:`set_command`.
        You can also set and send in one call using :meth:`send_command`."""
        # cmd_msg =  proto_msgs.GenericRxMsg()
        # cmd_msg.stamp = int(time.time() * 1e9)
        # cmd_msg.cmd = self._build_command_proto(self._next_joint_cmd)
        # msg_str = cmd_msg.SerializeToString()
        # self._out_cmd_socket.send(msg_str)

        self._out_cmd_socket.send(self._build_command_raw())
        t = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        # print(f"Sent command seq {self._cmd_seq} with delay {(t-self._cmd_stamp_ns)/1e6} ms")
        self._cmd_seq += 1


    def send_command(self, cmd : JointsCommand | None = None):
        """Send the next joint command to the robot. If cmd is not None, set it as the next command before sending.
        If cmd is None this is equivalent to calling :meth:`move`.
        If cmd is not None, then it will override commands set via :meth:`enableJoints`, :meth:`setPositionReference`, etc. """
        if cmd is not None:
            self.set_command(cmd)
        self.move()

    def set_command(self,  cmd : JointsCommand):
        """Set the next joint command to be sent to the robot via :meth:`move` or :meth:`send_command`.
        This does not send the command immediately.
        This overrides commands set via :meth:`enableJoints`, :meth:`setPositionReference`, etc.

        Parameters
        ----------
        cmd : JointsCommand
            The joint command to set as the next command to be sent.
        """
        self._next_joint_cmd = cmd
        self._cmd_stamp_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC) # Use this specific clock to try to use the same time here and in C++, so at least on the same machine things should match

    def get_joints_state(self, joints : List[str] | None = None) -> JointState:
        """Get the last sensed joint state from the robot. Current joint state is updated by calling :meth:`sense`.

        Parameters
        ----------
        joints : list of str, optional
            Joint names to query. Defaults to all joints (None).
        Returns
        -------
        JointState
            The last sensed state for the requested joints.
        """
        if joints is None:
            joints = self._joint_names
        tot_joints_num = len(self._joint_names)
        # print(f"self._last_joint_state_msg.joint_names = {js.joint_names}")
        indices = np.array([self._joint_names_to_idx[j] for j in joints], dtype=np.int32)
        # print(f"indices: {indices}")
        # joint_states_ppvvettpvekd = joint_states_ppvvettpvekd[:, [0,3]]
        js = JointState(self._last_joints_state_arr[indices])
        return js

    def enableJoints(self, jnames: list):
        """Configure the internal command to target the given joints with zero references.

        Resets the next command to address only the specified joints, with all
        references zeroed and control mode set to position (1). Call this once after
        :meth:`start` to select which joints to command, then set references with
        :meth:`setPositionReference` etc. before calling :meth:`send_command`.

        Using :meth:`set_command` or :meth:`send_command` with a custom JointsCommand overrides
        the joints enabled by this method.

        Parameters
        ----------
        jnames : list of str
            Ordered list of joint names to enable. Must be a subset of
            :meth:`get_joint_names`.
        """
        for j in jnames:
            if j not in self._joint_names:
                raise ValueError(f"Unknown joint name '{j}' in enableJoints, valid names are: {self._joint_names}")
        self._next_joint_cmd.joint_names = jnames
        self._next_joint_cmd.pvesd = np.zeros((len(jnames), 5))
        self._next_joint_cmd.ctrl_mode = np.ones((len(jnames), 1), dtype=np.int32) # default to position control

    def setPositionReference(self, pos_ref: np.ndarray):
        """Set position references for the currently enabled joints.

        Using :meth:`set_command` or :meth:`send_command` with a custom JointsCommand overrides
        the result of this method.

        Parameters
        ----------
        pos_ref : np.ndarray
            Array of shape (N,) with position targets in radians, one per joint
            in the order set by :meth:`enableJoints`.
        """
        self._next_joint_cmd.pvesd[:, 0] = pos_ref

    def setVelocityReference(self, vel_ref: np.ndarray):
        """Set velocity references for the currently enabled joints.

        Using :meth:`set_command` or :meth:`send_command` with a custom JointsCommand overrides
        the result of this method.

        Parameters
        ----------
        vel_ref : np.ndarray
            Array of shape (N,) with velocity targets in rad/s.
        """
        self._next_joint_cmd.pvesd[:, 1] = vel_ref

    def setEffortReference(self, tor_ref: np.ndarray):
        """Set effort (torque) feedforward references for the currently enabled joints.

        Using :meth:`set_command` or :meth:`send_command` with a custom JointsCommand overrides
        the result of this method.

        Parameters
        ----------
        tor_ref : np.ndarray
            Array of shape (N,) with torque feedforward values in N or Nm.
        """
        self._next_joint_cmd.pvesd[:, 2] = tor_ref

    def setStiffness(self, K: np.ndarray):
        """Set stiffness gains for the currently enabled joints.

        Using :meth:`set_command` or :meth:`send_command` with a custom JointsCommand overrides
        the result of this method.

        Parameters
        ----------
        K : np.ndarray
            Array of shape (N,) with position-gain values.
        """
        self._next_joint_cmd.pvesd[:, 3] = K

    def setDamping(self, D: np.ndarray):
        """Set damping gains for the currently enabled joints.

        Using :meth:`set_command` or :meth:`send_command` with a custom JointsCommand overrides
        the result of this method.

        Parameters
        ----------
        D : np.ndarray
            Array of shape (N,) with damping values.
        """
        self._next_joint_cmd.pvesd[:, 4] = D

    def setCtrlMode(self, ctrl_mode: np.ndarray):
        """Set the control mode bitmask for the currently enabled joints.

        Using :meth:`set_command` or :meth:`send_command` with a custom JointsCommand overrides
        the result of this method.

        Parameters
        ----------
        ctrl_mode : np.ndarray
            Integer array of shape (N, 1) with control mode flags per joint.
        """
        self._next_joint_cmd.ctrl_mode = ctrl_mode

    def get_imu_names(self) -> List[str]:
        """Return the ordered list of IMU names as known to the server."""
        return self._imu_names.copy()

    def getImuAngularVelocity(self, req_imu_names: Sequence[str] | None = None) -> np.ndarray:
        """Return the last sensed angular velocity for the requested IMUs. The current IMU state is updated by calling :meth:`sense`.

        Parameters
        ----------
        req_imu_names : sequence of str, optional
            IMU names to query. Defaults to all IMUs.

        Returns
        -------
        np.ndarray
            Array of shape (M, 3) with [wx, wy, wz] in rad/s for each requested IMU.
        """
        imu_names = self._imu_names
        if req_imu_names is None:
            req_imu_names = imu_names
        imu_idxs = np.array([imu_names.index(n) for n in req_imu_names])
        imu_state = self._last_imu_state_arr
        return imu_state[imu_idxs, 3:6]

    def getImuLinearAcceleration(self, req_imu_names: Sequence[str] | None = None) -> np.ndarray:
        """Return the last sensed linear acceleration for the requested IMUs. The current IMU state is updated by calling :meth:`sense`.

        Parameters
        ----------
        req_imu_names : sequence of str, optional
            IMU names to query. Defaults to all IMUs.

        Returns
        -------
        np.ndarray
            Array of shape (M, 3) with [ax, ay, az] in m/s² for each requested IMU.
        """
        imu_names = self._imu_names
        if req_imu_names is None:
            req_imu_names = imu_names
        imu_idxs = np.array([imu_names.index(n) for n in req_imu_names])
        imu_state = self._last_imu_state_arr
        return imu_state[imu_idxs, 0:3]

    def getImuOrientation(self, req_imu_names: Sequence[str] | None = None) -> np.ndarray:
        """Return the last sensed orientation for the requested IMUs as quaternions. The current IMU state is updated by calling :meth:`sense`.

        Parameters
        ----------
        req_imu_names : sequence of str, optional
            IMU names to query. Defaults to all IMUs.

        Returns
        -------
        np.ndarray
            Array of shape (M, 4) with quaternions in [x, y, z, w] order for each
            requested IMU.
        """
        imu_names = self._imu_names
        if req_imu_names is None:
            req_imu_names = imu_names
        imu_state = self._last_imu_state_arr
        imu_idxs = np.array([imu_names.index(n) for n in req_imu_names])
        return imu_state[imu_idxs, 6:10] # quaternion in xyzw order

    def setup_gc_for_control_loop(self, disable_fully : bool = False):
        """ Sets up garbage collection to minimize latencies introduced by the garbage collector.
            You can call this method at the beginning of your control loop, and call it again every time you restart the loop.
            In the case of and RL setting for example you would call this before each episode.
            It will do the following:
            - It first enables garbage collection if it was disabled
            - Then it unfreezes whatever is already frozen, to make it available for collection.
            - Then it performs a full garbage collection to clean up everything that needs to be collected.
            - Then it freezes all currently allocated objects, so they are ignored in future collections, to make gc calls faster.
            - Finally, if disable_fully is True, it disables garbage collection completely. This can lead to memory leaks if not used carefully.

        Parameters
        ----------
        disable_fully : bool
            Whether to disable garbage collection completely after the initial cleanup. Defaults to False.
        """
        gc.enable() # enable garbage collection if it was disabled, to be sure
        gc.unfreeze() # unfreezes whatever is already frozen
        gc.collect(2) # collects whatever nedds to be collected
        gc.freeze() # freeze currently allocated objects, so it is ignored in future collections, to make the fast
        if disable_fully:
            gc.disable()
