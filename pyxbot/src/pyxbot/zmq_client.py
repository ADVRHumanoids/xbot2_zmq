import zmq
import yaml 
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import List, Sequence
import pprint

from dataclasses import dataclass
import gc

@dataclass
class JointsCommand():
    pvesd : np.ndarray
    """ An array of shape (N, 5) where N is the number of joints. Each row contains [pos_ref, vel_ref, tor_ref, K, D] for a joint"""
    ctrl_mode : np.ndarray
    """ An array of shape (N,) where N is the number of joints. Each element is the control mode for the corresponding joint """
    joint_names : List[str]
    """ A list of N joint names corresponding to the rows in pvesd """

class JointState():
    def __init__(self, ppvvettpvekd) -> None:
        self._joint_states_ppvvettpvekd = ppvvettpvekd
        self._pve_idx = np.array([0,3,4])
        self._pvesd_refs_idx = np.array([7,8,9,10,11])

    def data(self) -> np.ndarray:
        return self._joint_states_ppvvettpvekd

    def pve(self) -> np.ndarray:
        return self._joint_states_ppvvettpvekd[:, self._pve_idx]
    
    def pvesd_refs(self) -> np.ndarray:
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

class XbotZmqClient:
    def __init__(self,  protocol : str = 'ipc',
                        remote_ip : str = 'localhost',
                        tcp_service_port : int = 5557,
                        tcp_pub_port : int = 5559,
                        tcp_cmd_port : int = 5558,
                        ipc_pub_path : str = '/tmp/xbot2_zmq_pub.ipc',
                        ipc_cmd_path : str = '/tmp/xbot2_zmq_cmd.ipc',
                        ipc_service_path : str = '/tmp/xbot2_zmq_rep.ipc'):

        if protocol not in ('tcp', 'ipc'):
            raise ValueError(f"Unknown protocol '{protocol}', expected 'tcp' or 'ipc'")
        self._protocol = protocol
        self._remote_ip = remote_ip
        self._tcp_service_port = tcp_service_port
        self._tcp_pub_port = tcp_pub_port
        self._tcp_cmd_port = tcp_cmd_port
        self._ipc_pub_path = ipc_pub_path
        self._ipc_cmd_path = ipc_cmd_path
        self._ipc_rep_path = ipc_service_path
        self._next_joint_cmd = JointsCommand(joint_names=[], pvesd=np.zeros((0,5)), ctrl_mode=np.zeros((0,1), dtype=np.int32))
        self._last_msg_seq = 0
        self._last_msg_stamp = 0.0
        self._cmd_seq = 0
        self._last_joints_state_arr : np.ndarray = None
        self._last_imu_state_arr : np.ndarray = None
        self._imu_states : dict[str,np.ndarray] = {}
        self._floating_base = True
        self._client_session_id = np.array([np.random.randint(0, np.iinfo(np.uint64).max, dtype=np.uint64)], dtype=np.uint64)

    def start(self) -> "XbotZmqClient":
        if self._protocol == 'tcp':
            self._request_reply_url = f"tcp://{self._remote_ip}:{self._tcp_service_port}"
            self._jointstates_url   = f"tcp://{self._remote_ip}:{self._tcp_pub_port}"
            self._out_cmd_url       = f"tcp://{self._remote_ip}:{self._tcp_cmd_port}"
        else:
            self._request_reply_url = f"ipc://{self._ipc_rep_path}"
            self._jointstates_url   = f"ipc://{self._ipc_pub_path}"
            self._out_cmd_url       = f"ipc://{self._ipc_cmd_path}"
        context = zmq.Context()
        self._request_reply_socket = context.socket(zmq.REQ)
        self._request_reply_socket.connect(self._request_reply_url)
        print(f"Connected to request-reply socket at {self._request_reply_url}")

        self._joint_names : List[str] = self._get_joint_names_remote()
        self._joints_num = len(self._joint_names)
        self._joint_names_to_idx = {name: idx for idx, name in enumerate(self._joint_names)}
        self._raw_joints_state_shape = (len(self._joint_names), 12) # fix this here, does not change anymore
        print(f"Got joint names: {self._joint_names}")

        self._imu_names : List[str] = self._get_imu_names_remote()
        self._imus_num = len(self._imu_names)
        self._imu_names_to_idx = {name: idx for idx, name in enumerate(self._imu_names)}
        self._raw_imu_state_shape = (len(self._imu_names), 10) # fix this here, does not change anymore
        print(f"Got IMU names: {self._imu_names}")

        self._jointstates_socket = context.socket(zmq.SUB)
        self._jointstates_socket.connect(self._jointstates_url)
        self._jointstates_socket.subscribe("")  # Subscribe to all topics
        self._jointstates_socket.setsockopt(zmq.CONFLATE, 1)  # last msg only.
        print(f"Connected to joint states socket at {self._jointstates_url}")

        self._out_cmd_socket = context.socket(zmq.PUB)
        self._out_cmd_socket.connect(self._out_cmd_url)
        print(f"Connected to command socket at {self._out_cmd_url}")

        self._urdf = self._get_urdf_remote()
        return self

    def _send_request(self, request : dict) -> dict:
        self._request_reply_socket.send_string(yaml.dump(request))
        response_str = self._request_reply_socket.recv_string()
        response = yaml.safe_load(response_str)
        return response

    def _get_joint_names_remote(self):
        response = self._send_request({"type": "joint_names"})
        names = response["data"]
        print(f"Received joint names from server: {names}")
        if self._floating_base:
            names = names[1:]  # Remove the floating base joint
        return names

    def _get_imu_names_remote(self):
        response = self._send_request({"type": "imu_names"})
        return response["data"]

    def set_filter_frequency_hz(self, cutoff_freq, enabled=True):
        resp = self._send_request({"type": "set_filter_frequency_hz", "enabled": enabled, "cutoff_hz": cutoff_freq})
        if not resp["success"]:
            raise RuntimeError("Failed to set filter frequency, reason: " + resp["message"])
    
    def _get_urdf_remote(self) -> str:
        response = self._send_request({"type": "urdf"})
        return response["data"]
    
    def get_urdf(self) -> str:
        return self._urdf
    
    def get_joint_names(self):
        return self._joint_names.copy()

    # def _extract_arrs_proto(self, msg):
    #     rx_msg = proto_msgs.GenericRxMsg.FromString(msg)
    #     seq = rx_msg.seq
    #     stamp = rx_msg.stamp

    #     if hasattr(rx_msg, 'js') and rx_msg.js is not None:
    #         joints_state_arr = np.frombuffer(rx_msg.js.joint_states_ppvvettpvekd, dtype=np.float64).reshape(self._raw_joints_state_shape, order='C')
    #         imu_state_arr = np.frombuffer(rx_msg.js.imu_linxyz_angxyz_quatsxyzw, dtype=np.float64).reshape(self._raw_imu_state_shape, order='C')
    #     return seq, stamp, joints_state_arr, imu_state_arr

    def _extract_arrs_raw(self, msg):
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


    def sense(self, timeout_s : float = float("+inf")):
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


    def _build_command_raw(self) -> bytes:
        """Build a raw bytes command following the recv_cmd_v3 format:
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

        if cmd.pvesd.shape != (self._joints_num, 5):
            raise ValueError(f"Invalid pvesd shape: {cmd.pvesd.shape}, expected: {(self._joints_num, 5)}")
        if cmd.ctrl_mode.shape != (self._joints_num, 1):
            raise ValueError(f"Invalid ctrl_mode shape: {cmd.ctrl_mode.shape}, expected: {(self._joints_num, 1)}")
        
        jnames = cmd.joint_names if cmd.joint_names is not None else self._joint_names
        joints_num = len(jnames)

        seq_arr = np.array([self._cmd_seq], dtype=np.uint32, order='C')
        stamp_arr = np.array([stamp_ns], dtype=np.uint64, order='C') # convert seconds to nanoseconds
        joints_num_arr = np.array([joints_num], dtype=np.uint32, order='C')
        client_session_id_arr = self._client_session_id
        joint_ids = np.array([self._joint_names_to_idx[n] for n in jnames], dtype=np.uint32, order='C')
        if self._floating_base:
            joint_ids = joint_ids + 1 # shift by one to account for the floating base joint at index 0
        pvesd = cmd.pvesd.astype(np.float64, order='C')
        ctrl  = cmd.ctrl_mode.flatten().astype(np.int32, order='C')
        return (  seq_arr.tobytes()
                + stamp_arr.tobytes()
                + joints_num_arr.tobytes()
                + client_session_id_arr.tobytes()
                + joint_ids.tobytes()
                + pvesd.tobytes()
                + ctrl.tobytes())

    def _build_command_proto(self, cmd: JointsCommand):
        if cmd.pvesd.shape != (self._joints_num, 5):
            raise ValueError(f"Invalid pvesd shape: {cmd.pvesd.shape}, expected: {(self._joints_num, 5)}")
        if cmd.ctrl_mode.shape != (self._joints_num, 1):
            raise ValueError(f"Invalid ctrl_mode shape: {cmd.ctrl_mode.shape}, expected: {(self._joints_num, 1)}")
        
        joint_cmd = proto_msgs.JointCommand()
        jnames = cmd.joint_names if cmd.joint_names is not None else self._joint_names
        joint_cmd.name.extend(jnames)
        # Let's be picky here with the inputs
        joint_cmd.joints_pvesd = cmd.pvesd.astype(np.float64, order="C").tobytes()
        joint_cmd.joints_ctrl = cmd.ctrl_mode.astype(np.int32, order="C").tobytes()
        return joint_cmd

    def move(self):
        """Send the next joint command to the robot"""
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
        """Send the next joint command to the robot. If cmd is not None, set it as the next command before sending"""
        if cmd is not None:
            self.set_command_v2(cmd)
        self.move()

    def set_command_v2(self,  cmd : JointsCommand):
        """Set the next joint command to be sent to the robot"""
        self._next_joint_cmd = cmd
        self._cmd_stamp_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC) # Use this specific clock to try to use the same time here and in C++, so at least on the same machine things should match

    def get_joints_state(self, joints : List[str] | None = None) -> JointState:
        """Get the last sensed joint state from the robot"""
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
        self._next_joint_cmd.joint_names = jnames
        self._next_joint_cmd.pvesd = np.zeros((len(jnames), 5))
        self._next_joint_cmd.ctrl_mode = np.ones((len(jnames), 1), dtype=np.int32) # default to position control

    def setPositionReference(self, pos_ref: np.ndarray):
        self._next_joint_cmd.pvesd[:, 0] = pos_ref

    def setVelocityReference(self, vel_ref: np.ndarray):
        self._next_joint_cmd.pvesd[:, 1] = vel_ref

    def setEffortReference(self, tor_ref: np.ndarray):
        self._next_joint_cmd.pvesd[:, 2] = tor_ref

    def setStiffness(self, K: np.ndarray):
        self._next_joint_cmd.pvesd[:, 3] = K

    def setDamping(self, D: np.ndarray):
        self._next_joint_cmd.pvesd[:, 4] = D

    def setCtrlMode(self, ctrl_mode: np.ndarray):
        self._next_joint_cmd.ctrl_mode = ctrl_mode

    def get_imu_names(self) -> List[str]:
        return self._imu_names.copy()
    
    def getImuAngularVelocity(self, req_imu_names: Sequence[str] | None = None) -> np.ndarray:
        imu_names = self._imu_names
        if req_imu_names is None:
            req_imu_names = imu_names
        imu_idxs = np.array([imu_names.index(n) for n in req_imu_names])
        imu_state = self._last_imu_state_arr
        return imu_state[imu_idxs, 3:6]
    
    def getImuLinearAcceleration(self, req_imu_names: Sequence[str] | None = None) -> np.ndarray:
        imu_names = self._imu_names
        if req_imu_names is None:
            req_imu_names = imu_names
        imu_idxs = np.array([imu_names.index(n) for n in req_imu_names])
        imu_state = self._last_imu_state_arr
        return imu_state[imu_idxs, 0:3]
    
    def getImuOrientation(self, req_imu_names: Sequence[str] | None = None) -> np.ndarray:
        imu_names = self._imu_names
        if req_imu_names is None:
            req_imu_names = imu_names
        imu_state = self._last_imu_state_arr
        imu_idxs = np.array([imu_names.index(n) for n in req_imu_names])
        return imu_state[imu_idxs, 6:10] # quaternion in xyzw order
    
    def setup_gc_for_control_loop(self, disable_fully : bool = False):
        """ Sets up garbage collection to minimize latencies introduced by the garbage collector.
            You can cal lthis method at the beginning of your control loop, and call it again every time you restart the loop.
            It will do the following:
            - It first enables garbage collection if it was disabled
            - Then it unfreezes whatever is already frozen, to make it available for collection.
            - Then it performs a full garbage collection to clean up everything that needs to be collected.
            - Then it freezes all currently allocated objects, so they are ignored in future collections, to make gc calls faster.
            - Finally, if disable_fully is True, it disables garbage collection completely. This can lead to memory leaks if not used carefully.
        """
        gc.enable() # enable garbage collection if it was disabled, to be sure
        gc.unfreeze() # unfreezes whatever is already frozen
        gc.collect(2) # collects whatever nedds to be collected
        gc.freeze() # freeze currently allocated objects, so it is ignored in future collections, to make the fast
        if disable_fully:
            gc.disable()