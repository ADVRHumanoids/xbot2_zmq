from pyxbot.zmq_client import XbotZmqClient, JointsCommand
import time
import numpy as np
import sys

if __name__ == "__main__":
    client = XbotZmqClient(protocol="tcp")
    client.start()
    joint_names = client.get_joint_names()
    print(f"joint names: {joint_names}")
    time.sleep(1)
    urdf = client._get_urdf_remote()
    print(f"URDF: {urdf}")
    time.sleep(1)
    t0 = time.time()
    pos0 = None
    posf = float(sys.argv[1]) if len(sys.argv)>1 else 0.0
    v = 0.05
    d = 10.0
    while time.time()-t0 < d:
        t = time.time()
        client.sense()
        joint_state = client.get_joints_state()
        print(f"joint state: {joint_state}")
        print(f"IMU angular velocity: {client.getImuAngularVelocity()}")
        print(f"IMU orientation: {client.getImuOrientation()}")
        print(f"IMU linear acceleration: {client.getImuLinearAcceleration()}")
        pos = joint_state.pos_joint
        pos0 = pos if pos0 is None else pos0
        a = min((t-t0)/d,1.0)
        pvesd = np.zeros((len(joint_names),5))
        pvesd[:,0] = pos0*(1-a)+posf*a
        pvesd[:,3] = 500
        pvesd[:,4] = 10
        print(f"sending command with pos:\n{pvesd[:,0]}")
        client.send_command(JointsCommand(   pvesd=pvesd,
                                            joint_names=joint_names,
                                            ctrl_mode=np.full((len(joint_names),1), 63, dtype=np.uint32)))
        time.sleep(0.1)