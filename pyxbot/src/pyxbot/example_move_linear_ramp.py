from pyxbot.zmq_client import XbotZmqClient, JointsCommand
import argparse
import time
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description="Move all the robot joints to a target position following a linear ramp.")
    parser.add_argument("--target_pos", type=float, default=0.0,
                        help="Target joint position [rad], applied to all joints.")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Duration of the position ramp [s].")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="Command sending rate [Hz].")
    parser.add_argument("--stiffness", type=float, default=500.0,
                        help="Joint impedance stiffness gain.")
    parser.add_argument("--damping", type=float, default=10.0,
                        help="Joint impedance damping gain.")
    parser.add_argument("--ctrl_mode", type=int, default=63,
                        help="Xbot2 control mode bitmask.")
    parser.add_argument("--protocol", choices=("tcp", "ipc"), default="tcp",
                        help="Transport used to reach the xbot2 server.")
    parser.add_argument("--remote_ip", type=str, default="localhost",
                        help="IP address of the xbot2 server, used only with --protocol tcp.")
    parser.add_argument("--verbose", action="store_true",
                        help="Print connection and discovery info.")
    parser.add_argument("--quiet", action="store_true",
                        help="Do not print the robot state at each control step.")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be strictly positive")
    if args.rate <= 0:
        parser.error("--rate must be strictly positive")
    return args


if __name__ == "__main__":
    args = parse_args()

    # Initialize the XbotZmqClient with the requested protocol
    client = XbotZmqClient(protocol=args.protocol,
                           remote_ip=args.remote_ip,
                           verbose=args.verbose)
    # Start the client and connect to the server XBot plugin
    client.start()

    joint_names = client.get_joint_names()
    print(f"joint names: {joint_names}")
    time.sleep(1)

    urdf = client._get_urdf_remote()
    print(f"URDF: {urdf}")
    time.sleep(1)

    # Move the robot to a desired position target_pos following a position ramp
    t0 = time.time()
    pos0 = None
    d = args.duration
    period = 1/args.rate
    ctrl_mode = np.full((len(joint_names), 1), args.ctrl_mode, dtype=np.uint32)
    while time.time()-t0 < d:
        t = time.time()
        client.sense() # Update the current robot state
        joint_state = client.get_joints_state() # Get the joint state (position, velocity, effort)
        if not args.quiet:
            print(f"joint state: {joint_state}")
            print(f"IMU angular velocity: {client.getImuAngularVelocity()}")
            print(f"IMU orientation: {client.getImuOrientation()}")
            print(f"IMU linear acceleration: {client.getImuLinearAcceleration()}")
        pos = joint_state.pos_joint
        pos0 = pos if pos0 is None else pos0
        a = min((t-t0)/d,1.0)
        pvesd = np.zeros((len(joint_names),5))
        pvesd[:,0] = pos0*(1-a)+args.target_pos*a
        pvesd[:,3] = args.stiffness
        pvesd[:,4] = args.damping
        if not args.quiet:
            print(f"sending command with pos:\n{pvesd[:,0]}")
        client.send_command(JointsCommand(  pvesd=pvesd,
                                            joint_names=joint_names,
                                            ctrl_mode=ctrl_mode))
        time.sleep(period)
