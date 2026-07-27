from pyxbot.zmq_client import XbotZmqClient
import argparse
import time
import numpy as np


def gravity_direction(quats_xyzw : np.ndarray) -> np.ndarray:
    """ Express the gravity direction in the local frame of each IMU.

    Parameters
    ----------
    quats_xyzw : np.ndarray
        Array of shape (M, 4) with the IMU orientations in the world frame,
        as quaternions in [x, y, z, w] order.

    Returns
    -------
    np.ndarray
        Array of shape (M, 3) with the unit gravity direction expressed in
        each IMU frame. It is [0, 0, -1] when the IMU is level.
    """
    q = np.asarray(quats_xyzw, dtype=float).reshape(-1, 4)
    q = q/np.linalg.norm(q, axis=1, keepdims=True)
    u = q[:, 0:3] # vector part
    w = q[:, 3:4] # scalar part
    # Gravity points down in the world frame, rotate it into the IMU frame with the inverse rotation
    g_world = np.zeros_like(u)
    g_world[:, 2] = -1.0
    uxg = np.cross(u, g_world)
    return g_world - 2*w*uxg + 2*np.cross(u, uxg)


def gravity_angle(grav_dir : np.ndarray) -> np.ndarray:
    """ Compute the tilt of each IMU, i.e. the angle between its z axis and the vertical.

    Parameters
    ----------
    grav_dir : np.ndarray
        Array of shape (M, 3) with the unit gravity direction in each IMU frame.

    Returns
    -------
    np.ndarray
        Array of shape (M,) with the tilt angle in radians, zero when the IMU is level.
    """
    g = np.asarray(grav_dir, dtype=float).reshape(-1, 3)
    return np.arccos(np.clip(-g[:, 2], -1.0, 1.0))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Print the robot state at a fixed rate. Read-only: no command is ever sent.")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="How long to keep reading the state [s]. Zero or negative means run until Ctrl+C.")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="State printing rate [Hz].")
    parser.add_argument("--protocol", choices=("tcp", "ipc"), default="tcp",
                        help="Transport used to reach the xbot2 server.")
    parser.add_argument("--remote_ip", type=str, default="localhost",
                        help="IP address of the xbot2 server, used only with --protocol tcp.")
    parser.add_argument("--verbose", action="store_true",
                        help="Print connection and discovery info.")
    parser.add_argument("--print_urdf", action="store_true",
                        help="Print the robot URDF at startup.")
    args = parser.parse_args()
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
    print(f"imu names: {client.get_imu_names()}")
    if args.print_urdf:
        print(f"URDF: {client.get_urdf()}")

    t0 = time.time()
    period = 1/args.rate
    try:
        while args.duration <= 0 or time.time()-t0 < args.duration:
            client.sense() # Update the current robot state
            joint_state = client.get_joints_state() # Get the joint state (position, velocity, effort)
            imu_orient = client.getImuOrientation()
            grav_dir = gravity_direction(imu_orient) # Gravity direction in each IMU frame
            grav_angle = gravity_angle(grav_dir) # Tilt of each IMU with respect to the vertical
            print(f"t = {time.time()-t0:.3f}s")
            print(f"joint state: {joint_state}")
            print(f"IMU angular velocity: {client.getImuAngularVelocity()}")
            print(f"IMU orientation: {imu_orient}")
            print(f"IMU linear acceleration: {client.getImuLinearAcceleration()}")
            print(f"gravity direction (IMU frame): {grav_dir}")
            print(f"gravity angle: {np.rad2deg(grav_angle)} deg")
            time.sleep(period)
    except KeyboardInterrupt:
        print("Interrupted, exiting.")
