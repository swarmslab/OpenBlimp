import sys
import time
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import socket
from scipy.spatial.transform import Rotation
import numpy as np

positions = {}
rotations = {}


def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    positions[robot_id] = position
    # rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)
    rotations[robot_id] = rotation_quaternion


if __name__ == "__main__":
    # clientAddress = socket.gethostbyname(socket.gethostname())
    clientAddress = "192.168.0.23"
    optitrackServerAddress = "192.168.0.4"
    robot_id = int(sys.argv[1])

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()
    while is_running:
        if robot_id in positions:
            rot = Rotation.from_quat(rotations[robot_id][0:4])
            roll_r, pitch_r, yaw_r = rot.as_euler("xyz")
            
            print('Last position X: %.2f Y: %.2f Z: %.2f\n Roll: %.2f Pitch: %.2f Yaw: %.2f' % (positions[robot_id][0], positions[robot_id][1], positions[robot_id][2], np.degrees(roll_r), np.degrees(pitch_r), np.degrees(yaw_r)))
            time.sleep(.2)
