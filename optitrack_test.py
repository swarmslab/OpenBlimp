import sys
import time
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import socket

positions = {}
rotations = {}


def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    positions[robot_id] = position
    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)
    rotations[robot_id] = rotz


if __name__ == "__main__":
    # clientAddress = socket.gethostbyname(socket.gethostname())
    clientAddress = "192.168.0.5"
    optitrackServerAddress = "192.168.0.4"
    robot_id = 14

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
            # last position
            print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
            time.sleep(.2)
