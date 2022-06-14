import logging
import sys
import time
from threading import Event
from threading import Thread

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
import pid_controller as pid

from scipy.spatial.transform import Rotation

import numpy as np

from NatNetClient import NatNetClient
# from util import quaternion_to_euler_angle_vectorized1
import socket


if len(sys.argv) != 5:
    if len(sys.argv) < 5:
        print("ERROR: Insufficient arguemnts\nPlease use the following form:\npython Position Control.py <radio adddress> <rigid body number> <client address> <optitrack server address>")
    else:
        print("ERROR: Too many arguemnts\nPlease use the following form:\npython Position Control.py <radio adddress> <rigid body number> <client address> <optitrack server address>")
    sys.exit()

radio_address = sys.argv[1]
uri_address = 'radio://0/'+radio_address+'/2M/E7E7E7E7E7'
uri = uri_helper.uri_from_env(default = uri_address)
rigid_body_id = int(sys.argv[2])
logging.basicConfig(level=logging.ERROR)

positions = {}
rotations = {}

def console_callback(text: str):
    '''A callback to run when we get console text from Crazyflie'''
    # We do not add newlines to the text received, we get them from the
    # Crazyflie at appropriate places.
    print(text, end='')

def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    positions[robot_id] = position
    # rot = Rotation.from_quat(rotation_quaternion)
    rotations[robot_id] = rotation_quaternion

def param_callback(name, value):
    print('The crazyflie has parameter ' + name + ' set at number: ' + value)

def set_param(cf, groupstr, namestr, value):
    full_name = groupstr+"."+namestr
    cf.param.add_update_callback(group=groupstr, name = namestr, cb = param_callback)
    # time.sleep(1)
    cf.param.set_value(full_name, value)
    # time.sleep(1)

def _connected(uri):
    print("Connected!")

def _connection_failed(self, uri, msg):
    """Callback when connection initial connection fails (i.e no Crazyflie
    at the specified address)"""
    print('Connection to %s failed: %s' % (uri, msg))

def _connection_lost(uri, msg):
    """Callback when disconnected after a connection has been made (i.e
    Crazyflie moves out of range)"""
    print('Connection to %s lost: %s' % (uri, msg))

def _disconnected(self, uri):
    """Callback when the Crazyflie is disconnected (called in all cases)"""
    print('Disconnected from %s' % uri)

def limitThrust(thrust):
    if thrust >= 65500:
        thrust = 65500
    if thrust <= 35000:
        thrust = 35000
    return thrust

def limitAngle(angle):
    if angle >= 45:
        angle = 45
    elif angle <= -45:
        angle = -45
    return angle

def initialize_optitrack(rigid_body_id):
    clientAddress = sys.argv[3]
    optitrackServerAddress = sys.argv[4]

    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)

    streaming_client.rigid_body_listener = receive_rigid_body_frame

    is_running = streaming_client.run()

    time.sleep(2)

    if is_running and streaming_client.connected():
        print("Connected to Optitrack")
        return streaming_client
    else:
        print("Not connected to Optitrack")
        assert False

def initialize_crazyflie():
    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache='./cache')
    # cf.commander.set_client_xmode(True)

    cf.connected.add_callback(_connected)
    cf.disconnected.add_callback(_disconnected)
    cf.connection_failed.add_callback(_connection_failed)
    cf.connection_lost.add_callback(_connection_lost)
    cf.console.receivedChar.add_callback(console_callback)
    print('Connecting to crazyflie at %s' % uri)

    cf.open_link(uri)
    time.sleep(1)
    return cf

if __name__ == '__main__':
    try:
        streaming_client = initialize_optitrack(rigid_body_id)
        # time.sleep(2)

        # print(rotations.keys())

        p_r = np.array(positions[rigid_body_id][0:2])
        cf = initialize_crazyflie()

        # Unlock startup thrust protection
        cf.commander.send_setpoint(0, 0, 0, 0)

        # set_param(cf, 'motorPowerSet', 'enable', 1)
        # set_param(cf, 'stabilizer', 'controller', 2)

        # CONSTANT DEF
        count = 0                   # Counter
        theta_r = 0                 # Rotation of Robot
        v_des = 0                   # Desired Velocity
        p_d = np.array([5.4, 0, 2])     # Desired Position
        r = .2                      # Destination Threshold
        mass = 0.040                 # the mass of the quadrotor in kg
        f_b = 0.250                   # the net lift force of the balloon in N
        g = 9.81                    # Accelleration due to gravity
        e3 = np.array([0,0,1])      # Z unit vector
        yaw_d = 0                   # Desired Yaw

        # We want to control x, y, z, and yaw
        pid_x   =   pid.PID(1.5, 0.0, 1.0)
        pid_y   =   pid.PID(1.5, 0.0, 1.0)
        pid_z   =   pid.PID(3.0, 0.05, 2.0)
        pid_yaw =   pid.PID(0.5, 0.0, 0.5, True, 180.0)

        current_time = time.time()
        print("Into the loop now!")
        while(True):
            # Position
            p_r = np.array(positions[rigid_body_id][0:3])

            # positional error:
            # a nice PID updater that takes care of the errors including
            # proportional, integral, and derivative terms
            err_p = p_d - p_r
            fx = pid_x.Update(err_p[0])
            fy = pid_y.Update(err_p[1])
            fz = pid_z.Update(err_p[2])

            # desired force that we want the robot to generate in the {world frame}
            fd = np.array([fx, fy, fz]) + (mass * g - f_b) * e3

            # orientation of the robot
            rot = Rotation.from_quat(rotations[rigid_body_id][0:4])
            # in the format of SO(3)
            rot_SO3 = rot.as_matrix()
            # yaw angle
            yaw_r = rot.as_euler("xyz")[2]
            err_yaw = yaw_d - yaw_r

            # desired force that we want the robot to generate in the {body frame}
            fd = rot_SO3.T.dot(fd)

            # desired torque along yaw
            rate_yaw = pid_yaw.Update(err_yaw)

            normfd = np.linalg.norm(fd) # Magnitude

            xid = np.array([np.cos(yaw_d), np.sin(yaw_d), 0]) # intermediate xd
            zfd = fd/normfd

            yfd = np.cross(zfd, xid)
            normyfd = np.linalg.norm(yfd)
            yfd = yfd/normyfd

            xfd = np.cross(yfd, zfd)
            normxfd = np.linalg.norm(xfd)
            xfd = xfd/normxfd

            # Desired Rotation Matrix
            Rd = np.hstack((np.asmatrix(xfd).T, np.asmatrix(yfd).T, np.asmatrix(zfd).T))
            rd = Rotation.from_matrix(Rd)
            # print( np.linalg.det(Rd))
            # assert np.allclose(np.linalg.det(Rd), 1)
            #
            # Desired Roll Pitch and Yaw angles
            angles = rd.as_euler('xyz', degrees = True)
            # print(angles)
            roll = angles[0]
            pitch = angles[1]
            yaw = angles[2]

            # desired thrust
            thrust_correction = fd.dot(rot_SO3.dot(e3))
            cf.commander.send_setpoint(0*roll, 0*pitch, 0.0*rate_yaw, limitThrust(int(32000*thrust_correction)))

            count += 1
            if count >= 300:
                count = 0
                # print("Orientation of the Robot: ", p_r, "Rotation:", theta_r,"\nDesitnation: ",p_d)
                print("Current thrust = ", limitThrust(32000*thrust_correction))
                print("Current Pitch = ", pitch)
                print("Current Roll = ", roll)
                print("Current Yaw = ", rate_yaw, "\n\n")

    except KeyboardInterrupt:
        set_param(cf, 'motorPowerSet', 'enable', 0)
        cf.commander.send_setpoint(0, 0, 0, 0)
        # time.sleep(0.1)
        print("Completed")
