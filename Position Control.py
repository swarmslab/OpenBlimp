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
import socket

def testArgs():
    if len(sys.argv) != 5:
        if len(sys.argv) < 5:
            print("ERROR: Insufficient arguemnts\nPlease use the following form:\npython Position Control.py <radio adddress> <rigid body number> <client address> <optitrack server address>")
        else:
            print("ERROR: Too many arguemnts\nPlease use the following form:\npython Position Control.py <radio adddress> <rigid body number> <client address> <optitrack server address>")
        sys.exit()

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
    cf.param.set_value(full_name, value)

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

def limitYawRate(yaw_rate):
    if yaw_rate >= 15:
        yaw_rate = 15
    if yaw_rate <= -15:
        yaw_rate = -15
    return yaw_rate
    

testArgs()
radio_address = sys.argv[1]
uri_address = 'radio://0/'+radio_address+'/2M/E7E7E7E7E7'
uri = uri_helper.uri_from_env(default = uri_address)
rigid_body_id = int(sys.argv[2])
logging.basicConfig(level=logging.ERROR)

positions = {}
rotations = {}

if __name__ == '__main__':
    try:
        streaming_client = initialize_optitrack(rigid_body_id)

        p_r = np.array(positions[rigid_body_id][0:2])
        cf = initialize_crazyflie()

        # Unlock startup thrust protection
        cf.commander.send_setpoint(0, 0, 0, 0)

        # CONSTANT DEF
        count = 0                            # Counter
        r = .2                               # Destination Threshold

        yaw_d = 0.0                          # Desired Yaw
        p_d = np.array([0, 0.0, 2.0])      # Desired Position
        
        # We want to control x, y, z, and yaw
        pid_x   =   pid.PID(.75, 0.0, 1.0)
        pid_y   =   pid.PID(.75, 0.0, 1.0)
        pid_z   =   pid.PID(3.0, 0.05, 2.0)
        pid_yaw =   pid.PID(.1, 0.000, 0.8, True, 180.0)

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

            # orientation of the robot
            rot = Rotation.from_quat(rotations[rigid_body_id][0:4])
            
            # yaw angle
            yaw_r = np.degrees(rot.as_euler("xyz")[2])
            err_yaw = yaw_d - yaw_r

            # desired torque along yaw
            rate_yaw = pid_yaw.Update(err_yaw) # Degrees/s
            
            cf.commander.send_force_setpoint(fx, fy, fz, 0)

            count += 1
            if count >= 200:
                count = 0

                print("fx = %.2f" % (fx))
                print("fy = %.2f" % (fy))
                print("fz = %.2f" % (fz))
                print("Yaw Rate = %.2f" % (rate_yaw))
                print("Robot Orientation = [x = %.2f, y = %.2f, z =%.2f, theta = %.2f]\n\n" % (p_r[0],p_r[1],p_r[2], yaw_r))

    except KeyboardInterrupt:
        set_param(cf, 'motorPowerSet', 'enable', 0)
        cf.commander.send_setpoint(0, 0, 0, 0)
        # time.sleep(0.1)
        print("Completed")
