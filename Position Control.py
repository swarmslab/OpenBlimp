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
import matplotlib.pyplot as plt

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
        timer = 0
        t = [0]
        log_yaw = [0]
        log_yaw_rate = [0]
        log_Cd = [0]
        r = .2                               # Destination Threshold
        mass = 0.04                          # the mass of the quadrotor in kg
        total_mass = .018                    # Total mass of the blimp in kg
        f_b = 0.25                           # the net lift force of the balloon in N
        g = 9.81                             # Accelleration due to gravity
        e3 = np.array([0,0,1])               # Z unit vector

        yaw_d = 0.0                          # Desired Yaw
        roll_d = 0.0                         # Desired Roll
        pitch_d = 0.0                        # Desired Pitch
        p_d = np.array([5.4, 0.0, 2.0])      # Desired Position

        # We want to control x, y, z, and yaw
        pid_x       =   pid.PID(0.0, 0.0, 0.0)
        pid_y       =   pid.PID(0.0, 0.0, 0.0)
        pid_z       =   pid.PID(3.0, 0.05, 2.0)
        pid_roll    =   pid.PID(0.1, 0.0, 0.1, True, 180.0)
        pid_pitch   =   pid.PID(0.1, 0.0, 0.1, True, 180.0)
        pid_yaw     =   pid.PID(20.0, 0.0, 30.0, True, 180.0)


        print("Into the loop now!")
        while(True):
            now = time.time()
            # Position
            p_r = np.array(positions[rigid_body_id][0:3])

            # positional error:
            # a nice PID updater that takes care of the errors including
            # proportional, integral, and derivative terms
            err_p = p_d - p_r
            fx = pid_x.Update(err_p[0],current_time = now)
            fy = pid_y.Update(err_p[1],current_time = now)
            fz = pid_z.Update(err_p[2],current_time = now)

            # desired force that we want the robot to generate in the {world frame}
            # fd = np.array([fx, fy, fz]) + (mass * g - f_b) * e3
            fd = np.array([fx, fy, fz]) + (total_mass*g)*e3

            # orientation of the robot
            rot = Rotation.from_quat(rotations[rigid_body_id][0:4])

            # yaw angle
            roll_r, pitch_r, yaw_r = rot.as_euler("xyz")
            err_roll = roll_d - roll_r
            err_pitch = pitch_d - pitch_r
            err_yaw = yaw_d - yaw_r

            # desired torque along roll pitch and yaw
            rate_roll = pid_roll.Update(err_roll,current_time = now)
            rate_pitch = pid_pitch.Update(err_pitch,current_time = now)
            rate_yaw = pid_yaw.Update(err_yaw,current_time = now)
            
            
            rot_compensate = Rotation.from_euler("xy", [rate_roll, rate_pitch]).as_matrix()
            
            fd = rot_compensate.T.dot(fd)
#--------------------------------------------fx------fy-----fz------yaw rate------------------------
# problems list:
# 1. Blimp constantly pitches backwards no matter what
# 2. The Roll correction is unstable and causes oscilations
# 3. The yaw control has a problem with momentum. When the yaw overcorrects, it spins so far past its 
#   desired location that it circles back around to having the same sign of yaw rate, adding energy to the spin


            cf.commander.send_force_setpoint(fd[0], fd[1], fd[2], rate_yaw)
            count += 1
            timer += 1
            t.append(timer)
            log_yaw.append(np.degrees(yaw_r))
            log_yaw_rate.append(rate_yaw)
            log_Cd.append(pid_yaw.Cd*pid_yaw.Kd)
            if count >= 20:
                count = 0
                print("fx = %.2f" % (fd[0]))
                print("fy = %.2f" % (fd[1]))
                print("fz = %.2f" % (fd[2]))
                print("Yaw Rate = %.2f" % (rate_yaw))
                print("Robot Orientation = [x = %.2f, y = %.2f, z =%.2f]\n[roll = %.2f, pitch = %.2f, yaw = %.2f]\n\n" % (p_r[0],p_r[1],p_r[2], np.degrees(roll_r), np.degrees(pitch_r), np.degrees(yaw_r)))

    except KeyboardInterrupt:   
        
        fig, ax = plt.subplots(1, figsize=(8, 6))

        # Set the title for the figure
        fig.suptitle('Yaw, Yaw Rate, and Yaw Error Derivative', fontsize=15)

        # Draw all the lines in the same plot, assigning a label for each one to be
        # shown in the legend.
        ax.plot(t, log_yaw, color="red", label="Yaw")
        ax.plot(t, log_yaw_rate, color="green", label="Yaw Rate")
        ax.plot(t, log_Cd, color="blue", label="Error Derivative")

        # Add a legend, and position it on the lower right (with no box)
        plt.legend(loc="lower right", title="Key", frameon=False)

        plt.show()

        set_param(cf, 'motorPowerSet', 'enable', 0)
        cf.commander.send_force_setpoint(0, 0, 0, 0)
        print("Completed")