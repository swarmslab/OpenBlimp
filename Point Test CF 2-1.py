from cmath import pi
import logging
from math import gamma
import sys
import time
from tkinter import N

import matplotlib
from matplotlib import markers
from matplotlib import projections
from matplotlib.patches import Circle, Wedge, Polygon, Ellipse
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import matplotlib.patches as matpatches
from matplotlib.patches import Circle, Wedge, Polygon
from matplotlib.collections import PatchCollection
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import Log, LogConfig
from cflib.utils import uri_helper
from scipy.spatial.transform import Rotation
import numpy as np
from NatNetClient import NatNetClient
import matplotlib.pyplot as plt
import pickle
import os


def testArgs():
    """Function to test if system arguments are correctly formatted"""
    if len(sys.argv) != 8:
        if len(sys.argv) < 8:
            print("ERROR: Insufficient arguemnts\nPlease use the following form:\npython Point Test CF 2-1.py <radio adddress> <rigid body number> <client address> <optitrack server address> <x coordinate> <y coordinate> <z coordinate>")
        else:
            print("ERROR: Too many arguemnts\nPlease use the following form:\npython Point Test CF 2-1.py <radio adddress> <rigid body number> <client address> <optitrack server address> <x coordinate> <y coordinate> <z coordinate>")
        sys.exit()

def console_callback(text: str):
    '''A callback to run when we get console text from Crazyflie'''
    # We do not add newlines to the text received, we get them from the
    # Crazyflie at appropriate places.
    print(text, end='')

def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    """Recieve the rigid body frame data identified with the parameter robot_id from the optitrack server

    Args:
        robot_id (int): rigid body frame id defined in the optitrack server
        position (array): position of the body frame specified
        rotation_quaternion : orientation of body frame specified in quaternion
    """
    positions[robot_id] = position
    # rot = Rotation.from_quat(rotation_quaternion)
    rotations[robot_id] = rotation_quaternion

def param_callback(name, value):
    """Callback for crazyflie parameter data

    Args:
        name (string): name of parameter
        value (int): value of parameter
    """
    print('The crazyflie has parameter ' + name + ' set at number: ' + value)

def set_param(cf, groupstr, namestr, value):
    """Function to set parameters in the firmware of crazyflie specified

    Args:
        cf (crazyflie): Crazyflie to have its parameters changed
        groupstr (string): Parameter group
        namestr (string): Parameter name
        value (float): New parameter value
    """
    full_name = groupstr+"."+namestr
    cf.param.add_update_callback(group=groupstr, name = namestr, cb = param_callback)
    cf.param.set_value(full_name, value)

def _connected(uri):
    print("Connected!")

def _connection_failed(self, link_uri, msg):
    """Callback when connection initial connection fails (i.e no Crazyflie
    at the specified address)"""
    print('Connection to %s failed: %s' % (uri, msg))

def _connection_lost(uri, msg):
    """Callback when disconnected after a connection has been made (i.e
    Crazyflie moves out of range)"""
    print('Connection to %s lost: %s' % (uri, msg))

def _disconnected(self, link_uri):
    """Callback when the Crazyflie is disconnected (called in all cases)"""
    print('Disconnected from %s' % uri)
    sys.exit()

def initialize_optitrack():
    """Function to initialize optitrack tracking of rigid body frame specified in system arguments

    Returns:
        Streaming Client Id: Optitrack streaming client
        rigid_body_id: Rigid body specified in arguments
    """
    rigid_body_id = int(sys.argv[2])
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
        return streaming_client, rigid_body_id
    else:
        print("Not connected to Optitrack")
        assert False

def initialize_crazyflie():
    """Function to initialize the crazyflie for flight

    Returns:
        cf: Crazyflie object used to send commands to the crazyflie
    """
    global uri
    radio_address = sys.argv[1]
    uri_address = 'radio://0/'+radio_address+'/2M/E7E7E7E7E7'
    uri = uri_helper.uri_from_env(default = uri_address)
    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache='./cache')
    # cf.commander.set_client_xmode(True)

    cf.connected.add_callback(_connected)
    cf.console.receivedChar.add_callback(console_callback)
    print('Connecting to crazyflie at %s' % uri)

    cf.open_link(uri)
    time.sleep(1)
    return cf

def _motor_log_error(logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

def _motor_log_data(timestamp, data, logconf):
    """Callback from the log API when data arrives"""
    global timestamp0
    global t2
    global log_m1
    global log_m2
    global log_m3
    global log_m4
    t2.append(timestamp0)
    for name, value in data.items():
        if name == "motor.m1":
            log_m1.append(value)
        elif name == "motor.m2":
            log_m2.append(value)
        elif name == "motor.m3":
            log_m3.append(value)
        elif name == "motor.m4":
            log_m4.append(value)
    timestamp0 += 1

def log(timestamp, data, logconf):
    global stop
    for name, value in data.items():
        if name == "ctrlMel.i_err_x" and stop == 0:
            print("i_error_x: %f" % (value))
            
def _log_kalman(timestamp, data, logconf):
    global stop
    for name, value in data.items():
        if name == "kalman.stateX" and stop == 0:
            print("X position: %f" % (value))


# assumes input quaternion is normalized. will fail if not.
def quatcompress(q):
    # we send the values of the quaternion's smallest 3 elements.
    i_largest = 0
    for i in range(1,4,1): 
        if (abs(q[i]) > abs(q[i_largest])):
            i_largest = i

    # since -q represents the same rotation as q,
    # transform the quaternion so the largest element is positive.
    # this avoids having to send its sign bit.
    negate = q[i_largest] < 0

    # 1/sqrt(2) is the largest possible value
    # of the second-largest element in a unit quaternion.

    # do compression using sign bit and 9-bit precision per element.
    comp = i_largest
    for i in range(0,4,1):
        if (i != i_largest):
            negbit = (q[i] < 0) ^ negate
            mag = int(((1 << 9) - 1) * (abs(q[i]) / 0.5**(0.5)) + 0.5)
            comp = (comp << 10) | (negbit << 9) | mag
    return comp


testArgs()
logging.basicConfig(level=logging.ERROR)

# GLOBAL VARIABLES ##############################################################################
stop = 0
positions = {}
rotations = {}
timestamp0 = 0
t1 = []
t2 = [0]
log_m1 = [0]
log_m2 = [0]
log_m3 = [0]
log_m4 = [0]
log_destination_x = []
log_destination_y = []
log_destination_z = []
log_position_x = []
log_position_y = []
log_position_z = []
log_error_x = []
log_error_y = []
log_error_z = []
log_error_yaw = []
log_error_vel_x = []
log_error_vel_y = []
log_error_vel_z = []


if __name__ == '__main__':
    try:
        # INITIALIZATIONS ########################################################################
        streaming_client, rigid_body_id = initialize_optitrack()
        cf = initialize_crazyflie()
        
        # INITIAL POSE
        p_r = np.array(positions[rigid_body_id][0:3])   # Position of Robot
        q_r = rotations[rigid_body_id][0:4]             # Orientation of Robot in Quaternion
        rot = Rotation.from_quat(q_r)                   # Rotation object of Robot Orientation
        roll_r, pitch_r, yaw_r = rot.as_euler("xyz")                       # Yaw of Robot
        
        # CONSTANT DEFINITIONS ####################################################################
        p_d = np.array([float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7]), 0])      # Desired Position and Yaw Angle
        p_r_p = p_r                          # Previous Position
        v_r = np.array([0.0, 0.0, 0.0])      # Velocity Error
        v_p = v_r                            # Previous Velocity
        now = time.time()                    # Current Time
        count = 0                            # Counter
        r = .2                               # Destination Threshold
        mass = .08                           # Total mass of the blimp in kg
        massThrust = 45000                   # Thrust Gain
        start_time = time.time()             # Start of flight
        t = 0                                # Ticker
        height = 0.7                         # Minimum flight height
        p = p_r
        
        # UNLOCK THRUST PROTECTION ################################################################
        cf.commander.send_setpoint(0, 0, 0, 0)
        
        # SET PARAMETERS ##########################################################################
        set_param(cf, 'stabilizer', 'estimator', 2) #Set estimator to Kalman Filter
        time.sleep(2)
        
        #Set initial pose in Kalman Filter
        set_param(cf, 'kalman', 'initialX', p_r[0])
        set_param(cf, 'kalman', 'initialY', p_r[1])
        set_param(cf, 'kalman', 'initialZ', p_r[2])
        set_param(cf, 'kalman', 'initialYaw', yaw_r)
        
        set_param(cf, 'kalman', 'resetEstimation', np.uint8(1)) #Set estimator to use initial data
        p[0] = p_r[0] * 1000; # data.x is float(m) and poses.x is int16(mm)
        p[1] = p_r[1] * 1000; #
        p[2] = p_r[2] * 1000; #
        q = quatcompress(q_r); # this function compresses the quaternion (4 floats) into one int32
        cf.loc.send_extpose(p_r, q_r)
        
        print("Kalman estimator initialized")
        
        set_param(cf, 'ctrlMel', 'mass', mass) # Set robot mass
        set_param(cf, 'ctrlMel', 'massThrust', massThrust) # Set robot thrust gain
        
        # X Position
        set_param(cf, 'ctrlMel', 'kp_x', 1.0) #Default: 0.4
        set_param(cf, 'ctrlMel', 'kd_x', 2.0) #Default: 0.2
        set_param(cf, 'ctrlMel', 'ki_x', 0.07) #Default: 0.05
        set_param(cf, 'ctrlMel', 'i_range_x', 2.0) #Default: 2.0
        # Y Position
        set_param(cf, 'ctrlMel', 'kp_y', 0.8) #Default: 0.4
        set_param(cf, 'ctrlMel', 'kd_y', 1.6) #Default: 0.2
        set_param(cf, 'ctrlMel', 'ki_y', 0.07) #Default: 0.05
        # Z position
        set_param(cf, 'ctrlMel', 'kp_z', 1.55) #Default: 1.25
        set_param(cf, 'ctrlMel', 'kd_z', 0.5) #Default: 0.4
        set_param(cf, 'ctrlMel', 'ki_z', 0.05) #Default: 0.05
        
        # Attitude
        
        # Pitch
        set_param(cf, 'ctrlMel', 'kR_x', 0.1) #Default: 0.3
        set_param(cf, 'ctrlMel', 'kw_x', 0.05) #Default: 0.15
        set_param(cf, 'ctrlMel', 'ki_m_x', 0.1) #Default: 0.0
        # Roll
        set_param(cf, 'ctrlMel', 'kR_y', 0.1) #Default: 0.3
        set_param(cf, 'ctrlMel', 'kw_y', 0.05) #Default: 0.15
        set_param(cf, 'ctrlMel', 'ki_m_y', 0.1) #Default: 0.0
        # Yaw
        set_param(cf, 'ctrlMel', 'kR_z', 20000) #Default: 20000
        set_param(cf, 'ctrlMel', 'kw_z', 8000) #Default: 8000
        set_param(cf, 'ctrlMel', 'ki_m_z', 100) #Default: 100
        # Roll and pitch angular velocity
        set_param(cf, 'ctrlMel', 'kd_omega_rp', 0.1) #Default: 0.1
        time.sleep(0.5)

        # LOGGING #############################################################################
        _lg_motor = LogConfig(name='motor', period_in_ms=100)
        _lg_motor.add_variable('motor.m1', 'uint32_t')
        _lg_motor.add_variable('motor.m2', 'uint32_t')
        _lg_motor.add_variable('motor.m3', 'uint32_t')
        _lg_motor.add_variable('motor.m4', 'uint32_t')
        
        _lg_i_error = LogConfig(name='ctrlMel', period_in_ms=100)
        _lg_i_error.add_variable('ctrlMel.i_err_x', 'float')
        
        _lg_kalman = LogConfig(name="kalman", period_in_ms=100)
        _lg_kalman.add_variable("kalman.stateX", "float")
        
        try:
            cf.log.add_config(_lg_motor)
            _lg_motor.data_received_cb.add_callback(_motor_log_data)
            _lg_motor.error_cb.add_callback(_motor_log_error)
            _lg_motor.start()
            
            cf.log.add_config(_lg_i_error)
            _lg_i_error.data_received_cb.add_callback(log)
            #_lg_i_error.start()
            
            cf.log.add_config(_lg_kalman)
            _lg_kalman.data_received_cb.add_callback(_log_kalman)
            #_lg_kalman.start()
            print("logs created")
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add log config, bad configuration.')


        # LOOP #######################################################################################
        print("Into the loop now!")
        start_time = time.time()
        then = start_time
        t = 0
        angle = 0
        while(True):
            t += 1
            p_d = np.array([float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7]), 0])
            v_d = 0
            
            # GET POSE ###############################################################################
            p_r_p = p_r
            p_r = np.array(positions[rigid_body_id][0:3])
            q_r = rotations[rigid_body_id][0:4]
            rot = Rotation.from_quat(q_r)
            roll_r, pitch_r, yaw_r = rot.as_euler("xyz", degrees = True)

            # SEND DATA ###############################################################################
            cf.commander.send_position_setpoint(p_d[0] ,p_d[1] ,p_d[2] ,p_d[3])
            p[0] = p_r[0] * 1000; # data.x is float(m) and poses.x is int16(mm)
            p[1] = p_r[1] * 1000; #
            p[2] = p_r[2] * 1000; #
            q = quatcompress(q_r); # this function compresses the quaternion (4 floats) into one int32
            cf.loc.send_extpose(p_r, q_r)
            
            # CALCULATE ERROR #########################################################################
            err_x = p_d[0] - p_r[0]
            err_y = p_d[1] - p_r[1]
            err_z = p_d[2] - p_r[2]
            err_yaw = p_d[3] - yaw_r
            
            dp_r = p_r - p_r_p
            then = now
            now = time.time()
            dt = now - then
            v_p = v_r
            if np.allclose(dt, 0):
                v_r = v_p
            else:
                v_r = dp_r/dt
            
            v_err = v_d - v_r
            
            # LOG DATA ################################################################################
            t1.append(t)
            log_destination_x.append(p_d[0])
            log_destination_y.append(p_d[1])
            log_destination_z.append(p_d[2])
            log_position_x.append(p_r[0])
            log_position_y.append(p_r[1])
            log_position_z.append(p_r[2])
            log_error_x.append(err_x)
            log_error_y.append(err_y)
            log_error_z.append(err_z)
            log_error_yaw.append(err_yaw)
            log_error_vel_x.append(v_err[0])
            log_error_vel_y.append(v_err[1])
            log_error_vel_z.append(v_err[2])
            
            # PRINT ###################################################################################
            count += 1
            if count >= 300:
                count = 0
                print("Robot Position: [x = %.2f, y = %.2f, z =%.2f]\nRobot Orientation:[roll = %.2f pitch = %.2f yaw = %.2f]\n\n" 
                      % (p_r[0],p_r[1],p_r[2], roll_r, pitch_r, yaw_r))
                print("Current Flight Duration: ", time.time()-start_time, " seconds")



    except KeyboardInterrupt:
        stop = 1
        end_time = time.time()
        flightTime = end_time - start_time
        print("Total flight time: ", flightTime)
        
        # PICKLE ###################################################################################
        directory = ("Experiments/Trajectory Experiment-"+time.asctime(time.localtime())).replace(":", "-")
        os.mkdir(directory)
        with open("./"+directory+"/log_dest_x.dat", "wb") as file:
            pickle.dump(log_destination_x, file)
        with open("./"+directory+"/log_dest_y.dat", "wb") as file:
            pickle.dump(log_destination_y, file)
        with open("./"+directory+"/log_dest_z.dat", "wb") as file:
            pickle.dump(log_destination_z, file)
        with open("./"+directory+"/log_pos_x.dat", "wb") as file:
            pickle.dump(log_position_x, file)
        with open("./"+directory+"/log_pos_y.dat", "wb") as file:
            pickle.dump(log_position_y, file)
        with open("./"+directory+"/log_pos_z.dat", "wb") as file:
            pickle.dump(log_position_z, file)
        with open("./"+directory+"/log_err_x.dat", "wb") as file:
            pickle.dump(log_error_x, file)
        with open("./"+directory+"/log_err_y.dat", "wb") as file:
            pickle.dump(log_error_y, file)
        with open("./"+directory+"/log_err_z.dat", "wb") as file:
            pickle.dump(log_error_z, file)
        with open("./"+directory+"/log_err_yaw.dat", "wb") as file:
            pickle.dump(log_error_yaw, file)
        with open("./"+directory+"/t.dat", "wb") as file:
            pickle.dump(t, file)
        with open("./"+directory+"/time.dat", "wb") as file:
            pickle.dump(flightTime, file)
        with open("./"+directory+"/log_m1", "wb") as file:
            pickle.dump(log_m1, file)
        with open("./"+directory+"/log_m2", "wb") as file:
            pickle.dump(log_m2, file)
        with open("./"+directory+"/log_m3", "wb") as file:
            pickle.dump(log_m3, file)
        with open("./"+directory+"/log_m4", "wb") as file:
            pickle.dump(log_m4, file)
        with open("./"+directory+"/t2.dat", "wb") as file:
            pickle.dump(t2, file)
        
        
        
        # TRAJECTORY ###################################################################################
        trajectory, axes = plt.subplots(nrows=1, ncols=1, figsize = (6,6))
        axes = plt.axes(projection="3d")
        plt.suptitle("Path")
        axes.scatter(log_destination_x, log_destination_y, log_destination_z, color="blue",marker="x",label="Trajectory")
        # r = 0
        # g = 0
        # b = 0
        # cycle = 0
        # for i in range(len(log_position_x)):
        #     cycle = i // 255
        #     if r < 255:
        #         r = i - 255 * cycle
        #     elif g < 255:
        #         g = i - 255 * cycle
        #     elif b < 255:
        #         b = i - 255 * cycle
        #     elif r == g and g == b and b == 255:
        #         r = 0
        #         g = 0
        #         b = 0
        #     axes.scatter(log_position_x[i], log_position_y[i], log_position_z[i], color=(r/255.0,g/255.0,b/255.0), label="Position")
        axes.plot3D(log_position_x, log_position_y, log_position_z, color="orange", label="Position")
        
        axes.minorticks_on()
        axes.grid(visible=True, which='major', color='#666666', linestyle='-')
        axes.grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
        trajectory.tight_layout()
        plt.savefig("./"+directory+"/Trajectory.png")
        
        position, ax1 = plt.subplots(nrows = 3, ncols=1, figsize=(6, 8))
        plt.suptitle("Position")
        ax1[0].plot(t1, log_position_x, color="orange", label="Robot")
        ax1[0].plot(t1, log_destination_x, color="blue", label="Destination")
        ax1[0].set_title("X Position")
        
        ax1[1].plot(t1, log_position_y, color="red", label="Robot")
        ax1[1].plot(t1, log_destination_y, color="blue", label="Destination")
        ax1[1].set_title("Y Position")
        
        ax1[2].plot(t1, log_position_z, color="green", label="Robot")
        ax1[2].plot(t1, log_destination_z, color="blue", label="Destination")
        ax1[2].set_title("Z Position")
        
        ax1[0].minorticks_on()
        ax1[0].grid(visible=True, which='major', color='#666666', linestyle='-')
        ax1[0].grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)

        ax1[1].minorticks_on()
        ax1[1].grid(visible=True, which='major', color='#666666', linestyle='-')
        ax1[1].grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)

        ax1[2].minorticks_on()
        ax1[2].grid(visible=True, which='major', color='#666666', linestyle='-')
        ax1[2].grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
        position.tight_layout()
        plt.savefig("./"+directory+"/Position.png")
        
        
        # MOTOR POWER ###################################################################################
        motorPower, ax2 = plt.subplots(nrows = 4, figsize=(6,8))
        plt.suptitle("Motor Power")

        ax2[0].plot(t2, log_m1, color="red", label="m1")
        ax2[0].set_title("M1")
        ax2[1].plot(t2, log_m2, color="blue", label="m2")
        ax2[1].set_title("M2")
        ax2[2].plot(t2, log_m3, color="green", label="m3")
        ax2[2].set_title("M3")
        ax2[3].plot(t2, log_m4, color="orange", label="m4")
        ax2[3].set_title("M4")

        ax2[0].minorticks_on()
        ax2[0].grid(visible=True, which='major', color='#666666', linestyle='-')
        ax2[0].grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)

        ax2[1].minorticks_on()
        ax2[1].grid(visible=True, which='major', color='#666666', linestyle='-')
        ax2[1].grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)

        ax2[2].minorticks_on()
        ax2[2].grid(visible=True, which='major', color='#666666', linestyle='-')
        ax2[2].grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)

        ax2[3].minorticks_on()
        ax2[3].grid(visible=True, which='major', color='#666666', linestyle='-')
        ax2[3].grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
        motorPower.tight_layout()
        plt.savefig("./"+directory+"/Motor Power.png")
        
        # ERROR ###################################################################################
        error, ax3 = plt.subplots(nrows=5, ncols=1, figsize=(6,8))
        ax3[0].plot(t1, log_error_x, color="red")
        ax3[0].set_title("X error")
        ax3[1].plot(t1, log_error_y, color="blue")
        ax3[1].set_title("Y error")
        ax3[2].plot(t1, log_error_z, color="green")
        ax3[2].set_title("Z error")
        ax3[3].plot(t1, log_error_yaw, color="orange")
        ax3[3].set_title("Yaw error")
        ax3[4].plot(t1, log_error_vel_x, color="red")
        ax3[4].plot(t1, log_error_vel_y, color="blue")
        ax3[4].plot(t1, log_error_vel_z, color="green")
        ax3[4].set_title("Velocity Error")

        ax3[0].minorticks_on()
        ax3[0].grid(visible=True, which='major', color='#666666', linestyle='-')
        ax3[0].grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)

        ax3[1].minorticks_on()
        ax3[1].grid(visible=True, which='major', color='#666666', linestyle='-')
        ax3[1].grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)

        ax3[2].minorticks_on()
        ax3[2].grid(visible=True, which='major', color='#666666', linestyle='-')
        ax3[2].grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)

        ax3[3].minorticks_on()
        ax3[3].grid(visible=True, which='major', color='#666666', linestyle='-')
        ax3[3].grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
        
        ax3[4].minorticks_on()
        ax3[4].grid(visible=True, which='major', color='#666666', linestyle='-')
        ax3[4].grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
        
        error.tight_layout()
        plt.savefig("./"+directory+"/Error.png")
        
        # PAPER FIGURE ##################################################################################
        figure, ax4 = plt.subplots(nrows= 2, ncols=1, figsize=(6,10), sharex="col")
        
        index = 10
        
        t1 = list(map(lambda x: x/t1[-1]*flightTime,t1))
    
        ax4[1].set_xlabel("Time (s)")
        ax4[0].set_ylabel("Position Error (m)")
        ax4[1].set_ylabel("Velocity Error (m/s)")
        
        
        ax4[0].plot(t1[index:], log_error_x[index:], color="red", label="X Error")
        ax4[0].plot(t1[index:], log_error_y[index:], color="magenta", label="Y Error")
        ax4[0].plot(t1[index:], log_error_z[index:], color="green", label="Z Error")
        ax4[1].plot(t1[index:], log_error_vel_x[index:], color="red", label="X Velocity Error")
        ax4[1].plot(t1[index:], log_error_vel_y[index:], color="magenta", label="Y Velocity Error")
        ax4[1].plot(t1[index:], log_error_vel_z[index:], color="green", label="Z Velocity Error")

        ax4[0].minorticks_on()
        ax4[0].grid(visible=True, which='major', color='#666666', linestyle='-')
        ax4[0].grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)

        ax4[1].minorticks_on()
        ax4[1].grid(visible=True, which='major', color='#666666', linestyle='-')
        ax4[1].grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)

        ax4[0].legend()
        ax4[1].legend()
        figure.tight_layout()
        plt.savefig("./"+directory+"/Paper_fig.png")
        
        
        plt.show()
        set_param(cf, 'motorPowerSet', 'enable', 0)
