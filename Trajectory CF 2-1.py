from cmath import pi
import logging
from math import gamma
import sys
import time
from tkinter import N

from cv2 import circle
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from scipy.spatial.transform import Rotation
import numpy as np
from NatNetClient import NatNetClient
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import pickle
import os

# plt.rc('font', family='serif')
# plt.rc('text', usetex=True)
# plt.rc('xtick', labelsize=20)
# plt.rc('ytick', labelsize=20)
# plt.rc('figure', figsize=(6,12))
# plt.rc('legend', fontsize=20)
# plt.rc('axes', titlesize="large")
# plt.rc("axes", labelsize="large")


def test_args():
    """Function to test if system arguments are correctly formatted"""
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

def circle_trjactory(t, radius, center_x, center_y, height, yaw):
    """Q-AB trajectory in a circle of set radius around a given point

    Args:
        t (int): counter
        radius (float): radius of trajectory in m
        center_x (float): x coordinate of center of circle
        center_y (float): y coordinate of center of circle
        height (float): height for Q-AB to fly at in m
        yaw (float): desired yaw angle in degrees

    Returns:
        Array: Desired position at cycle t
    """
    global type
    type = "c"
    return np.array([radius*np.cos(.000175*t) + center_x, radius*np.sin(.000175*t) + center_y, height, yaw]), np.array([-0.000175*radius*np.sin(0.000175*t), 0.000175*radius*np.sin(0.000175*t), 0.0])

def circle_trjactory_vertical(t, radius, center_x, center_y, center_z, yaw):
    """Q-AB trajectory in a circle of set radius around a given point

    Args:
        t (int): counter
        radius (float): radius of trajectory in m
        center_x (float): x coordinate of center of circle
        center_y (float): y coordinate of center of circle
        center_z (float): z coordinate of center of circle
        yaw (float): desired yaw angle in degrees

    Returns:
        Array: Desired position at cycle t
    """
    global type
    type = "cv"
    return np.array([radius*np.cos(np.radians(.01*t-90)) + center_x, center_y, radius*np.sin(np.radians(.01*t-90))+ center_z, yaw])

def helix_trajectory(t, radius, center_x, center_y, max_height, yaw):
    """Q-AB trajectory in a helix of set radius around a given point

    Args:
        t (int): counter
        radius (float): radius of trajectory in m
        center_x (float): x coordinate of center of circle
        center_y (float): y coordinate of center of circle
        max_height (float): max height for Q-AB to reach
        yaw (float): desired yaw angle in degrees

    Returns:
        Array: Desired position at cycle t
    """
    global height
    global type
    type = "h"
    height += 0.00002
    if height > max_height:
        height = max_height
    return np.array([radius*np.cos(.000349*t) + center_x, radius*np.sin(.000349*t) + center_y, height, yaw]), np.array([-radius*.000349*np.sin(.000349*t), radius*.000349*np.cos(.000349*t), .00002])

test_args()
logging.basicConfig(level=logging.ERROR)

# GLOBAL VARIABLES ##############################################################################
type = ""
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
        p_d = np.array([0.0, 0.0, 0.0, 0.0]) # Desired Position and Yaw Angle
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
        
        # MOTOR LOGGING #############################################################################
        _lg_motor = LogConfig(name='motor', period_in_ms=100)
        _lg_motor.add_variable('motor.m1', 'uint32_t')
        _lg_motor.add_variable('motor.m2', 'uint32_t')
        _lg_motor.add_variable('motor.m3', 'uint32_t')
        _lg_motor.add_variable('motor.m4', 'uint32_t')
        try:
            cf.log.add_config(_lg_motor)
            _lg_motor.data_received_cb.add_callback(_motor_log_data)
            _lg_motor.error_cb.add_callback(_motor_log_error)
            _lg_motor.start()
            print("motor power log created")
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')


        # LOOP #######################################################################################
        print("Into the loop now!")
        while(True):
            t += 1
            p_d, v_d = helix_trajectory(t, 2, 0, 0, 2.5,0)
            
            
            # GET POSE ###############################################################################
            p_r_p = p_r
            p_r = np.array(positions[rigid_body_id][0:3])
            q_r = rotations[rigid_body_id][0:4]
            rot = Rotation.from_quat(q_r)
            roll_r, pitch_r, yaw_r = rot.as_euler("xyz", degrees = True)
            
            # SEND DATA ###############################################################################
            cf.loc.send_extpose(p_r, q_r)
            cf.commander.send_position_setpoint(p_d[0] ,p_d[1] ,p_d[2] ,p_d[3])
            
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
                print("Destination: [%.2f, %.2f, %.2f, %.2f]\n" % (p_d[0], p_d[1], p_d[2], p_d[3]))
                # # print("Location: [%.2f, %.2f, %.2f, %.2f]\n" % (p_r[0], p_r[1], p_r[2], yaw_r))
                print("Robot Orientation = [x = %.2f, y = %.2f, z =%.2f]\n[roll = %.2f pitch = %.2f yaw = %.2f]\n\n" % (p_r[0],p_r[1],p_r[2], roll_r, pitch_r, yaw_r))


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
        with open("./"+directory+"/log_err_vel_x.dat", "wb") as file:
            pickle.dump(log_error_vel_x, file)
        with open("./"+directory+"/log_err_vel_y.dat", "wb") as file:
            pickle.dump(log_error_vel_y, file)
        with open("./"+directory+"/log_err_vel_z.dat", "wb") as file:
            pickle.dump(log_error_vel_z, file)
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
        plt.suptitle("Path")
        if type != "c":
            axes = plt.axes(projection="3d")
            axes.plot3D(log_destination_x, log_destination_y, log_destination_z, color="blue",label="Trajectory")
            axes.plot3D(log_position_x, log_position_y, log_position_z, color="orange", label="Position")
        else:  
            axes.plot(log_destination_x,log_destination_y, color="blue", label="Trajectory")
            axes.plot(log_position_x, log_position_y, color="orange", label="Path")
        
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
        index = 10
        figure = plt.figure(figsize=(6,10))
        if type != "c":
            ax4 = figure.add_subplot(3,1,1, projection="3d")
            ax4.plot3D(log_destination_x[index:], log_destination_y[index:], log_destination_z[index:], color="blue",label="Trajectory")
            ax4.plot3D(log_position_x[index:], log_position_y[index:], log_position_z[index:], color="orange", label="Position")
            ax4.set_xlabel("x (m)")
            ax4.set_ylabel("y (m)")
            ax4.set_zlabel("z (m)")
            ax4.set_box_aspect((1,1,1))
        else:  
            ax4 = figure.add_subplot(3,1,1)
            ax4.plot(log_destination_x[index:],log_destination_y[index:], color="blue", label="Trajectory")
            ax4.plot(log_position_x[index:], log_position_y[index:], color="orange", label="Path")
            ax4.set_xlabel("x (m)")
            ax4.set_ylabel("y (m)")
            
        ax4.minorticks_on()
        ax4.grid(visible=True, which='major', color='#666666', linestyle='-')
        ax4.grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
            
        t1 = list(map(lambda x: x/t1[-1]*flightTime,t1))
    
        ax4 = figure.add_subplot(3,1,2)
        
        ax4.set_ylabel("Position Error (m)")
        ax4.plot(t1[index:], log_error_x[index:], color="red", label="X Error")
        ax4.plot(t1[index:], log_error_y[index:], color="magenta", label="Y Error")
        ax4.plot(t1[index:], log_error_z[index:], color="green", label="Z Error")
        ax4.minorticks_on()
        ax4.grid(visible=True, which='major', color='#666666', linestyle='-')
        ax4.grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
        ax4.legend()
        
        ax4 = figure.add_subplot(3,1,3)
        
        ax4.set_xlabel("Time (s)")
        ax4.set_ylabel("Velocity Error (m/s)")
        ax4.plot(t1[index:], log_error_vel_x[index:], color="red", label="X Velocity Error")
        ax4.plot(t1[index:], log_error_vel_y[index:], color="magenta", label="Y Velocity Error")
        ax4.plot(t1[index:], log_error_vel_z[index:], color="green", label="Z Velocity Error")

        ax4.minorticks_on()
        ax4.grid(visible=True, which='major', color='#666666', linestyle='-')
        ax4.grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)

        ax4.legend()
        figure.tight_layout()
        plt.savefig("./"+directory+"/Paper_fig.png")
        
        
        plt.show()
        set_param(cf, 'motorPowerSet', 'enable', 0)