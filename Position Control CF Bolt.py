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
from cflib.crazyflie.localization import Localization
import pid_controller as pid
from scipy.spatial.transform import Rotation
import numpy as np
from NatNetClient import NatNetClient
import matplotlib.pyplot as plt


def testArgs():
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
    radio_address = sys.argv[1]
    uri_address = 'radio://0/'+radio_address+'/2M/E7E7E7E706'
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

testArgs()
logging.basicConfig(level=logging.ERROR)

# GLOBAL VARIABLES ##############################################################################
positions = {}
rotations = {}
timestamp0 = 0
t2 = [0]
log_m1 = [0]
log_m2 = [0]
log_m3 = [0]
log_m4 = [0]

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
        p_d = np.array([5.4, 0.0, 1.5, 0])      # Desired Position and Yaw Angle
        count = 0                            # Counter
        r = .2                               # Destination Threshold
        mass = .15                          # Total mass of the blimp in kg
        massThrust = 32000                   # Thrust Gain
        
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
        
        # XY Position
        set_param(cf, 'ctrlMel', 'kp_xy', 0.0) #Default: 0.4
        set_param(cf, 'ctrlMel', 'kd_xy', 0.0) #Default: 0.2
        set_param(cf, 'ctrlMel', 'ki_xy', 0.0) #Default: 0.05
        # Z position
        set_param(cf, 'ctrlMel', 'kp_z', 1.4) #Default: 1.25
        set_param(cf, 'ctrlMel', 'kd_z', 0.6) #Default: 0.4
        set_param(cf, 'ctrlMel', 'ki_z', 0.0) #Default: 0.05
        # Attitude
        set_param(cf, 'ctrlMel', 'kR_xy', 0.15) #Default: 0.3
        set_param(cf, 'ctrlMel', 'kw_xy', 0.2) #Default: 0.15
        set_param(cf, 'ctrlMel', 'ki_m_xy', 0.0) #Default: 0.0
        # Yaw
        set_param(cf, 'ctrlMel', 'kR_z', 20000.0) #Default: 20000
        set_param(cf, 'ctrlMel', 'kw_z', 8000.0) #Default: 8000
        set_param(cf, 'ctrlMel', 'ki_m_z', 100.0) #Default: 100
        # Roll and pitch angular velocity
        set_param(cf, 'ctrlMel', 'kd_omega_rp', 0.0) #Default: 0.1
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
        start_time = time.time()
        while(True):
            now = time.time()
            
            # GET POSE ###############################################################################
            p_r = np.array(positions[rigid_body_id][0:3])
            q_r = rotations[rigid_body_id][0:4]
            rot = Rotation.from_quat(q_r)
            roll_r, pitch_r, yaw_r = rot.as_euler("xyz", degrees = True)
            
            # SEND DATA ###############################################################################
            cf.loc.send_extpose(p_r, q_r)
            cf.commander.send_position_setpoint(p_d[0] ,p_d[1] ,p_d[2] ,p_d[3])
            
            # PRINT ###################################################################################
            # count += 1
            # if count >= 300:
            #     count = 0
            #     print("Robot Orientation = [x = %.2f, y = %.2f, z =%.2f]\n[roll = %.2f pitch = %.2f yaw = %.2f]\n\n" % (p_r[0],p_r[1],p_r[2], roll_r, pitch_r, yaw_r))



    except KeyboardInterrupt:
        end_time = time.time()
        

        fig2, ax = plt.subplots(nrows = 2, ncols = 2)
        plt.suptitle("Motor Power")

        ax[0,1].plot(t2, log_m1, color="red", label="m1")
        ax[0,1].set_title("M1")
        ax[1,1].plot(t2, log_m2, color="blue", label="m2")
        ax[1,1].set_title("M2")
        ax[1,0].plot(t2, log_m3, color="green", label="m3")
        ax[1,0].set_title("M3")
        ax[0,0].plot(t2, log_m4, color="orange", label="m4")
        ax[0,0].set_title("M4")

        ax[0,0].minorticks_on()
        ax[0,0].grid(visible=True, which='major', color='#666666', linestyle='-')
        ax[0,0].grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)

        ax[0,1].minorticks_on()
        ax[0,1].grid(visible=True, which='major', color='#666666', linestyle='-')
        ax[0,1].grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)

        ax[1,0].minorticks_on()
        ax[1,0].grid(visible=True, which='major', color='#666666', linestyle='-')
        ax[1,0].grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)

        ax[1,1].minorticks_on()
        ax[1,1].grid(visible=True, which='major', color='#666666', linestyle='-')
        ax[1,1].grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)

        print("Total flight time: ", end_time - start_time)
        plt.show()
        
        print("Landing initiated")
        
        cf.commander.send_position_setpoint(p_r[0],p_r[1],0,yaw_r)
        
        set_param(cf, 'motorPowerSet', 'enable', 0)
        print("Completed")
