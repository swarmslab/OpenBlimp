# openBlimp
A project that uses a Crazyflie Quad-rotors to actuate a blimp as well as a simulator that can be used to test factors that affect performance in the real world. 

## Getting Started
Fork and clone this repository to your local machine. 

### Prerequisites

To utilize this repository properly you will need the following:
- Optitrack camera system and server
- Crazyflie 2.X
- Crazyradio
- WiFi connection

## Usage
To use this project, the primary files are "Trajectory CF 2-1.py" and "Point Test CF 2-1.py". These two files control the Crazyflie 2.1 by sending external pose and either a dynamically changing destination setpoint, or a fixed destination setpoint.

To use "Point Test CF 2-1.py", use the following format:
```
$ python3 'Point Test CF 2-1.py' <radio address> <rigid body id> <client address> <optitrack server address> <x coordinate> <y coordinate> <z coordinate>
```
To use "Trajectory CF 2-1.py" use the following format:
```
$ python3 'Trajectory CF 2-1.py' <radio address> <rigid body id> <client address> <optitrack server address>
```
The desired trajectory can be changed within "Trajectory CF 2-1.py" by changing which function is called in the `while(true)` loop. The current options are `circle_trajectory`, `figure_eight_trajectory`, and `helix_trajectory`. Each of these functions take a number of parameters that describe the trajectory.

To stop the flight one may kill the python instance, shake the drone somewhat forcefully, or use the command `ctrl + c` which will exit the loop, generate a folder structure, store the data from that flight, and generate a series of graphs to display flight metrics.

## Addition notes
- The crazyradio can have issues maintaining connectivity unless it recieves a constant and uninterrupted supply of power. A powered external usb hub can aid in this.
- There is a parameter in each of the trajectory functions `rate` that is currently arbitrary but does control the speed at which the setpoint moves.
- Currently there is an issue with the python library cflib that causes problems when the yaw angle of the blimp exceeds 90 degrees or -90 degrees.
- Within "Trajectory CF 2-1.py" and "Point Test CF 2-1.py"
    - Under the heading `CONSTANT DEFINITIONS` there are a number of variables that are dependant on the drone being flown
    - Under the heading `SET PARAMETERS` there are a number of parameters being set that tune the onboard controller to the specific drone being flown.



- The primary script for this project is "Trajectory CF 2-1.py". This script is used to control a blimp actuated by a crazyflie 2.X by sending it its position via the Optitrack positioning system, as well as a dynamically changing setpoint. Currently there are three trajectory methods: `circle_trajectory`, `figure_eight_trajectory`, and `helix_trajectory`. These methods can be called in the while loop of the main method to change the desired trajectory for the quadrotor-blimp.


## Examples
- PUT EXAMPLES HERE

# Dependencies that is already included
- cflib (https://github.com/bitcraze/crazyflie-lib-python) is an API written in Python that is used to communicate with the Crazyflie
and Crazyflie 2.0 quadcopters. It is intended to be used by client software to
communicate with and control a Crazyflie quadcopter. 

- (Temporary) natnet for Optitrack in Python (https://github.com/swarmslab/optitrack_natnet) is a simple Python script to connect to the Optitrack Motion Capture System.
