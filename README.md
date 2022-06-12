# openBlimp
A project uses Crazyflies to control a blimp. 
- Currently we are starting to fly things. One script that is (should be) under rapid update is "Position Control.py" which drives the crazyflie to a desired position under the localization provided by an Optitrack system and onboard sensors.
- Before moving to the next step, we have to make sure the position tracking works no matter the starting point.
- Then, next step
  - Work on the stability analysis with the angle hold -> assigned to Jiawei
  - Onboard localization methods (Since crazyflie provides pretty solid orientation, we would like to work on obtaining the position of the drone)
  - Onboard computations (Phone? Arduino? Raspberry Pi? Others?)
  - Object recognition (Camera! Others? Do we need a depth sensor?)

# Demonstration
Once we had most of the todos work, we will come back to create some demonstrations.

# Dependencies that is already included
- cflib (https://github.com/bitcraze/crazyflie-lib-python) is an API written in Python that is used to communicate with the Crazyflie
and Crazyflie 2.0 quadcopters. It is intended to be used by client software to
communicate with and control a Crazyflie quadcopter. 

- (Temporary) natnet for Optitrack in Python (https://github.com/swarmslab/optitrack_natnet) is a simple Python script to connect to the Optitrack Motion Capture System.
