# UR_Kinematics
Kinematics of a UR5 robot arm and Basler CigE cameras.

This repository stores my work on the forward kinematics of a UR5 Universal Robot, to present objects to a set of cameras.


The way this repo is set up follows the following logic:
* RobotGUI.py is the heart of the application
* Corpus.py contains all the different elements like the robot, the modbus, and the cameras

We have the following components:
* CameraManagement.py contains the top camera and the detail camera
* RobotClass.py contains the robot functionality (sending commands, moving, etc)
* Readers.py contains the lower level interface of reading from the modbus

Then come the modules:
* ImageModule.pu is for treating images
* KinematicsModule computes the forward kinematics of the robot

The KinematicsModule is largely replaced by c scripts that can be found in the src directory. To compile these scripts we need a Makefile and a setup.py file.