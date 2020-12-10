''' DESCRIPTION
This is a short tutorial how to use the python module 'Robot_module.py' to control JLI's Robots.

The following program will show a simpel step by ste tutorial how to set up the robot to run the program through some specific positions.
Use the python interpreter "Python 3.5 (Classify_Lego_Bricks)
'''
import math
import time


#Step 0 - Import the module
import Robot_module as Rm



# Step 1 - Set up the connection to the robot.

# Define a initial position for the joints.
# The robot got 6 joints, which is can be set in the range -2*pi to 2*pi degress
joint_pos_rest = [61.42*math.pi/180, -93*math.pi/180, 94.65*math.pi/180, -91.59*math.pi/180, -90*math.pi/180, 0*math.pi/180]

# Init the robot connection, by making the robot go to the joint_idle position and open the gripper.
# The function returns a connection to the robot, which we will use to communicate with it.
Robot_connection = Rm.robot_init(joint_pos_rest)



# Step 2 - Set up positions for the robot
# The Easiest way to set up a position is to use the robot controller and move the robot to where you want it.
# When the robot is in the position you want, you can read the coordinates directly from the move panel ( remember to set the view to 'Base'.)
# Read the position from the display - conver from mm to m for the robot to understand.
# The structure is [X,Y,Z,RX,RY,RZ], where X,Y and Z are coordinates and Rx, Ry and RZ is rotations/angles.
pos_brick_up = [0.251,   -0.437,  0.325,  3.14,  0.00,  0.00]

# Another way to get a robot position could be to move the robot to the position you want and then run the following command to get your.
#pos_brick_up = getPosition()

#Other positions
pos_brick_up =   [ 0.251,   -0.437,  0.325,  3.14,  0.00,  0.00]
pos_brick_down = [ 0.251,   -0.437,  0.014,  3.14,  0.00,  0.00]
pos_brick_drop = [ 0.085,   -0.516,  0.050,  0.00,  3.14,  0.00]
pos_read_up =    [-0.48135, -0.13387,  0.325,   1.29,  -1.29,  0.00]
pos_read_down =  [-0.48135, -0.13387,  0.0530,  -1.29,  1.29,  0.00]



# Step 3 - Pick up object

# Move the robot using the command robot_movej - optimal move.
# p: Defines weather the position is a position og a joint position.
Rm.robot_movej(Robot_connection, pos_brick_up, wait=1, p=True)
# Move the robot using the command robot_movel - move in a straight line for better control.
Rm.robot_movel(Robot_connection, pos_brick_down, wait=1, p=True)


# Open  the gripper by setting thd digital I/O 0 to false.
Robot_connection.send(b'set_digital_out(0, False)' + b"\n")
time.sleep(0.5) #Wait for the gripper

# Move robot
Rm.robot_movel(Robot_connection, pos_brick_up, wait=1, p=True)

# Move the robot using the command robot_movel - move in a straight line for better control.
# Always go through a control position, like this one, to make sure the robot does hit the glass or spin to much.
Rm.robot_movej(Robot_connection, joint_pos_rest , wait=1, p=False)



# Step 4 - Move object in front of camera and rotate it

# If you know the specific position but you want a another angle you can use the function 'RPY2rotvec' which takes in
# 3 rotation around the x, y and z axis in radians converts it to a measure the robot understands.
# You can see the axis on the robot controller. Use only 2 axis - it makes it easier.
pos_read_up [3], pos_read_up [4], pos_read_up [5] = Rm.RPY2rotvec(math.pi*3/4, 0, math.pi/2)
pos_read_down[3:] = pos_read_up[3:]


# Move robot - Show object to camera in different angles.
# Move directly over the camera position first, to avoid collisions with the camera or light sources
Rm.robot_movej(Robot_connection, pos_read_up, wait=1, p=True)
Rm.robot_movel(Robot_connection, pos_read_down, wait=1, p=True)

pos_read_down[3], pos_read_down[4], pos_read_down[5] = Rm.RPY2rotvec(math.pi, 0, math.pi/2)
Rm.robot_movel(Robot_connection, pos_read_down, wait=1, p=True)

pos_read_down[3], pos_read_down[4], pos_read_down[5] = Rm.RPY2rotvec(math.pi, 0, 0)
Rm.robot_movel(Robot_connection, pos_read_down, wait=1, p=True)

pos_read_down[3], pos_read_down[4], pos_read_down[5] = Rm.RPY2rotvec(math.pi, 0, -math.pi/2)
Rm.robot_movel(Robot_connection, pos_read_down, wait=1, p=True)

pos_read_down[3], pos_read_down[4], pos_read_down[5] = Rm.RPY2rotvec(math.pi*3/4, 0, -math.pi/2)
Rm.robot_movel(Robot_connection, pos_read_down, wait=1, p=True)
pos_read_up[3:] = pos_read_down[3:]
Rm.robot_movel(Robot_connection, pos_read_up, wait=1, p=True)




# Step 6 - Drop item
Rm.robot_movej(Robot_connection, joint_pos_rest , 1, False)
Rm.robot_movej(Robot_connection, pos_brick_drop)
Robot_connection.send(b'set_digital_out(0,True)' + b"\n")  # Close the gripper by setting thd digital I/O 0 to false.
time.sleep(0.3)




# Step 7 - Return to resting position

# Move robot - return to idle position
Rm.robot_movej(Robot_connection, joint_pos_rest, wait=1, p=False)


