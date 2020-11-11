import time
import socket
import threading
import binascii
from struct import *
import numpy as np
import math
import cv2
import ImageModule as Im
from matplotlib import pyplot as plt
import os



# Robot connection
HOST = "192.168.1.17" # Robot ip
PORT = 30003# Robot port


class RobotSocket(socket.socket):
    def __init__(self):
        super(RobotSocket, self).__init__(socket.AF_INET, socket.SOCK_STREAM)
        self.buffer_length = 1024
        self.test = b'test'
        self.settimeout(10)
        self.connectionThread = None
        self.isConnected = False
        self.startTimeNotConnected = 0
        self.elapsedTimeNotConnected = 0

    def renew(self):
        super(RobotSocket, self).__init__(socket.AF_INET, socket.SOCK_STREAM)

    def connectSafely(self, address):
        self.connect(address)
        self.connectionThread = threading.Thread(target=self.testConnection, args=(address,), daemon=True)
        self.connectionThread.start()

    def testConnection(self, address):
        while True:
            try:
                self.send(self.test)
                self.recv(self.buffer_length)
            except OSError as error:
                # print("Connection failed: {}".format(error))
                # set connection status and recreate socket
                self.isConnected = False
                self.startTimeNotConnected = time.time()
                print("Robot connection lost ... reconnecting")
                while not self.isConnected:
                    self.elapsedTimeNotConnected = time.time() - self.startTimeNotConnected
                    if self.elapsedTimeNotConnected > 5:
                        return
                    try:
                        self.renew()
                        self.connect(address)
                        self.isConnected = True
                        print("Server connected again")
                    except OSError as error:
                        # print("Connection failed: {}".format(error))
                        time.sleep(0.5)


# Robot angle converison functions
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def RPY2rotvec(roll, pitch, yaw):
    yawMatrix = np.matrix([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    pitchMatrix = np.matrix([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    rollMatrix = np.matrix([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    R = yawMatrix * pitchMatrix * rollMatrix
    theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2)
    multi = 1 / (2 * math.sin(theta+1e-5))

    rx = multi * (R[2, 1] - R[1, 2]) * theta
    ry = multi * (R[0, 2] - R[2, 0]) * theta
    rz = multi * (R[1, 0] - R[0, 1]) * theta

    rot = np.zeros((3, 1))
    cv2.Rodrigues(R, rot)

    rx = rot[0, 0]
    ry = rot[1, 0]
    rz = rot[2, 0]

    return rx, ry, rz


def Angles2rotvec(theta1, theta2):
    theta1 = math.pi - theta1
    R_x = np.array([[1, 0, 0], [0, np.cos(theta1), -np.sin(theta1)], [0, np.sin(theta1), np.cos(theta1)]])
    #R_y = np.array([[np.cos(theta1), 0, np.sin(theta1)], [0, 1, 0], [-np.sin(theta1), 0, np.cos(theta1)]])
    R_z = np.array([[np.cos(theta2), -np.sin(theta2), 0], [np.sin(theta2), np.cos(theta2), 0], [0, 0, 1]])

    R = R_x.dot(R_z)

    RPY = rotationMatrixToEulerAngles(R)
    RX, RY, RZ = RPY2rotvec(RPY[0],RPY[1],RPY[2])
    return RX, RY, RZ


def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

# Robot behaviour functions
def getPosition(s = None):
    '''
    DESCRIPTION: Returns the current position of the robot.
    :param s: robot connection
    :return: [X,Y,Z,RX,RY,RZ] - robots current position
    '''
    X = -1
    Y = -1
    Z = -1
    RX = -1
    RY = -1
    RZ = -1
    # Connect to robot
    #if s is None or True: # IF the connection doesn't get reestablish, it doesn't work for some reason.
    #    #print("New robot connection")
    #    try:
    #        print("UPDATE SOCKET")
    #        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #        s.settimeout(0.5)
    #        s.connect((HOST, PORT))
    #    except:
    #        print("Recoonect socket error")
    #        return np.array([X,Y,Z,RX,RY,RZ])

    time.sleep(0.1)
    # Trials
    count = 0
    # Connect

    while count < 100:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.5)
            s.connect((HOST, PORT))
            # Send message to recived new information
            #s.send(b'' + b"\n")
            time.sleep(0.1)

            # Other information
            package_length = unpack('!i', bytes.fromhex(str(binascii.hexlify(s.recv(4)).decode("utf-8"))))[0]
            print("Package length:", package_length)
            s.recv(584)
            # Position
            X  = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            Y  = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            Z  = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            RX = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            RY = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            RZ = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            #print("Current position:", X,Y,Z,RX,RY,RZ)

            # Other information
            s.recv(168)

            # Position
            ROBO_MODE  = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            #print("Robot mode:", ROBO_MODE)

            break
        except socket.error as socketerror:
            # Problems with the connection
            print("Error: ", socketerror)
            count += 1
            s.close()

    return np.array([X,Y,Z,RX,RY,RZ])

def getPosition_modbus(s = None):
    program_run = 0


    try:
        if s is None:
            print("CONNECT to robot")
            PORT_502 = 502
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(10)
            s.connect((HOST, PORT_502))
            time.sleep(0.05)
        reg_400 = ""
        s.send (b"\x00\x04\x00\x00\x00\x06\x00\x03\x01\x90\x00\x01") #request data from register 128-133 (cartisian data)
        reg_400 = s.recv(1024)
        reg_400 = reg_400.replace (b"\x00\x04\x00\x00\x00\x05\x00\x03\x02", b"")
        reg_400 = reg_400.hex() #convert the data from \x hex notation to plain hex

        if reg_400 == "":
            reg_400 = "0000"
        reg_400_i = int(reg_400,16)
        if reg_400_i < 32768:
            reg_400_f = float(reg_400_i)/10
        if reg_400_i > 32767:
            reg_400_i = 65535 - reg_400_i
            reg_400_f = float(reg_400_i)/10*-1
        X = reg_400_f

        reg_401 = ""
        s.send (b"\x00\x04\x00\x00\x00\x06\x00\x03\x01\x91\x00\x01") #request data from register 128-133 (cartisian data)
        reg_401 = s.recv(1024)
        reg_401 = reg_401.replace (b"\x00\x04\x00\x00\x00\x05\x00\x03\x02", b"")
        reg_401 = reg_401.hex() #convert the data from \x hex notation to plain hex
        if reg_401 == "":
            reg_401 = "0000"
        reg_401_i = int(reg_401,16)
        if reg_401_i < 32768:
            reg_401_f = float(reg_401_i)/10
        if reg_401_i > 32767:
            reg_401_i = 65535 - reg_401_i
            reg_401_f = float(reg_401_i)/10*-1
        Y = reg_401_f

        reg_402 = ""
        s.send (b"\x00\x04\x00\x00\x00\x06\x00\x03\x01\x92\x00\x01") #request data from register 128-133 (cartisian data)
        reg_402 = s.recv(1024)
        reg_402 = reg_402.replace (b"\x00\x04\x00\x00\x00\x05\x00\x03\x02", b"")
        reg_402 = reg_402.hex() #convert the data from \x hex notation to plain hex
        if reg_402 == "":
            reg_402 = "0000"
        reg_402_i = int(reg_402,16)
        if reg_402_i < 32768:
            reg_402_f = float(reg_402_i)/10

        if reg_402_i > 32767:
            reg_402_i = 65535 - reg_402_i
            reg_402_f = float(reg_402_i)/10*-1
        Z = reg_402_f

        reg_403 = ""
        s.send (b"\x00\x04\x00\x00\x00\x06\x00\x03\x01\x93\x00\x01") #request data from register 128-133 (cartisian data)
        reg_403 = s.recv(1024)
        reg_403 = reg_403.replace (b"\x00\x04\x00\x00\x00\x05\x00\x03\x02", b"")
        reg_403 = reg_403.hex() #convert the data from \x hex notation to plain hex
        if reg_403 == "":
            reg_403 = "0000"
        reg_403_i = int(reg_403,16)
        if reg_403_i < 32768:
            reg_403_f = float(reg_403_i)/1000
        if reg_403_i > 32767:
            reg_403_i = 65535 - reg_403_i
            reg_403_f = float(reg_403_i)/1000*-1
        Rx = reg_403_f

        reg_404 = ""
        s.send (b"\x00\x04\x00\x00\x00\x06\x00\x03\x01\x94\x00\x01") #request data from register 128-133 (cartisian data)
        reg_404 = s.recv(1024)
        reg_404 = reg_404.replace (b"\x00\x04\x00\x00\x00\x05\x00\x03\x02", b"")
        reg_404 = reg_404.hex() #convert the data from \x hex notation to plain hex
        if reg_404 == "":
            reg_404 = "0000"
        reg_404_i = int(reg_404,16)
        if reg_404_i < 32768:
            reg_404_f = float(reg_404_i)/1000
        if reg_404_i > 32767:
            reg_404_i = 65535 - reg_404_i
            reg_404_f = float(reg_404_i)/1000*-1
        Ry = reg_404_f

        reg_405 = ""
        s.send (b"\x00\x04\x00\x00\x00\x06\x00\x03\x01\x95\x00\x01") #request data from register 128-133 (cartisian data)
        reg_405 = s.recv(1024)
        reg_405 = reg_405.replace (b"\x00\x04\x00\x00\x00\x05\x00\x03\x02", b"")
        reg_405 = reg_405.hex() #convert the data from \x hex notation to plain hex
        if reg_405 == "":
            reg_405 = "0000"
        reg_405_i = int(reg_405,16)
        if reg_405_i < 32768:
            reg_405_f = float(reg_405_i)/1000
        if reg_405_i > 32767:
            reg_405_i = 65535 - reg_405_i
            reg_405_f = float(reg_405_i)/1000*-1
        Rz = reg_405_f

        home_status = 1
        program_run += 1
        position = (X,Y,Z,Rx,Ry,Rz)
        print("\nPosition:", position)
        return position

    except socket.error as socketerror:
        print("Error: ", socketerror)
        return (-1,-1,-1,-1,-1,-1)

def connect_robot_modbus(Port=502):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(10)
    s.connect((HOST, PORT))
    time.sleep(0.05)
    return s

def getJointPosition(s = None):
    '''
    DESCRIPTION: Returns the current position of the robot.
    :param s: robot connection
    :return: [X,Y,Z,RX,RY,RZ] - robots current position
    '''
    # Connect to robot
    if s is None or True: # IF the connection doesn't get reestablish, it doesn't work for some reason.
        #print("New robot connection")
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        s.connect((HOST, PORT))
        time.sleep(0.1)

    joint_position_actual = np.array([-1, -1, -1, -1, -1, -1])
    joint_position_target = np.array([-1, -1, -1, -1, -1, -1])

    # Trials
    count = 0
    # Connect
    while count < 100:
        try:
            # Send message to recived new information
            s.send(b'get joint speeds()' + b"\n")
            s.send(b'get joint positions()' + b"\n")
            # Other information
            s.recv(12)
            # Target Joint Position
            X  = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            Y  = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            Z  = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            RX = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            RY = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            RZ = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]

            joint_position_target = np.array([X,Y,Z,RX,RY,RZ])
            # Other information
            s.recv(192)

            # Actual Joint Position
            X  = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            Y  = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            Z  = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            RX = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            RY = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            RZ = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]

            joint_position_actual = np.array([X, Y, Z, RX, RY, RZ])

            break
        except socket.error as socketerror:
            # Problems with the connection
            print("Error: ", socketerror)
            count += 1
            s.close()

    return joint_position_target, joint_position_actual


def getSpeed(s=None):
    '''
    DESCRIPTION: Returns the speed of the robot.
    :param s: robot connection
    :return: [X,Y,Z,RX,RY,RZ] - robots current speed
    '''

    # Connect to robot
    if s is None or True:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        s.connect((HOST, PORT))
        time.sleep(0.1)


    # Trials
    count = 0

    # Connect
    while count < 100:
        try:
            # Send message to recived new information
            s.send(b"")

            # Other information
            s.recv(300)

            # Speed
            X  = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            Y  = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            Z  = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            RX = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            RY = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]
            RZ = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]

            print("Target position:", X, Y, Z, RX, RY, RZ)
            print("Total Speed:",  abs(X)+abs(Y)+abs(Z)+abs(RX)+abs(RY)+abs(RZ))

            # Position
            ROBO_MODE  = unpack('!d', bytes.fromhex(str(binascii.hexlify(s.recv(8)).decode("utf-8"))))[0]

            #print("Robot mode:", ROBO_MODE)

            break
        except socket.error as socketerror:
            # Problems with the connection
            print("Error: ", socketerror)
            count += 1
            s.close()

    return np.array([X,Y,Z,RX,RY,RZ])


def waitPos(pos_targ, s=None, check = False, check_pose = [0,0,0,0,0,0], compare_pos = True):
    '''
    DESCRIPTION: Wait for the robot to reach the target position
    :param pos_targ: target position for robot
    :param s: robot connection
    :return: Bool - True if reached
    '''

    # In position counter
    count = 0
    max_count = 0

    pos_targ = np.array(pos_targ)
    #pos_targ[3:] += (pos_targ[3:] < 0) * 2 * math.pi
    pos_targ *= 1000
    # Run until robot reports the correct position 3 times
    pos_cur_pre = np.zeros(pos_targ.shape)
    pos_targ = pos_targ.astype(np.int)
    if compare_pos:
        pos_targ[3:] = np.abs(pos_targ[3:])
    print("START WAIT POS")
    while count < 2:
        try:
            pos_cur = (getPosition(s)*1000).astype(int)
            #print("Cur:", pos_cur)
            #print("Target:", pos_targ)
            #print("DIFF TARG:", pos_cur-pos_targ)
            #print("Diff value:", (np.sum(np.abs(pos_cur-pos_targ))))
            #speed_cur = sum(abs(getSpeed(s) * 1000).astype(int))

            #joint_position_target, joint_position_actual = getJointPosition(s)
            #joint_position_actual *= 1000
            #joint_position_actual = joint_position_actual.astype(np.int)
            #print(joint_position_target)
            #print("cur angle:", joint_position_actual)
            #print("Target:", pos_targ)
            #print("DIFF TARG angle:", joint_position_actual - pos_targ)
            #print("DIFF:", np.sum(np.abs(joint_position_target*1000-joint_position_actual*1000)))
            #DIFF = np.sum(np.abs(joint_position_target*1000-joint_position_actual*1000))

            #pos_diff = np.abs(pos_cur - pos_targ)
            #pos_diff[3:] -= (pos_diff[3:] > 6) * 2 * math.pi
            # Check if in position
            #print("Cur:", np.round(pos_cur, 5))
            #print("Pre", np.round(pos_cur_pre, 5))
            #print('Count:',count)
            #print(np.array_equal(pos_cur, pos_cur_pre))
            #print(np.abs(np.sum(pos_cur-pos_cur_pre)))
            # Maybe try to use the target position more
            # Pre version (np.sum(pos_diff) < 10e-2) or np.array_equal(pos_cur, pos_cur_pre):
            distance = (np.sum(np.abs(pos_cur-pos_cur_pre)))
            print("DIstance to target:", distance)
            #print("SPEED:", speed_cur)
            #if compare_pos:
            #    pos_cur[3:] = np.abs(pos_cur[3:])
            #    print("Position difference:", (np.sum(np.abs(pos_cur-pos_targ))))
            #    distance = (np.sum(np.abs(pos_cur-pos_targ)))
            #else:
            #    print("Angle difference:", (np.sum(np.abs(joint_position_actual - pos_targ))))
            #    distance = (np.sum(np.abs(joint_position_actual - pos_targ)))


            if check:
                return np.sum(np.abs(np.array(check_pose-pos_cur))) < 20

            if distance < 10:#DIFF < 0.3:
                count += 1
                break
            else:
                count = 0
            pos_cur_pre = pos_cur.copy()


            max_count += 1
        except:
            print("Wait pose error")

    print("END WAIT POS")
    return True


def robot_init(pos_init, s = None):
    #return None
    '''
    DESCRIPTION: Set the gripper voltage and open it. Move the robot to a initialization position.
    :param pos_init: initialization position
    :return s: robot connection
    '''
    # Connect to robot
    if s is None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #s = RobotSocket()
        #.connectSafely((HOST, PORT))
        s.connect((HOST, PORT))

    angle_brick_drop = [87.28 * math.pi / 180, -74.56 * math.pi / 180, 113.86 * math.pi / 180, -129.29 * math.pi / 180, -89.91 * math.pi / 180, -2.73 * math.pi / 180]

    # Set gripper  voltage - avoids the current error
    s.send(b'set_tool_voltage(12)' + b'\n')
    time.sleep(0.5)
    s.send(b'set_tool_voltage(24)' + b'\n')
    time.sleep(0.5)

    #Check if in position
    pos_check = waitPos(pos_init, s, check = True, check_pose=[-125.92, -459.28, 293.57, 775, 3044, 0])
    print("Pose check:", pos_check)
    # Drop of current object
    if not pos_check:
        # Get position to move up vertically.
        pos_cur = getPosition()
        if(pos_cur[2] < 0.325):
            pos_cur[2] = 0.317

        # Move robot to pos_init
        robot_movej(s, pos_cur, 1, True)
        robot_movej(s, pos_init, 1, False)

        robot_movej(s, angle_brick_drop, 1, False)
        s.send(b'set_digital_out(8,False)' + b"\n")  # Open gripper
        time.sleep(0.1)

        robot_movej(s, pos_init, 1, False)
    else:
        # Open gripper
        s.send(b'set_digital_out(8, False)' + b"\n")
        time.sleep(0.5)
    return s


def robot_movej(s, pos, wait = True, p = True):
    '''
    DESCRIPTION: Moves the robot with the movej command to pos
    :param s: Robot connection
    :param pos: position [X,Y,Z,RX,RY,RZ]
    :param wait: If true the robot program, waits until the robot is in its position.
    :param p: Defines weather the position is a position og a joint position.
    :return string: command send to the robot
    '''
    # Command string
    if p:
        string = "movej(p[%s, %s, %s, %s, %s, %s]) \n" % (pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
    else:
        string = "movej([%s, %s, %s, %s, %s, %s]) \n" % (pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
    #print('Send')

    # Send command
    s.send(str.encode(string))
    # Wait for bot to reach its position
    print("Wait?", wait)
    if wait:
        waitPos(pos, s, compare_pos=p)
    #print('In position')

    return string


def robot_movel(s, pos, wait = True, p = True):
    '''
    DESCRIPTION: Moves the robot with the movel command to pos
    :param s: Robot connection
    :param pos: position [X,Y,Z,RX,RY,RZ]
    :param wait: If true the robot program, waits until the robot is in its position.
    :param p: Defines weather the position is a position og a joint position.
    :return string: command send to the robot
    '''

    # Command string
    if p:
        string = "movel(p[%s, %s, %s, %s, %s, %s]) \n" % (pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
    else:
        string = "movel([%s, %s, %s, %s, %s, %s]) \n" % (pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
    #print('Send')

    # Send command
    s.send(str.encode(string)) # comment out for camera

    # Wait for bot to reach its position
    #print('Wait')
    if wait != 0:
        waitPos(pos, s, compare_pos=p)
    #print('In position')
    time.sleep(0.2)

    return string


def open_gripper(s = None):
    # Connect to robot
    if s is None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    s.send(b'set_digital_out(8, False)' + b"\n")


def close_gripper(s = None):
    # Connect to robot
    if s is None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    s.send(b'set_digital_out(8, True)' + b"\n")


def robot_line_sweep( pick_point, view_point, cam_pos, temp, img, save_image, show_image, path, s = None, optimize = False, camera = None):
    print("PICK", pick_point)
    pick_point[1] = pick_point[3]

    if s is None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    final_position, _, y_offset = generate_pos(cam_pos, view_point, pick_point, img, temp, s, optimize, camera)
    start_position, _, _ = generate_pos(cam_pos, [view_point[2],view_point[3],view_point[0],view_point[1],view_point[4],view_point[5],view_point[6],view_point[7], [8]], pick_point, img, temp, s)
    start_position[1] += y_offset
    position = final_position.copy()
    angle = math.pi / 2 + math.atan2(view_point[3] - view_point[1], view_point[2] - view_point[0])
    angle = -1 * math.atan2(view_point[1] - view_point[3], view_point[0] - view_point[2])
    angle = angle - abs(math.pi - temp)

    direction = 1 if angle >= 0 and angle <= math.pi else -1
    length = math.sqrt((view_point[2] - view_point[0]) ** 2 + (view_point[3] - view_point[1]) ** 2)
    print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
    for j in reversed(range(view_point[6])):
        position[1] = final_position[1] - (j/(view_point[6]-1)) * (-0.3125 + 0.5627) * abs(length) / img.shape[1]
        # Move robot
        print("Position:", position)
        robot_movej(s, position)
        # Get image
        print("Save?", save_image, path)
        if show_image or save_image:
            img_num = os.listdir(path)
            if path is not None:
                path_save = path + '/img_' + str(len(img_num)) + '_' + time.strftime("%Y%m%d-%H%M%S") + ".png"
            else:
                path_save = path
            print("Save image", j, "/", view_point[6])
        else:
            path_save = None

        Im.save_image(save_image, path_save, show_image, "Line sweep - image %d of %d" %(view_point[6]-j, view_point[6]), camera=camera)


def robot_point_view( pick_point, view_point, cam_pos, temp, img, save_image, show_image, path, s = None, optimize = False, camera = None):
    if s is None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    position, opt_image, _= generate_pos(cam_pos, view_point, pick_point, img, temp, s, optimize, camera)

    print("View point", view_point)
    print("Cam pos", cam_pos)
    print("Position", position)

    if opt_image is None:
        # Move robot
        robot_movej(s, position)

    # Get image
    if show_image or save_image:
        if path is not None:
            path_save = path + '/img_' + time.strftime("%Y%m%d-%H%M%S") + ".png"
        else:
            path_save = path

        Im.save_image(save_image, path_save, show_image, "", opt_image, camera=camera)


def robot_point_rotation( pick_point, view_point, cam_pos, temp, img, save_image, show_image, path, s = None):
    if s is None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    #view_point[0],view_point[1],view_point[2],view_point[3]= view_point[2],view_point[3],view_point[0],view_point[1]
    position, _, _ = generate_pos(cam_pos, view_point, pick_point, img, temp, s)
    position[1], a = rotate((cam_pos[1], cam_pos[2]), (position[1], position[2]), -(view_point[5] * math.pi / 180) / 2)
    position[2] +=  abs(position[2]-a)

    angle_view_h = -1 * math.atan2(view_point[1] - view_point[3], view_point[0] - view_point[2]) - abs(math.pi - temp)
    for j in range(1, 1 + view_point[6]):
        position[1], a = rotate((cam_pos[1], cam_pos[2]), (position[1], position[2]), (view_point[5] * math.pi / 180) / (max(view_point[6] - 1,1)))
        position[2] += abs(position[2] - a)
        angle_view_v = (view_point[4]) * math.pi / 180 + math.pi + 0.5 * view_point[5] * math.pi / 180 - j * (view_point[5] * math.pi / 180) / (max(view_point[6] - 1,1))
        angle_view_v -= 2*math.pi#-2*math.pi/180

        position[3], position[4], position[5] = Angles2rotvec(angle_view_v, angle_view_h)

        # Move robot
        robot_movej(s, position)

        # Get image
        if show_image or save_image:
            if path is not None:
                path_save = path + '/img_' + time.strftime("%Y%m%d-%H%M%S") + ".png"
            else:
                path_save = path

            Im.save_image(save_image, path_save, show_image, "Point rotation - Angle: %2.f"  %(0.5 * view_point[5]  - (j-1) * (view_point[5] ) / (max(view_point[6] - 1,1))))


def generate_pos(cam_pos, view_point, pick_point, img, temp , s, optimize = False, camera = None):
    print("GET POSITION START")
    position = cam_pos.copy()
    # Translate pick -> view
    length = math.sqrt(((pick_point[2] + pick_point[0]) / 2 - view_point[2]) ** 2 + ( (pick_point[3] + pick_point[1]) / 2 - view_point[3]) ** 2)
    angle_trans_pick = math.pi / 2 - math.atan2((view_point[3] - (pick_point[3] + pick_point[1]) / 2), view_point[2] - (pick_point[2] + pick_point[0]) / 2)

    position[0] += math.sin(angle_trans_pick) * (-0.3125 + 0.5627) * abs(length) / img.shape[1]
    position[1] += math.cos(angle_trans_pick) * (-0.3125 + 0.5627) * abs(length) / img.shape[1]

    # Translate cam
    angle_trans_cam = - math.atan2(view_point[3] -view_point[1], view_point[2] - view_point[0]) + math.pi / 2
    position[0], position[1] = rotate((cam_pos[0], cam_pos[1]), (position[0], position[1]), angle_trans_cam)

    # Rotations
    temp2 = -1 * math.atan2(view_point[1] - view_point[3], view_point[0] - view_point[2])
    angle_hoiz = temp2 - abs(math.pi - temp)

    angle_ver = (view_point[4]) * math.pi / 180 + math.pi #-2*math.pi/180

    position[3], position[4], position[5] = Angles2rotvec(angle_ver, angle_hoiz)

    #TODO: Optimize this part
    #position[2] -= 0.008*math.sin(abs(view_point[4]* math.pi / 180))
    #position[1] -= 0.0035 * math.sin(abs(view_point[4] * math.pi / 180))
    if abs(view_point[4]) == 90:
        position[2] += 0.008

    position[1], position[2] = rotate((cam_pos[1], cam_pos[2]), (position[1], position[2]), -view_point[4] * math.pi / 180)

    if abs(view_point[4]) == 90:
        position[2] -= 0.003

    opt_img = None
    y_offset = 0
    if optimize:
        position, opt_img, y_offset = optimize_view(s, position.copy(), angle_hoiz, angle_ver, camera)

    print("GET POSITION END")
    return position, opt_img, y_offset


def optimize_view(s, position, angle_h, angle_v, camera = None):
    opt_position = position.copy()
    opt_image = None

    #Demo purpose
    #position[1] -= 0.02

    print("Optimizing angle")
    variance = 0
    for offset in range(-5, 6):
        position[3], position[4], position[5] = Angles2rotvec(angle_v + 0.3 * offset * math.pi / 180, angle_h)
        robot_movej(s, position)
        img = Im.get_image(F"21565643", camera)
        # Show image
        img = cv2.putText(img, "Optimizing view angle", (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2,cv2.LINE_AA)
        cv2.imwrite('C:/Prj/Robopick/Depot.svn/images/temp/temp_save.png', img)
        cv2.imwrite('C:/Prj/Robopick/Depot.svn/images/temp/temp.png', img)

        hist_1, _, _ = plt.hist(img.ravel(), 256, [0, 256], histtype='step');

        temp = math.sqrt(np.var(img))
        print("Std:", temp)
        if temp > variance:
            variance = temp
            opt_position[3:6] = position[3:6]
            opt_image = img.copy()
        elif temp < variance * 1.1:
            break

    sharpness = 0
    pre_variance = 0
    start_height = position[2]
    position[2] -= 0.003
    print("Optimizing height")
    position[3:6] = opt_position[3:6].copy()
    for offset in range(30):
        robot_movej(s, position)
        img = Im.get_image(F"21565643", camera)
        #Show image
        img = cv2.putText(img, "Optimizing view height", (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2, cv2.LINE_AA)
        cv2.imwrite('C:/Prj/Robopick/Depot.svn/images/temp/temp_save.png', img)
        cv2.imwrite('C:/Prj/Robopick/Depot.svn/images/temp/temp.png', img)

        hist_1, _, _ = plt.hist(img.ravel(), 256, [0, 256], histtype='step');
        #temp = math.sqrt(np.var(img))
        array = np.asarray(img, dtype=np.int32)
        gy, gx = np.gradient(array)
        gnorm = np.sqrt(gx ** 2 + gy ** 2)
        temp = np.average(gnorm)
        #temp = cv2.Laplacian(img, cv2.CV_64F).var()

        print("Sharpness:", temp, position[2], offset)
        if temp > sharpness:
            sharpness = temp
            opt_position[2] = position[2]
            opt_image = img.copy()
        elif temp*1.10 < sharpness and temp >1.5:
            break

        position[2] += 0.0003


    #offset y-coordinate with respect to height

    y_offset = (opt_position[2]-start_height)*math.cos(75*math.pi/180)
    opt_position[1] += y_offset

    print("OPITMAL", opt_position)

    variance = 0
    position[2] = opt_position[2]
    position[1] -= 0.005
    print("Optimize y axis")
    for offset in range(10):
        robot_movej(s, position)
        img = Im.get_image(F"21565643", camera)
        img_draw = img.copy()
        img_draw = cv2.putText(img_draw, "Optimizing X position", (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2, cv2.LINE_AA)
        cv2.imwrite('C:/Prj/Robopick/Depot.svn/images/temp/temp_save.png', img_draw)
        cv2.imwrite('C:/Prj/Robopick/Depot.svn/images/temp/temp.png', img_draw)
        #cv2.imshow("Variance test", img)
        #cv2.waitKey(10)
        hist_1, _, _ = plt.hist(img.ravel(), 256, [0, 256], histtype='step');
        temp = np.mean(img)#math.sqrt(np.var(img))
        print("Std:", temp)
        print("Mean", np.mean(img))
        if temp > variance:
            variance = temp
            opt_position[1] = position[1]
            opt_image = img.copy()
        elif temp < variance * 1.05:
            break
        position[1] += 0.001


    while True:
        max = np.max(opt_image)
        print("Max:",np.max(img) )
        print("Exposure time:", camera.ExposureTimeAbs.GetValue())
        exposure_time = camera.ExposureTimeAbs.GetValue()
        if max == 255:
            camera.ExposureTimeAbs = exposure_time - 5
        elif max < 230:
            camera.ExposureTimeAbs = exposure_time + 5
        else:
            break
        opt_image = Im.get_image(F"21565643", camera)
        img_draw = opt_image.copy()
        img_draw = cv2.putText(img_draw, "Optimizing exposure time", (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1,255, 2, cv2.LINE_AA)
        cv2.imwrite('C:/Prj/Robopick/Depot.svn/images/temp/temp_save.png', img_draw)
        cv2.imwrite('C:/Prj/Robopick/Depot.svn/images/temp/temp.png', img_draw)



    time.sleep(0.5)
    if os.path.isfile('C:/Prj/Robopick/Depot.svn/images/temp/temp_save.png'):
        os.remove('C:/Prj/Robopick/Depot.svn/images/temp/temp_save.png')

    return  opt_position, opt_image, y_offset


if __name__ == '__main__':
    while True:
        print('The module test will run the robot through some positions and set the gripper.')
        print('Make sure that the robot is not blocked.')
        choice = input('Are you sure you want to test the Robot_modul(y/n)?')
        if choice in ['y','n']:
            break
    if choice == 'y':
        print('Starting test...')
        # - Positions encoding: X, Y, Z, RX, RY, RZ
        pos_idle        = [-0.126, -0.460,  0.325,  2.22,  2.22,  0.00]
        pos_brick_up   = [ 0.259, -0.207,  0.325,  0.00,  3.14,  0.00]
        pos_brick_down = [ 0.259, -0.207,  0.040,  0.00,  3.14,  0.00]
        pos_brick_drop  = [-0.522,  0.100,  0.050, -3.14,  0.08,  0.00]
        pos_read_up     = [-0.591, -0.491,  0.325, -2.91,  1.17, -0.03]
        pos_read_down   = [-0.591, -0.491,  0.040, -2.91,  1.17, -0.03]


        # Initialize Robot
        s = robot_init(pos_idle)

        # Grab brick
        robot_movej(s, pos_brick_up)
        robot_movel(s, pos_brick_down)
        s.send(b'set_digital_out(8,False)' + b"\n")# Open gripper
        time.sleep(0.5)
        robot_movel(s, pos_brick_up)

        # Move brick in front of read camera
        robot_movej(s, pos_read_up)
        robot_movel(s, pos_read_down)
        robot_movel(s, pos_read_up)

        # Drop brick
        robot_movej(s, pos_brick_drop)
        s.send(b'set_digital_out(8,True)' + b"\n") # Open gripper
        time.sleep(0.5)

        # todo Robot: Move to idle
        robot_movej(s, pos_idle)

        # todo Robot: End connection
        pos_final = getPosition()
        pos_final[:2] = pos_final[:2] * 1000
        s.close()
        print("Final position", pos_final)
        print("Test done")



