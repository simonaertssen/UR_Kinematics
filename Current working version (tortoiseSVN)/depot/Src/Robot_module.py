import time
import sys
import socket
import threading
import binascii
from struct import *
import numpy as np
import math
import cv2
import Image_module as Im
from matplotlib import pyplot as plt


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

    # Connect to robot
    if s is None or True: # IF the connection doesn't get reestablish, it doesn't work for some reason.
        #print("New robot connection")
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
            s.recv(588)

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


def waitPos(pos_targ, s=None, check = False, check_pose = [0,0,0,0,0,0]):
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

    # Run until robot reports the correct position 3 times
    pos_cur_pre = np.zeros(pos_targ.shape)
    while count < 2:
        pos_cur = (getPosition(s)*1000).astype(int)
        pos_diff = np.abs(pos_cur - pos_targ)
        pos_diff[3:] -= (pos_diff[3:] > 6) * 2 * math.pi
        # Check if in position
        #print("Cur:", np.round(pos_cur, 5))
        #print("Pre", np.round(pos_cur_pre, 5))
        #print('Count:',count)
        #print(np.array_equal(pos_cur, pos_cur_pre))
        #print(np.abs(np.sum(pos_cur-pos_cur_pre)))
        # Maybe try to use the target position more
        # Pre version (np.sum(pos_diff) < 10e-2) or np.array_equal(pos_cur, pos_cur_pre):
        if (np.sum(np.abs(pos_cur-pos_cur_pre))) < 20:
            count += 1
        else:
            count = 0
        pos_cur_pre = pos_cur.copy()

        if check:
            return np.sum(np.abs(np.array(check_pose-pos_cur))) < 20


        max_count += 1

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

    angle_brick_drop = [87.28 * math.pi / 180, -73.11 * math.pi / 180, 114.73 * math.pi / 180, -131.61 * math.pi / 180, -89.91 * math.pi / 180, -2.73 * math.pi / 180]

    # Set gripper  voltage - avoids the current error
    s.send(b'set_tool_voltage(12)' + b'\n')
    time.sleep(0.5)
    s.send(b'set_tool_voltage(24)' + b'\n')
    time.sleep(0.5)

    #Check if in position
    pos_check = waitPos(pos_init, s, check = True, check_pose=[-126, -459, 316, 775, 3044,1])
    print("Pose check:", pos_check)
    # Drop of current object
    if not pos_check:
        # Get position to move up vertically.
        pos_cur = getPosition()
        print("pos_cur", pos_cur)
        if pos_cur[2] < 0.325:
            pos_cur[2] = 0.317

        # Move robot to pos_init
        print("KOM NU")
        robot_movej(s, pos_cur, 1, True)
        print(1)
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
    """
    DESCRIPTION: Moves the robot with the movej command to pos
    :param s: Robot connection
    :param pos: position [X,Y,Z,RX,RY,RZ]
    :param wait: If true the robot program, waits until the robot is in its position.
    :param p: Defines weather the position is a position og a joint position.
    :return string: command send to the robot
    """
    # Command string
    if p:
        string = "movej(p[%s, %s, %s, %s, %s, %s]) \n" % (pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
    else:
        string = "movej([%s, %s, %s, %s, %s, %s]) \n" % (pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
    print("string =", string)
    # Send command
    s.send(str.encode(string))

    # Wait for bot to reach its position
    if wait:
        waitPos(pos, s)

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
        waitPos(pos, s)
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


def robot_line_sweep( pick_point, view_point, cam_pos, temp, img, save_image, show_image, path, s = None, optimize = False):
    if s is None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    final_position = generate_pos(cam_pos, view_point, pick_point, img, temp, s, optimize)
    start_position = generate_pos(cam_pos, [view_point[2],view_point[3],view_point[0],view_point[1],view_point[4],view_point[5],view_point[6],view_point[7], [8]], pick_point, img, temp, s)
    # print("Start", start_position)
    # print("End", final_position)

    position = final_position.copy()
    angle = math.pi / 2 + math.atan2(view_point[3] - view_point[1], view_point[2] - view_point[0])
    angle = -1 * math.atan2(view_point[1] - view_point[3], view_point[0] - view_point[2])
    angle = angle - abs(math.pi - temp)
    # print("Angle:", angle)
    direction = 1 if angle >= 0 and angle <= math.pi else -1
    length = math.sqrt((view_point[2] - view_point[0]) ** 2 + (view_point[3] - view_point[1]) ** 2)
    for j in reversed(range(view_point[6])):
        #position[0] = final_position[0] + (j/(view_point[6]-1))*math.sin(angle) * (-0.3125 + 0.5627) * abs(length) / img.shape[1]
        position[1] = final_position[1] - (j/(view_point[6]-1)) * (-0.3125 + 0.5627) * abs(length) / img.shape[1]
        #print("Y:", position[1])
    #for j in range(1,view_point[6]+1):
    #    position[0] = start_position[0] + (final_position[0]- start_position[0])*j/view_point[6]
    #   position[1] = start_position[1] + (final_position[1] - start_position[1]) * j / view_point[6]

        # Move robot
        robot_movej(s, position)

        # Get image
        if show_image or save_image:
            print("saving on path_save =", path_save)
            if path is not None:
                path_save = path + '/img_' + time.strftime("%Y%m%d-%H%M%S") + ".png"
            else:
                path_save = path
            print("saving on path_save =", path_save)
            Im.save_image(save_image, path_save, show_image, "Line sweep - image %d of %d" % (view_point[6]-j, view_point[6]))


def robot_point_view( pick_point, view_point, cam_pos, temp, img, save_image, show_image, path, s = None, optimize = False):
    if s is None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    position= generate_pos(cam_pos, view_point, pick_point, img, temp, s, optimize)
    print("View point", view_point)
    print("Cam pos", cam_pos)
    print("Position", position)
    # Move robot
    robot_movej(s, position)

    # Get image
    if show_image or save_image:
        if path is not None:
            path_save = path + '/img_' + time.strftime("%Y%m%d-%H%M%S") + ".png"
        else:
            path_save = path

        Im.save_image(save_image, path_save, show_image, "")


def robot_point_rotation( pick_point, view_point, cam_pos, temp, img, save_image, show_image, path, s = None):
    if s is None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    #view_point[0],view_point[1],view_point[2],view_point[3]= view_point[2],view_point[3],view_point[0],view_point[1]
    position = generate_pos(cam_pos, view_point, pick_point, img, temp, s)
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


def generate_pos(cam_pos, view_point, pick_point, img, temp , s, optimize = False):
    position = cam_pos.copy()
    print("position", position)
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
    print("Optimize?", optimize)
    position[3], position[4], position[5] = Angles2rotvec(angle_ver, angle_hoiz)

    #TODO: Optimize this part
    #position[2] -= 0.008*math.sin(abs(view_point[4]* math.pi / 180))
    #position[1] -= 0.0035 * math.sin(abs(view_point[4] * math.pi / 180))
    if abs(view_point[4]) == 90:
        position[2] += 0.008
    print("position 2", position, -view_point[4])
    position[1], position[2] = rotate((cam_pos[1], cam_pos[2]), (position[1], position[2]), -view_point[4] * math.pi / 180)
    print("position 3", position)
    if abs(view_point[4]) == 90:
        position[2] -= 0.003

    if optimize:
        position = optimize_view(s, position.copy(), angle_hoiz, angle_ver)

    return position


def optimize_view(s, position, angle_h, angle_v):
    opt_position = position.copy()

    print("Optimizing angle")
    variance = 0
    for offset in range(-5, 6):
        position[3], position[4], position[5] = Angles2rotvec(angle_v + 0.3 * offset * math.pi / 180, angle_h)
        robot_movej(s, position)
        img = Im.get_image(F"21565643")
        # cv2.imshow("Variance test", img)
        # cv2.waitKey(10)
        # hist_1, _, _ = plt.hist(img.ravel(), 256, [0, 256], histtype='step')

        temp = math.sqrt(np.var(img))
        print("Std:", temp)
        if temp > variance:
            variance = temp
            opt_position[3:6] = position[3:6]
        elif temp < variance * 1.05:
            break

    sharpness = 0
    pre_variance = 0
    print("Optimizing view")
    position[2] -= 0.005
    print("Optimizing height")
    position[3:6] = opt_position[3:6].copy()
    for offset in range(20):
        robot_movej(s, position)
        img = Im.get_image(F"21565643")
        # cv2.imshow("Variance test", img)
        # cv2.waitKey(10)
        # hist_1, _, _ = plt.hist(img.ravel(), 256, [0, 256], histtype='step');
        #temp = math.sqrt(np.var(img))
        array = np.asarray(img, dtype=np.int32)
        gy, gx = np.gradient(array)
        gnorm = np.sqrt(gx ** 2 + gy ** 2)
        temp = np.average(gnorm)

        print("Sharpness:", temp, position[2])
        if temp > sharpness:
            sharpness = temp
            opt_position[2] = position[2]
        elif temp*1.10 < variance and temp >1.5:
            break
        if offset > 1 and False:
            position[2] += (temp-pre_variance)*0.001
        else:
            position[2] += 0.0005

        pre_variance = temp

    #variance = 0
    #position[2] = opt_position[2]
    #position[1] -= 0.005
    #for offset in range(10):
    #    robot_movej(s, position)
    #    img = Im.get_image(F"21565643")
    #    cv2.imshow("Variance test", img)
    #    cv2.waitKey(10)
    #    hist_1, _, _ = plt.hist(img.ravel(), 256, [0, 256], histtype='step');

    #    temp = math.sqrt(np.var(img))
    #    print("Std:", temp)
    #    print("Mean", np.mean(img))
    #    if temp > variance:
    #        variance = temp
    #        opt_position[1] = position[1]
    #    elif temp < variance * 1.05:
    #        break

    #    position[1] += 0.001

    # cv2.destroyWindow("Variance test")

    return opt_position


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



