import numpy as np
import cv2
import os
import time
from pypylon import pylon
from pypylon_opencv_viewer import BaslerOpenCVViewer
import glob
import socket
import binascii
from struct import *


HOST = "192.168.111.2"
PORT = 30003

# Robot functions
def getPosition(s):
    count = 0
    while count < 1000:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(10)
            s.connect((HOST, PORT))
            time.sleep(0.1)

            s.recv(588) # Other information

            # Position
            X  = unpack('!d', bytes.fromhex(str( binascii.hexlify( s.recv(8)).decode("utf-8"))))[0]
            Y  = unpack('!d', bytes.fromhex(str( binascii.hexlify( s.recv(8)).decode("utf-8"))))[0]
            Z  = unpack('!d', bytes.fromhex(str( binascii.hexlify( s.recv(8)).decode("utf-8"))))[0]
            RX = unpack('!d', bytes.fromhex(str( binascii.hexlify( s.recv(8)).decode("utf-8"))))[0]
            RY = unpack('!d', bytes.fromhex(str( binascii.hexlify( s.recv(8)).decode("utf-8"))))[0]
            RZ = unpack('!d', bytes.fromhex(str( binascii.hexlify( s.recv(8)).decode("utf-8"))))[0]
            break
        except socket.error as socketerror:
            print("Error: ", socketerror)
            count += 1

    return [X,Y,Z,RX,RY,RZ]

def waitPos(pos_targ, s):

    count = 0
    while count < 3:
        pos_cur = getPosition(s)

        pos_diff = np.abs(np.array([pos_cur]) - np.array([pos_targ]))
        print(np.sum(pos_diff))
        if np.sum(pos_diff) < 10e-3:
            count += 1

    return True



def robot_init(pos_init):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))  # Connect to robot

    robot_movej(s, pos_init)
    s.send(b'set_tool_voltage(12)' + b'\n')  # Set gripper voltage - avoids the current error
    time.sleep(0.5)
    s.send(b'set_tool_voltage(24)' + b'\n')  # Set gripper voltage - avoids the current error
    time.sleep(0.5)
    s.send(b'set_digital_out(8,False)' + b"\n")  # Open gripper
    time.sleep(1)
    return s

def robot_movej(s, pos):
    string = "movej(p[%s, %s, %s, %s, %s, %s]) \n" % (pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
    #print(string)
    s.send(str.encode(string)) # comment out for camera TEST
    waitPos(pos, s)
    return string

def robot_movel(s, pos):
    string = "movel(p[%s, %s, %s, %s, %s, %s]) \n" % (pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
    #print(string)
    s.send(str.encode(string)) # comment out for camera TEST
    waitPos(pos, s)
    return string

# Camera functions
def cam_init(features):
    serial_number = '22290932'
    info = None
    for i in pylon.TlFactory.GetInstance().EnumerateDevices():
        if i.GetSerialNumber() == serial_number:
            info = i
            break
    else:
        print('Camera with {} serial number not found'.format(serial_number))

    # VERY IMPORTANT STEP! To use Basler PyPylon OpenCV viewer you have to call .Open() method on you camera
    if info is not None:
        camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(info))
        camera.Open()

    viewer = BaslerOpenCVViewer(camera)
    #viewer.set_features(features)
    return viewer

if __name__ == '__main__':
    # - Positions encoding: X, Y, Z, RX, RY, RZ
    pos_idle        = [-0.126, -0.460,  0.325,  2.22,  2.22,  0.00]
    pos_brick_up   = [ 0.259, -0.207,  0.325,  0.00,  3.14,  0.00]
    pos_brick_down = [ 0.259, -0.207,  0.040,  0.00,  3.14,  0.00]
    pos_brick_drop  = [-0.522,  0.100,  0.050, -3.14,  0.08,  0.00]
    pos_read_up     = [-0.591, -0.491,  0.325, -2.91,  1.17, -0.03]
    pos_read_down   = [-0.591, -0.491,  0.040, -2.91,  1.17, -0.03]


    # Initialize Robot
    s = robot_init(pos_idle)
    s.send(b'set_tool_voltage(24)' + b'\n') # Set gripper voltage - avoids the current error
    time.sleep(0.5)
    s.send(b'set_digital_out(8,False)' + b"\n") # Open gripper
    time.sleep(0.5)

    # Grab brick
    robot_movej(s, pos_brick_up)
    robot_movel(s, pos_brick_down)
    s.send(b'set_digital_out(8,True)' + b"\n") # Open gripper
    time.sleep(0.5)
    robot_movel(s, pos_brick_up)

    # Move brick in front of read camera
    robot_movej(s, pos_read_up)
    robot_movel(s, pos_read_down)
    robot_movel(s, pos_read_up)

    # Drop brick
    robot_movej(s, pos_brick_drop)
    s.send(b'set_digital_out(8,False)' + b"\n") # Open gripper
    time.sleep(0.5)

    # todo Robot: Move to idle
    robot_movej(s, pos_idle)

    # todo Robot: End connection
    pos_final = getPosition(s) * 1000
    s.close()
    print("Fianl position", pos_final)
    print("End Program")


