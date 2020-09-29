# Echo client program
import socket
import time
import Robot_module as Rm
import math
import cv2
import numpy as np
import Image_module as Im
while True:
    img = Im.get_image(F"21565643")

    cv2.imshow("", img)
    array = np.asarray(img, dtype=np.int32)
    gy, gx = np.gradient(array)
    gnorm = np.sqrt(gx ** 2 + gy ** 2)
    temp = np.average(gnorm)
    print("Gradient:", temp)

    temp = cv2.Laplacian(img, cv2.CV_64F).var()
    print("Laplacian:", temp)
    cv2.waitKey()






if False:
    HOST = "192.168.1.17" # Robot ip

    PORT_502 = 502

    robo_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    robo_connection.settimeout(0.5)
    robo_connection.connect((HOST, 30003))

    #robo_connection = Rm.robot_init([61.42 * math.pi / 180, -93 * math.pi / 180, 94.65 * math.pi / 180, -91.59 * math.pi / 180, -90 * math.pi / 180, 0 * math.pi / 180])

    #angle_brick_up = [6.49 * math.pi / 180, -88.63 * math.pi / 180, 90.35 * math.pi / 180, -91.73 * math.pi / 180,-89.91 * math.pi / 180, 0 * math.pi / 180]
    #Rm.robot_movej(robo_connection, angle_brick_up, False, False)

    pos_brick_up = [0.251, -0.437, 0.05, 3.14, 0.00, 0.00]
    Rm.robot_movej(robo_connection, pos_brick_up, wait=False)

    print(Rm.getPosition(robo_connection))
    time.sleep(0.1)
    robo_connection.send(b'set payload(0.1)' + b"\n")
    print(Rm.getPosition(robo_connection))
    time.sleep(0.1)
    robo_connection.send(b'set payload(0.1)' + b"\n")
    print(Rm.getPosition(robo_connection))
    time.sleep(0.1)
    robo_connection.send(b'set payload(0.1)' + b"\n")
    print(Rm.getPosition(robo_connection))
    time.sleep(0.1)
    robo_connection.send(b'set payload(0.1)' + b"\n")
    print(Rm.getPosition(robo_connection))

    #robo_connection.detach()
    #time.sleep(1)
    #
    #s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #s.settimeout(10)
    #s.connect((HOST, PORT_502))
    #time.sleep(0.05)
    #
    #while(1):
    #    Rm.getPosition_modbus(s)
    #    time.sleep(1)