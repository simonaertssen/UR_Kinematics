# Echo client program
import socket
import time
import cv2
import image_processing as ip
import numpy as np
import random
import glob
import math

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
    multi = 1 / (2 * math.sin(theta))

    rx = multi * (R[2, 1] - R[1, 2]) * theta
    ry = multi * (R[0, 2] - R[2, 0]) * theta
    rz = multi * (R[1, 0] - R[0, 1]) * theta

    return rx, ry, rz


HOST = "192.168.111.22"  # This computer - remember to set in in the robot!!!
PORT = 30002  # The same port as used by the server

images = [cv2.imread(file) for file in glob.glob('../Test_Images/*bmp')]
s_img = images[random.randint(0,len(images)-1)]#cv2.imread('C:/Users/Paul Toft Duizer/Desktop/Mads Lyngsaae Nielsen/Lego robot/Test_Images/4_blocks_9.bmp')
# Find contours
kernel = np.ones((7, 7), np.uint8)  # avoid colapsing contours, when objects are close together

imgray = cv2.cvtColor(s_img, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(imgray, 100, 255, cv2.THRESH_BINARY_INV)
contours, hierarchy = cv2.findContours(cv2.erode(thresh, kernel, iterations=1), cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)
pos = []
orientation = []
angle = []
shape = []
dim = []
empty = np.zeros(thresh.shape, np.uint8)
for i in range(len(contours)):
    area = cv2.contourArea(contours[i])
    if area > 1000 and area < 100000:
        rect = cv2.minAreaRect(contours[i])
        mask = empty.copy()
        cv2.drawContours(mask, [contours[i]], -1, 255, cv2.FILLED)

        m_cropped, s_cropped, top_ori = ip.getSubImage(rect, imgray, mask)
        shape.append(ip.findShape(s_cropped, m_cropped, contours[i]))

        pos.append(rect[0])
        angle.append(rect[2])
        dim.append(rect[1])


        # Show result
        if shape[-1][0] is not None:
            img_contour = s_img.copy()
            cv2.drawContours(img_contour, [contours[i]], -1, (0, 255, 0), 3)
            cv2.namedWindow('Contour', cv2.WINDOW_NORMAL)
            #cv2.bitwise_or(img_contour, img_ori, img_contour)
            img_ori = ip.drawOrientation(img_contour, pos[-1], angle[-1], dim[-1], shape[-1][2], top_ori)
            cv2.imshow('Contour', img_ori)
            cv2.namedWindow('Brick type', cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
            cv2.imshow('Brick type', shape[-1][0])
            cv2.waitKey(1)
            #Robot
            #print("Brick located: Press any key to process highlighted brick")
            #cv2.waitKey(0)
            done = False
            print(shape[-1][2], angle[-1], angle[-1] * math.pi / 180)
            while( done == False):
                print(1)
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind((HOST, PORT))  # Bind to the port
                s.listen(5)  # Now wait for client connection.
                c, addr = s.accept()  # Establish connection with client.
                print(2)

                try:
                    msg = c.recv(1024)
                    print(msg)
                    time.sleep(1)
                    if msg == b'asking_for_data':
                        print(shape[-1][2], angle[-1], angle[-1] * math.pi / 180)
                        print("Message 1:", msg)

                        # Calculate orientations

                        rx, ry, rz = RPY2rotvec(math.pi, math.pi / 4, angle[-1] * math.pi / 180)
                        if shape[-1][2] == 'side':
                            rx, ry, rz = RPY2rotvec(math.pi, 1*math.pi/180, angle[-1] * math.pi / 180)
                        else:
                            rx, ry, rz = RPY2rotvec(math.pi, math.pi / 4, angle[-1] * math.pi / 180)
                        # Process picture: Find location
                        location = [-0.525, 0.01, 0.02, rx, ry, rz]
                        time.sleep(0.5)
                        time.sleep(0.5)
                        c.send(str.encode("(%s, %s, %s, %s, %s, %s)" % (
                        location[0], location[1], location[2], location[3], location[4], location[5])));
                    elif msg == b'take_picture':
                        print("Message 2: ", msg)
                        #frame = cv2.imread(
                        #    'C:/Users/Paul Toft Duizer/Desktop/Mads Lyngsaae Nielsen/Lego robot/Test_Images/4_blocks_9.bmp')
                        #cv2.imshow('Scan', frame)
                        #cv2.waitKey(1)
                        c.send(b"(1)")
                    elif msg == b'idle':
                        print("Message 3: ", msg)
                        done = True

                except socket.error as socketerror:
                    print(1)

                # Process picture: find number

                c.close()
                s.close()
