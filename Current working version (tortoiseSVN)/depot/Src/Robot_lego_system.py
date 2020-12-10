# Import libraries
import numpy as np
import time
import cv2
import math
import os
from tkinter import *

import Image_module as Im
import Robot_module as Rm

# Gui to select the type of objects we want to find
def SelectObjects():
    selected = list()
    top = Tk()

    mb=  Menubutton ( top, text="Select library types", relief=RAISED )
    mb.grid()
    mb.menu  =  Menu ( mb, tearoff = 0 )
    mb["menu"]  =  mb.menu

    dirs = sorted(os.listdir('C:/Prj/Robopick/Depot.svn/Library'), key=int)
    items = list()
    for d, i in zip(dirs,range(len(dirs))):
        items.append(IntVar())
        mb.menu.add_checkbutton(label=d, variable=items[i])


    def close():
        for i, counter in zip(items,range(len(dirs))):
            if i.get() == 1:
                selected.append(dirs[counter])
        top.destroy()

    mb_close= Button( top, text="Start robot", command=close )

    mb.pack()
    mb_close.pack()
    top.mainloop()

    print("Types selected:", selected)
    return selected


def Robot(Robo_info, s = None,  reset = False, camera = None):
        print("Run robot", len(Robo_info))
        print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
        #return s, False
        # Default positions
        # Positions encoding: X, Y, Z, RX, RY, RZ
        pos_idle =       [-0.126,   -0.460,  0.317,  0.00,  3.14,  0.00]
        joint_idle =     [61.42*math.pi/180, -93*math.pi/180, 94.65*math.pi/180, -91.39*math.pi/180, -90*math.pi/180, 0*math.pi/180]
        pos_brick_drop = [ 0.085,   -0.516,  0.050,  0.00,  3.14,  0.00]
        angle_brick_drop = [87.28 * math.pi / 180, -74.56 * math.pi / 180, 113.86 * math.pi / 180, -129.29 * math.pi / 180, -89.91 * math.pi / 180, -2.73 * math.pi / 180]

        # Initialize Robot
        if s is None:
            s = Rm.robot_init(joint_idle)
        elif reset:
            s = Rm.robot_init(joint_idle, s)

        if Robo_info[0] is None or len(Robo_info) != 13:
            if Robo_info[-1] is not None:
                drop_objecct(s, Robo_info[-1], Robo_info[5])

            s.send(b'set_digital_out(8,False)' + b"\n")  # Open gripper
            time.sleep(0.1)
            Rm.robot_movej(s, joint_idle, 1, False)
            print("ERROR: RETURN FROM ROBOT", Robo_info[0] is None, len(Robo_info) != 13, len(Robo_info))
            return s, False

        # Work directory
        rootdir = "../Library"#r"C:/Prj/Robopick/Depot.svn/Library"
        shape, pos, angle, dim, contours, img, _, save_image, show_image, data_type, contour_number, optimize_view, drop_position = Robo_info
        angle *= -1
        # Show result
        print("DATA", data_type)
        contour = contours[contour_number]

        img_contour = img.copy()
        cv2.drawContours(img_contour, [contour], -1, (0, 255, 0), 3)
        #cv2.bitwise_or(img_contour, img_ori, img_contour)
        angle, img_ori = Im.drawOrientation(img_contour, pos, angle, dim, shape[2])
        #cv2.imshow('Brick type', shape[-1][0])
        #cv2.waitKey(1)

        # Run Robot
        # Get pick points
        pick_points = np.load(rootdir + '/' + shape[1] + '/points/' + shape[2] + '_pickpoints.npy')
        #print("Pick points", pick_points)

        # Grab brick

        if drop_position is not None:
            drop_objecct(s,drop_position,img)
        else:
            s.send(b'set_digital_out(8,False)' + b"\n")  # Open gripper
            time.sleep(0.1)

        angle_view_v = get_objecct(s, shape, pos, angle , pick_points, img)

        #Rm.robot_movej(s, joint_idle, 1, False)

        if pick_points[1, 0] != -1 or pick_points[int(pick_points[0,4]+1), 0] != -1:
            angle_brick_up = [6.49 * math.pi / 180, -88.63 * math.pi / 180, 90.35 * math.pi / 180, -91.73 * math.pi / 180, -89.91 * math.pi / 180, 0 * math.pi / 180]
            Rm.robot_movej(s, angle_brick_up, 1, False)

        # Present object to camera 1
        if pick_points[1,0] != -1 and False:
            present_object_cam1(pick_points[: int(pick_points[0,4]+1),:],  s, img, angle_view_v)

        # Present object to camera 2
        if pick_points[int(pick_points[0,4]+1), 0] != -1:
            # Turn on light
            #Rm.set_Light(s, False)
            Rm.set_IO_PORT(s, 0, True)
            
            present_object_cam2(pick_points, s, img, angle_view_v, Robo_info[0][1], data_type, save_image, show_image, optimize_view, camera)

            # Turn off light
            #Rm.set_Light(s, True)
            Rm.set_IO_PORT(s, 0, False)

        # Drop brick
        #Rm.robot_movej(s, joint_idle, 1, False)
        #Rm.robot_movej(s, pos_brick_drop)
        Rm.robot_movej(s, angle_brick_drop, 1, False)

        # TODO: MAKE THIS WORK SO IT DROPS IF IT ONLY USES ONE BRICK
        #if drop_position is None:
            #s.send(b'set_digital_out(8,False)' + b"\n")  # Open gripper
            #time.sleep(0.1)


        #Rm.robot_movej(s, joint_idle, 1, False)

        pos_final = Rm.getPosition()
        pos_final[:2] = pos_final[:2] * 1000




        return s , True

def get_objecct(s, shape, pos, angle, points, img, get = True):
    pos_brick_up = [0.251, -0.437, 0.05, 3.14, 0.00, 0.00]
    pos_brick_down = [0.251, -0.437, 0.008, 3.14, 0.00, 0.00]  # [ 0.251,   -0.437,  0.034,  3.14,  0.00,  0.00]

    # Get brick positio5
    x_offset = shape[0].shape[1] / 2
    y_offset = shape[0].shape[0] / 2

    # Offset line
    y = shape[0].shape[0] - (points[0, 3] + points[0, 1]) / 2 - y_offset # make y image coordinate to normal coordinates
    test2 = math.atan2(y ,((points[0, 2] + points[0, 0]) / 2 - x_offset))

    angle_temp = (angle * math.pi / 180 + test2)
    length = math.sqrt(((points[0, 2] + points[0, 0]) / 2 - x_offset) ** 2 + ((points[0, 3] + points[0, 1]) / 2 - y_offset) ** 2)

    # Scale and offset position to real world coordinates -  1.01  and 0.98 used for calibration
    x = Im.imgLen2RealLen((pos[1]*1.01 - math.sin(angle_temp) * length), 'x', img) + pos[0]/img.shape[1]*0.003
    print(pos[0]/img.shape[1])
    y = Im.imgLen2RealLen((pos[0] + math.cos(angle_temp) * length), 'y', img)
    print(y, pos[0],  angle_temp, length, math.cos(angle_temp) * length)
    #cv2.waitKey()

    # Set pick positions
    pos_brick_up[0:2] = [x, y]
    pos_brick_down[0:2] = [x, y]

    # Set picking
    # Set view angles
    temp = -1 * (math.atan2( points[0, 3] - points[0, 1], points[0, 2] - points[0, 0]))
    temp2 = -1 * (math.atan2( points[1, 1] - points[1, 3], points[1, 0] - points[1, 2]))
    temp3 = 3

    if temp < 0:
        temp += math.pi
        temp3 = 5

    #Adjust angle to real world
    angle = -angle + temp * 180 / math.pi
    angle_view_v = points[1, 4] * math.pi / 180

    if temp2 < 0:
        temp2 += math.pi

    # check if it is possible to grip the object with an angle
    if angle_view_v >= math.pi / 4 and abs(abs(temp2 - temp) - math.pi / 2) < math.pi * 0.03 and points.shape[0] == 2:
        angle_view_v -= math.pi / 4
        pos_brick_up[3], pos_brick_up[4], pos_brick_up[5] = Rm.RPY2rotvec(0, math.pi * temp3 / 4, (180 - angle) * math.pi / 180)
    else:
        angle_view_v = None
        pos_brick_up[3], pos_brick_up[4], pos_brick_up[5] = Rm.RPY2rotvec(0, math.pi, (180 - angle) * math.pi / 180)

    pos_brick_down[3:] = pos_brick_up[3:]

    # Move robot
    Rm.robot_movej(s, pos_brick_up)
    Rm.robot_movel(s, pos_brick_down)
    if get:
        s.send(b'set_digital_out(8, True)' + b"\n")  # Open gripper
    else:
        s.send(b'set_digital_out(8, False)' + b"\n")

    time.sleep(0.25)
    Rm.robot_movel(s, pos_brick_up)

    return angle_view_v


def drop_objecct(s, pos, img, drop = True):
    pos_brick_up = [0.251, -0.437, 0.05, 3.14, 0.00, 0.00]
    pos_brick_down = [0.251, -0.437, 0.01, 3.14, 0.00, 0.00]  # [ 0.251,   -0.437,  0.034,  3.14,  0.00,  0.00]

    # Scale and offset position to real world coordinates -  1.01  and 0.98 used for calibration
    x = Im.imgLen2RealLen(pos[1], 'x', img)
    y = Im.imgLen2RealLen(pos[0], 'y', img)
    pos_brick_up[0:2] = [x, y]
    pos_brick_down[0:2] = [x, y]

    # check if it is possible to grip the object with an angle
    pos_brick_up[3], pos_brick_up[4], pos_brick_up[5] = Rm.RPY2rotvec(0, math.pi, (180 - pos[2]) * math.pi / 180)
    pos_brick_down[3:] = pos_brick_up[3:]

    # Move robot
    Rm.robot_movej(s, pos_brick_up)
    Rm.robot_movel(s, pos_brick_down)
    if drop:
        s.send(b'set_digital_out(8, False)' + b"\n")
    else:
        s.send(b'set_digital_out(8, True)' + b"\n")  # Open gripper

    time.sleep(0.1)
    Rm.robot_movel(s, pos_brick_up)


def present_object_cam1(points, s, img, angle_view_v): # TODO: Present the object correct in all all plane. Seems to be switched around in XY plane or Z direction
    pre_angle =  -1000
    pos_read_down = [-0.47835, -0.17087, 0.070, -1.29, 1.29, 0.00]
    ori_point = pos_read_down[0:3].copy()

    temp = -1 * (math.atan2(points[0, 3] - points[0, 1], points[0, 2] - points[0, 0]))

    if temp < 0:
        temp += math.pi

    for i in range(1, points.shape[0]):
        pos_read_up = [-0.48135, -0.17087, 0.317, 1.29, -1.29, 0.00]
        pos_read_down = [-0.48135, -0.17087, 0.08, -1.29, 1.29, 0.00]

        length = math.sqrt(((points[0, 2] + points[0, 0]) / 2 - points[i, 2]) ** 2 + ((points[0, 3] + points[0, 1]) / 2 - points[i, 3]) ** 2)
        temp2 = math.atan2((points[i, 3] - (points[0, 3] + points[0, 1]) / 2), points[i, 2] - (points[0, 2] + points[0, 0]) / 2) + math.pi
        temp3 = math.atan2(points[i, 3] - points[i, 1], points[i, 2] - points[i, 0])

        pos_read_up[0] += math.sin(temp2) * (-0.3125 + 0.5627) * abs(length) / img.shape[1]
        pos_read_up[1] += math.cos(temp2) * (-0.3125 + 0.5627) * abs(length) / img.shape[1]


        pos_read_up[0], pos_read_up[1] = Rm.rotate((ori_point[0], ori_point[1]), (pos_read_up[0], pos_read_up[1]), temp3)
        _, pos_read_down[2] = Rm.rotate((ori_point[1], ori_point[2]), (pos_read_up[1], pos_read_down[2]), points[i, 4] * math.pi / 180)
        temp2 = -1 * math.atan2(points[i, 1] - points[i, 3], points[i, 0] - points[i, 2])
        angle_view_h = temp2 - abs(math.pi - temp)
        if True  or angle_view_v is None:
            angle_view_v = points[i, 4] * math.pi / 180
            pos_read_down[2] += 0.005 * math.sin(angle_view_v)
            pos_read_down[1] -= 0.006 * math.sin(angle_view_v)


        pos_read_up[3], pos_read_up[4], pos_read_up[5] = Rm.Angles2rotvec(angle_view_v, angle_view_h)

        #if points[i, 4] != 0:
        #    pos_read_up[1] = -0.16387

        pos_read_down[0] = pos_read_up[0]
        pos_read_down[3:6]= pos_read_up[3:6]

        # Move robot
        if points[i, 4] != pre_angle:
            Rm.robot_movej(s, pos_read_up)

        Rm.robot_movej(s, pos_read_down)
        Im.send_in_pos_message('Hello, World!')
        cv2.waitKey(1000)

        angle_view_v = None
        pre_angle = points[i, 4]

    Rm.robot_movej(s, pos_read_up)


def present_object_cam2(points, s, img, angle_view_v, type_name, data_type, save_image, show_image, optimize_view, camera):

    pos_brick_angle = [-0.068*math.pi/180, -91.45*math.pi/180, 94.01*math.pi/180, -92.59*math.pi/180, 87*math.pi/180, 180*math.pi/180]
    #pos_brick_angle = [-1.68 * math.pi / 180, -91.63 * math.pi / 180, 78.43 * math.pi / 180, -76.98 * math.pi / 180, 84.27 * math.pi / 180, 1.74 * math.pi / 180]

    cam_pos = [-0.476, -0.131 , 0.649, 0.041, 0.0412, 1.5704]

    if points[points[0,4]+1,0] != -51 and points[points[0,4]+1,1] != -51:
        Rm.robot_movej(s,pos_brick_angle, 1, False)
        path = None

        if save_image:
            path = 'C:/Prj/Robopick/Depot.svn/images/' + type_name
            if not os.path.exists(path):
                os.mkdir(path)

            path += '/'+data_type
            if not os.path.exists(path) and data_type != "Manual select":
                os.mkdir(path)


        temp = -1 * (math.atan2(points[0, 3] - points[0, 1], points[0, 2] - points[0, 0]))

        if temp < 0:
            temp += math.pi

        for i in range(points[0,4]+1,points[0,4]+1+points[int(points[0,4]+1):,:].shape[0]):
            if points[i, 7] == 0:
                print("Robot performing point view")
                Rm.robot_point_view(points[0, :], points[i, :], cam_pos, temp, img, save_image, show_image, path, s, optimize_view, camera=camera)
            elif points[i, 7] == 2:
                print("Robot performing point view rotation")
                Rm.robot_point_rotation(points[0, :], points[i, :], cam_pos, temp, img, save_image, show_image, path, s)
            elif points[i, 7] == 1:
                print("Robot performing line sweep")
                try:
                    if type_name == "Demo":
                        points[i, 0] = points[i, 2]
                        points[i,1] = 370
                    Rm.robot_line_sweep(points[0, :], points[i, :], cam_pos, temp, img, save_image, show_image, path, s, optimize_view, camera=camera)
                except:
                    print("LINE SWEEP ERROR!!!")

            Rm.robot_movej(s, pos_brick_angle, 1, False)


if __name__ == '__main__':
    Lookup = SelectObjects()
    while(True):
        img = Im.get_image()
        cv2.namedWindow('Contour', cv2.WINDOW_NORMAL)
        cv2.imshow('Contour', img)
        cv2.waitKey(1)
        Robo_info = Im.findObject(img, Lookup)
        Robot(Robo_info)
