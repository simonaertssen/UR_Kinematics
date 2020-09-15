import numpy as np
import cv2
import glob
import math
import os
from pypylon import pylon
import socket
from PyQt5 import QtCore, QtWidgets
import sys

def send_in_pos_message(MESSAGE):
    TCP_IP = '192.168.1.30'
    TCP_PORT = 20000
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(1)
        s.connect((TCP_IP, TCP_PORT))
        s.send(str.encode(MESSAGE))
        s.close()
    except socket.error as socketerror:
        # Problems with the connection
        #print("Error: ", socketerror)
        pass

    return


def get_image_old(name = F"Basler acA1920-40gm#00305322F4F4#192.168.1.0:3956"):
    # Connect to camera
    print(pylon.CInfoBase.GetFullName(pylon.TlFactory.GetInstance().EnumerateDevices()[0]))
    print(pylon.CInfoBase.GetFullName(pylon.TlFactory.GetInstance().EnumerateDevices()[1]))
    #return cv2.imread("C:\Prj\Robopick\Depot.svn\Lib_backup\\2\mask\side.png")

    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(name))
    camera.open()
    camera.StartGrabbingMax(1)

    while camera.IsGrabbing():
        grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

        if grabResult.GrabSucceeded():
            # Access the image data.
            img = grabResult.Array
            # Crop image
            if name == F"Basler acA1920-40gm#00305322F4F4#192.168.1.0:3956":
                img_crop = img[50:grabResult.Height-65, 140: grabResult.Width - 215]

        grabResult.Release()

    return img_crop


def get_image(serial_number = '22290932'):
    print("Trying to get an image!!")
    # Test code
    #img = cv2.imread("C:\Prj\Robopick\Depot.svn\images\\temp\\cam1_example.png")
    #img_crop = img[55:img.shape[0] - 65, 140: img.shape[1] - 248]
    #return img_crop


    # Pypylon get camera by serial number
    info = None
    img = None

    while info is None:
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
        camera.StartGrabbingMax(1)

        while camera.IsGrabbing():
            grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

            if grabResult.GrabSucceeded():
                # Access the image data.
                img = grabResult.Array
                if serial_number == '22290932':
                    img = img[55:grabResult.Height - 65, 145: grabResult.Width - 248]
                else:
                    img = cv2.flip(img,-1)
        camera.Close()
        camera.DetachDevice()
    return img


def save_image(save, path_save, show, text = "", path_show=r"C:/Users/simon/Documents/JLI Projects/ROBOPICK (Mads L)/Current working version (tortoiseSVN)/depot.svn/Images/temp/temp_save.png", cam_serial=F"21565643"):
    counter = 0
    while True:
        if counter == 10:
            print("ERROR: Could not get an image.")
            return
        try:
            img = get_image(cam_serial)
            break
        except:
            counter += 1
    while save:
        # files = os.listdir('C:/Prj/Robopick/Depot.svn/images/temp')
        files = os.listdir(r"C:/Users/simon/Documents/JLI Projects/ROBOPICK (Mads L)/Current working version (tortoiseSVN)/depot.svn/Images/temp")
        if not 'temp_save.png' in files:
            print("No temp_save.png available")
            break

    if show:
        img_show = img.copy()
        img_show = cv2.putText(img_show, text, (10, img_show.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2, cv2.LINE_AA)
        cv2.imwrite(path_show, img_show)
        #cv2.waitKey(1000)
    if save:
        path_parts = path_save.split("/")
        if path_parts[-2] == "Manual select":
            cv2.imwrite(r"C:/Users/simon/Documents/JLI Projects/ROBOPICK (Mads L)/Current working version (tortoiseSVN)/depot.svn/Images/temp/manual.png", img)
        else:
            cv2.imwrite(path_save, img)


def imgLen2RealLen(length, dimension, img):
    if dimension == 'x':
        return 0.145 + (0.315 - 0.145) * length / img.shape[0]
    elif dimension == 'y':
        return -0.563 + (-0.3205 + 0.563)* length / img.shape[1]

    return length


def template_matching(src, src_mask, template, template_mask):
    template_res = cv2.resize(template, (src.shape[1], src.shape[0]), interpolation=cv2.INTER_CUBIC)
    src_norm = cv2.normalize(src, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    template_res_norm = cv2.normalize(template_res, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F) #todo Make it work with a mask
    template_mask_res = cv2.resize(template_mask, (src_mask.shape[1], src_mask.shape[0]), interpolation=cv2.INTER_CUBIC) #todo Make it work with a mask
    template_res_norm = cv2.cvtColor(template_res_norm, cv2.COLOR_BGR2GRAY)
    template_mask_res = cv2.cvtColor(template_mask_res, cv2.COLOR_BGR2GRAY)
    res_src = cv2.matchTemplate(src_norm, template_res_norm, cv2.TM_SQDIFF_NORMED)
    res_mask = cv2.matchTemplate(src_mask, template_mask_res, cv2.TM_SQDIFF_NORMED)

    return res_src[0, 0] + 5*res_mask[0, 0]


def rotate_image(mat, angle):
    """
    Rotates an image (angle in degrees) and expands image to avoid cropping
    """
    height, width = mat.shape[:2] # image shape has 3 dimensions
    image_center = (width/2, height/2) # getRotationMatrix2D needs coordinates in reverse order (width, height) compared to shape

    rotation_mat = cv2.getRotationMatrix2D(image_center, angle, 1.)

    # rotation calculates the cos and sin, taking absolutes of those.
    abs_cos = abs(rotation_mat[0,0])
    abs_sin = abs(rotation_mat[0,1])

    # find the new width and height bounds
    bound_w = int(height * abs_sin + width * abs_cos)
    bound_h = int(height * abs_cos + width * abs_sin)

    # subtract old image center (bringing image back to origo) and adding the new image center coordinates
    rotation_mat[0, 2] += bound_w/2 - image_center[0]
    rotation_mat[1, 2] += bound_h/2 - image_center[1]

    # rotate image with the new bounds and translated rotation matrix
    rotated_mat = cv2.warpAffine(mat, rotation_mat, (bound_w, bound_h))
    return rotated_mat


def getSubImage(rect, src , mask, check=0):
    # Get center, size, and angle from rect
    center, size, theta = rect
    # Convert to int
    center, size = tuple(map(int, center)), tuple(map(int, size))

    # Get rotation matrix for rectangle
    M = cv2.getRotationMatrix2D( center, theta, 1)
    # Perform rotation on src image
    dst_mask = cv2.warpAffine(mask, M, (mask.shape[0]+500, mask.shape[1]+500))
    dst_src = cv2.warpAffine(src, M, (src.shape[0] + 500, src.shape[1] + 500))

    out_mask = cv2.getRectSubPix(dst_mask, size, center)
    out_src = cv2.getRectSubPix(dst_src, size, center)

    final_mask = np.zeros((out_mask.shape[0]+30,out_mask.shape[1]+30), np.uint8)

    final_src = np.ones((out_mask.shape[0]+30,out_mask.shape[1]+30), np.uint8) * 255


    final_mask[15:-15, 15:-15] = out_mask
    final_src[15:-15, 15:-15] = out_src
    top_ori = 0


    #area = [np.mean(final_mask[:int(final_mask.shape[0]/2), :]), np.mean(final_mask[int(final_mask.shape[0]/2):, :]), np.mean(final_mask[:, :int(final_mask.shape[1]/2)]), np.mean(final_mask[:, int(final_mask.shape[1]/2):])]
    area = [np.mean(final_mask[:, int(final_mask.shape[1] / 2):]),
            np.mean(final_mask[:int(final_mask.shape[0] / 2), :]),
            np.mean(final_mask[:, :int(final_mask.shape[1] / 2)]),
            np.mean(final_mask[int(final_mask.shape[0] / 2):, :])]

    area_index = np.argsort(area)
    #if final_mask.shape[0] < final_mask.shape[1]:
        #top_ori = 1

    #print(area,  np.argmin(area))
    #cv2.imshow("",final_mask)
    #cv2.imshow("2",final_mask[int(final_mask.shape[0] / 2):, :])
    #cv2.waitKey()
    if area[area_index[0]] < area[area_index[1]]*0.98:
        top_ori = area_index[0]
        final_mask = rotate_image(final_mask, -90*top_ori)
        final_src  = rotate_image(final_src, -90*top_ori)
    else:
        if final_mask.shape[0] < final_mask.shape[1]:
            final_mask = rotate_image(final_mask, 90)
            final_src = rotate_image(final_src, 90)
            top_ori = -1


    #return final_mask, cv2.bitwise_and(final_src, final_mask, None), top_ori
    return final_mask[10:-10, 10:-10], final_src[10:-10, 10:-10], top_ori


def findShape(src, mask, s_contour):
    rootdir = 'Library2'
    first_run = True
    type_img = None
    type_name = None
    orientation = None
    d = 10000
    for subdir, dirs, files in os.walk(rootdir):
        for file in files:
            template = cv2.imread(os.path.join(subdir, file), cv2.IMREAD_GRAYSCALE)
            ret, thresh = cv2.threshold(template, 100, 255, cv2.THRESH_BINARY_INV)
            temp_contour, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            rect = cv2.minAreaRect(temp_contour[0])
            temp_mask_cropped, temp_cropped, top_ori = getSubImage(rect, template, thresh)

            if abs(mask.shape[0]/mask.shape[1] - temp_cropped.shape[0]/temp_cropped.shape[1]) < 1:
                d = template_matching(src, mask, temp_cropped, temp_mask_cropped)

            #print(d)
            if d < 1.5:
                if first_run:
                    first_run = False
                    d_min = d
                    type_img = template.copy()
                    type_name = subdir.split(os.path.sep)[-1]
                    orientation = os.path.splitext(file)[0]
                    #cv2.destroyWindow('Cropped src')
                    #cv2.destroyWindow('Cropped template')
                    #cv2.imshow('Cropped src', src)
                    #cv2.imshow('Cropped template', temp_cropped)
                elif d < d_min:
                    d_min = d
                    type_img = template.copy()
                    type_name = subdir.split(os.path.sep)[-1]
                    orientation = os.path.splitext(file)[0]
                    #cv2.destroyWindow('Cropped src')
                    #cv2.destroyWindow('Cropped template')
                    #cv2.imshow('Cropped src', src)
                    #cv2.imshow('Cropped template', temp_cropped)

    return type_img, type_name, orientation


def findShape_v2(src, mask, s_contour , Lookup = None):
    rootdir = r"C:/Prj/Robopick/Depot.svn/Library"
    rootdir = r"C:/Users/simon/Documents/JLI Projects/ROBOPICK (Mads L)/Current working version (tortoiseSVN)/depot.svn/Library"

    first_run = True
    type_img = None
    type_name = None
    orientation = None
    d = 10000
    if Lookup == None:
        library = os.listdir(rootdir)
    else:
        library = Lookup if isinstance(Lookup, list) else [Lookup]

    for subdir in library:
        for path in glob.glob(os.path.join(rootdir, subdir + "/src/*png")):
            file = os.path.split(path)[1]
            temp_cropped = cv2.imread(path).astype(np.uint8)
            temp_mask_cropped = cv2.imread(os.path.join(rootdir, subdir + "/mask/" + file)).astype(np.uint8)

            if abs(mask.shape[0]/mask.shape[1] - temp_cropped.shape[0]/temp_cropped.shape[1]) < 1:
                d = template_matching(src, mask, temp_cropped, temp_mask_cropped)

            #print(d)
            if d < 0.7:
                if first_run:
                    first_run = False
                    d_min = d
                    type_img = temp_mask_cropped.copy()
                    type_name = subdir.split(os.path.sep)[-1]
                    orientation = os.path.splitext(file)[0]

                elif d < d_min:
                    d_min = d
                    type_img = temp_mask_cropped.copy()
                    type_name = subdir.split(os.path.sep)[-1]
                    orientation = os.path.splitext(file)[0]

    return type_img, type_name, orientation


def drawOrientation(s_img, pos, angle, dim, orientation):  # Drawing cross showing contours 2 main axis

    img_ori = s_img.copy()
    pos_cross = [math.cos(math.radians(angle)), math.sin(math.radians(angle))]
    # Brick orientation
    cv2.line(img_ori, (int(pos[0] + pos_cross[0]*dim[0]*0.25), int(pos[1] + pos_cross[1]*dim[0]*0.25)), (int(pos[0] - pos_cross[0]*dim[0]*0.25), int(pos[1] - pos_cross[1]*dim[0]*0.25)), (0, 0, 255), 2)
    cv2.line(img_ori, (int(pos[0] - pos_cross[1]*dim[1]*0.25), int(pos[1] + pos_cross[0]*dim[1]*0.25)), (int(pos[0] + pos_cross[1]*dim[1]*0.25), int(pos[1] - pos_cross[0]*dim[1]*0.25)), (0, 0, 255), 2)
    angle_ori = angle

    # Gripper positions
    var_1 = 0
    if dim[0] < dim[1]:
        var_1 = 1

    gripper_width = 10000
    if gripper_width > dim[0]:
        grip_pose_1 = (int(pos[0] + pos_cross[0]*dim[0]*0.5 + 15*pos_cross[0]), int(pos[1] + pos_cross[1]*dim[0]*0.5 + 15*pos_cross[1]))
        grip_pose_2 = (int(pos[0] - pos_cross[0]*dim[0]*0.5 - 15*pos_cross[0]), int(pos[1] - pos_cross[1]*dim[0]*0.5 - 15*pos_cross[1]))
    else:
        if gripper_width > dim[1]:
            grip_pose_1 = (int(pos[0] - pos_cross[1]*dim[1]*0.5 - 15*pos_cross[1]), int(pos[1] + pos_cross[0]*dim[1]*0.5 + 15*pos_cross[0]))
            grip_pose_2 = (int(pos[0] + pos_cross[1]*dim[1]*0.5 + 15*pos_cross[1]), int(pos[1] - pos_cross[0]*dim[1]*0.5 - 15*pos_cross[0]))
        else:
            print('Error: Brick to big')


    cv2.circle(img_ori, grip_pose_1, 10, (255, 0, 125), 2)
    cv2.circle(img_ori, grip_pose_2, 10, (255, 0, 125), 2)


    # Gripper orientation

    return angle_ori, img_ori


def calcBrickInfo(s_img):  # Find which type of brick and it orientation
    # Find contours
    kernel = np.ones((7, 7), np.uint8)  # avoid colapsing contours, when objects are close together

    imgray = cv2.cvtColor(s_img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray, 200, 255, cv2.THRESH_BINARY_INV)
    #cv2.imshow("test", thresh)
    #cv2.waitKey()
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

            m_cropped, s_cropped, top_ori = getSubImage(rect, imgray, mask)
            shape.append(findShape(s_cropped, m_cropped, contours[i]))

            pos.append(rect[0])
            angle.append(rect[2])
            dim.append(rect[1])


            # Show result
            if shape[-1][0] is not None:
                img_contour = s_img.copy()
                cv2.drawContours(img_contour, [contours[i]], -1, (0, 255, 0), 3)
                cv2.namedWindow('Contour', cv2.WINDOW_NORMAL)
                #cv2.bitwise_or(img_contour, img_ori, img_contour)
                img_ori = drawOrientation(img_contour, pos[-1], angle[-1], dim[-1], shape[-1][2])
                cv2.imshow('Contour', img_ori)
                cv2.namedWindow('Brick type', cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
                cv2.imshow('Brick type', shape[-1][0])

                cv2.waitKey(0)

    return pos, orientation, angle, shape


def findObject(img, Lookup, object_index = -1):
    object_shape = None
    object_pos = None
    object_angle = None
    object_dim  = None
    object_error = False
    object_contours = []
    objects_all_info = []
    none_object_contours = []
    index = -1

    # Convert image format
    if len(img.shape) == 2:
        s_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

        s_img[:, :, 0] = img
        s_img[:, :, 1] = img
        s_img[:, :, 2] = img
        img = s_img
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find contours
    kernel = np.ones((7, 7), np.uint8)  # avoid colapsing contours, when objects are close together
    ret, thresh = cv2.threshold(imgray, 150, 255, cv2.THRESH_BINARY_INV)
    # SIMON
    _, contours, hierarchy = cv2.findContours(cv2.erode(thresh, kernel, iterations=1), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    empty = np.zeros(thresh.shape, np.uint8)
    # Go through all contours
    for i in range(len(contours)):
        area = cv2.contourArea(contours[i])
        if area > 500 and area < 500000:
            # Draw contour
            rect = cv2.minAreaRect(contours[i])
            mask = empty.copy()
            cv2.drawContours(mask, [contours[i]], -1, 255, cv2.FILLED)

            # Get cropped figures
            m_cropped, s_cropped, top_ori = getSubImage(rect, imgray, mask)
            # Find shape to figure out how to grip the object
            shape = findShape_v2(s_cropped, m_cropped, contours[i], Lookup)
            if shape[0] is not None:
                index += 1
                if object_index == -1 or object_index == -2 or object_index == index: #or cv2.pointPolygonTest(contours[i], pos, True) < 30:
                    object_shape = shape
                    object_pos = rect[0]
                    object_angle = (rect[2] - 90 * (top_ori))
                    object_dim = rect[1]

                    #object_shape.append(shape)
                #object_pos.append(rect[0])
                #object_angle.append((rect[2] - 90 * (top_ori)))
                #object_dim.appned(rect[1])

                objects_all_info.append([shape, rect[0], (rect[2] - 90 * (top_ori)),rect[1], [contours[i]], img, object_error])
                object_contours.append(contours[i])
            else:
                none_object_contours.append(contours[i])

        if area > 100000:
            object_error = True

    return [object_shape, object_pos, object_angle, object_dim, object_contours, img, object_error], none_object_contours, objects_all_info


def findOptimalImage(image):
    print("Finding optimal image")
    pass

class Dialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super(Dialog, self).__init__(parent)
        self.val = 'Unknown'
        self.initUI()

    def initUI(self):
        select_label = QtWidgets.QLabel("Select class")
        select_label.setAlignment(QtCore.Qt.AlignCenter)
        error_button = QtWidgets.QPushButton('1: Error')
        error_button.clicked.connect(lambda: self.on_clicked('Error'))
        correct_button = QtWidgets.QPushButton('2: Correct')
        correct_button.clicked.connect(lambda: self.on_clicked('Correct'))
        unknow_button = QtWidgets.QPushButton('3: Unknown')
        unknow_button.clicked.connect(lambda: self.on_clicked('Unknown'))
        lay = QtWidgets.QVBoxLayout(self)
        lay.addWidget(select_label)
        lay.addWidget(error_button)
        lay.addWidget(correct_button)
        lay.addWidget(unknow_button)
        self.setWindowTitle(str(self.val))

        select_label.setStyleSheet('font-weight: bold;')
        error_button.setStyleSheet('QPushButton{Text-align:left}')
        correct_button.setStyleSheet('QPushButton{Text-align:left}')
        unknow_button.setStyleSheet('QPushButton{Text-align:left}')

        self.show()

    @QtCore.pyqtSlot()
    def on_clicked(self, object_class):
        self.val = object_class
        self.accept()
        print("hej")

    def keyPressEvent(self, event):
        print("HEJ HEJ")
        if event.key() == QtCore.Qt.Key_1:
            self.val = 'Error'
            self.accept()
        if event.key() == QtCore.Qt.Key_2:
            self.val = 'Correct'
            self.accept()
        if event.key() == QtCore.Qt.Key_3:
            self.val = 'Unknow'
            self.accept()


if __name__ == '__main__':
    # Test functions

    #Read in image
    images = [cv2.imread(file) for file in glob.glob('../Test_Images/*bmp')]
    for i in range(len(images)):
        img_raw = images[i]
        pos, orientation, angle, shape = calcBrickInfo(img_raw)

