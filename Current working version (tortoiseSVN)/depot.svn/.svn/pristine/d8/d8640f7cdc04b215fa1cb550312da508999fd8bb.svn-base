import numpy as np
import cv2
import glob
import math
import os


def template_matching(src, src_mask, template, template_mask):
    template_res = cv2.resize(template, (src.shape[1], src.shape[0]), interpolation=cv2.INTER_CUBIC)
    src_norm = cv2.normalize(src, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    template_res_norm = cv2.normalize(template_res, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F) #todo Make it work with a mask
    template_mask_res = cv2.resize(template_mask, (src_mask.shape[1], src_mask.shape[0]), interpolation=cv2.INTER_CUBIC) #todo Make it work with a mask
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
    final_mask = np.zeros((out_mask.shape[0]+10,out_mask.shape[1]+10), np.uint8)
    final_src = final_mask.copy()
    final_mask[5:-5, 5:-5] = out_mask
    final_src[5:-5, 5:-5] = out_src

    area = [np.mean(final_mask[:int(final_mask.shape[0]/2), :]), np.mean(final_mask[int(final_mask.shape[0]/2):, :]), np.mean(final_mask[:, :int(final_mask.shape[1]/2)]), np.mean(final_mask[:, int(final_mask.shape[1]/2):])]

    area_index = np.argsort(area)

    if area[area_index[0]] < area[area_index[1]]*0.98:
        top_ori = 0
        if area_index[0] == 0:
            final_mask = rotate_image(final_mask, -90)
            final_src  = rotate_image(final_src, -90)
            top_ori = 1
        elif area_index[0] == 1:
            final_mask = rotate_image(final_mask, 90)
            final_src  = rotate_image(final_src, 90)
            top_ori = 2
        elif area_index[0] == 2:
            final_mask = rotate_image(final_mask, 180)
            final_src  = rotate_image(final_src, 180)
            top_ori = 3
    else:
        top_ori = None
        if final_mask.shape[0] < final_mask.shape[1]:
            final_mask = rotate_image(final_mask, 90)
            final_src = rotate_image(final_src, 90)

    return final_mask, cv2.bitwise_and(final_src, final_mask, None), top_ori


def findShape(src, mask, s_contour):
    rootdir = '..\Library'
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

            if d < 1:
                if first_run:
                    first_run = False
                    d_min = d
                    type_img = template.copy()
                    type_name = subdir.split(os.path.sep)[-1]
                    orientation = os.path.splitext(file)[0]
                    cv2.destroyWindow('Cropped src')
                    cv2.destroyWindow('Cropped template')
                    cv2.imshow('Cropped src', src)
                    cv2.imshow('Cropped template', temp_cropped)
                elif d < d_min:
                    d_min = d
                    type_img = template.copy()
                    type_name = subdir.split(os.path.sep)[-1]
                    orientation = os.path.splitext(file)[0]
                    cv2.destroyWindow('Cropped src')
                    cv2.destroyWindow('Cropped template')
                    cv2.imshow('Cropped src', src)
                    cv2.imshow('Cropped template', temp_cropped)

    return type_img, type_name, orientation


def drawOrientation(s_img, pos, angle, dim, orientation, top_ori):  # Drawing cross showing contours 2 main axis
    img_ori = np.zeros(s_img.shape, np.uint8)
    img_ori = s_img.copy()
    pos_cross = [math.cos(math.radians(angle)), math.sin(math.radians(angle))]
    # Brick orientation
    cv2.line(img_ori, (int(pos[0] + pos_cross[0]*dim[0]*0.25), int(pos[1] + pos_cross[1]*dim[0]*0.25)), (int(pos[0] - pos_cross[0]*dim[0]*0.25), int(pos[1] - pos_cross[1]*dim[0]*0.25)), (0, 0, 255), 2)
    cv2.line(img_ori, (int(pos[0] - pos_cross[1]*dim[1]*0.25), int(pos[1] + pos_cross[0]*dim[1]*0.25)), (int(pos[0] + pos_cross[1]*dim[1]*0.25), int(pos[1] - pos_cross[0]*dim[1]*0.25)), (0, 0, 255), 2)


    # Gripper positions
    var_1 = 0
    if dim[0] < dim[1]:
        var_1 = 1

    gripper_width = 10000
    if dim[0] > dim[1] and gripper_width > dim[0]:
        grip_pose_1 = (int(pos[0] + pos_cross[0]*dim[0]*0.5 + 15*pos_cross[0]), int(pos[1] + pos_cross[1]*dim[0]*0.5 + 15*pos_cross[1]))
        grip_pose_2 = (int(pos[0] - pos_cross[0]*dim[0]*0.5 - 15*pos_cross[0]), int(pos[1] - pos_cross[1]*dim[0]*0.5 - 15*pos_cross[1]))
        if orientation != 'side':
            if top_ori == 2:# Not necessary  - top_ori can be removed
                grip_orientation = (int(pos[0] - pos_cross[1] * dim[1]), int(pos[1] + pos_cross[0]*dim[1]))
            else:
                grip_orientation = (int(pos[0] + pos_cross[1] * dim[1]), int(pos[1] - pos_cross[0] * dim[1]))
        else:
            grip_orientation = (int(pos[0]), int(pos[1]))
    else:
        if gripper_width > dim[1]:
            grip_pose_1 = (int(pos[0] - pos_cross[1]*dim[1]*0.5 - 15*pos_cross[1]), int(pos[1] + pos_cross[0]*dim[1]*0.5 + 15*pos_cross[0]))
            grip_pose_2 = (int(pos[0] + pos_cross[1]*dim[1]*0.5 + 15*pos_cross[1]), int(pos[1] - pos_cross[0]*dim[1]*0.5 - 15*pos_cross[0]))
            if orientation != 'side':
                if top_ori == 1:# Not necessary
                    grip_orientation = (int(pos[0] + pos_cross[0] * dim[0]), int(pos[1] + pos_cross[1] * dim[0]))
                else:
                    grip_orientation = (int(pos[0] - pos_cross[0] * dim[0]), int(pos[1] - pos_cross[1] * dim[0]))
            else:
                grip_orientation = (int(pos[0]), int(pos[1]))
        else:
            print('Error: Brick to big')


    cv2.circle(img_ori, grip_pose_1, 10, (255, 0, 125), 2)
    cv2.circle(img_ori, grip_pose_2, 10, (255, 0, 125), 2)
    cv2.circle(img_ori, grip_orientation, 10, (255, 125, 0), 2)

    # Gripper orientation

    return img_ori


def calcBrickInfo(s_img):  # Find which type of brick and it orientation
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
                img_ori = drawOrientation(img_contour, pos[-1], angle[-1], dim[-1], shape[-1][2], top_ori)
                cv2.imshow('Contour', img_ori)
                cv2.namedWindow('Brick type', cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
                cv2.imshow('Brick type', shape[-1][0])

                cv2.waitKey(0)

    return pos, orientation, angle, shape

if __name__ == '__main__':
    # Test functions

    #Read in image
    images = [cv2.imread(file) for file in glob.glob('../Test_Images/*bmp')]
    for i in range(len(images)):
        img_raw = images[i]
        pos, orientation, angle, shape = calcBrickInfo(img_raw)

