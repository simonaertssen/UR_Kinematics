import time
import numpy as np
import cv2 as cv
import os


def findObjectsToPickUp(image_to_extract):
    # Light box dimensions:
    LIGHT_BOX_LENGTH = 0.252  # m
    LIGHT_BOX_WIDTH = 0.177  # m

    # Extract image in a brute way
    # image_to_extract = image_to_extract[50:-53, 158:-198].copy()  # Calibrated
    image_to_extract = image_to_extract[47:-65, 158:-198].copy()  # Calibrated

    image_to_extract = image_to_extract[::-1, ::-1]  # Flip completely to assign origin
    _, image_to_analyse = cv.threshold(image_to_extract, 70, 255, cv.THRESH_BINARY_INV)

    kernel = np.ones((9, 9), np.uint8)
    image_to_analyse = cv.dilate(image_to_analyse, kernel, iterations=1)
    _, contours, hierarchy = cv.findContours(image_to_analyse, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    image_width, image_height = image_to_extract.shape

    draw_on_me = cv.cvtColor(image_to_extract.copy(), cv.COLOR_GRAY2RGB)
    outputInfo = []
    contour_num = 0
    for contour in contours:
        area = cv.contourArea(contour)
        area_percentage = area/image_to_extract.size*100
        if area_percentage < 0.05:  # Do not proceed if less than 0.05 percent of the image
            continue
        if area_percentage > 10:  # Do not proceed if more than 10 percent of the image
            continue
        _, _, _w, _h = cv.boundingRect(contour)
        if _w/_h > 10 or _h/_w > 10:
            continue
        M = cv.moments(contour)
        if M['m00'] == 0:
            break
        contour_X = M['m10'] / M['m00']
        contour_Y = M['m01'] / M['m00']
        contour_centre = (int(contour_X), int(contour_Y))

        rect = cv.minAreaRect(contour)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        (rectangle_X, rectangle_Y), (rectangle_width, rectangle_height), rectangle_angle = rect
        rectangle_centre = (int(rectangle_X), int(rectangle_Y))

        # Compute angle between centroid of contour and bounding rectangle: if the pointer vector
        # is large enough the difference is large enough and we have a lego brick. The pointer angle
        # comes from the signed clockwise angle between the pointer and the normal [-1,0],
        # multiplied with -1 because both axes are flipped.
        d_X = rectangle_X - contour_X
        d_Y = rectangle_Y - contour_Y
        pointer_length = np.sqrt(d_X**2 + d_Y**2)
        pointer_angle = -np.arctan2(0*d_X + 1*d_Y, 1*d_X + 0*d_Y)*180/np.pi if pointer_length > 1.0 else 0

        # Get angle from the rotated rectangle, depending on the longest edge:
        rectangle_angle += 90  # In the (0, 90] range
        conditions = [rectangle_width < rectangle_height, pointer_angle < 0.0]
        extra_rotations = [90, -180]  # 90 degrees for the leading edge, 180 if upside down
        adjustments = [r for r, c in zip(extra_rotations, conditions) if c]
        rectangle_angle += sum(adjustments)

        # Save all information
        outputInfo.append(((image_width-contour_Y)/image_width*LIGHT_BOX_WIDTH, contour_X/image_height*LIGHT_BOX_LENGTH, rectangle_angle*np.pi/180.0))

        # Draw info on the image:
        draw_on_me = cv.polylines(draw_on_me, [box], True, (0, 255, 0), thickness=5)
        # draw_on_me = cv.putText(draw_on_me, str(np.round(rectangle_angle, 2)), contour_centre, cv.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 2, cv.LINE_AA)
        draw_on_me = cv.circle(draw_on_me, contour_centre, 5, (0, 0, 255), -1)
        contour_num += 1
    return draw_on_me, outputInfo


def markTextOnImage(image, message):
    if not isinstance(message, str):
        message = str(message)
    if not isinstance(image, np.ndarray):
        return
    im_shape = image.shape
    TEXT_THICKNESS = 0.75
    TEXT_SIZE = 4
    if len(im_shape) == 2:
        image = cv.putText(image, message, (5, int(40*TEXT_THICKNESS)), cv.FONT_HERSHEY_SIMPLEX, TEXT_THICKNESS, 255, TEXT_SIZE, cv.LINE_AA)
    else:
        image = cv.putText(image, message, (5, int(40*TEXT_THICKNESS)), cv.FONT_HERSHEY_SIMPLEX, TEXT_THICKNESS, (255, 255, 255), TEXT_SIZE, cv.LINE_AA)
    return image


def markTimeDateOnImage(image):
    if not isinstance(image, np.ndarray):
        return
    message = time.asctime()
    im_shape = image.shape
    TEXT_THICKNESS = 0.75
    TEXT_SIZE = 3
    # if len(im_shape) == 2:
    #     image = cv.putText(image, message, (5, int(40*TEXT_THICKNESS)), cv.FONT_HERSHEY_SIMPLEX, TEXT_THICKNESS, 255, TEXT_SIZE, cv.LINE_AA)
    #     image = cv.putText(image, message, (5, int(80*TEXT_THICKNESS)), cv.FONT_HERSHEY_SIMPLEX, TEXT_THICKNESS,   0, TEXT_SIZE, cv.LINE_AA)
    # else:
    #     image = cv.putText(image, message, (5, int(40*TEXT_THICKNESS)), cv.FONT_HERSHEY_SIMPLEX, TEXT_THICKNESS, (255, 255, 255), TEXT_SIZE, cv.LINE_AA)
    #     image = cv.putText(image, message, (5, int(80*TEXT_THICKNESS)), cv.FONT_HERSHEY_SIMPLEX, TEXT_THICKNESS, (  0,   0,   0), TEXT_SIZE, cv.LINE_AA)
    return image


def cropRectangle(image_original):
    S = 5.0
    image = cv.resize(image_original, None, fx=1/S, fy=1/S, interpolation=cv.INTER_AREA)
    image = image/image.max()
    h, w = image.shape
    image_to_analyse = cv.inRange(image, 0.25, 1.0)

    kernel = np.ones((3, 3), np.uint8)
    image_to_analyse = cv.morphologyEx(image_to_analyse, cv.MORPH_OPEN, kernel)
    image_to_analyse = cv.morphologyEx(image_to_analyse, cv.MORPH_CLOSE, kernel)

    _, contours, hierarchy = cv.findContours(image_to_analyse, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    c = max(contours, key=cv.contourArea)
    rect = cv.minAreaRect(c)
    (rectangle_X, rectangle_Y), (rectangle_width, rectangle_height), rectangle_angle = rect
    if rectangle_angle < -80:
        # The the rectangle is aligned to the other side
        rect = tuple(((rectangle_Y, rectangle_X), (rectangle_height, rectangle_width), rectangle_angle + 90))
    box = cv.boxPoints(rect)
    box = np.int0(box*S)

    image_original = cv.polylines(image_original, [box], True, 255, thickness=5)

    width = int(rect[1][0])
    height = int(rect[1][1])
    src_pts = box.astype("float32")
    dst_pts = np.array([[0, height - 1], [0, 0], [width - 1, 0], [width - 1, height - 1]], dtype="float32")
    M = cv.getPerspectiveTransform(src_pts, dst_pts)
    image = cv.warpPerspective(image_original, M, (width, height))

    return image


def saveImage(image_to_save, stop_event):
    if stop_event and stop_event.isSet():
        return
    if not isinstance(image_to_save, np.ndarray):
        return
    w, h = image_to_save.shape[0:2]
    if image_to_save is None or w == 0 or h == 0:
        print("Image does not contain information")
        return
    here = os.path.join(os.getcwd(), 'Images')
    if not os.path.exists(here):
        os.makedirs(here)
    date_time_string = time.asctime().replace(" ", "_").replace(":", "_")
    image_name = os.path.join(here, date_time_string + ".png")
    cv.imwrite(image_name, image_to_save)


def imageSharpness(image):
    if not isinstance(image, np.ndarray):
        return np.nan
    image_laplace = cv.Laplacian(image, cv.CV_8U, ksize=1)
    _, std = cv.meanStdDev(image_laplace)
    return std[0][0]*std[0][0]


def imageContrast(image):
    if not isinstance(image, np.ndarray):
        return np.nan
    h_tv = np.power((image[1:, :] - image[:-1, :]), 2).sum()
    w_tv = np.power((image[:, 1:] - image[:, :-1]), 2).sum()
    return np.sqrt(h_tv + w_tv).sum()

