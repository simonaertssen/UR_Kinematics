import time
import numpy as np
import cv2 as cv
import os


def findObjectsToPickUp(image_to_extract):
    # Lightbox dimensions:
    LIGHTBOX_LENGTH = 0.250  # m
    LIGHTBOX_WIDTH = 0.176  # m

    # Extract image in a brute way
    image_to_extract = image_to_extract[50:-53, 154:-198].copy()  # Calibrated
    image_to_extract = image_to_extract[::-1, ::-1]  # Flip completely to assign origin
    _, image_to_analyse = cv.threshold(image_to_extract, 70, 255, cv.THRESH_BINARY_INV)

    kernel = np.ones((9, 9), np.uint8)
    image_to_analyse = cv.dilate(image_to_analyse, kernel, iterations=1)
    _, contours, hierarchy = cv.findContours(image_to_analyse, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    w, h = image_to_extract.shape

    drawonme = cv.cvtColor(image_to_extract.copy(), cv.COLOR_GRAY2RGB)
    outputInfo = []
    contournum = 0
    for contour in contours:
        area = cv.contourArea(contour)
        if area/image_to_extract.size*100 < 0.1:  # Do not proceed if less than 0.1 percent of the image
            continue
        if area / image_to_extract.size * 100 > 10:  # Do not proceed if more than 10 percent of the image
            continue
        x, y, w, h = cv.boundingRect(contour)
        if w/h > 10 or h/w > 10:
            continue
        M = cv.moments(contour)
        if M['m00'] == 0:
            break
        X = int(M['m10'] / M['m00'])
        Y = int(M['m01'] / M['m00'])

        rect = cv.minAreaRect(contour)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        # Find midpoints:
        midX = [np.int0((box[i - 1, 0] + box[i, 0]) / 2) for i in range(4)]
        midY = [np.int0((box[i - 1, 1] + box[i, 1]) / 2) for i in range(4)]
        # Find longest side:
        candidates = np.sqrt(np.array([(midX[i] - midX[j]) ** 2 + (midY[i] - midY[j]) ** 2 for i, j in zip([0, 1], [2, 3])]))
        longest_side = candidates.argmax()
        sign = np.sign(midX[longest_side + 2] - midX[longest_side])
        # Find angle:
        pts = np.array([[midX[i], midY[i]] for i in longest_side + [0, 2]])
        angle = sign * np.arccos(np.abs(pts[1] - pts[0]).dot(np.array([1, 0])) / np.linalg.norm(pts[0] - pts[1]))
        outputInfo.append(((w-Y)/w, X/h, np.pi/2 - angle))
        # Draw info on the image:
        drawonme = cv.polylines(drawonme, [box], True, (0, 255, 0), thickness=5)
        # drawonme = cv.putText(drawonme, str(np.round(angle*180/np.pi, 2)), (X, Y), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv.LINE_AA)
        drawonme = cv.circle(drawonme, (X, Y), 5, (0, 0, 255), -1)
        for i in range(4):
            drawonme = cv.circle(drawonme, (midX[i], midY[i]), 5, (255, 0, 0), -1)
        contournum += 1
    return drawonme, outputInfo


def markTimeDateOnImage(image):
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


def saveImage(image_to_save):
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

