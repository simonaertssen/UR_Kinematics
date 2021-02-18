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
        if area_percentage < 0.05:  # Do not proceed if less than 5 percent of the image
            continue
        if area_percentage > 0.5:  # Do not proceed if more than 50 percent of the image
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
        rectangle_X, rectangle_Y = rect[0]
        rectangle_centre = (int(rectangle_X), int(rectangle_Y))
        d_X = rectangle_X - contour_X
        d_Y = rectangle_Y - contour_Y

        end_point = (int(contour_X + d_X*30), int(contour_Y + d_Y*30))
        draw_on_me = cv.arrowedLine(draw_on_me, contour_centre, end_point, (255, 0, 0), 5)

        # Find midpoints of each of the sides:
        middle_X = [np.int0((box[i - 1, 0] + box[i, 0]) / 2) for i in range(4)]
        middle_Y = [np.int0((box[i - 1, 1] + box[i, 1]) / 2) for i in range(4)]
        # Find longest side:
        candidates = np.sqrt(np.array([(middle_X[i] - middle_X[j]) ** 2 + (middle_Y[i] - middle_Y[j]) ** 2 for i, j in zip([0, 1], [2, 3])]))
        longest_side = candidates.argmax()
        sign = np.sign(middle_X[longest_side + 2] - middle_Y[longest_side])
        # Find angle:
        pts = np.array([[middle_X[i], middle_Y[i]] for i in longest_side + [0, 2]])
        angle = sign * np.arccos(np.abs(pts[1] - pts[0]).dot(np.array([1, 0])) / np.linalg.norm(pts[0] - pts[1]))
        # Save all information
        outputInfo.append(((image_width-contour_Y)/image_width*LIGHT_BOX_WIDTH, contour_X/image_height*LIGHT_BOX_LENGTH, np.pi/2 - angle))

        # Draw info on the image:
        draw_on_me = cv.polylines(draw_on_me, [box], True, (0, 255, 0), thickness=5)
        # draw_on_me = cv.putText(draw_on_me, str(np.round(angle*180/np.pi, 2)), (X, Y), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv.LINE_AA)
        draw_on_me = cv.circle(draw_on_me, contour_centre, 5, (0, 0, 255), -1)
        # for i in range(4):
        #     draw_on_me = cv.circle(draw_on_me, (middle_X[i], middle_Y[i]), 5, (255, 0, 0), -1)
        contour_num += 1
    return draw_on_me, outputInfo


def findObjectsToPickUpTemplate(image_to_extract):
    # Light box dimensions:
    LIGHT_BOX_LENGTH = 0.252  # m
    LIGHT_BOX_WIDTH = 0.177  # m

    # Obtain template image for matching:
    path_to_template = os.path.join(os.getcwd(), "Library", "3", "Mask", "side.png")
    template = cv.imread(path_to_template, cv.IMREAD_GRAYSCALE)
    template_w, template_h = template.shape

    # Extract image in a brute way
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
        if area_percentage < 0.05:  # Do not proceed if less than 5 percent of the image
            continue
        if area_percentage > 0.5:  # Do not proceed if more than 50 percent of the image
            continue
        rect_x, rect_y, rect_w, rect_h = cv.boundingRect(contour)
        if rect_w/rect_h > 10 or rect_h/rect_w > 10:
            continue
        M = cv.moments(contour)
        if M['m00'] == 0:
            break
        X = int(M['m10'] / M['m00'])
        Y = int(M['m01'] / M['m00'])
        print(M['m10'] / M['m00'], M['m01'] / M['m00'])

        rect = cv.minAreaRect(contour)
        print(rect[0])
        box = cv.boxPoints(rect)
        box = np.int0(box)

        rectX, rectY = rect[0]
        rectX, rectY = int(rectX), int(rectY)

        rect_height = int(rect[1][1])

        # src_pts = box.astype("float32")
        # dst_pts = np.array([[0, height - 1], [0, 0], [width - 1, 0], [width - 1, height - 1]], dtype="float32")
        # M = cv.getPerspectiveTransform(src_pts, dst_pts)
        # print(np.arccos(M[0, 0]), np.arccos(M[1, 1]), np.arcsin(M[1, 0]), np.arcsin(-M[0, 1]))
        # candidate_image = cv.warpPerspective(image_to_extract, M, (width, height))
        # candidate_template = cv.resize(template, (candidate_image.shape[1], candidate_image.shape[0]), interpolation=cv.INTER_CUBIC)
        # print(candidate_image.shape, candidate_template.shape)
        # # c_w, c_h = candidate_image.shape[0:2]
        # # draw_on_me[0:c_w, 0:c_h, 0] = candidate_image
        # candidate = cv.resize(image_to_extract, (rect_h, rect_w), interpolation=cv.INTER_CUBIC)

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
        # Rotate and scale template to match the candidate
        # candidate_template = cv.resize(template, (rect_width, rect_width), interpolation=cv.INTER_CUBIC)
        # rows, cols = candidate_template.shape
        # M = cv.getRotationMatrix2D((cols / 2, rows / 2), -angle*180/3.14159, 1)
        # candidate_template = cv.warpAffine(candidate_template, M, (cols, rows))
        #
        # matching = cv.matchTemplate(image_to_analyse, candidate_template, cv.TM_SQDIFF_NORMED)
        # _, _, top_left, _ = cv.minMaxLoc(matching)
        # bottom_right = (top_left[0] + template_w, top_left[1] + template_h)
        # draw_on_me = cv.rectangle(draw_on_me, top_left, bottom_right, 255, 2)
        # try:
        #     M = cv.getRotationMatrix2D((X, Y), angle * 180 / 3.14159, 1)
        #     top_left = np.array(top_left).dot(M[0:2, 0:2])
        #     bottom_right = np.array(bottom_right).dot(M[0:2, 0:2])
        # except Exception as e:
        #     print(e)

        # try:
        #     draw_on_me = cv.circle(draw_on_me, (int(top_left[0]), int(top_left[1])), 5, (0, 0, 255), -1)
        #     draw_on_me = cv.circle(draw_on_me, (int(bottom_right[0]), int(bottom_right[1])), 5, (0, 0, 255), -1)
        #     # draw_on_me = cv.rectangle(draw_on_me, bottom_right, top_left, 255, 2)
        # except Exception as e:
        #     print(e)

        # Save all information
        outputInfo.append(((image_width-Y)/image_width*LIGHT_BOX_WIDTH, X/image_height*LIGHT_BOX_LENGTH, np.pi/2 - angle))

        # # Draw info on the image:
        draw_on_me = cv.polylines(draw_on_me, [box], True, (0, 255, 0), thickness=5)
        # # draw_on_me = cv.putText(draw_on_me, str(np.round(angle*180/np.pi, 2)), (X, Y), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv.LINE_AA)
        draw_on_me = cv.circle(draw_on_me, (X, Y), 5, (0, 0, 255), -1)
        draw_on_me = cv.circle(draw_on_me, (rectX, rectY), 5, (0, 0, 255), -1)
        draw_on_me = cv.drawContours(draw_on_me, [contour], 0, (255, 0, 0), 3)

        # for i in range(4):
        #     draw_on_me = cv.circle(draw_on_me, (midX[i], midY[i]), 5, (255, 0, 0), -1)
        contour_num += 1
    return draw_on_me, outputInfo


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

