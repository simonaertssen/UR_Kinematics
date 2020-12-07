import numpy as np
import cv2 as cv


def findObjectsToPickUp(image_to_extract):
    image_to_extract = image_to_extract[60:-90, 170:-210].copy()
    image_to_extract = image_to_extract[::-1, ::-1]  # Flip completely to assign origin
    _, image_to_analyse = cv.threshold(image_to_extract, 70, 255, cv.THRESH_BINARY_INV)

    kernel = np.ones((9, 9), np.uint8)
    image_to_analyse = cv.dilate(image_to_analyse, kernel, iterations=1)
    _, contours, hierarchy = cv.findContours(image_to_analyse, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    drawonme = cv.cvtColor(image_to_extract.copy(), cv.COLOR_GRAY2RGB)
    output = list()
    for contour in contours:
        area = cv.contourArea(contour)
        if area/image_to_extract.size*100 < 0.1:  # Do not proceed if less than 0.1 percent of the image
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
        output.append((X, Y, angle * 180 / np.pi))
        # Draw info on the image:
        drawonme = cv.polylines(drawonme, [box], True, (0, 255, 0), thickness=5)
        drawonme = cv.circle(drawonme, (X, Y), 5, (255, 0, 0), -1)
        for i in range(4):
            drawonme = cv.circle(drawonme, (midX[i], midY[i]), 5, (255, 0, 0), -1)

    output.insert(0, drawonme)
    return output
