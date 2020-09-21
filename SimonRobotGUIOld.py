import sys
import time
import numpy as np
import cv2 as cv
from PyQt5 import QtGui, QtCore, QtWidgets
from CameraManagement import CameraArray
from RobotSocket import Robot


class updateViewThread(QtCore.QThread):
    newImageIsAvailable = QtCore.pyqtSignal(tuple)

    def __init__(self, image_dimensions, camera_to_rotate, call_function_to_get_images):
        super(updateViewThread, self).__init__()
        self.imageDimensions = image_dimensions
        self.rotateCamera = camera_to_rotate
        self.getImages = call_function_to_get_images
        self.startTime = time.time()

    def run(self):
        while True:
            image, cameraIndex = self.getImages()
            if image is not None:
                shape = self.imageDimensions[cameraIndex]
                if cameraIndex == self.rotateCamera:
                    image = cv.rotate(image, cv.ROTATE_90_COUNTERCLOCKWISE)
                image = cv.resize(image, shape, interpolation=cv.INTER_CUBIC)
                self.newImageIsAvailable.emit((image, cameraIndex))
                if cameraIndex == 0:
                    now = time.time()
                    # print("FPS in thread=", 1 / (now - self.startTime))
                    self.startTime = now


class SurfaceInspectionWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(SurfaceInspectionWindow, self).__init__()
        self.setWindowTitle("Surface scratch inspection")
        self.width  = 800
        self.height = 600
        self.splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        # Image-related attributes:
        self.cameraSerials = ["21565643", "22290932"]
        self.cameras = CameraArray(self.cameraSerials)
        self.rotateThisCameraIndex = 1
        self.analyseThisCameraIndex = 1

        # Set labels/windows programmatically
        self.views = []
        self.viewSizes = []
        for i, (w, h) in enumerate(zip(self.cameras.pixelWidths, self.cameras.pixelHeights)):
            view = QtWidgets.QLabel(self)
            if i == self.rotateThisCameraIndex:
                shape = (self.height, int(h / w * self.height))
            else:
                shape = (int(w / h * self.height), self.height)
            self.updateQImage(shape, view, np.zeros(shape, dtype=np.uint8))
            self.views.append(view)
            self.viewSizes.append(shape)
            self.splitter.addWidget(view)
        # Flip image dimensions for 90 degree rotation
        self.viewSizes[self.rotateThisCameraIndex] = self.viewSizes[self.rotateThisCameraIndex][::-1]
        self.viewThread = updateViewThread(self.viewSizes, self.rotateThisCameraIndex, self.cameras.grabImage)
        self.viewThread.newImageIsAvailable.connect(self.processImagesFromThread)

        self.setCentralWidget(self.splitter)
        self.setGeometry(50, 50, self.width, self.height)

        # Start the robot:
        self.robot = Robot()
        self.objectsToPickUp = 0

        # Start threads
        self.viewThread.start()
        # self.robot.initialise()
        self.show()

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Escape:
            self.closeEvent(event)

    def closeEvent(self, event):
        # close = QtWidgets.QMessageBox()
        # close.setText("Are you sure you wish to quit the application?")
        # close.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.Cancel)
        # close = close.exec()
        # if close == QtWidgets.QMessageBox.Yes:
        #     event.accept()
        # else:
        #     event.ignore()
        self.robot.shutdownRobot()
        self.cameras.Destroy()
        self.close()

    def processImagesFromThread(self, image_and_index):
        image, camera_index = image_and_index
        if camera_index == self.analyseThisCameraIndex:
            image, self.objectsToPickUp = self.findAllObjects(image)
        self.updateQImage(self.viewSizes[camera_index], self.views[camera_index], image)

    def findAllObjects(self, image):
        print("image.shape", image.shape, "image.strides", image.strides, type(image[0, 0]))
        inverted = 255 - image
        _, contours, hierarchy = cv.findContours(inverted, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        drawme = np.zeros(image.shape, dtype=np.uint8)

        print(image.shape)
        print(drawme.shape)
        cv.drawContours(drawme, contours, -1, (255, 255, 255), thickness=cv.FILLED)

        print("drawme.shape", drawme.shape, "drawme.strides", drawme.strides, type(drawme[0, 0]))

        print(image.mean())
        imageMask = 255*(image < 200)
        print(imageMask.shape)
        return drawme, 2

    def updateQImage(self, shape, view, image):
        w, h = shape
        # print("image.shape", image.shape, "image.strides", image.strides, type(image[0, 0]))
        QI = QtGui.QImage(image.data, w, h, image.strides[0], QtGui.QImage.Format_Indexed8) #.rgbSwapped()
        view.setPixmap(QtGui.QPixmap.fromImage(QI)) # .scaled(w, h, QtCore.Qt.KeepAspectRatio))


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    w = SurfaceInspectionWindow()
    sys.exit(app.exec_())




