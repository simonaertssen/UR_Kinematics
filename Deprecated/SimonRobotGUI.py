import sys
import time
import numpy as np
import cv2 as cv
from PyQt5 import QtCore, QtWidgets
from CameraManagement import Camera
from Deprecated.RobotSocket import Robot
from vispy import scene


class updateViewThread(QtCore.QThread):
    newImageIsAvailable = QtCore.pyqtSignal(tuple)

    def __init__(self, active_camera, image_dimensions, call_me_to_get_images):
        super(updateViewThread, self).__init__()
        self.activeCamera = active_camera
        self.imageDimensions = image_dimensions
        self.getImages = call_me_to_get_images
        self.active = True
        self.starttime = time.time()

    def switchActiveState(self):
        self.active = not self.active
        print("updateViewThread is active?", self.active)

    def renewGetImageFunction(self, camera_index_and_new_call_me_to_get_images):
        self.activeCamera, call_me_to_get_images = camera_index_and_new_call_me_to_get_images
        self.getImages = call_me_to_get_images

    def run(self):
        while True:
            if not self.active:
                time.sleep(0.01)
                continue
            try:
                image = self.getImages()
                if image is not None:
                    w, h = image.shape
                    image = cv.resize(image, (int(h/3), int(w/3)), interpolation=cv.INTER_CUBIC)
                    self.newImageIsAvailable.emit((self.activeCamera, image))

                    now = time.time()
                    print("FPS =", 1 / (now - self.starttime))
                    self.starttime = now
            except Exception as e:
                print(e)


class View(scene.SceneCanvas):
    def __init__(self):
        scene.SceneCanvas.__init__(self)
        self.unfreeze()
        self.view = self.central_widget.add_view()
        self.view.camera = scene.PanZoomCamera(aspect=1, interactive=False)
        self.view.camera.flip = (0, 1, 0)
        self.Image = scene.visuals.Image(parent=self.view.scene, cmap='grays', interpolation='bilinear')
        self.currentCamera = 0
        self.freeze()

    def display(self, current_camera, image):
        self.Image.set_data(image)
        if self.currentCamera != current_camera:
            self.currentCamera = current_camera
            self.view.camera.set_range()
        self.update()


class SurfaceInspectionWindow(QtWidgets.QMainWindow):
    pauseOrActivateViewThread = QtCore.pyqtSignal()
    switchActiveCameraSignal  = QtCore.pyqtSignal(tuple)

    def __init__(self):
        super(SurfaceInspectionWindow, self).__init__()
        self.setWindowTitle("Surface scratch inspection")
        self.setFocus()
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.width  = 800
        self.height = 600

        self.mutexLock = QtCore.QMutex()

        # Image-related attributes:
        self.cameraView = View()
        self.activeCamera = 1
        self.cameraView.currentCamera = (self.activeCamera + 1) % 2  # Change Viewer camera so that first image will set range
        self.cameraSerials = ["21565643", "22290932"]
        self.camera = Camera(self.cameraSerials[self.activeCamera])
        self.analyseThisCamera = 1
        self.viewThread = updateViewThread(self.activeCamera, (0, 0), self.camera.grabImage)
        # test_img = cv.imread('jli_logo_aflang.png')
        # self.cameraView.display(test_img)
        # Connect signals and slots
        self.viewThread.newImageIsAvailable.connect(self.processImagesFromThread)
        self.pauseOrActivateViewThread.connect(self.viewThread.switchActiveState)
        self.switchActiveCameraSignal.connect(self.viewThread.renewGetImageFunction)

        self.setCentralWidget(self.cameraView.native)
        self.centralWidget().setFocusPolicy(QtCore.Qt.NoFocus)
        self.setGeometry(50, 50, self.width, self.height)

        # Start the robot:
        self.robot = Robot()
        self.objectsToPickUp = []

        # Start threads
        print(type(self.viewThread))
        self.viewThread.start()
        self.robot.initialise()
        self.show()

    def switchActiveCamera(self):
        self.pauseOrActivateViewThread.emit()
        self.camera.Destroy()
        self.activeCamera = (self.activeCamera + 1) % len(self.cameraSerials)
        self.camera = Camera(self.cameraSerials[self.activeCamera])
        self.camera.Open()
        self.switchActiveCameraSignal.emit((self.activeCamera, self.camera.grabImage))
        self.pauseOrActivateViewThread.emit()

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Escape:
            self.closeEvent(event)
        if event.key() == QtCore.Qt.Key_Space:
            self.switchActiveCamera()
        if event.key() == QtCore.Qt.Key_Enter:
            print("grabbing")
            self.robot.pickUpObject(self.objectsToPickUp[0])

    def closeEvent(self, event):
        if self.robot is not None:
            self.robot.shutdownRobot()
        self.camera.Destroy()
        self.close()

    def processImagesFromThread(self, camera_and_image):
        currentCamera, image = camera_and_image
        if currentCamera  == self.analyseThisCamera:
            image = self.findAllObjects(image)
        self.cameraView.display(currentCamera, image)

    def findAllObjects(self, imageOriginal):
        image = imageOriginal.copy()
        image = image[30:-60, 70:-120]
        # Otsu binarization to get contours:
        blur = cv.GaussianBlur(image, (5, 5), 0)
        _, image = cv.threshold(blur, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        bricks = (image < 20).astype(np.uint8)
        _, contours, hierarchy = cv.findContours(bricks, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            M = cv.moments(contour)
            if M['m00'] <= 0 or M['m00'] >= 5000:
                break
            X = int(M['m10'] / M['m00'])
            Y = int(M['m01'] / M['m00'])

            image = cv.circle(image, (X, Y), 4, 255, -1)

            rect = cv.minAreaRect(contour)
            (x, y), (w, h), angle = rect
            angle = angle * 180 / np.pi % 360
            print("angle", 90 - angle)

            box = cv.boxPoints(rect)
            box = np.int0(box)
            midX = [np.int0((box[i - 1, 0] + box[i, 0]) / 2) for i in range(4)]
            midY = [np.int0((box[i - 1, 1] + box[i, 1]) / 2) for i in range(4)]
            distances = [np.sqrt((x - X) ** 2 + (y - Y) ** 2) for x, y in zip(midX, midY)]
            d = max(distances)
            idx = [distance / d < 0.9 for distance in distances]
            print(distances)
            pts = np.array([[x, y] for i, (x, y) in enumerate(zip(midX, midY)) if idx[i] is True])
            print(pts)
            for pt in pts:
                image = cv.circle(image, (pt[0], pt[1]), 10, 255, -1)
            try:
                angle = np.arccos(np.abs(pts[1] - pts[0]).dot(np.array([1, 0])) / np.linalg.norm(pts[1] - pts[0]))
            except Exception as e:
                print(e)
            print("angle", int(angle * 180 / np.pi))

            entry = (X, Y, angle)
            add = True
            for object in self.objectsToPickUp:
                if object == entry:
                    add = False
            if add:
                self.objectsToPickUp.append(entry)
            print(len(self.objectsToPickUp))
        return image


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    w = SurfaceInspectionWindow()
    sys.exit(app.exec_())




