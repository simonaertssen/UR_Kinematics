import time
from queue import Queue

import cv2 as cv
import numpy as np
from pypylon import pylon, genicam
from ImageModule import findObjectsToPickUp, markTimeDateOnImage

import tracemalloc


class ImageEventHandler(pylon.ImageEventHandler):
    r"""
    Class used to represent pylon c class that catches images from the cameras.

    Attributes:
    -------
    imageQueue : Queue
        The queue in which taken images are stored for threadsafe access.
    """

    imageQueue = Queue()

    def OnImageGrabbed(self, camera, grab_result):
        r"""
        Safely acquire an image from the pylon c buffer. The GetArrayZeroCopy()
        method was shown to be superior in terms of speed.
        """
        while not self.imageQueue.empty():
            self.imageQueue.get()
        try:
            if grab_result.GrabSucceeded():
                cameraContextValue = grab_result.GetCameraContext()
                with grab_result.GetArrayZeroCopy() as ZCArray:
                    self.imageQueue.put((cameraContextValue, ZCArray.data))
        except genicam.GenericException as e:
            print("ImageEventHandler Exception: {}".format(e))
        finally:
            grab_result.Release()


class Camera:
    r"""
    Class used to represent a pylon camera, holding all the required information
    for robust performance. Cameras are initialised using their serial number
    (see the CW serial number). Subclassing a pylon.InstantCamera is not
    supported, so this camera1 will be used as an attribute.
    See https://github.com/basler/pypylon/issues/196

    Attributes:
    -------
    serialNumber : int
        The serial number of the camera.
    grayScale : bool
        Turn the acquired image in grayscale format or not.
    info : pylon.CDeviceInfo
        The camera information used by pylon.
    camera : pylon.InstantCamera
        The camera is listed as an attribute as subclassing it is not supported.
    imageEventHandler: ImageEventHandler
        The class that acquires an image from the camera safely.
    Connected : bool
        The camera is connected or not.
    """

    def __init__(self, serial_number=None, grayscale=True):
        super(Camera, self).__init__()
        self.serialNumber = serial_number
        self.grayScale = grayscale
        # Parameters for continuous extraction of data:
        self.info = pylon.CDeviceInfo()
        self.camera = None
        self.imageEventHandler = ImageEventHandler()
        self.Connected = False
        self.setCamera()
        self.registerGrabbingStrategy()

        # Open camera briefly to avoid errors while registering properties.
        self.camera.Open()
        self.camera.TriggerMode.SetValue('Off')
        self.pixelWidth = self.camera.Width.Value
        self.pixelHeight = self.camera.Height.Value
        self.camera.Close()

    def getShape(self):
        r"""
        Acquire the dimensions of images taken by the current camera.
        """
        return self.pixelHeight, self.pixelWidth

    def setCamera(self):
        r"""
        Find the camera with the pylon internals using the serial number. If that
        is not given, use the first camera that pylon finds. An error is raised
        if no cameras could be found.
        """
        try:
            if self.serialNumber is None:
                self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
            else:
                self.info.SetSerialNumber(str(self.serialNumber))
                self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(self.info))
            # Close all connections if they exist:
            self.camera.Close()
            self.Connected = True
            print("Camera {} is connected.".format(self.serialNumber))
        except genicam.GenericException as e:
            self.shutdownSafely()
            raise ConnectionError('Camera {} could not be found: {}'.format(self.serialNumber, e))

    def isConnected(self):
        return self.Connected

    def open(self):
        if not self.camera.IsOpen():
            self.camera.Open()

    def close(self):
        if self.camera.IsGrabbing():
            self.camera.StopGrabbing()
        if self.camera.IsOpen():
            self.camera.Close()

    def shutdownSafely(self):
        self.Connected = False
        if self.camera:
            self.close()
            self.camera.DetachDevice()
            self.camera.DestroyDevice()

    def registerGrabbingStrategy(self):
        r"""
        Load a strategy for the camera to follow on hw to acquire images. The
        fastest strategy was chosen here.
        """
        self.camera.RegisterConfiguration(pylon.SoftwareTriggerConfiguration(), pylon.RegistrationMode_ReplaceAll, pylon.Cleanup_Delete)
        self.camera.RegisterImageEventHandler(self.imageEventHandler, pylon.RegistrationMode_Append, pylon.Cleanup_Delete)

    def toGrayScale(self, image_to_gray):
        if len(image_to_gray.shape) == 3 and self.grayScale:
            image_to_gray = cv.cvtColor(image_to_gray, cv.COLOR_RGB2GRAY)
        return image_to_gray

    def manipulateImage(self, image_to_manipulate):
        r"""
        Wrapper which contains all the functions we wish to apply on images
        coming from this camera.
        """
        image_to_manipulate = self.toGrayScale(image_to_manipulate)
        return image_to_manipulate

    def grabImage(self):
        if not self.Connected:
            return None
        self.open()
        grabbedImage, info, cam_num = None, None, None
        if not self.camera.IsGrabbing():
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly, pylon.GrabLoop_ProvidedByInstantCamera)
        try:
            if self.camera.WaitForFrameTriggerReady(0, pylon.TimeoutHandling_Return):
                self.camera.ExecuteSoftwareTrigger()
            cam_num, grabbedImage = self.imageEventHandler.imageQueue.get()
            grabbedImage, info = self.manipulateImage(np.asarray(grabbedImage))
        except genicam.RuntimeException as e:
            print('genicam Runtime Exception: {}'.format(e))
        except RuntimeError as e:
            print('Runtime Exception: {}'.format(e))
        finally:
            return grabbedImage, info, cam_num


class TopCamera(Camera):
    r"""
    Class used to represent the camera looking down on the light box.
    """
    def __init__(self, serial_number=22290932, grayscale=True):
        super(TopCamera, self).__init__(serial_number, grayscale)

    def manipulateImage(self, image_to_manipulate):
        info = None
        # Overload to deal with images in the right way
        image_to_manipulate = self.toGrayScale(image_to_manipulate)
        image_to_manipulate, info = findObjectsToPickUp(image_to_manipulate)
        image_to_manipulate = markTimeDateOnImage(image_to_manipulate)
        return image_to_manipulate, info


class DetailCamera(Camera):
    r"""
    Class used to represent the camera looking down on the items presented by
    the robot arm.
    """
    def __init__(self, serial_number=21565643, grayscale=True):
        super(DetailCamera, self).__init__(serial_number, grayscale)

    def manipulateImage(self, image_to_manipulate):
        info = None
        # Overload to deal with images in the right way
        image_to_manipulate = self.toGrayScale(image_to_manipulate)
        image_to_manipulate = markTimeDateOnImage(image_to_manipulate)
        return image_to_manipulate, info


def runSingleCamera(camera):
    # This works well
    testWindow = 'window1'
    cv.namedWindow(testWindow)
    cv.moveWindow(testWindow, 20, 20)

    h, w = camera.getShape()

    start = time.time()
    while True:
        image, info, cam_num = camera.grabImage()
        if image is None:
            continue

        cv.imshow(testWindow, cv.resize(image, (int(w/4), int(h/4))))
        if cv.waitKey(1) & 0xFF == 27:  # Exit upon escape key
            break
        now = time.time()
        print("FPS =", 1 / (time.time() - start))
        start = now
    camera.shutdownSafely()
    cv.destroyAllWindows()


def debugCameraForMemoryLeaks(camera):
    # This works well
    testWindow = 'window1'
    cv.namedWindow(testWindow)
    cv.moveWindow(testWindow, 20, 20)

    h, w = camera.getShape()
    tracemalloc.start(10)

    counter = 0
    while True:
        counter += 1
        image, info, cam_num = camera.grabImage()
        if image is None:
            continue

        cv.imshow(testWindow, cv.resize(image, (int(w / 4), int(h / 4))))
        if cv.waitKey(1) & 0xFF == 27:  # Exit upon escape key
            break
        if counter >= 100:
            counter = 0
            snapshot = tracemalloc.take_snapshot()
            for i, stat in enumerate(snapshot.statistics('filename')[:5], 1):
                print(str(stat))
    camera.shutdownSafely()
    cv.destroyAllWindows()


def runCamerasAlternate(cameraOn, cameraOff):
    testWindow = 'window1'
    cv.namedWindow(testWindow)
    cv.moveWindow(testWindow, 20, 20)

    h, w = cameraOn.getShape()

    start = time.time()
    while True:
        image, info, cam_num = cameraOn.grabImage()
        if image is None:
            continue

        cv.imshow(testWindow, cv.resize(image, (int(w / 4), int(h / 4))))
        if cv.waitKey(1) & 0xFF == 27:  # Exit upon escape key
            break

        if cv.waitKey(5) & 0xFF == ord('q'):  # Switch cameras with q
            cameraOn.close()
            cameraOn, cameraOff = cameraOff, cameraOn
            cameraOn.open()
            h, w = cameraOn.getShape()

        now = time.time()
        print("FPS =", 1 / (time.time() - start))
        start = now
    cameraOn.shutdownSafely()
    cameraOn.shutdownSafely()
    cv.destroyAllWindows()


if __name__ == '__main__':
    runCamerasAlternate(TopCamera(), DetailCamera())
