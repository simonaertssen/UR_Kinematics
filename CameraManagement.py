import time
import cv2 as cv
import numpy as np

from queue import Queue
from pypylon import pylon, genicam


class ImageEventHandler(pylon.ImageEventHandler):
    imageQueue = Queue()

    def OnImageGrabbed(self, camera, grab_result):
        try:
            while not self.imageQueue.empty():
                self.imageQueue.get()
            if grab_result.GrabSucceeded():
                cameraContextValue = grab_result.GetCameraContext()
                with grab_result.GetArrayZeroCopy() as ZCArray:
                    grabbedImage = ZCArray
                self.imageQueue.put((cameraContextValue, grabbedImage.data))
        except genicam.GenericException as e:
            print("ImageEventHandler Exception: {}".format(e))
        finally:
            grab_result.Release()


class Camera:
    # This class will save all the information coming from the pypylon package, so that two instances of a CW can be created.
    # Cameras are initialised using their serial number (see the CW serial number).
    # Subclassing a pylon.InstantCamera is not supported, so this camera1 will be used as an attribute.
    # See https://github.com/basler/pypylon/issues/196
    def __init__(self, serial_number=None, grayscale=True):
        super(Camera, self).__init__()
        self.serialNumber = serial_number
        self.grayScale = grayscale
        # Parameters for continuous extraction of data:
        self.info = pylon.CDeviceInfo()
        self.camera = None
        self.imageEventHandler = ImageEventHandler()
        self.grabbedImage = None
        self.setCamera()
        self.registerGrabbingStrategy()

        # Open camera briefly to avoid errors.
        self.camera.Open()
        self.pixelWidth = self.camera.Width.Value
        self.pixelHeight = self.camera.Height.Value
        self.camera.Close()

    def formatImage(self, image_to_format):
        if len(image_to_format.shape) == 3 and self.grayScale:
            image_to_format = cv.cvtColor(image_to_format, cv.COLOR_RGB2GRAY)
        return image_to_format.astype('uint8')

    def setCamera(self):
        try:
            if self.serialNumber is None:
                self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
            else:
                self.info.SetSerialNumber(self.serialNumber)
                self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(self.info))
            print("Camera {} is connected.".format(self.serialNumber))
        except genicam.GenericException as e:
            raise SystemExit('Camera {} could not be found: {}'.format(self.serialNumber, e))

    def Open(self):
        if not self.camera.IsOpen():
            self.camera.Open()

    def Close(self):
        if self.camera.IsGrabbing():
            self.camera.StopGrabbing()
        if self.camera.IsOpen():
            self.camera.Close()

    def Destroy(self):
        self.Close()
        self.camera.DetachDevice()
        self.camera.DestroyDevice()

    def registerGrabbingStrategy(self):
        self.camera.RegisterConfiguration(pylon.SoftwareTriggerConfiguration(), pylon.RegistrationMode_ReplaceAll, pylon.Cleanup_Delete)
        self.camera.RegisterImageEventHandler(self.imageEventHandler, pylon.RegistrationMode_Append, pylon.Cleanup_Delete)

    def grabImage(self):
        self.Open()
        grabbedImage = None
        if not self.camera.IsGrabbing():
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly, pylon.GrabLoop_ProvidedByInstantCamera)
        if self.camera.WaitForFrameTriggerReady(0, pylon.TimeoutHandling_Return):
            self.camera.ExecuteSoftwareTrigger()
        while grabbedImage is None:
            cameraContextValue, grabbedImage = self.imageEventHandler.imageQueue.get()
        return self.formatImage(grabbedImage)


class CameraArray:
    def __init__(self, serial_numbers=[], grayscale=True):
        super(CameraArray, self).__init__()
        self.serialNumbers = serial_numbers
        self.grayScale = grayscale
        self.maxCamerasToUse = 4
        self.inProcessOfClosing = False
        # Parameters for continuous extraction of data:
        self.numCameras = len(self.serialNumbers)
        if self.numCameras == 0:
            raise ValueError
        self.cameraArray = pylon.InstantCameraArray(min(self.numCameras, self.maxCamerasToUse))
        self.pixelWidths, self.pixelHeights = [], []
        self.imageEventHandler = ImageEventHandler()
        self.setCameras()

        # Open cameras briefly to avoid errors.
        for camera in self.cameraArray:
            camera.Open()
            self.pixelWidths.append(camera.Width.Value)
            self.pixelHeights.append(camera.Height.Value)
            camera.Close()

    def formatImage(self, image_to_format):
        if len(image_to_format.shape) == 3 and self.grayScale:
            image_to_format = cv.cvtColor(image_to_format, cv.COLOR_RGB2GRAY)
        return image_to_format.astype('uint8')

    def registerGrabbingStrategy(self, camera):
        camera.RegisterConfiguration(pylon.SoftwareTriggerConfiguration(), pylon.RegistrationMode_ReplaceAll, pylon.Cleanup_Delete)
        camera.RegisterImageEventHandler(self.imageEventHandler, pylon.RegistrationMode_ReplaceAll, pylon.Cleanup_Delete)

    def setCameras(self):
        try:
            for serialNumber, camera in zip(self.serialNumbers, self.cameraArray):
                info = pylon.CDeviceInfo()
                info.SetSerialNumber(serialNumber)
                device = pylon.TlFactory.GetInstance().CreateDevice(info)
                camera.Attach(device)
                self.registerGrabbingStrategy(camera)
            print("Cameras", self.serialNumbers, "are connected.")
        except genicam.GenericException as e:
            raise SystemExit('Cameras {} could not be found: {}'.format(self.serialNumbers, e))

    def Open(self):
        self.cameraArray.Open()

    def Close(self):
        if self.cameraArray.IsGrabbing():
            self.cameraArray.StopGrabbing()
        if self.cameraArray.IsOpen():
            self.cameraArray.Close()

    def Destroy(self):
        self.Close()
        self.cameraArray.DestroyDevice()

    def imageFromQueue(self):
        return self.imageEventHandler.imageQueue.get

    def grabImage(self, grabbedImages):
        alreadyReplaced = 0
        if not self.cameraArray.IsGrabbing():
            self.cameraArray.StartGrabbing(pylon.GrabStrategy_LatestImageOnly, pylon.GrabLoop_ProvidedByInstantCamera)

        try:
            for cam in self.cameraArray:
                if cam.WaitForFrameTriggerReady(10, pylon.TimeoutHandling_Return):
                    cam.ExecuteSoftwareTrigger()
            for cam in self.cameraArray:
                context, grabbedImage = self.imageEventHandler.imageQueue.get()
                grabbedImages[context] = self.formatImage(grabbedImage)
            return 0
        except genicam.RuntimeException as e:
            # raise SystemExit('Runtime Exception: {}'.format(self.serialNumbers, e))
            return -1


def runSingleCamera():
    # This works well
    camera = Camera("21565643")
    testWindow = 'window1'
    cv.namedWindow(testWindow)
    cv.moveWindow(testWindow, 20, 20)

    start = time.time()

    while True:
        image = camera.grabImage()
        w, h = image.shape
        image = cv.resize(image, (int(h/4), int(w/4)))

        cv.imshow(testWindow, image)
        if cv.waitKey(1) & 0xFF == 27:  # Exit upon escape key
            break
        now = time.time()
        print("FPS =", 1 / (time.time() - start))
        start = now

    camera.Destroy()
    cv.destroyAllWindows()


def runDuoCamera():
    # This does not work well
    camera1 = Camera("21565643")
    camera2 = Camera("22290932")
    testWindow1 = 'window1'
    testWindow2 = 'window2'
    cv.namedWindow(testWindow1)
    cv.moveWindow(testWindow1, 20, 20)
    cv.namedWindow(testWindow2)
    cv.moveWindow(testWindow2, 500, 20)

    start = time.time()

    while True:
        # image1 = camera1.grabImage()
        # w, h = image1.shape
        # image1 = cv.resize(image1, (int(h/8), int(w/8)))
        # cv.imshow(testWindow1, image1)

        image2 = camera2.grabImage()
        w, h = image2.shape
        image2 = cv.resize(image2, (int(h / 8), int(w / 8)))
        cv.imshow(testWindow2, image2)
        if cv.waitKey(1) & 0xFF == 27:  # Exit upon escape key
            break
        now = time.time()
        print("FPS =", 1 / (time.time() - start))
        start = now

    camera1.Destroy()
    camera2.Destroy()
    cv.destroyAllWindows()


def hconcat_resize_min(im_list, interpolation=cv.INTER_CUBIC):
    im_list = [cv.resize(image, (int(image.shape[1] / 4), int(image.shape[0] / 4))) for image in im_list]
    h_min = min(im.shape[0] for im in im_list)
    im_list_resize = [cv.resize(im, (int(im.shape[1] * h_min / im.shape[0]), h_min), interpolation=interpolation)
                      for im in im_list]
    return cv.hconcat(im_list_resize)


def runCameraArray():
    cameras = CameraArray(["21565643", "22290932"])
    testWindow = 'window1'
    cv.namedWindow(testWindow)
    cv.moveWindow(testWindow, 20, 20)

    start = time.time()

    images = [None, None]
    while True:
        if cameras.grabImage(images) != 0:
            continue

        cv.imshow(testWindow, hconcat_resize_min(images))
        if cv.waitKey(1) & 0xFF == 27:  # Exit upon escape key
            break
        now = time.time()
        print("FPS =", 1 / (time.time() - start))
        start = now

    cameras.Destroy()
    cv.destroyAllWindows()


if __name__ == '__main__':
    runSingleCamera()

