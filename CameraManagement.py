import sys
from pypylon import pylon
from pypylon import genicam
import cv2 as cv
import time
import numpy as np

# Inherit camera1 attributes and methods programmatically?
# cam.Open()
# for attributeName in dir(cam):
#     try:
#         attribute = getattr(cam, attributeName)
#         if callable(attribute):
#             setattr(self, attributeName, attribute)
#     except genicam.GenericException as e:
#         print("OS error: {0}".format(e))
# cam.Close()


class ImageEventHandler(pylon.ImageEventHandler):
    def __init__(self, callback):
        super(ImageEventHandler).__init__()
        self.returnImageCallback = callback
        self.cameraContextValue = None
        self.grabbedImage = None
        self.start = time.time()

    def OnImageGrabbed(self, camera, grabResult):
        try:
            if grabResult.GrabSucceeded():
                with grabResult.GetArrayZeroCopy() as ZCArray:
                    self.grabbedImage = ZCArray
                    # self.cameraContextValue = grabResult.GetCameraContext()

                # print("FPS =", 1 / (time.time() - self.start))
                self.start = time.time()
                self.returnImageCallback(self.grabbedImage)

        except genicam.GenericException as e:
            print("An exception occurred. {}".format(e))


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
        self.imageEventHandler = ImageEventHandler(self.getLatestImage)
        self.grabbedImage = None
        self.setCamera()

        # Open camera1 briefly to avoid errors.
        self.camera.Open()
        self.pixelWidth = self.camera.Width.Value
        self.pixelHeight = self.camera.Height.Value
        # self.fps = max(100, self.camera1.AcquisitionFrameRate.Value)
        self.camera.Close()

    def getLatestImage(self, image=None):
        self.grabbedImage = image

    def formatImage(self, imageToFormat):
        if len(imageToFormat.shape) == 3 and self.grayScale:
            imageToFormat = cv.cvtColor(imageToFormat, cv.COLOR_RGB2GRAY)
        return imageToFormat

    def setCamera(self):
        if self.serialNumber is None:
            self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        else:
            self.info.SetSerialNumber(self.serialNumber)
            self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(self.info))
        print("Camera {} is connected.".format(self.serialNumber))

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
        self.camera.RegisterConfiguration(pylon.SoftwareTriggerConfiguration(), pylon.RegistrationMode_Append, pylon.Cleanup_Delete)
        self.camera.RegisterImageEventHandler(self.imageEventHandler, pylon.RegistrationMode_Append, pylon.Cleanup_Delete)

    def grabImage(self):
        self.Open()
        grabbedImage = None
        if not self.camera.IsGrabbing():
            self.registerGrabbingStrategy()
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly, pylon.GrabLoop_ProvidedByInstantCamera)
            # self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly, pylon.GrabLoop_ProvidedByInstantCamera)
        if self.camera.WaitForFrameTriggerReady(100, pylon.TimeoutHandling_Return):
            self.camera.ExecuteSoftwareTrigger()
        while grabbedImage is None:
            grabbedImage = self.imageEventHandler.grabbedImage
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
        self.imageEventHandlers = []
        self.setCameras()

        # Open cameras briefly to avoid errors.
        for camera in self.cameraArray:
            # print(dir(camera))
            camera.Open()
            self.pixelWidths.append(camera.Width.Value)
            self.pixelHeights.append(camera.Height.Value)
            # self.fps = max(100, self.camera1.AcquisitionFrameRate.Value)
            camera.AcquisitionFrameRateAbs.SetValue(100)
            camera.Close()
        self.grabbedImages = [np.zeros((w, h)).astype('uint8') for w, h in zip(self.pixelWidths, self.pixelHeights)]

    def getGrabbedImage(self, image):
        self.grabbedImage = image

    def formatImage(self, imageToFormat):
        if len(imageToFormat.shape) == 3 and self.grayScale:
            imageToFormat = cv.cvtColor(imageToFormat, cv.COLOR_RGB2GRAY)
        return imageToFormat.astype('uint8')

    def setCameras(self):
        # self.imageEventHandler = ImageEventHandler(self.getGrabbedImage)
        for serialNumber, camera in zip(self.serialNumbers, self.cameraArray):
            info = pylon.CDeviceInfo()
            info.SetSerialNumber(serialNumber)
            device = pylon.TlFactory.GetInstance().CreateDevice(info)
            camera.Attach(device)

            # handler = ImageEventHandler(self.getGrabbedImage)
            # self.imageEventHandlers.append(handler)
            # camera.RegisterConfiguration(pylon.SoftwareTriggerConfiguration(), pylon.RegistrationMode_ReplaceAll, pylon.Cleanup_Delete)
            # camera.RegisterImageEventHandler(self.imageEventHandler, pylon.RegistrationMode_Append, pylon.Cleanup_Delete)
        print("Cameras", self.serialNumbers, "are connected.")

    def Open(self):
        self.cameraArray.Open()
        # for camera in self.cameraArray:
        #     if not camera.IsOpen():
        #         camera.Open()

    def Close(self):
        self.inProcessOfClosing = True
        if self.cameraArray.IsGrabbing():
            self.cameraArray.StopGrabbing()
        if self.cameraArray.IsOpen():
            self.cameraArray.Close()

        # for camera in self.cameraArray:
        #     if camera.IsGrabbing():
        #         camera.StopGrabbing()
        #     if camera.IsOpen():
        #         camera.Close()

    def Destroy(self):
        self.Close()
        self.cameraArray.DestroyDevice()

    # def registerGrabbingStrategy(self, camera):
    #     camera.RegisterConfiguration(pylon.SoftwareTriggerConfiguration(), pylon.RegistrationMode_ReplaceAll, pylon.Cleanup_Delete)
    #     camera.RegisterImageEventHandler(self.imageEventHandler, pylon.RegistrationMode_Append, pylon.Cleanup_Delete)

    def grabImage(self):
        grabbedImage = None
        cameraContextValue = None

        if self.inProcessOfClosing:
            return grabbedImage, cameraContextValue

        if not self.cameraArray.IsGrabbing():
            self.cameraArray.StartGrabbing()
        try:
            grabResult = self.cameraArray.RetrieveResult(100)
            if grabResult.GrabSucceeded():
                cameraContextValue = grabResult.GetCameraContext()
                with grabResult.GetArrayZeroCopy() as ZCArray:
                    grabbedImage = self.formatImage(ZCArray)
        except genicam.GenericException as e:
            print("An exception occurred. {}".format(e))
        return grabbedImage, cameraContextValue

    def grabImageTests(self):
        camera_done = [0, 0]
        grabbedImage = None

        if not self.cameraArray.IsGrabbing():
            self.cameraArray.StartGrabbing()

        # for i, camera in enumerate(self.cameraArray):
        #     try:
        #         if not camera.IsGrabbing():
        #             # self.registerGrabbingStrategy(camera)
        #             camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly, pylon.GrabLoop_ProvidedByInstantCamera)
        #             print("Started grabbing")
        #         if camera.WaitForFrameTriggerReady(10, pylon.TimeoutHandling_Return):
        #             camera.ExecuteSoftwareTrigger()
        #         while grabbedImage is None:
        #             print("waiting")
        #             grabbedImage = self.imageEventHandlers[i].grabbedImage
        #         self.grabbedImages[self.imageEventHandlers[i].cameraContextValue] = grabbedImage
        #     except genicam.GenericException as e:
        #         print("An exception occurred. {}".format(e))

        # Kind of works, but with delays...
        try:
            grabResult = self.cameraArray.RetrieveResult(50)
            if grabResult.GrabSucceeded():
                cameraContextValue = grabResult.GetCameraContext()
                print(cameraContextValue)
                if camera_done[cameraContextValue] == 0:
                    with grabResult.GetArrayZeroCopy() as ZCArray:
                        grabbedImage = ZCArray
                        self.grabbedImages[cameraContextValue] = self.formatImage(grabbedImage)
                        camera_done[cameraContextValue] = 1
        except genicam.GenericException as e:
            print("An exception occurred. {}".format(e))

        # try:
        #     # if self.cameraArray.WaitForFrameTriggerReady(100, pylon.TimeoutHandling_ThrowException):
        #     #     self.cameraArray.ExecuteSoftwareTrigger()
        #
        #     grabResult = self.cameraArray.RetrieveResult(100, pylon.TimeoutHandling_Return)
        #     if grabResult.GrabSucceeded():
        #         cameraContextValue = grabResult.GetCameraContext()
        #         if camera_done[cameraContextValue] == 0:
        #             with grabResult.GetArrayZeroCopy() as ZCArray:
        #                 grabbedImage = ZCArray
        #                 self.grabbedImages[cameraContextValue] = self.formatImage(grabbedImage)
        #                 camera_done[cameraContextValue] = 1
        # except genicam.GenericException as e:
        #     print("An exception occurred. {}".format(e))

        return self.grabbedImages

    def grabImageOld(self):
        grabbedImages = []
        for i, camera in enumerate(self.cameraArray):
            camera.Open()
            grabbedImage = None
            if not camera.IsGrabbing():
                # self.registerGrabbingStrategy(camera_number)
                camera.StartGrabbing(pylon.GrabStrategy_OneByOne, pylon.GrabLoop_ProvidedByInstantCamera)
            if camera.WaitForFrameTriggerReady(100, pylon.TimeoutHandling_Return):
                camera.ExecuteSoftwareTrigger()
            while grabbedImage is None:
                grabbedImage = self.imageEventHandlers[i].grabbedImage
            grabbedImages.append(self.formatImage(grabbedImage))
        return grabbedImages


if __name__ == '__main__':

    serials = ["22290932", "21565643"]
    cameras = [Camera(serial_number) for serial_number in serials]

    window = "Show me"
    cv.namedWindow(window)
    cv.moveWindow(window, 20, 20)

    start = time.time()
    cameraindex = 0
    camera = cameras[cameraindex]

    while True:
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        image = camera.grabImage()
        image = cv.resize(image, (int(camera.pixelWidth / 3), int(camera.pixelHeight / 3)))

        cv.imshow(window, image)

        if cv.waitKey(1) & 0xFF == ord('c'):
            camera.Close()
            cameraindex += 1
            cameraindex = cameraindex % 2
            camera = cameras[cameraindex]
            camera.Open()

        now = time.time()
        print("Camera FPS =", 1 / (time.time() - start))
        start = now

    for camera in cameras:
        camera.Close()

    cv.destroyAllWindows()
    sys.exit()

    import threading
    import multiprocessing
    import queue

    def runCamera(queue, serial_number):
        camera = Camera(serial_number)

        cv.namedWindow(serial_number)
        cv.moveWindow(serial_number, 20, 20)

        start = time.time()
        while True:
            lock = queue.get()
            lock.acquire()
            image = camera.grabImage()
            lock.release()
            queue.put(lock)

            image = cv.resize(image, (int(camera.pixelWidth / 3), int(camera.pixelHeight / 3)))

            cv.imshow(serial_number, image)

            if cv.waitKey(1) & 0xFF == ord('q'):
                break
            now = time.time()
            print("Camera FPS =", 1 / (time.time() - start))
            start = now

        camera.Close()
        cv.destroyAllWindows()

    serials = ["22290932", "21565643"]
    threadLock = threading.Lock()
    lockQueue = queue.Queue()
    lockQueue.put(threadLock)
    cameraThread1 = threading.Thread(target=runCamera, args=(lockQueue, "22290932", ), daemon=True)
    cameraThread1.start()

    cameraThread2 = threading.Thread(target=runCamera, args=(lockQueue, "21565643",), daemon=True)
    cameraThread2.start()
    cameraThread2.join()

    sys.exit()

    # # Example on how to use a single camera and how to threshold:
    # camera1 = Camera("22290932")
    # camera1.Open()
    # print(dir(camera1.camera))
    # print(dir(camera1.camera.TriggerMode))
    # print(camera1.camera.TriggerMode.GetValue())
    # camera1.Close()
    #
    # camera1 = Camera("22290932")
    # camera2 = Camera("21565643")
    #
    # testWindow1 = 'window1'
    # testWindow2 = 'window2'
    # cv.namedWindow(testWindow1)
    # cv.moveWindow(testWindow1, 20, 20)
    #
    # # cv.namedWindow(testWindow2)
    # # cv.moveWindow(testWindow2, 500, 20)
    # start = time.time()
    #
    # while True:
    #     image1 = camera1.grabImage()
    #     image1 = cv.resize(image1, (int(camera1.pixelWidth / 2), int(camera1.pixelHeight / 2)))
    #
    #     image1 = image1[30:-60, 70:-120]
    #
    #     image1 = ((image1 - np.min(image1)) / (np.max(image1) - np.min(image1))).astype('uint8')
    #
    #     # blur = cv.GaussianBlur(image1, (5, 5), 0)
    #     # _, bricks = cv.threshold(blur, 50, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    #     # cv.imshow(testWindow1, bricks)
    #     # continue
    #
    #     bricks = 255*(image1 <= 20).astype('uint8')
    #
    #     _, contours, hierarchy = cv.findContours(bricks, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    #     drawme = np.zeros(image1.shape, dtype=np.uint8)
    #     cv.drawContours(drawme, contours, -1, (255, 255, 255), thickness=cv.FILLED)
    #
    #     for contour in contours:
    #         M = cv.moments(contour)
    #         if M['m00'] == 0:
    #             break
    #         X = int(M['m10'] / M['m00'])
    #         Y = int(M['m01'] / M['m00'])
    #
    #         image1 = cv.circle(image1, (X, Y), 4, 255, -1)
    #
    #         rect = cv.minAreaRect(contour)
    #         (x, y), (w, h), angle = rect
    #         box = cv.boxPoints(rect)
    #         box = np.int0(box)
    #         print(box, box.shape)
    #
    #         lengths = np.array([np.sqrt((box[i - 1, 0] - box[i, 0]) ** 2 + (box[i - 1, 1] - box[i, 1]) ** 2) for i in range(4)])
    #         print("lengths:\n", lengths)
    #
    #         # midx = np.array([(box[i-1] + (box[i-1] - box[i])/2).astype(np.int) for i in range(4)])
    #         # print("midx:\n", midx)
    #         # image1 = cv.line(image1, (midx[0, 0], midx[0, 1]), (midx[1, 0], midx[1, 1]), 255, 5)
    #
    #         start = (X, Y)
    #         end = (int(X + lengths[0] * np.cos(angle)), int(Y + lengths[0] * np.sin(angle)))
    #         image1 = cv.line(image1, start, end, 255, 5)
    #
    #         for pt in box:
    #             image1 = cv.circle(image1, (pt[0], pt[1]), 4, 255, -1)
    #         # one, two, three, four = box
    #         # print(((four - two)/2))
    #         # print(type(tuple(np.rint((four - two)/2))))
    #         # start = tuple(np.rint(two + (four - two)/2).astype(np.int))
    #         # end   = tuple(np.rint(two + (three - one)/2).astype(np.int))
    #         # print(start, end)
    #         # image1 = cv.line(image1, start, end, 255, 5)
    #
    #         # image2 = camera2.grabImage()
    #     # image2 = cv.resize(image2, (int(camera2.pixelWidth / 4), int(camera2.pixelHeight / 4)))
    #
    #     cv.imshow(testWindow1, bricks)
    #     # cv.imshow(testWindow2, image2)
    #
    #     if cv.waitKey(1) & 0xFF == ord('q'):
    #         break
    #     now = time.time()
    #     # print("FPS =", 1 / (time.time() - start))
    #     start = now
    #
    # camera1.Close()
    # camera2.Close()
    # cv.destroyAllWindows()

    # # Example on how to use multiple cameras:
    # serials = ["21565643", "22290932"]
    # cameras = CameraArray(serials)
    # print(dir(cameras.cameraArray))
    # sys.exit()
    #
    # testWindow = 'Test window'
    # windowWidth = 500
    # windowHeight = 500
    # cv.namedWindow(testWindow)
    # cv.moveWindow(testWindow, 20, 20)
    # start = time.time()
    #
    # def hconcat_resize_min(im_list, maxWidth, interpolation=cv.INTER_CUBIC):
    #     h_min = min(min(im.shape[0] for im in im_list), maxWidth)
    #     im_list_resize = [cv.resize(im, (int(im.shape[1] * h_min / im.shape[0]), h_min), interpolation=interpolation) for im in im_list]
    #     return cv.hconcat(im_list_resize)
    #
    # while True:
    #     images = cameras.grabImage()
    #     # images[0] = cv.rotate(images[0], cv.ROTATE_90_CLOCKWISE)
    #     # print("len(images)", len(images))
    #     # print("images[0]", images[0].shape, type())  # (1024, 1280) with r = 0.8001561280249805
    #     # print("images[1]", images[1].shape)  # (1200, 1920) with r = 0.625
    #
    #     for im in images:
    #         w, h = im.shape
    #         print("(w, h) =", w, ",", h, "type =", type(im[0, 0]))
    #         im = cv.resize(im, (int(w/h*windowHeight), windowHeight))
    #         print((int(w/h*windowHeight), windowHeight))
    #         # print("(w, h) =", w, ",", h, "type =", type(im[0, 0]))
    #
    #     image = hconcat_resize_min(images, windowWidth)
    #     print(image.shape)
    #
    #     cv.imshow(testWindow, image)
    #     if cv.waitKey(1) & 0xFF == ord('q'):
    #         break
    #     now = time.time()
    #     print("FPS =", 1 / (time.time() - start))
    #     start = now
    #
    # cameras.Close()
    # cv.destroyAllWindows()
