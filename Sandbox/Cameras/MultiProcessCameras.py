import os
import time
import cv2 as cv
from pypylon import pylon, genicam
from multiprocessing import Process


class CamProcess(Process):
    def __init__(self, queue, serial_number):
        Process.__init__(self)
        self.data_queue = queue
        self.serial_number = serial_number
        self.flag = True
        self.id = None
        self.cam = None

    def run(self):
        if self.id is None:
            self.id = os.getpid()
        self.cam = Basler(self.serial_number, flag=0)
        while self.flag:
            ret, frame = self.cam.read()
            if ret:
                self.data_queue.put([self.id, frame])
            # else:
            #     self.data_queue.put(None)

    def stop(self):
        self.flag = False
        self.cam.release()


class Basler:
    def __init__(self, serial_number, flag=0, exposure=None):
        self.info = pylon.CDeviceInfo()
        self.info.SetSerialNumber(serial_number)
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(self.info))
        self.camera.Open()

        # Grabing Continously (video) with minimal delay
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        self.converter = pylon.ImageFormatConverter()

        # converting to opencv bgr format
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        """
        The code below works and it is similar to the code wrapper in 
        "read" function. Calling that function gives error
        """

        # self.some =self.camera.RetrieveResult (5000, pylon.TimeoutHandling_ThrowException)
        # self.image=self.converter.Convert (self.some)
        # self.img=self.image.GetArray ()
        # print(self.img)

    def read(self):
        success = False
        grabbedImage = None
        try:
            grabResult = self.camera.RetrieveResult(100)
            if grabResult.GrabSucceeded():
                with grabResult.GetArrayZeroCopy() as ZCArray:
                    grabbedImage = ZCArray
                    success = True
        except genicam.GenericException as e:
            print("An exception occurred. {}".format(e))
        return success, grabbedImage

        # grabResult = self.camera.RetrieveResult(1000, pylon.TimeoutHandling_ThrowException)
        # if grabResult.GrabSucceeded():
        #     with grabResult.GetArrayZeroCopy() as ZCArray:
        #         # with grabResult.GetArray() as ZCArray:
        #         grabbedImage = ZCArray
        #     return True, img
        # else:
        #     return False, None

    def release(self):
        self.camera.StopGrabbing()
        self.camera.Close()
        del self.camera


if __name__ == '__main__':
    from multiprocessing import Queue

    serials = ["22290932", "21565643"]

    im_q1 = Queue(maxsize=1)
    serial1 = "22290932"
    camera1 = CamProcess(im_q1, serial1)
    camera1.start()

    cv.namedWindow(serial1)
    cv.moveWindow(serial1, 20, 20)
    cv.resizeWindow(serial1, 10, 10)

    im_q2 = Queue(maxsize=1)
    serial2 = "21565643"
    camera2 = CamProcess(im_q2, serial2)
    camera2.start()

    cv.namedWindow(serial2)
    cv.moveWindow(serial2, 200, 20)
    cv.resizeWindow(serial2, 10, 10)

    # while camera2.id is None:
    #     time.sleep(0.1)

    time.sleep(1)

    start = time.time()
    while True:
        process, image1 = im_q1.get()
        h, w = image1.shape
        image1 = cv.resize(image1, (int(w / 3), int(h / 3)))
        cv.imshow(serial1, image1)

        process, image2 = im_q2.get()
        h, w = image2.shape
        image2 = cv.resize(image2, (int(w / 3), int(h / 3)))
        cv.imshow(serial2, image2)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        now = time.time()
        print("Camera FPS =", 1 / (time.time() - start))
        start = now

    camera1.stop()
    camera2.stop()
    cv.destroyAllWindows()
