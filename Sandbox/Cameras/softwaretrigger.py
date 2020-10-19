import time
from queue import Queue
import cv2 as cv
from pypylon import pylon, genicam


class SampleImageEventHandler(pylon.ImageEventHandler):
    returnImageCallback = Queue()

    def OnImageGrabbed(self, camera, grab_result):
        print("CSampleImageEventHandler::OnImageGrabbed called.")
        try:
            if grab_result.GrabSucceeded():
                with grab_result.GetArrayZeroCopy() as ZCArray:
                    grabbedImage = ZCArray
                    self.returnImageCallback.put(grabbedImage)
        except genicam.GenericException as e:
            print("ImageEventHandler Exception: {}".format(e))
        finally:
            grab_result.Release()


def sftrigger():
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    camera.Open()

    IEH = SampleImageEventHandler()
    camera.RegisterConfiguration(pylon.SoftwareTriggerConfiguration(), pylon.RegistrationMode_ReplaceAll, pylon.Cleanup_Delete)
    camera.RegisterImageEventHandler(SampleImageEventHandler(), pylon.RegistrationMode_Append, pylon.Cleanup_Delete)

    camera.StartGrabbing(pylon.GrabStrategy_OneByOne, pylon.GrabLoop_ProvidedByInstantCamera)

    while True:
        image = None
        time.sleep(0.05)
        print('Test')
        if camera.WaitForFrameTriggerReady(10, pylon.TimeoutHandling_Return):
            camera.ExecuteSoftwareTrigger()
        while image is None:
            image = IEH.returnImageCallback.get()


if __name__ == '__main__':
    sftrigger()

