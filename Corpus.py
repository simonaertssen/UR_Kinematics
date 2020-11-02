import time
from RobotClass import Robot
from CameraManagement import TopCamera, DetailCamera
from threading import Thread


class MainManager(Thread):
    def __init__(self):
        super(MainManager, self).__init__(daemon=True)
        self.robot = None
        self.topCam = None
        self.detailCam = None
        self.actions = dict()
        self.tryConnect()
        self.start()

    def startAsync(self, attribute, constructor):
        setattr(self, attribute, constructor())

    def tryConnect(self):
        startThreads = [Thread(target=self.startAsync, args=('robot', Robot,)),
                        Thread(target=self.startAsync, args=('topCam', TopCamera,)),
                        Thread(target=self.startAsync, args=('detailCam', DetailCamera,))]
        for thread in startThreads:
            thread.start()
        for thread in startThreads:
            thread.join()
        print('All connected')

    def run(self):
        while True:
            for function_name, function_to_call in self.actions.items():
                # print(function_name)
                try:
                    function_to_call()
                except TypeError:
                    print('An uncallable function was encountered.')

    def checkComponentsAreConnected(self):
        return self.isRobotConnected() and self.isModBusConnected() and self.isTopCameraConnected() and self.isDetailCameraConnected()

    def isRobotConnected(self):
        try:
            return self.robot.RobotCCO.Connected
        except:
            return False

    def isModBusConnected(self):
        try:
            return self.robot.ModBusReader.Connected
        except:
            return False

    def isTopCameraConnected(self):
        try:
            return self.topCam.Connected
        except:
            return False

    def isDetailCameraConnected(self):
        try:
            return self.detailCam.Connected
        except:
            return False

    def getContinuousImages(self, continuous_image_callback, continuous_info_callback):
        if not callable(continuous_image_callback):
            raise ValueError('Continuous Image Callback not callable')
        if not callable(continuous_info_callback):
            raise ValueError('Continuous Info Callback not callable')

        def wrap_callback():
            image_to_yield, info_to_yield = self.topCam.grabImage()
            continuous_image_callback(image_to_yield)
            continuous_info_callback(info_to_yield)
        self.actions[str(continuous_image_callback)] = wrap_callback

    def test(self):
        print('Testing')


if __name__ == '__main__':
    print('Before')
    c = MainManager()
    time.sleep(10)
    print(c.checkComponentsAreConnected())
    time.sleep(10)
    print('After')
