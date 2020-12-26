import time
from RobotClass import Robot as robot
# from CameraManagement import TopCamera as topCamera
# from CameraManagement import DetailCamera as detailCamera
from threading import Thread, Event, enumerate


class topCamera:
    def __init__(self):
        print("topCamera started")


class detailCamera:
    def __init__(self):
        print("detailCamera started")


class MainManager:
    def __init__(self):
        self.Robot = None
        self.TopCam = None
        self.DetailCam = None
        self.Actions = dict()
        self.ImageInfoList = []

        self.StopEvent = Event()
        self.Task = Thread(target=self.run, args=[self.running], daemon=True, name='MainManagerTask')

        self.tryConnect()
        self.Task.start()

    def tryConnect(self):
        def startAsync(attribute, constructor):
            setattr(self, attribute, constructor())
        startThreads = [Thread(target=startAsync, args=('robot', Robot,), name='Corpus.robot.startAsync'),
                        Thread(target=startAsync, args=('topCam', TopCamera,), name='Corpus.robot.startAsync'),
                        Thread(target=startAsync, args=('detailCam', DetailCamera,), name='Corpus.robot.startAsync')]
        [x.start() for x in startThreads]
        [x.join() for x in startThreads]

        def testConnection(obj):
            if obj is None:
                raise ConnectionError(obj, "is not connected.")
        [testConnection(x) for x in [self.Robot, self.TopCam, self.DetailCam]]

    def run(self, stopevent):
        while stopevent.is_set():
            for function_name, function_to_call in self.Actions.items():
                try:
                    function_to_call()
                except TypeError as e:
                    print('An uncallable function was encountered: {}'.format(e))

    def shutdownAllComponents(self):
        self.running.clear()
        self.Task.join()
        shutdownThreads = [Thread(target=self.Robot.shutdownSafely, name='Corpus.robot.shutdownSafely'),
                           Thread(target=self.TopCam.Shutdown, name='Corpus.topCam.Shutdown'),
                           Thread(target=self.DetailCam.Shutdown, name='Corpus.detailCam.Shutdown')]
        [x.start() for x in shutdownThreads]
        [x.join() for x in shutdownThreads]

    def checkComponentsAreConnected(self):
        return self.isRobotConnected() and self.isModBusConnected() and self.isTopCameraConnected() and self.isDetailCameraConnected()

    def isRobotConnected(self):
        try:
            return self.Robot.RobotCCO.Connected
        except Exception as e:
            return False

    def isModBusConnected(self):
        try:
            return self.Robot.ModBusReader.Connected
        except Exception as e:
            return False

    def isTopCameraConnected(self):
        try:
            return self.TopCam.Connected
        except Exception as e:
            return False

    def isDetailCameraConnected(self):
        try:
            return self.DetailCam.Connected
        except Exception as e:
            return False

    def getContinuousImages(self, continuous_image_callback, continuous_info_callback):
        if not callable(continuous_image_callback):
            raise ValueError('Continuous Image Callback not callable')
        if not callable(continuous_info_callback):
            raise ValueError('Continuous Info Callback not callable')

        def wrap_callback():
            output = self.TopCam.grabImage()
            if output:
                continuous_image_callback(output[0])
            if len(output) > 1:
                self.ImageInfoList = output[1]
                continuous_info_callback(output[1:])
        self.Actions[str(continuous_image_callback)] = wrap_callback

    def test(self):
        print('Testing')

    def openGripper(self):
        self.Robot.openGripper()

    def closeGripper(self):
        self.Robot.closeGripper()

    def startRobot(self):
        try:
            self.Robot.pickUpObject(self.ImageInfoList)
            self.Robot.goHome()
            self.Robot.dropObject()
        except Exception as e:
            print(e)
        finally:
            self.Robot.goHome()

    def stopRobot(self):
        self.Robot.send(b'stop(10) + \n')

    def moveToolTo(self, target_position, move, wait=True, check_collisions=True):
        self.Robot.moveToolTo(target_position, move, wait, check_collisions)

    def moveJointsTo(self, target_position, move, wait=True, check_collisions=True):
        self.Robot.moveJointsTo(target_position, move, wait, check_collisions)


if __name__ == '__main__':
    print('Before')
    c = MainManager()
    time.sleep(10)
    print(c.checkComponentsAreConnected())
    time.sleep(10)
    print('After')
