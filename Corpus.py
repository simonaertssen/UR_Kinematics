import time
from RobotClass import Robot
from CameraManagement import TopCamera, DetailCamera
from threading import Thread, enumerate

from multiprocessing import Process, Event


class MainManager:
    def __init__(self):
        # super(MainManager, self).__init__(name='MainManagerThread')
        self.robot = None
        self.topCam = None
        self.detailCam = None
        self.actions = dict()
        self.imageInfoList = []
        self.running = Event()
        self.task = Thread(target=self.run, args=(self.running, ), daemon=True, name='MainManagerTask')
        self.tryConnect()
        self.running.set()
        self.task.start()

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
        [testConnection(x) for x in [self.robot, self.topCam, self.detailCam]]

    def run(self, stopevent):
        while stopevent.is_set():
            for function_name, function_to_call in self.actions.items():
                try:
                    function_to_call()
                except TypeError as e:
                    print('An uncallable function was encountered: {}'.format(e))

    def shutdownAllComponents(self):
        self.running.clear()
        self.task.join()
        shutdownThreads = [Thread(target=self.robot.shutdownSafely, name='Corpus.robot.shutdownSafely'),
                           Thread(target=self.topCam.Shutdown, name='Corpus.topCam.Shutdown'),
                           Thread(target=self.detailCam.Shutdown, name='Corpus.detailCam.Shutdown')]
        [x.start() for x in shutdownThreads]
        [x.join() for x in shutdownThreads]

    def checkComponentsAreConnected(self):
        return self.isRobotConnected() and self.isModBusConnected() and self.isTopCameraConnected() and self.isDetailCameraConnected()

    def isRobotConnected(self):
        try:
            return self.robot.RobotCCO.Connected
        except Exception as e:
            return False

    def isModBusConnected(self):
        try:
            return self.robot.ModBusReader.Connected
        except Exception as e:
            return False

    def isTopCameraConnected(self):
        try:
            return self.topCam.Connected
        except Exception as e:
            return False

    def isDetailCameraConnected(self):
        try:
            return self.detailCam.Connected
        except Exception as e:
            return False

    def getContinuousImages(self, continuous_image_callback, continuous_info_callback):
        if not callable(continuous_image_callback):
            raise ValueError('Continuous Image Callback not callable')
        if not callable(continuous_info_callback):
            raise ValueError('Continuous Info Callback not callable')

        def wrap_callback():
            output = self.topCam.grabImage()
            if output:
                continuous_image_callback(output[0])
            if len(output) > 1:
                self.imageInfoList = output[1]
                continuous_info_callback(output[1:])
        self.actions[str(continuous_image_callback)] = wrap_callback

    def test(self):
        print('Testing')

    def openGripper(self):
        self.robot.openGripper()

    def closeGripper(self):
        self.robot.closeGripper()

    def stopRobot(self):
        self.robot.send(b'stopl(10) + \n')

    def moveToolTo(self, target_position, move, wait=True, check_collisions=True):
        self.robot.moveToolTo(target_position, move, wait, check_collisions)

    def moveJointsTo(self, target_position, move, wait=True, check_collisions=True):
        self.robot.moveJointsTo(target_position, move, wait, check_collisions)

    def pickUpObject(self):
        LIGHTBOX_LENGTH = 0.250  # m
        LIGHTBOX_WIDTH = 0.176  # m
        print(self.imageInfoList)
        if len(self.imageInfoList) < 1:
            return
        X, Y, angle = self.imageInfoList
        target_position = self.robot.ToolPositionLightBox
        print(target_position)

        target_position[0] += Y * LIGHTBOX_WIDTH
        target_position[1] -= X * LIGHTBOX_LENGTH
        self.moveToolTo(target_position, 'movel')
        try:
            target_position = self.robot.getJointAngles()
            print(type(target_position))
            print(target_position)
            target_position[-1] = angle
            self.moveJointsTo(target_position, 'movej')
        except Exception as e:
            print(e)

        time.sleep(2)
        self.moveJointsTo(self.robot.JointAngleInit, "movej")
        print(target_position)


if __name__ == '__main__':
    print('Before')
    c = MainManager()
    time.sleep(10)
    print(c.checkComponentsAreConnected())
    time.sleep(10)
    print('After')
