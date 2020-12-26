import time

from threading import Thread, Event
from queue import Queue

from RobotClass import Robot as robot
# from CameraManagement import TopCamera as topCamera
# from CameraManagement import DetailCamera as detailCamera


class topCamera:
    def __init__(self):
        print("topCamera started")
    def shutdownSafely(self):
        print("topCamera shut down")


class detailCamera:
    def __init__(self):
        print("detailCamera started")
    def shutdownSafely(self):
        print("detailCamera shut down")


class MainManager:
    def __init__(self):
        self.Robot = robot
        self.TopCam = topCamera
        self.DetailCam = detailCamera
        self.StopEvent = Event()

        self.Actions = dict()
        self.ImageInfoList = []

        self.Tasks = Thread(target=self.run, args=[self.StopEvent], daemon=True, name='MainManager tasks')
        self.tryConnect()
        self.Tasks.start()

    def tryConnect(self):
        # Start these parts safely before anything else:
        ReturnErrorMessageQueue = Queue()
        def startAsync(error_queue, constructor):
            try:
                setattr(self, str(constructor), constructor())
            except Exception as e:
                # Startup of this part has failed and we need to shutdown all parts
                error_queue.put(e)

        startThreads = [Thread(target=startAsync, args=[ReturnErrorMessageQueue, partname], name='{} startAsync'.format(partname)) for partname in [robot, topCamera, detailCamera]]
        [x.start() for x in startThreads]
        [x.join() for x in startThreads]

        # Get first error, if any, and raise it to warn the instance and the parent
        if not ReturnErrorMessageQueue.empty():
            self.shutdownSafely()
            raise ReturnErrorMessageQueue.get()

    def shutdownSafely(self):
        if not self.StopEvent.isSet():
            self.StopEvent.set()
        if self.Tasks.is_alive():
            self.Tasks.join()
            self.StopEvent.clear()

        def shutdownAsync(object):
            if object is None:
                return
            try:
                object.shutdownSafely()
            except Exception as e:
                raise SystemExit("Safe shutdown failed due to {}. Aborting".format(e))

        shutdownThreads = [Thread(target=shutdownAsync, args=[part], name='{} shutdownSafely'.format(part)) for part in [self.Robot, self.TopCam, self.DetailCam]]
        [x.start() for x in shutdownThreads]
        [x.join() for x in shutdownThreads]

    def run(self, stop_event):
        while not stop_event.is_set():
            for function_name, function_to_call in self.Actions.items():
                try:
                    function_to_call()
                except TypeError as e:
                    print('An uncallable function was encountered: {}'.format(e))

    def isConnected(self):
        answer = [False]*4
        for i, part in enumerate([self.Robot.ModBusReader, self.Robot.RobotCCO, self.TopCam, self.DetailCam]):
            try:
                answer[i] = part.isConnected()
            except Exception as e:
                answer[i] = False
        return all(answer)

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

    def openGripper(self):
        self.Robot.openGripper()

    def closeGripper(self):
        self.Robot.closeGripper()

    def startRobot(self):
        def startRobotHandle(stop_event_as_argument):
            self.Robot.pickUpObject(self.ImageInfoList, stop_event=stop_event_as_argument)
            self.Robot.goHome(stop_event=stop_event_as_argument)
            self.Robot.dropObject(stop_event=stop_event_as_argument)
            self.Robot.goHome(stop_event=stop_event_as_argument)
        self.Robot.wrapInThread(startRobotHandle, stop_event_as_argument)

    def stopRobot(self):
        self.Robot.halt()

    def moveToolTo(self, target_position, move, wait=True, check_collisions=True):
        self.Robot.moveToolTo(target_position, move, wait, check_collisions)

    def moveJointsTo(self, target_position, move, wait=True, check_collisions=True):
        self.Robot.moveJointsTo(target_position, move, wait, check_collisions)


if __name__ == '__main__':
    c = MainManager()
    time.sleep(10)
    print(c.checkComponentsAreConnected())
    time.sleep(10)
    print('After')
