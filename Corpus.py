import time

from threading import Thread, Event
from queue import Queue

from RobotClass import Robot
from CameraManagement import TopCamera
from CameraManagement import DetailCamera


class MainManager:
    def __init__(self):
        self.Robot = Robot
        self.TopCamera = TopCamera
        self.DetailCamera = DetailCamera
        self.StopEvent = Event()

        self.ImageQueue = Queue()
        self.ImageTaskThread = Thread(target=self.runImageTasks, args=[self.ImageQueue, self.StopEvent], daemon=True, name='Async Robot tasks')

        self.RobotTaskQueue = Queue()
        self.RobotTaskThread = Thread(target=self.runRobotTasks, args=[self.RobotTaskQueue, self.Robot.StopEvent], daemon=True, name='Async Robot tasks')

        self.tryConnect()
        self.ImageTaskThread.start()
        self.RobotTaskThread.start()

    def tryConnect(self):
        # Start these parts safely before anything else:
        ReturnErrorMessageQueue = Queue()

        def startAsync(this, error_queue, constructor):
            try:
                print(constructor.__name__)
                setattr(this, constructor.__name__, constructor())
            except Exception as e:
                # Startup of this part has failed and we need to shutdown all parts
                error_queue.put(e)

        startThreads = [Thread(target=startAsync, args=[self, ReturnErrorMessageQueue, partname], name='{} startAsync'.format(partname)) for partname in [Robot, TopCamera, DetailCamera]]
        [x.start() for x in startThreads]
        [x.join() for x in startThreads]

        # Get first error, if any, and raise it to warn the instance and the parent
        if not ReturnErrorMessageQueue.empty():
            print('There is an error')
            self.shutdownSafely()
            raise ReturnErrorMessageQueue.get()

    def shutdownSafely(self):
        if not self.StopEvent.isSet():
            self.StopEvent.set()
        if self.RobotTaskThread.is_alive():
            self.Robot.StopEvent.set()
            self.RobotTaskThread.join()
            self.Robot.StopEvent.clear()
        if self.RobotTaskThread.is_alive():
            self.StopEvent.set()
            self.ImageTaskThread.join()
            self.StopEvent.clear()

        def shutdownAsync(part):
            if part is None:
                return
            try:
                part.shutdownSafely()
            except Exception as e:
                raise SystemExit("Safe shutdown failed due to {}. Aborting".format(e))

        shutdownThreads = [Thread(target=shutdownAsync, args=[part], name='{} shutdownSafely'.format(part)) for part in [self.Robot, self.TopCamera, self.DetailCamera]]
        [x.start() for x in shutdownThreads]
        [x.join() for x in shutdownThreads]

    def runImageTasks(self, image_queue, stop_event):
        while not stop_event.is_set():
            try:
                image, info, cam_num = self.TopCamera.grabImage()
                if image is None:
                    continue
                image_queue.put(image, info, cam_num)
            except Exception as e:
                print('An exception occurred while retrieving images: {}'.format(e))

    @staticmethod
    def runRobotTasks(robot_task_queue, robot_stop_event):
        while not robot_stop_event.is_set():
            task_handle = robot_task_queue.get()
            try:
                task_handle(robot_stop_event)
            except TypeError as e:
                print('An uncallable function was encountered: {}'.format(e))

    def isConnected(self):
        answer = [False]*4
        for i, part in enumerate([self.Robot.ModBusReader, self.Robot.RobotCCO, self.TopCamera, self.DetailCamera]):
            try:
                answer[i] = part.isConnected()
            except Exception as e:
                answer[i] = False
        return all(answer)

    def switchActiveCamera(self):
        if not self.TopCamera.isConnected() or not self.DetailCamera.isConnected():
            print("Error switching cameras: one is not connected.")
            return
        self.TopCamera, self.DetailCamera = self.DetailCamera, self.TopCamera

    def giveRobotParallelTask(self, function_handle):
        self.RobotTaskQueue.put(function_handle)

    def openGripper(self):
        self.giveRobotParallelTask(self.Robot.openGripper)

    def closeGripper(self):
        self.giveRobotParallelTask(self.Robot.closeGripper)

    def startRobot(self, info):
        def startRobotHandle(stop_event_as_argument):
            self.Robot.pickUpObject(stop_event_as_argument, info)
            self.Robot.goHome(stop_event_as_argument)
            self.Robot.dropObject(stop_event_as_argument)
            self.Robot.goHome(stop_event_as_argument)

        self.giveRobotParallelTask(startRobotHandle)

    def stopRobot(self):
        self.giveRobotParallelTask(self.Robot.halt)


if __name__ == '__main__':
    c = MainManager()
    time.sleep(2)
    print('After')
