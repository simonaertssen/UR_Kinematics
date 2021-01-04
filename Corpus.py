import time

from threading import Thread, Event, enumerate as list_threads
from queue import Queue

from RobotClass import Robot
from CameraManagement import TopCamera
from CameraManagement import DetailCamera


class MainManager:
    def __init__(self):
        self.Robot = Robot
        self.TopCamera = TopCamera
        self.DetailCamera = DetailCamera

        self.StopImageTaskEvent = Event()
        self.ImageQueue = Queue()
        self.ImageInfo = None
        self.ImageTaskThread = Thread(target=self.runImageTasks, args=[self.ImageQueue, self.StopImageTaskEvent], daemon=True, name='Fetch image from camera')

        self.StopRobotTaskEvent = Event()
        self.RobotTaskQueue = Queue()
        self.RobotTaskThread = Thread(target=self.runRobotTasks, args=[self.RobotTaskQueue, self.StopRobotTaskEvent], daemon=True, name='Async Robot tasks')

        self.tryConnect()
        self.ImageTaskThread.start()
        self.RobotTaskThread.start()

    def tryConnect(self):
        # Start these parts safely before anything else:
        ReturnErrorMessageQueue = Queue()

        def startAsync(this, error_queue, constructor):
            try:
                setattr(this, constructor.__name__, constructor())
            except Exception as e:
                # Startup of this part has failed and we need to shutdown all parts
                error_queue.put(e)

        startThreads = [Thread(target=startAsync, args=[self, ReturnErrorMessageQueue, partname], name='{} startAsync'.format(partname)) for partname in [Robot, TopCamera, DetailCamera]]
        [x.start() for x in startThreads]
        [x.join() for x in startThreads]

        # Get first error, if any, and raise it to warn the instance and the parent
        if not ReturnErrorMessageQueue.empty():
            self.shutdownSafely()
            raise ReturnErrorMessageQueue.get()

    def shutdownSafely(self):
        print("Active threads: ", [t.name for t in list_threads()])
        if self.RobotTaskThread.is_alive():
            self.StopRobotTaskEvent.set()
            self.RobotTaskThread.join()
            self.StopRobotTaskEvent.clear()
        if self.ImageTaskThread.is_alive():
            self.StopImageTaskEvent.set()
            self.ImageTaskThread.join()
            self.StopImageTaskEvent.clear()

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
                while not image_queue.empty():
                    image_queue.get()
                self.ImageInfo = info
                image_queue.put((image, info, cam_num))
            except Exception as e:
                print('An exception occurred while retrieving images: {}'.format(e))

    @staticmethod
    def runRobotTasks(robot_task_queue, robot_stop_event):
        while not robot_stop_event.is_set():
            if robot_task_queue.empty():
                time.sleep(0.001)
                continue

            task_handle = robot_task_queue.get(block=False)
            print(task_handle, callable(task_handle))
            try:
                task_handle(robot_stop_event)
            except TypeError as e:
                print('An uncallable function was encountered: {}'.format(e))
            finally:
                robot_task_queue.task_done()

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

    def startRobot(self):
        def startRobotHandle(stop_event_as_argument):
            r"""" Feed the first found object into the pickup function. """
            self.Robot.pickUpObject(stop_event_as_argument, self.ImageInfo[0])
            print("Picked up")
            self.Robot.goHome(stop_event_as_argument)
            print("Went Home")
            self.Robot.dropObject(stop_event_as_argument)
            self.Robot.goHome(stop_event_as_argument)
        self.giveRobotParallelTask(startRobotHandle)

    def stopRobot(self):
        self.Robot.StopEvent.isSet()  # Halt the execution of the current task

        def stopAndReturn(stop_event_as_argument):
            self.Robot.halt(stop_event_as_argument)
            self.Robot.goHome(stop_event_as_argument)
        self.giveRobotParallelTask(stopAndReturn)


if __name__ == '__main__':
    c = MainManager()
    time.sleep(2)
    print('After')
