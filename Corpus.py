import time

from threading import Thread, Event, enumerate as list_threads
from queue import SimpleQueue, Empty, Full

from RobotClass import Robot
from CameraManagement import TopCamera
from CameraManagement import DetailCamera


class MainManager:
    def __init__(self):
        self.Robot = Robot
        self.TopCamera = TopCamera
        self.DetailCamera = DetailCamera

        self.StopImageTaskEvent = Event()
        self.ImageQueue = SimpleQueue()  # Maximum one image at a time
        self.ImageInfo = None
        RobotTaskQueueArguments = [self.ImageQueue, self.StopImageTaskEvent]
        self.ImageTaskThread = Thread(target=self.runImageTasks, args=RobotTaskQueueArguments, daemon=True, name='Async Image retrieval')

        self.StopRobotTaskEvent = Event()
        self.RobotTaskFinishedEvent = Event()
        self.RobotTaskQueue = SimpleQueue()
        RobotTaskQueueArguments = [self.RobotTaskQueue, self.StopRobotTaskEvent, self.Robot.StopEvent, self.RobotTaskFinishedEvent]
        self.RobotTaskThread = Thread(target=self.runRobotTasks, args=RobotTaskQueueArguments, daemon=True, name='Async Robot tasks')

        self.tryConnect()
        # self.switchActiveCamera()
        self.ImageTaskThread.start()
        self.RobotTaskThread.start()

    def tryConnect(self):
        # Start these parts safely before anything else:
        ReturnErrorMessageQueue = SimpleQueue()

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
            self.Robot.StopEvent.set()
            self.RobotTaskThread.join()
        if self.ImageTaskThread.is_alive():
            self.StopImageTaskEvent.set()
            self.ImageTaskThread.join()

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
                self.ImageInfo = info

                try:
                    image_queue.put_nowait((image, info, cam_num))
                except Full as e:
                    # Yes, you know that a full queue raises an error
                    continue
            except Exception as e:
                print('An exception occurred while retrieving images: {}'.format(e))
        print("runImageTasks stopped")

    @staticmethod
    def runRobotTasks(robot_task_queue, task_stop_event, robot_stop_event, robot_task_finished_event):
        while not robot_stop_event.is_set():  # Continue for as long as the robot is running
            try:  # See if there is a new task
                task_handle = robot_task_queue.get(timeout=0.01)
            except Empty as e:
                # Yes, you know that emptying the queue raises an error
                continue  # To the next iteration of the loop

            try:  # See if we can execute the function
                task_handle(task_stop_event)
            except TypeError as e:
                print('An uncallable function was encountered: {}'.format(e))
            finally:  # Signal that the task was performed in some way
                robot_task_finished_event.set()
                if task_stop_event.isSet():
                    # If the event was raised, clear it and signal that the task was stopped.
                    # This yields the robot ready for the coming task.
                    task_stop_event.clear()
        print("runRobotTasks stopped")

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
        print("Cameras switched")

    def giveRobotParallelTask(self, function_handle):
        # Signal that the given task is not finished yet
        self.RobotTaskFinishedEvent.clear()
        self.RobotTaskQueue.put(function_handle)

    def openGripper(self):
        self.giveRobotParallelTask(self.Robot.openGripper)

    def closeGripper(self):
        self.giveRobotParallelTask(self.Robot.closeGripper)

    def startRobotTask(self):
        def startRobotHandle(stop_event_as_argument):
            r"""" Feed the first found object into the pickup function. """
            # Pick up and present:
            # self.Robot.pickUpObject(stop_event_as_argument, self.ImageInfo[0])
            # self.Robot.moveJointsTo(stop_event_as_argument, self.Robot.JointAngleReadObject.copy(), 'movej')
            # self.switchActiveCamera()
            # time.sleep(10.0)
            # self.switchActiveCamera()
            # self.Robot.dropObject(stop_event_as_argument)
            # self.Robot.goHome(stop_event_as_argument)

            # Move around:
            # self.Robot.moveToolTo(stop_event_as_argument, self.Robot.ToolPositionLightBox.copy(), 'movej')
            # self.Robot.goHome(stop_event_as_argument)
            self.Robot.dropObject(stop_event_as_argument)
            self.Robot.goHome(stop_event_as_argument)

            # Stay around the camera:
            # self.Robot.closeGripper(stop_event_as_argument)
            # target = self.Robot.JointAngleReadObject.copy()
            # self.Robot.moveJointsTo(stop_event_as_argument, target, 'movej')
            # self.switchActiveCamera()
            # time.sleep(10.0)
            # target[0] += 0.1
            # self.Robot.moveJointsTo(stop_event_as_argument, target, 'movej')
            # target[0] += 0.1
            # self.switchActiveCamera()

            # Old pickup:
            # self.Robot.pickUpObject(stop_event_as_argument, self.ImageInfo[0])
            # self.Robot.goHome(stop_event_as_argument)
            # self.Robot.dropObject(stop_event_as_argument)
            # self.Robot.goHome(stop_event_as_argument)
        self.giveRobotParallelTask(startRobotHandle)

    def stopRobotTask(self):
        self.StopRobotTaskEvent.set()  # Halt the execution of the current task and wait until the robot is stopped
        self.RobotTaskFinishedEvent.wait(timeout=3.0)

        def stopAndReturn(stop_event_as_argument):
            self.Robot.halt(stop_event_as_argument)
            self.Robot.goHome(stop_event_as_argument)
        self.giveRobotParallelTask(stopAndReturn)


if __name__ == '__main__':
    c = MainManager()
    time.sleep(2)
    print('After')
