import time

from threading import Thread, Event, enumerate as list_threads
from multiprocessing import Process

from queue import SimpleQueue, LifoQueue, Empty, Full

from RobotClass import Robot
from CameraManagement import TopCamera
from CameraManagement import DetailCamera


class MainManager:
    Robot = Robot
    TopCamera = TopCamera
    DetailCamera = DetailCamera
    ImageInfo = []

    def __init__(self):
        self.tryConnect()

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
        # Raise events:
        self.Robot.StopTaskEvent.set()
        self.Robot.StopEvent.set()
        # Join threads:
        if self.Robot.TaskThread.is_alive():
            self.Robot.TaskThread.join()

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

    def isConnected(self):
        answer = [False]*4
        for i, part in enumerate([self.Robot.ModBusReader, self.Robot.RobotCCO, self.TopCamera, self.DetailCamera]):
            try:
                answer[i] = part.isConnected()
            except Exception as e:
                answer[i] = False
        return all(answer)

    def grabImage(self):
        return self.TopCamera.grabImage()

    def switchActiveCamera(self):
        if not self.TopCamera.isConnected() or not self.DetailCamera.isConnected():
            print("Error switching cameras: one is not connected.")
            return
        self.TopCamera, self.DetailCamera = self.DetailCamera, self.TopCamera
        print("Cameras switched")

    def openGripper(self):
        self.Robot.giveTask(self.Robot.openGripper)

    def closeGripper(self):
        self.Robot.giveTask(self.Robot.closeGripper)

    def startRobotTask(self):
        def startRobotHandle(stop_event_as_argument):
            r"""" Feed the first found object into the pickup function. """
            # Pick up and present:
            self.Robot.pickUpObject(stop_event_as_argument, self.ImageInfo[0])
            self.switchActiveCamera()
            self.Robot.moveJointsTo(stop_event_as_argument, self.Robot.JointAngleReadObject.copy(), 'movej')
            time.sleep(100.0)
            self.switchActiveCamera()
            self.Robot.dropObject(stop_event_as_argument)
            self.Robot.goHome(stop_event_as_argument)

            # Move around:
            # self.Robot.moveToolTo(stop_event_as_argument, self.Robot.ToolPositionLightBox.copy(), 'movej')
            # self.Robot.goHome(stop_event_as_argument)
            # self.Robot.dropObject(stop_event_as_argument)
            # self.Robot.goHome(stop_event_as_argument)

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
        self.Robot.giveTask(startRobotHandle)

    def stopRobotTask(self):
        self.Robot.halt(self.Robot.StopTaskEvent.set())  # Halt the execution of the current task and wait until the robot is stopped
        self.Robot.TaskFinishedEvent.wait(timeout=1.0)

        def stopAndReturn(stop_event_as_argument):
            self.Robot.halt(stop_event_as_argument)
            self.Robot.goHome(stop_event_as_argument)
        self.Robot.giveTask(stopAndReturn)


if __name__ == '__main__':
    c = MainManager()
    time.sleep(2)
