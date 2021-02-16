import time

from threading import Thread, Event, enumerate as list_threads
from multiprocessing import Process

from queue import SimpleQueue, LifoQueue, Empty, Full

from RobotClass import Robot
from CameraManagement import TopCamera
from CameraManagement import DetailCamera
from Functionalities import sleep


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
        stopPrintingEvent = Event()

        def printActiveThreadsContinuously(stop_printing_event):  # Continuously print which threads are still active
            while not stop_printing_event.is_set():
                print("Active threads: ", [t.name for t in list_threads()])
                sleep(1.0, stop_printing_event)
        Thread(target=printActiveThreadsContinuously, args=[stopPrintingEvent], name="Print Active Threads Continuously", daemon=True).start()

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
        stopPrintingEvent.set()

    def isConnected(self):
        answer = [False]*4
        for i, part in enumerate([self.Robot.ModBusReader, self.Robot.RobotCCO, self.TopCamera, self.DetailCamera]):
            try:
                answer[i] = part.isConnected()
            except Exception as e:
                answer[i] = False
        return all(answer)

    def grabImage(self):
        # Intercept to be able to use the info for the robot
        image, self.ImageInfo, cam_num = self.TopCamera.grabImage()
        return image, self.ImageInfo, cam_num

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

    def goHome(self):
        self.Robot.giveTask(self.Robot.goHome)

    def startRobotTask(self):
        def task(stop_event_as_argument):
            # self.Robot.moveToolTo(stop_event_as_argument, self.Robot.ToolPositionTestCollision.copy(), 'movej')
            # self.Robot.moveToolTo(stop_event_as_argument, self.Robot.ToolPositionLightBox.copy(), 'movel')
            self.Robot.closeGripper(stop_event_as_argument)
            # # self.Robot.moveJointsTo(stop_event_as_argument, self.Robot.JointAngleDropObject.copy(), 'movej')
            # # self.Robot.pickUpObject(stop_event_as_argument, self.ImageInfo[0])
            self.switchActiveCamera()
            # # self.Robot.presentObject(stop_event_as_argument)
            self.Robot.moveJointsTo(stop_event_as_argument, self.Robot.JointAngleReadObject.copy(), 'movej')
            sleep(10.0, stop_event_as_argument)
            self.Robot.openGripper(stop_event_as_argument)
            # self.Robot.dropObject(stop_event_as_argument)
            self.switchActiveCamera()
            self.Robot.goHome(stop_event_as_argument)
        self.Robot.giveTask(task)

    def stopRobotTask(self):
        print("Stopping robot task")
        self.Robot.halt()
        # self.Robot.giveTask(self.Robot.goHome)


if __name__ == '__main__':
    c = MainManager()
    time.sleep(2)
