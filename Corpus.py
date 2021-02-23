import time
import sys
import os

from threading import Thread, Event, enumerate as list_threads

from queue import SimpleQueue

from RobotClass import Robot
from CameraManagement import TopCamera
from CameraManagement import DetailCamera
from Functionalities import communicateError, sleep
from ImageModule import saveImage, imageSharpness, markTextOnImage, imageContrast
from Functionalities import pi, testMemoryDemand
from KinematicsModule.Kinematics import RPY2RotVec  # Slow Python implementation


class MainManager:
    Robot = Robot
    TopCamera = TopCamera
    DetailCamera = DetailCamera
    Image = None
    ImageInfo = []
    ImageAvailable = Event()

    mem_leak_thread = Thread(target=testMemoryDemand, daemon=True, name='Test memory demand')
    mem_leak_thread.start()
    print("Started")

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
        self.Image, self.ImageInfo, cam_num = self.TopCamera.grabImage()
        if not self.ImageAvailable.isSet():
            self.ImageAvailable.set()
        return self.Image, self.ImageInfo, cam_num

    def switchActiveCamera(self, stop_event=None):
        r"""
        Switch the active camera. Because items are initialised using their correct class name,
        the TopCamera will always be the active camera instance. That means when we switch
        reference between cameras, we can run another camera without having to change names.
        """
        if stop_event is None:  # Replace with a random event when none is given
            stop_event = Event()
        if stop_event.isSet():
            return

        if not self.TopCamera.isConnected() or not self.DetailCamera.isConnected():
            print("Error switching cameras: one is not connected.")
            return
        self.TopCamera.close()
        self.TopCamera, self.DetailCamera = self.DetailCamera, self.TopCamera
        # Wait for the new camera to start grabbing
        while not stop_event.isSet() and not self.TopCamera.camera.IsGrabbing():
            time.sleep(0.01)
        # Wait for the new image to become available. We do not want to use the .wait() method
        # on the event, because we need to interrupt execution at any time.
        self.ImageAvailable.clear()
        while not stop_event.isSet() and not self.ImageAvailable.isSet():
            time.sleep(0.01)
        print("Camera switched")

    def openGripper(self):
        self.Robot.giveTask(self.Robot.openGripper)

    def closeGripper(self):
        self.Robot.giveTask(self.Robot.closeGripper)

    def goHome(self):
        self.Robot.giveTask(self.Robot.goHome)

    def optimiseFocus(self, stop_event):
        start_time = time.time()
        current_position = self.Robot.getToolPosition()
        MAX_STEPS = 20
        step = 10
        sign = 1
        while True:
            if stop_event.isSet():
                break
            while not self.ImageAvailable.isSet():
                # Wait for new image:
                sleep(0.01, stop_event)
            if start_time - time.time() > 10.0:
                print("Over time")
                break

            if step > MAX_STEPS:
                step = 1
                sign *= -1

            # Calibrated
            current_position[1] += sign * 0.0002
            current_position[2] -= sign * 0.001

            self.Robot.moveToolTo(stop_event, current_position, 'movej', velocity=0.25)
            sleep(0.1, stop_event)

            if self.Image is not None and not stop_event.isSet():
                image_candidate = self.Image.copy()
                # print(imageContrast(image_candidate))
            step += 1

    def startRobotTask(self):
        def testTask(stop_event_as_argument):
            self.Robot.pickUpObject(stop_event_as_argument, self.ImageInfo[0])

        def pickupTask(stop_event_as_argument):
            # Delete all previous images:
            image_folder = os.path.join(os.getcwd(), 'Images')
            for image_file in os.listdir(image_folder):
                os.remove(os.path.join(image_folder, image_file))

            # if not self.ImageInfo:
            #     raise ValueError("ImageInfo should not be None")

            self.Robot.turnWhiteLampON(stop_event_as_argument)
            # self.Robot.pickUpObject(stop_event_as_argument, self.ImageInfo[0])

            # Move to desired position from the Home position
            # current_joints = self.Robot.getJointAngles()
            # desired_change = [i * pi / 180 for i in [-50.0, -25.0, 25.0, 0, 180.0, 0]]
            # current_joints = [a + b for a, b in zip(current_joints, desired_change)]
            # self.Robot.moveJointsTo(stop_event_as_argument, current_joints, 'movej')
            self.Robot.moveJointsTo(stop_event_as_argument, self.Robot.JointAngleReadObject.copy(), 'movej')

            self.switchActiveCamera(stop_event_as_argument)

            try:
                self.optimiseFocus(stop_event_as_argument)
            except Exception as e:
                communicateError(e)

            self.switchActiveCamera(stop_event_as_argument)
            # Move back to initial position
            self.Robot.moveJointsTo(stop_event_as_argument, self.Robot.JointAngleReadObject.copy(), 'movej')

            # Move back to temporary position, and back home
            # self.Robot.moveJointsTo(stop_event_as_argument, current_joints, 'movej')
            # current_joints = [a - b for a, b in zip(current_joints, desired_change)]
            # self.Robot.moveJointsTo(stop_event_as_argument, current_joints, 'movej')

            # self.Robot.dropObject(stop_event_as_argument)
            # self.Robot.turnWhiteLampOFF(stop_event_as_argument)
        self.Robot.giveTask(pickupTask)

    def stopRobotTask(self):
        print("Stopping robot task")
        self.Robot.halt()


if __name__ == '__main__':
    c = MainManager()
    time.sleep(2)
