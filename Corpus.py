import time
import os
import numpy as np

from threading import Thread, Event, enumerate as list_threads

from queue import SimpleQueue

from RobotClass import Robot
from CameraManagement import TopCamera, DetailCamera

from Functionalities import communicateError, sleep
from ImageModule import saveImage, imageSharpness, markTextOnImage, imageContrast, cropToRectangle
from Functionalities import pi, testMemoryDemand
from KinematicsLib.cKinematics import toolPositionDifference, jointAngleDifference, spatialDifference


class MainManager:
    Robot = Robot
    TopCamera = TopCamera
    DetailCamera = DetailCamera
    _image = None
    _imageInfo = []
    ImageAvailable = Event()

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
        image, image_info, cam_num = None, None, None
        try:
            image, image_info, cam_num = self.TopCamera.grabImage()
            if image is not None and image_info is not None:
                self._image = image.copy()
                self._imageInfo = image_info.copy()
                if not self.ImageAvailable.isSet():
                    self.ImageAvailable.set()
        except Exception as e:
            communicateError(e, "bullshit")
        return image, image_info, cam_num

    def waitForNextAvailableImage(self, stop_event):
        if stop_event.isSet():
            return
        MAX_TIME = 1.0
        start_time = time.time()

        self.ImageAvailable.clear()
        while not stop_event.isSet() and not self.ImageAvailable.isSet():
            time.sleep(0.01)
            if start_time - time.time() > MAX_TIME:
                raise TimeoutError("Waiting for image took too long.")

        if self._image is not None:
            return self._image.copy()
        else:
            raise ReferenceError("_image not found, reference was deleted.")

    @staticmethod
    def deleteAllImages(stop_event):
        if stop_event.isSet():
            return
        # Delete all previous images:
        image_folder = os.path.join(os.getcwd(), 'Images')
        for image_file in os.listdir(image_folder):
            os.remove(os.path.join(image_folder, image_file))

    def optimiseExposure(self, stop_event):
        if stop_event.isSet():
            return
        self.TopCamera.open()

        step = 0
        MAX_STEPS = 10
        step_value = 500
        TARGET = 250.0
        while not stop_event.isSet() and step < MAX_STEPS:
            latest_image = cropToRectangle(self.waitForNextAvailableImage(stop_event))
            max_value = latest_image.max()
            print(max_value)
            if abs(max_value - TARGET) < 1.0:
                break
            current_value = self.TopCamera.camera.ExposureTimeAbs.GetValue()
            new_value = current_value - (max_value - TARGET)*step_value
            print("New value =", latest_image.mean())
            if new_value < 0.0:
                break
            self.TopCamera.camera.ExposureTimeAbs.SetValue(new_value)
            sleep(1.0, stop_event)
            step += 1

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
            sleep(0.01, stop_event)
        # Wait for the new image to become available. We do not want to use the .wait() method
        # on the event, because we need to interrupt execution at any time.
        self.ImageAvailable.clear()
        while not stop_event.isSet() and not self.ImageAvailable.isSet():
            sleep(0.01, stop_event)

    def openGripper(self):
        self.Robot.giveTask(self.Robot.openGripper)

    def closeGripper(self):
        self.Robot.giveTask(self.Robot.closeGripper)

    def goHome(self):
        self.Robot.giveTask(self.Robot.goHome)

    def optimise(self, stop_event, info, objective, transform_position, new_target):
        if stop_event.isSet():
            return

        # Guards:
        idx = info['idx']
        if not isinstance(idx, int):
            raise TypeError("Index is not an integer")
        if not callable(transform_position) or not callable(objective) or not callable(new_target):
            raise TypeError("Optimisation functions are not callable")

        def sample_objective(position, stop_event_as_argument):
            if stop_event.isSet():
                return np.NAN

            # Set position, record an image and record the score
            self.Robot.moveToolTo(stop_event_as_argument, position, 'movel', velocity=0.1)
            sleep(0.25, stop_event_as_argument)  # Let vibrations dissipate
            try:
                value = objective(self.waitForNextAvailableImage(stop_event_as_argument))
                print("Objective value =", value)
                return value
            except Exception as e:
                communicateError(e)

        # Set loop parameters
        iteration = 1
        MAX_ITERATIONS = 30
        start_time = time.time()
        MAX_TIME = 60.0                          # 1 minute
        MAX_TOLERANCE = 0.001                    # 1 mm
        MAX_DEVIATION = info['MAX_DEVIATION']    # 20 cm
        MAX_STEP = info['MAX_STEP']              # 3 mm

        # Gather initial data (3 points for a 2nd order polynomial)
        original_position = self.Robot.getToolPosition()
        current_position = original_position.copy()
        data = np.empty((2, MAX_ITERATIONS))
        data[:] = np.NAN
        for index, d_pos in enumerate(info['init']):
            current_position = transform_position(d_pos, current_position)
            data[:, index] = np.array([current_position[idx], sample_objective(current_position, stop_event)])
            # Invert the transformation because we are just taking estimates around the initial position
            current_position = transform_position(-d_pos, current_position)

        while iteration + 3 < MAX_ITERATIONS and not stop_event.isSet() and start_time - time.time() < MAX_TIME:
            # Remove nans from computation and fit the 2nd order polynomial:
            target = new_target(data)
            d_pos = target - current_position[idx]
            d_pos = (d_pos / abs(d_pos)) * min(abs(d_pos), MAX_STEP)
            print(f"Now = {round(current_position[idx], 4)}, new = {round(target, 4)}, d = {round(d_pos, 4)}")

            current_position = transform_position(d_pos, current_position)
            data[:, iteration + 2] = np.array([current_position[idx], sample_objective(current_position, stop_event)])
            # If deviation is too large then move back to the beginning
            if sum(toolPositionDifference(original_position, current_position)) > MAX_DEVIATION:
                print("Too much deviation")
                data[:, iteration + 3] = np.NAN
                self.Robot.moveToolTo(stop_event, original_position, 'movel')
                iteration -= 1
                continue
            if abs(d_pos) < MAX_TOLERANCE:  # Because we cannot move closer than 1 mm
                break
            iteration += 1
        return current_position

    def optimiseFocus(self, stop_event):
        if stop_event.isSet():
            return

        # Calibrated transformations
        def apply_z_to_x_transformation(d_z, position):
            position[1] -= d_z * 0.2
            position[2] += d_z
            return position

        def objective(image):
            value = imageSharpness(image)
            return value

        def new_value(data):
            # Fit a polynomial to the focused images and find optimal focus
            mask = ~np.isnan(data)
            poly = np.polyfit(data[0, mask[0]], data[1, mask[1]], 2)
            return -poly[1] / (2 * poly[0])  # -b/2a

        opt_info = {'idx': 2, 'MAX_STEP': 0.001, 'init': [-0.001, 0.000, 0.001]}
        return self.optimise(stop_event, opt_info, objective, apply_z_to_x_transformation, new_value)

    def optimiseReflectionAngle(self, stop_event):
        if stop_event.isSet():
            return

        # Calibrated transformations
        def apply_angle_transformation(d_z, position):
            position[3] += d_z
            return position

        def objective(image):
            image = cropToRectangle(image)
            num_255 = (image == 255).sum()
            value = imageSharpness(image)
            return num_255

        def new_value(data):
            xs = data[0, ~np.isnan(data[0, :])]
            ys = data[1, ~np.isnan(data[1, :])]
            x_new = xs[-1] - 0.003 * (ys[-2] - ys[-1])/(xs[-2] - xs[-1])
            return x_new

        opt_info = {'idx': 3, 'MAX_STEP': 0.005, 'init': [0.00, 0.005]}
        return self.optimise(stop_event, opt_info, objective, apply_angle_transformation, new_value)

    def startRobotTask(self):
        def continuousTask(stop_event_as_argument):
            if not self._imageInfo:
                raise ValueError("Image info should not be None, possibly no items.")
            for i in range(len(self._imageInfo)):
                try:
                    pickupTask(stop_event_as_argument)
                except Exception as e:
                    communicateError(e)
                    break

        def testPickup(stop_event_as_argument):
            self.Robot.pickUpObject(stop_event_as_argument, self._imageInfo[0])
            # Move to desired position from the Home position
            current_joints = self.Robot.getJointAngles()
            desired_change = [i * pi / 180 for i in [-50.0, -25.0, 25.0, 0, 180.0, 0]]
            current_joints = [a + b for a, b in zip(current_joints, desired_change)]
            self.Robot.moveJointsTo(stop_event_as_argument, current_joints, 'movej')
            self.Robot.moveJointsTo(stop_event_as_argument, self.Robot.JointAngleReadObject.copy(), 'movej')
            self.Robot.moveJointsTo(stop_event_as_argument, current_joints, 'movej')
            current_joints = [a - b for a, b in zip(current_joints, desired_change)]
            self.Robot.moveJointsTo(stop_event_as_argument, current_joints, 'movej')

            self.Robot.dropObject(stop_event_as_argument)

        def testPresentation(stop_event_as_argument):
            self.Robot.moveJointsTo(stop_event_as_argument, self.Robot.JointAngleReadObject.copy(), 'movej')

            self.switchActiveCamera(stop_event_as_argument)
            self.optimiseFocus(stop_event_as_argument)
            self.optimiseReflectionAngle(stop_event_as_argument)
            best_image = self.waitForNextAvailableImage(stop_event_as_argument)
            saveImage(best_image, stop_event_as_argument)
            self.switchActiveCamera(stop_event_as_argument)

            # Move back to initial position
            self.Robot.moveJointsTo(stop_event_as_argument, self.Robot.JointAngleReadObject.copy(), 'movej')

        def pickupTask(stop_event_as_argument):
            if not self._imageInfo:
                raise ValueError("Image info should not be None, possibly no items.")

            self.Robot.turnWhiteLampON(stop_event_as_argument)
            self.Robot.pickUpObject(stop_event_as_argument, self._imageInfo[0])

            # Move to desired position from the Home position
            current_joints = self.Robot.getJointAngles()
            desired_change = [i * pi / 180 for i in [-50.0, -25.0, 25.0, 0, 180.0, 0]]
            current_joints = [a + b for a, b in zip(current_joints, desired_change)]
            self.Robot.moveJointsTo(stop_event_as_argument, current_joints, 'movej')
            self.Robot.moveJointsTo(stop_event_as_argument, self.Robot.JointAngleReadObject.copy(), 'movej')

            self.switchActiveCamera(stop_event_as_argument)
            self.optimiseFocus(stop_event_as_argument)
            self.optimiseReflectionAngle(stop_event_as_argument)
            best_image = self.waitForNextAvailableImage(stop_event_as_argument)
            saveImage(best_image, stop_event_as_argument)
            self.switchActiveCamera(stop_event_as_argument)

            # Move back to initial position
            self.Robot.moveJointsTo(stop_event_as_argument, self.Robot.JointAngleReadObject.copy(), 'movej')

            # Move back to temporary position, and back home
            self.Robot.moveJointsTo(stop_event_as_argument, current_joints, 'movej')
            current_joints = [a - b for a, b in zip(current_joints, desired_change)]
            self.Robot.moveJointsTo(stop_event_as_argument, current_joints, 'movej')

            self.Robot.dropObject(stop_event_as_argument)

        self.Robot.giveTask(testPresentation)

    def stopRobotTask(self):
        self.Robot.halt()


if __name__ == '__main__':
    c = MainManager()
    time.sleep(2)
