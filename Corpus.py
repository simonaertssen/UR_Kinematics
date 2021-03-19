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
from KinematicsModule.Kinematics import RotVec2RPY  # Slow Python implementation
from KinematicsLib.cKinematics import toolPositionDifference, jointAngleDifference, spatialDifference


class MainManager:
    Robot = Robot
    TopCamera = TopCamera
    DetailCamera = DetailCamera
    _image = None
    _imageInfo = []
    currentObject = ()
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
        step_value = 10
        TARGET = 250.0
        while not stop_event.isSet() and step < MAX_STEPS:
            latest_image = cropToRectangle(self.waitForNextAvailableImage(stop_event))
            max_value  = latest_image.max()
            mean_value = latest_image.mean()
            print(max_value)
            if abs(max_value - TARGET) < 1.0 or mean_value < 150:
                break
            current_value = self.TopCamera.camera.ExposureTimeAbs.GetValue()
            new_value = current_value - (max_value - TARGET)*step_value
            print("New value =", latest_image.mean())
            if new_value < 10.0:  # Lowest exposure time from manufacturer
                break
            self.TopCamera.camera.ExposureTimeAbs.SetValue(new_value)
            sleep(0.01, stop_event)
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

            MAX_SAMPLES = 3
            samples = np.empty(MAX_SAMPLES)
            samples[:] = np.NAN

            # Set position, record an image and record the score
            self.Robot.moveToolTo(stop_event_as_argument, position, 'movel', velocity=0.1)
            sleep(0.25, stop_event_as_argument)  # Let vibrations dissipate
            try:
                for i in range(MAX_SAMPLES):
                    samples[i] = objective(self.waitForNextAvailableImage(stop_event_as_argument))
            except Exception as e:
                communicateError(e)
            return np.nanmean(samples)

        # Set loop parameters
        iteration = 1
        MAX_ITERATIONS = 30
        start_time = time.time()
        MAX_TIME = 60.0                          # 1 minute
        MAX_TOLERANCE = 0.0005                   # mm, half of the robot precision
        MAX_DEVIATION = info['MAX_DEVIATION']    # mm
        MAX_STEP = info['MAX_STEP']              # mm

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
            if np.isnan(d_pos) or np.isnan(current_position[idx]):
                # Nan value from the optimisation
                print(f"NAN: target = {target}, current_position[idx] = {current_position[idx]}")
                break
            d_pos_old = d_pos
            d_pos = (d_pos / abs(d_pos)) * min(abs(d_pos), MAX_STEP)
            print(f"Now = {round(current_position[idx], 4)}, new = {round(target, 4)}, d = {round(d_pos, 4)} instead of {round(d_pos_old, 4)}")

            current_position = transform_position(d_pos, current_position)
            data[:, iteration + 2] = np.array([current_position[idx], sample_objective(current_position, stop_event)])
            if abs(d_pos) < MAX_TOLERANCE:  # Because we cannot move closer than 1 mm
                # instead of measuring how close we are to  the optimal value of the objective
                # function (which runs across multiple orders of magnitude) we stop optimising
                # if the next step is less than half of our precision away
                break
            if sum(toolPositionDifference(original_position, current_position)) > MAX_DEVIATION:
                # If deviation is too large then move back to the beginning
                data[:, iteration + 2] = np.NAN
                self.Robot.moveToolTo(stop_event, original_position, 'movel')
                iteration -= 1
                continue
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
            return imageSharpness(image)

        def new_value(data):
            # Fit a polynomial to the focused images and find optimal focus
            mask = ~np.isnan(data)
            poly = np.polyfit(data[0, mask[0]], data[1, mask[1]], 2)
            return -poly[1] / (2 * poly[0])  # -b/2a is the top of a parabola

        # Distance: all numbers are in millimeter
        opt_info = {'idx': 2,                       # Index of the position we optimise on
                    'MAX_STEP': 0.004,              # Maximum change of the parameter per step
                    'MAX_DEVIATION': 0.020,         # Total maximum change wrt the initial position
                    'init': [-0.001, 0.000, 0.001]  # Initial samples
                    }
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
            return (image == 255).sum()

        def new_value(data):
            # Gradient method
            xs = data[0, ~np.isnan(data[0, :])]
            ys = data[1, ~np.isnan(data[1, :])]
            grad = 0.0
            try:
                grad = (ys[-2] - ys[-1])/(xs[-2] - xs[-1] + 1.0e-10)
            except IndexError as e:
                print(ys)
                communicateError(e)
            lr = 1.0e-7
            x_new = xs[-1] - lr * grad
            return x_new

        # Angle: all numbers are in milli rad
        opt_info = {'idx': 3,                # Index of the position we optimise on
                    'MAX_STEP': 0.01,       # Maximum change of the parameter per step
                    'MAX_DEVIATION': 0.030,  # Total maximum change wrt the initial position
                    'init': [0.00, 0.002]    # Initial samples
                    }
        return self.optimise(stop_event, opt_info, objective, apply_angle_transformation, new_value)

    def startRobotTask(self):
        def continuousTask(stop_event_as_argument):
            while not stop_event_as_argument.isSet():
                if not self._imageInfo:
                    raise ValueError("Image info should not be None, possibly no items.")
                try:
                    pickupTask(stop_event_as_argument)
                except Exception as e:
                    communicateError(e)
                    break

        def testPickup(stop_event_as_argument):
            self.Robot.pickUpObject(stop_event_as_argument, self._imageInfo[0])
            # self.Robot.moveToolTo(stop_event_as_argument, self.Robot.ToolPositionLightBox.copy(), 'movel')

        def testPresentation(stop_event_as_argument):
            joint_angle_read = self.Robot.JointAngleReadObject.copy()
            joint_angle_read[-1] -= np.pi / 2
            self.Robot.moveJointsTo(stop_event_as_argument, joint_angle_read, 'movej')

            self.switchActiveCamera(stop_event_as_argument)
            # self.optimiseExposure(stop_event_as_argument)
            # self.optimiseFocus(stop_event_as_argument)
            # self.optimiseReflectionAngle(stop_event_as_argument)

            resize_factor = 61.16 / 390.27  # irl length / image length
            # real_length = h * resize_factor
            real_length = 61.16
            PIECE_LENGTH = 3.0
            num_pieces = np.ceil(real_length / PIECE_LENGTH).astype(int)  # Cut up in pieces of 10 mm
            print("num_pieces = ", num_pieces)
            tool_position = self.Robot.getToolPosition()
            print("tool_position = ", tool_position)
            yaw, pitch, roll = RotVec2RPY(*tool_position[3:])  # Unpack into function input
            print(yaw, pitch, roll)

            # Get rotation matrices for the conversion:
            sin = np.sin
            cos = np.cos
            yawMatrix = np.array([
                [cos(yaw), -sin(yaw), 0],
                [sin(yaw), cos(yaw), 0],
                [0, 0, 1]])
            pitchMatrix = np.array([
                [cos(pitch), 0, sin(pitch)],
                [0, 1, 0],
                [-sin(pitch), 0, cos(pitch)]])
            rollMatrix = np.array([
                [1, 0, 0],
                [0, cos(roll), -sin(roll)],
                [0, sin(roll), cos(roll)]])

            print(-real_length/2.0*1.0e-3)
            CALIBRATION = 0.005
            new_pos = np.array([-CALIBRATION, 0, real_length/2.0*1.0e-3]).dot(yawMatrix.dot(pitchMatrix.dot(rollMatrix)))  # Calibrated

            tool_position[0] += new_pos[0]
            tool_position[1] += new_pos[1]
            tool_position[2] += new_pos[2]
            for i in range(num_pieces):
                self.Robot.moveToolTo(stop_event_as_argument, tool_position, 'movel', velocity=0.1)
                sleep(0.35, stop_event_as_argument)

                best_image = self.waitForNextAvailableImage(stop_event_as_argument)
                saveImage(best_image, stop_event_as_argument)

                new_pos = np.array([CALIBRATION*2.0/num_pieces, 0, -PIECE_LENGTH * 1.0e-3]).dot(yawMatrix.dot(pitchMatrix.dot(rollMatrix)))
                tool_position[0] += new_pos[0]
                tool_position[1] += new_pos[1]
                tool_position[2] += new_pos[2]

            best_image = self.waitForNextAvailableImage(stop_event_as_argument)
            saveImage(best_image, stop_event_as_argument)
            self.switchActiveCamera(stop_event_as_argument)

        def pickupTask(stop_event_as_argument):
            if not self._imageInfo:
                print("self._imageInfo =", self._imageInfo)
                raise ValueError("Image info should not be None, possibly no items.")

            self.Robot.turnWhiteLampON(stop_event_as_argument)
            self.currentObject = self._imageInfo[0]
            X, Y, w, h, angle = self.Robot.pickUpObject(stop_event_as_argument, self.currentObject)
            # small lego brick: 76.47 x 103.75 pixels
            # big lego brick:   76.38 x 204.11 pixels
            # big alum piece:  131.27 x 389.68 pixels
            line_sweep = max(w, h) > 175.0  # Then we need to rotate the object and do a line sweep

            # Move to desired position from the Home position
            current_joints = self.Robot.getJointAngles()
            desired_change = [i * pi / 180 for i in [-50.0, -25.0, 25.0, 0, 180.0, 0]]
            current_joints = [a + b for a, b in zip(current_joints, desired_change)]
            self.Robot.moveJointsTo(stop_event_as_argument, current_joints, 'movej')

            joint_angle_read = self.Robot.JointAngleReadObject.copy()
            if line_sweep:
                joint_angle_read[-1] -= np.pi/2
            self.Robot.moveJointsTo(stop_event_as_argument, joint_angle_read, 'movej')

            self.switchActiveCamera(stop_event_as_argument)
            self.optimiseExposure(stop_event_as_argument)
            self.optimiseFocus(stop_event_as_argument)
            self.optimiseReflectionAngle(stop_event_as_argument)

            if line_sweep:
                resize_factor = 61.16 / 390.27  # irl length / image length
                real_length = h * resize_factor
                PIECE_LENGTH_SUGGESTION = 3.0  # millimeter
                num_pieces = np.ceil(real_length / PIECE_LENGTH_SUGGESTION).astype(int)  # Cut up in pieces of 10 mm
                PIECE_LENGTH = real_length/num_pieces

                real_length -= 3*PIECE_LENGTH  # Remove two pieces from the object
                num_pieces -= 3

                tool_position = self.Robot.getToolPosition()
                yaw, pitch, roll = RotVec2RPY(*tool_position[3:])  # Unpack into function input

                # Get rotation matrices for the conversion:
                sin = np.sin
                cos = np.cos
                yawMatrix = np.array([
                    [cos(yaw), -sin(yaw), 0],
                    [sin(yaw), cos(yaw), 0],
                    [0, 0, 1]])
                pitchMatrix = np.array([
                    [cos(pitch), 0, sin(pitch)],
                    [0, 1, 0],
                    [-sin(pitch), 0, cos(pitch)]])
                rollMatrix = np.array([
                    [1, 0, 0],
                    [0, cos(roll), -sin(roll)],
                    [0, sin(roll), cos(roll)]])

                # Move to one of the ends:
                CALIBRATION = 0.005  # millimeter
                new_pos = np.array([-CALIBRATION, 0, (real_length - PIECE_LENGTH) / 2.0 * 1.0e-3]).dot(yawMatrix.dot(pitchMatrix.dot(rollMatrix)))  # Calibrated
                tool_position[0] += new_pos[0]
                tool_position[1] += new_pos[1]
                tool_position[2] += new_pos[2]
                self.Robot.moveToolTo(stop_event_as_argument, tool_position, 'movel', velocity=0.1)
                sleep(0.35, stop_event_as_argument)

                for i in range(num_pieces):
                    new_pos = np.array([CALIBRATION * 2.0 / num_pieces, 0, -PIECE_LENGTH * 1.0e-3]).dot(yawMatrix.dot(pitchMatrix.dot(rollMatrix)))
                    tool_position[0] += new_pos[0]
                    tool_position[1] += new_pos[1]
                    tool_position[2] += new_pos[2]
                    self.Robot.moveToolTo(stop_event_as_argument, tool_position, 'movel', velocity=0.1)
                    sleep(0.35, stop_event_as_argument)
                    best_image = self.waitForNextAvailableImage(stop_event_as_argument)
                    saveImage(best_image, stop_event_as_argument)

            best_image = self.waitForNextAvailableImage(stop_event_as_argument)
            saveImage(best_image, stop_event_as_argument)
            self.switchActiveCamera(stop_event_as_argument)

            # Move back to initial position
            self.Robot.moveJointsTo(stop_event_as_argument, joint_angle_read, 'movej')

            # Move back to temporary position, and back home
            self.Robot.moveJointsTo(stop_event_as_argument, current_joints, 'movej')
            current_joints = [a - b for a, b in zip(current_joints, desired_change)]
            self.Robot.moveJointsTo(stop_event_as_argument, current_joints, 'movej')

            self.Robot.dropObject(stop_event_as_argument)

        self.Robot.giveTask(pickupTask)

    def stopRobotTask(self):
        self.Robot.halt()


if __name__ == '__main__':
    c = MainManager()
    time.sleep(2)
