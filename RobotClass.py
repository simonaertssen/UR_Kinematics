import time
import sys

from threading import Thread, Event
from queue import Queue
# import winsound

from KinematicsModule.Kinematics import RPY2RotVec  # Slow Python implementation
from KinematicsLib.KinematicsModule import ForwardKinematics, detectCollision  # Fast C and Cython implementation

from Readers import ModBusReader, RobotCCO


class Robot:
    """
    Class used to represent the UR5 robot, which consists of the ModBusReader to
    listen to and aqcuire all the robot information, and the RobotCCO, to which
    we can send commands. This class implements the move commands as a blocking
    thread, so that we can wait for the target position to be reached while
    using full concurrency.

    Attributes:
    -------
    ModBusReader : ModBusReader
        The instance of the ModBusReader class to listen to parameters like the
        current joint angles or tool position.
    RobotCCO : RobotCCO
        The instance of the RobotCCO class to send commands via URscript.

    pidiv180 : float
        The value of pi/180 for conversion between angles in degrees to radians.
    ToolHoverHeight : float
        The height of the tool before picking up an object.
    ToolPickUpHeight : float
        The height of the tool when picking up an object.
    JointAngleInit : list of angles in radians
        The joint angles required for the initial position.
    JointAngleBrickDrop: list of angles in radians
        The joint angles required for an item to be dropped in the bucket.
    ToolPositionBrickDrop : list of tool positions in mm and angles in radians
        The tool position required for an item to be dropped in the bucket.
    ToolPositionLightBox : list of tool positions in mm and angles in radians
        The tool position of the tool positioned at the lower left corner of the
        light box, with the tool aligned vertically, facing down.
    """

    def __init__(self):
        super(Robot, self).__init__()
        self.ModBusReader = None
        self.RobotCCO = None
        self.tryConnect()

        # Save some important positions as attributes:
        self.pidiv180 = 3.14159265359/180
        self.ToolHoverHeight = 0.06
        self.ToolPickUpHeight = 0.009

        self.JointAngleInit = [i * self.pidiv180 for i in [61.42, -93.00, 94.65, -91.59, -90.0, 0.0]]
        self.JointAngleBrickDrop = [i * self.pidiv180 for i in [87.28, -74.56, 113.86, -129.29, -89.91, -2.73]]

        self.ToolPositionBrickDrop = [0.08511, -0.51591, 0.04105, 0.00000, 0.00000, 0.00000]
        self.ToolPositionLightBox = [0.14912, -0.30970, 0.05, 0.000, 3.14159, 0.000]

        self.StopEvent = Event()
        self.initialise()

    def tryConnect(self):
        # Start these parts safely before anything else:
        ReturnErrorMessageQueue = Queue()
        def startAsync(error_queue, constructor):
            try:
                setattr(self, str(constructor), constructor())
            except Exception as e:
                # Startup of this part has failed and we need to shutdown all parts
                error_queue.put(e)

        startThreads = [Thread(target=startAsync, args=[ReturnErrorMessageQueue, partname], name='{} startAsync'.format(partname)) for partname in [ModBusReader, RobotCCO]]
        [x.start() for x in startThreads]
        [x.join() for x in startThreads]

        # Get first error, if any, and raise it to warn the instance and the parent
        if not ReturnErrorMessageQueue.empty():
            self.shutdownSafely()
            raise ReturnErrorMessageQueue.get()

    def shutdownSafely(self):
        # self.initialise()  # Only initialise if we want to reset the robot entirely
        if self.RobotCCO is not None and self.RobotCCO.isConnected():
            self.halt()

        def shutdownAsync(object):
            if object is None:
                return
            try:
                object.shutdownSafely()
            except Exception as e:
                raise SystemExit("Safe shutdown failed due to {}. Aborting".format(e))

        shutdownThreads = [Thread(target=shutdownAsync, args=[part], name='{} shutdownSafely'.format(part)) for part in [self.ModBusReader, self.RobotCCO]]
        [x.start() for x in shutdownThreads]
        [x.join() for x in shutdownThreads]

    def send(self, message):
        try:
            self.RobotCCO.send(message)
        except Exception as e:
            print("Sending failed due to {}".format(e))

    def stop(self):
        self.send(b'stop(5)')

    def halt(self):
        if not self.StopEvent.isSet():
            self.StopEvent.set()
        self.stop()
        self.StopEvent.clear()

    def receive(self):
        try:
            return self.RobotCCO.recv(self.RobotCCO.BufferLength)
        except Exception as e:
            print("Receiving failed due to {}".format(e))
            return b''  # An empty bytes object

    def getToolBitInfo(self):
        return self.ModBusReader.getToolBitInfo()

    def getToolPosition(self):
        return self.ModBusReader.getToolPosition()

    def getJointAngles(self):
        return self.ModBusReader.getJointAngles()

    def getJointPositions(self):
        X, Y, Z, _, _, _ = self.getToolPosition()
        return ForwardKinematics(tuple(self.getJointAngles()), (X, Y, Z))

    def set_IO_PORT(self, port_number, on):
        """
        DESCRIPTION: Set robot I/O port described with by 'port_number' and the status of the port described by 'on'.
        :param port_number: integer
        :param on: boolean
        :return: None
        """
        if not (0 <= port_number <= 8) or not isinstance(port_number, int):
            print("set_IO_PORT: port number value error")
            return
        if not isinstance(on, bool):
            print("set_IO_PORT: boolean value error")
            return
        command = str.encode('set_digital_out({},{}) \n'.format(port_number, on))
        self.send(command)

    def turnWhiteLampON(self):
        self.set_IO_PORT(0, True)
        time.sleep(2)

    def turnWhiteLampOFF(self):
        self.set_IO_PORT(0, False)

    def isGripperOpen(self):
        tool_bit, _ = self.getToolBitInfo()
        return tool_bit == 0

    def openGripper(self):  # Equals a tool bit of 0
        print('Open Gripper')
        if self.isGripperOpen():
            print('Gripper is already open')
            return
        self.send(b'set_digital_out(8, False)' + b"\n")
        self.waitForGripperToRead(0)
        print("Gripper Opened")

    def closeGripper(self):  # Equals a tool bit of 1
        print('Close Gripper')
        if not self.isGripperOpen():
            print('Gripper is already closed')
            return
        self.send(b'set_digital_out(8, True)' + b"\n")
        self.waitForGripperToRead(1)
        print("Gripper Closed")

    def waitForGripperToRead(self, bit_value):
        while True:
            tool_bit, settled = self.getToolBitInfo()
            if tool_bit is bit_value and settled:
                break

    def detectCollision(self):
        return detectCollision(self.getJointPositions())

    def moveTo(self, target_position, move, stop_event, p=True, wait=True, check_collisions=True):
        """
        DESCRIPTION: Moves the robot to the target.
        :param move: movej (find best move) or movel (move in a line).
        :param target_position: target joint angles (p=False) or tool position (given by p).
        :param wait: wait for the program to reach the required position (blocking or not).
        :param p: defines weather the target is a set of joint angles (p=False) or a tool position (p=True).
        :param check_collisions: check whether a collision occurs during the move.
        :param stop_event: a threading event designed to halt the execution of a thread if necessary.
        """
        if stop_event.isSet():
            return

        if p:
            current_position = self.getToolPosition
        else:
            current_position = self.getJointAngles

        command = str.encode("{}({}{}) \n".format(move, "p" if p is True else "", target_position))
        self.send(command)

        start_position = current_position()
        if wait:
            try:
                self.waitUntilTargetReached(current_position, target_position, check_collisions, stop_event)
            except TimeoutError as e:  # Time ran out to test for object position
                print(e)
            except RuntimeError as e:  # Collision raises RuntimeError, so move to startpoition
                self.stop()
                time.sleep(0.1)
                self.moveTo(start_position, "movel", stop_event, wait=True, p=p, check_collisions=False)
            finally:
                time.sleep(0.075)  # To let momentum fade away

    @staticmethod
    def spatialDifference(current_position, target_position):
        x1, y1, z1, _, _, _ = current_position
        x2, y2, z2, _, _, _ = target_position
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2) ** 0.5

    def waitUntilTargetReached(self, current_position, target_position, check_collisions, stop_event):
        difference = [1000.0 for _ in target_position]
        start_time = time.time()

        totalDifferenceTolerance = 5e-3
        while not stop_event.isSet() and sum(difference) >= totalDifferenceTolerance:
            difference = [abs(joint - pos) for joint, pos in zip(current_position(), target_position)]
            if time.time() - start_time > 15.0:
                raise TimeoutError('Movement took too long')
            if check_collisions and self.detectCollision():
                raise RuntimeError('Bumping in to stuff!')

    def waitForParallelTask(function, arguments=None, information=None):
        if information:
            print('Task received:', information)
        # The function is a handle that has all the necessary arguments already,
        # only the event needs to be passed when the robot needs to be stopped.
        # But leave possibility to extend with more args.
        threadargs = [self.StopEvent]
        if arguments is not None:
            threadargs.extend(arguments)
        thread = Thread(target=function, args=threadargs, daemon=True, name=information)
        thread.start()
        thread.join()
        if self.StopEvent.isSet():
            self.StopEvent.clear()

    def wrapInThread(function_handle, stop_event):
        if stop_event:
            function_handle(stop_event)
        else:
            self.waitForParallelTask(function=function_handle, arguments=None, information=str(function_handle))

    def moveToolTo(self, target_position, move, wait=True, check_collisions=True, stop_event=None):
        # If the stop_event is given, then a thread is calling this function.
        # If it is not given, then we want to start a thread.
        # This avoids having two different funcions with two different but similar names.
        def moveToolHandle(stop_event_as_argument):
            self.moveTo(target_position, move, stop_event_as_argument, wait=wait, p=True, check_collisions=check_collisions)
        self.wrapInThread(moveToolHandle, stop_event)

    def moveJointsTo(self, target_position, move, wait=True, check_collisions=True, stop_event=None):
        # If the stop_event is given, then a thread is calling this function.
        # If it is not given, then we want to start a thread.
        # This avoids having two different funcions with two different but similar names.
        def moveJointsHandle(stop_event_as_argument):
            self.moveTo(target_position, move, stop_event_as_argument, wait=wait, p=False, check_collisions=check_collisions)
        self.wrapInThread(moveJointsHandle, stop_event)

    def goHome(self, stop_event=None):
        def goHomeHandle(stop_event_as_argument):
            self.moveJointsTo(self.JointAngleInit.copy(), "movej", stop_event=stop_event_as_argument)
        self.wrapInThread(goHomeHandle, stop_event)

    def dropObject(self, stop_event=None):
        def dropObjectHandle(stop_event_as_argument):
            self.moveJointsTo(self.JointAngleBrickDrop.copy(), "movej", stop_event=stop_event_as_argument)
            self.openGripper()
        self.wrapInThread(dropObjectHandle, stop_event)

    def pickUpObject(self, object_position, stop_event=None):
        def pickUpObjectHandle(stop_event_as_argument):
            LIGHTBOX_LENGTH = 0.250  # m
            LIGHTBOX_WIDTH = 0.176  # m
            print("object_position: ", object_position)
            if len(object_position) < 1:
                return
            X, Y, angle = object_position

            # Adjust position to the object
            target_position = self.ToolPositionLightBox.copy()
            target_position[0] += X * LIGHTBOX_WIDTH   # adjust X position
            target_position[1] -= Y * LIGHTBOX_LENGTH  # adjust Y position
            target_position[2] = self.ToolHoverHeight
            # Get right orientation from Rodrigues conversion
            a, b, c = RPY2RotVec(0, 3.1415926, -angle)
            target_position[3] = a
            target_position[4] = b
            target_position[5] = c
            self.moveToolTo(target_position, 'movel', stop_event=stop_event_as_argument)
            # Go down and pickup the object
            target_position[2] = self.ToolPickUpHeight
            self.moveToolTo(target_position, 'movel', stop_event=stop_event_as_argument)
            self.closeGripper()
            # Go back up
            target_position[2] = self.ToolPickUpHeight
            self.moveToolTo(target_position, 'movel', stop_event=stop_event_as_argument)
        self.wrapInThread(pickUpObjectHandle, stop_event)


    def initialise(self):
        def initialiseInThread(stop_event_as_argument):
            currentJointPosition = self.getJointAngles()
            distanceFromAngleInit = sum([abs(i - j) for i, j in zip(currentJointPosition, self.JointAngleInit.copy())])
            currentToolPosition = self.getToolPosition()
            if self.isGripperOpen():
                if currentToolPosition[2] < 0.300:
                    targetToolPosition = currentToolPosition.copy()
                    targetToolPosition[2] = 0.300
                    # Move towards first location, don't check collisions
                    # because we might start from a bad position.
                    self.moveToolTo(targetToolPosition, "movel", check_collisions=False, stop_event=stop_event)
            else:
                if self.spatialDifference(currentToolPosition, self.ToolPositionBrickDrop) < 0.5:
                    if currentToolPosition[2] < 0.07:
                        targetToolPosition = currentToolPosition.copy()
                        targetToolPosition[2] = 0.07
                        self.moveToolTo(targetToolPosition, "movel", stop_event=stop_event_as_argument)
                else:
                    if distanceFromAngleInit > 0.05:
                        self.goHome(stop_event=stop_event_as_argument)

                self.dropObject(stop_event=stop_event_as_argument)
            if distanceFromAngleInit > 0.05:
                self.goHome(stop_event=stop_event_as_argument)
            print("Initialisation Done")
        self.waitForParallelTask(function=initialiseInThread, arguments=None, information="Initialising")

    def test(self):
        print('Testing the gripper')
        self.closeGripper()
        self.openGripper()

    @staticmethod
    def beep():
        # winsound.PlaySound("SystemHand", winsound.SND_NOSTOP)
        pass


if __name__ == '__main__':
    robot = Robot()
    robot.moveToolTo(robot.ToolPositionLightBox, "movel", wait=False)
    robot.beep()
