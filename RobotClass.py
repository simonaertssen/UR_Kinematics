import time
import sys

from threading import Thread, Event
from queue import Queue
# import winsound

from KinematicsModule.Kinematics import detectCollision, RPY2RotVec
from KinematicsLib.KinematicsModule import ForwardKinematics

from Readers import ModBusReader, RobotCCO


class Robot:
    def __init__(self):
        super(Robot, self).__init__()
        self.ModBusReader = None
        self.RobotCCO = None

        # Start these parts safely before anything else:
        ReturnErrorMessageQueue = Queue()
        def startAsync(constructor, error_queue):
            try:
                # print('starting {} async: type = '.format(constructor, type(constructor)))
                setattr(self, str(constructor), constructor())
            except Exception as e:
                # Startup of this part has failed and we need to shutdown all parts
                error_queue.put(e)

        parts = [ModBusReader, RobotCCO]
        startThreads = [Thread(target=startAsync, args=(partname,ReturnErrorMessageQueue,), name='{} startAsync'.format(partname)) for partname in parts]
        [x.start() for x in startThreads]
        [x.join() for x in startThreads]

        # Get first error, if any, and raise it to warn the instance and the parent
        if not ReturnErrorMessageQueue.empty():
            self.shutdownSafely()
            raise ReturnErrorMessageQueue.get()

        # Save some important positions as attributes:
        self.pidiv180 = 3.14159265359/180
        self.ToolPickUpHeight = 0.009
        self.ToolHoverHeight = 0.06

        self.JointAngleInit = [i * self.pidiv180 for i in [61.42, -93.00, 94.65, -91.59, -90.0, 0.0]]
        self.JointAngleBrickDrop = [i * self.pidiv180 for i in [87.28, -74.56, 113.86, -129.29, -89.91, -2.73]]

        self.ToolPositionBrickDrop = [0.08511, -0.51591, 0.04105, 0.00000, 0.00000, 0.00000]
        self.ToolPositionLightBox = [0.14912, -0.30970, 0.05, 0.000, 3.14159, 0.000]

        self.ToolPositionCollisionStart = [0.08838, -0.46649, 0.24701, -0.3335, 3.11, 0.0202]
        self.ToolPositionTestCollision = [0.08838, -0.76649, 0.24701, -0.3335, 3.11, 0.0202]

        self.Running = Event()
        self.initialise()
        self.Running.set()

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

        shutdownThreads = [Thread(target=shutdownAsync, args=(part,), name='{} shutdownSafely'.format(part)) for part in [self.ModBusReader, self.RobotCCO]]
        [x.start() for x in shutdownThreads]
        [x.join() for x in shutdownThreads]

    def send(self, message):
        self.RobotCCO.send(message)

    def receive(self):
        return self.RobotCCO.recv(self.RobotCCO.BufferLength)

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

    def stop(self):
        self.send(b'stop(5)')

    def halt(self):
        if self.Running.isSet():
            self.Running.clear()
        self.stop()

    def detectCollision(self):
        return detectCollision(self.getJointPositions())

    def moveTo(self, target_position, move, wait=True, check_collisions=True, p=True):
        """
        DESCRIPTION: Moves the robot to the target
        :param move: movej (find best move) or movel (move in a line)
        :param target_position: target joint angles (p=False) or tool position (given by p)
        :param wait: wait for the program to reach the required position (blocking or not)
        :param p: defines weather the target is a set of joint angles (p=False) or a tool position (p=True).
        :param check_collisions: check whether a collision occurs during the move
        """
        if p:
            current_position = self.getToolPosition
        else:
            current_position = self.getJointAngles

        command = str.encode("{}({}{}) \n".format(move, "p" if p is True else "", target_position))
        self.send(command)

        start_position = current_position()
        if wait:
            try:
                self.waitUntilTargetReached(current_position, target_position, check_collisions)
            except TimeoutError as e:  # Time ran out to test for object position
                print(e)
            except RuntimeError as e:  # Collision raises RuntimeError
                # self.set_IO_PORT(1, False)
                # time.sleep(0.1)
                self.stop()
                time.sleep(0.1)
                self.moveTo(start_position, "movel", wait=True, p=p, check_collisions=False)
            time.sleep(0.075)  # To let momentum fade away

    @staticmethod
    def spatialDifference(current_position, target_position):
        x1, y1, z1, _, _, _ = current_position
        x2, y2, z2, _, _, _ = target_position
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2) ** 0.5

    def waitUntilTargetReached(self, current_position, target_position, check_collisions):
        all_differences = 100 * [0]
        difference = [1000.0 for _ in target_position]
        start_time = time.time()

        totalDifferenceTolerance = 5e-3
        while sum(difference) >= totalDifferenceTolerance:
            difference = [abs(joint - pos) for joint, pos in zip(current_position(), target_position)]
            all_differences.pop(0)
            all_differences.append(sum(difference))
            if sum(all_differences) == 5e-3*100:
                print('Too long in one position ')
                break
            if time.time() - start_time > 15.0:
                raise TimeoutError('Movement took too long')
            if check_collisions and self.detectCollision():
                raise RuntimeError('Bumping in to stuff!')

    @staticmethod
    def waitForParallelTask(function, arguments=None, information=None):
        if information:
            print('Task received:', information)
        thread = Thread(target=function, args=[], daemon=True, name=information)
        thread.start()
        thread.join()

    def moveToolTo(self, target_position, move, wait=True, check_collisions=True):
        def moveToolToInThread():
            self.moveTo(target_position, move, wait=wait, check_collisions=check_collisions, p=True)
        self.waitForParallelTask(function=moveToolToInThread, arguments=None, information="Moving Tool Head")

    def moveJointsTo(self, target_position, move, wait=True, check_collisions=True):
        def moveJointsToInThread():
            self.moveTo(target_position, move, wait=wait, check_collisions=check_collisions, p=False)
        self.waitForParallelTask(function=moveJointsToInThread, arguments=None, information="Moving Joints")

    def goHome(self):
        self.moveJointsTo(self.JointAngleInit.copy(), "movej")

    def initialise(self):
        def initialiseInThread():
            currentJointPosition = self.getJointAngles()
            distanceFromAngleInit = sum([abs(i - j) for i, j in zip(currentJointPosition, self.JointAngleInit.copy())])
            currentToolPosition = self.getToolPosition()
            if self.isGripperOpen():
                if currentToolPosition[2] < 0.300:
                    targetToolPosition = currentToolPosition.copy()
                    targetToolPosition[2] = 0.300
                    self.moveToolTo(targetToolPosition, "movel", wait=True, check_collisions=False)
            else:
                if self.spatialDifference(currentToolPosition, self.ToolPositionBrickDrop) < 0.5:
                    if currentToolPosition[2] < 0.07:
                        targetToolPosition = currentToolPosition.copy()
                        targetToolPosition[2] = 0.07
                        self.moveToolTo(targetToolPosition, "movel", wait=True)
                else:
                    if distanceFromAngleInit > 0.05:
                        self.moveJointsTo(self.JointAngleInit.copy(), "movej", wait=True)

                self.moveJointsTo(self.JointAngleBrickDrop.copy(), "movej", wait=True)
                self.openGripper()
            if distanceFromAngleInit > 0.05:
                self.moveJointsTo(self.JointAngleInit.copy(), "movej", wait=True)
            print("Initialisation Done")
        self.waitForParallelTask(function=initialiseInThread, arguments=None, information="Initialising")

    def dropObject(self):
        self.moveJointsTo(self.JointAngleBrickDrop.copy(), "movej", wait=True)
        self.openGripper()

    def pickUpObject(self, object_position):
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
        self.moveToolTo(target_position, 'movel')
        # Go down and pickup the object
        target_position[2] = self.ToolPickUpHeight
        self.moveToolTo(target_position, 'movel')
        self.closeGripper()
        # Go down and pickup the object
        target_position[2] = self.ToolPickUpHeight
        self.moveToolTo(target_position, 'movel')

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
    # time.sleep(200)
