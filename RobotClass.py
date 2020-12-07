import time
from threading import Thread
import winsound

from Kinematics import detectCollision
from KinematicsLib.KinematicsModule import ForwardKinematics

from Readers import ModBusReader, RobotChiefCommunicationOfficer


class Robot:
    def __init__(self):
        super(Robot, self).__init__()
        self.ModBusReader = None
        self.RobotCCO = None

        def startAsync(attribute, constructor):
            setattr(self, attribute, constructor())

        startThreads = [Thread(target=startAsync, args=('ModBusReader', ModBusReader,)),
                        Thread(target=startAsync, args=('RobotCCO', RobotChiefCommunicationOfficer,))]
        [x.start() for x in startThreads]
        [x.join() for x in startThreads]

        def testConnection(obj):
            if obj is None:
                raise ConnectionError(obj, "is not connected.")
        [testConnection(x) for x in [self.ModBusReader, self.RobotCCO]]

        # Save some important positions as attributes:
        pi180 = 3.14159265359/180
        self.JointAngleInit = [i * pi180 for i in [61.42, -93.00, 94.65, -91.59, -90.0, 0.0]]
        self.JointAngleBrickDrop = [i * pi180 for i in [87.28, -74.56, 113.86, -129.29, -89.91, -2.73]]
        self.ToolPositionBrickDrop = [0.08511, -0.51591, 0.04105, 0.00000, 0.00000, 0.00000]
        self.ToolPositionLightBox = [0.14882, -0.30662, 0.0944, 0.00000, 0.00000, 0.00000]

        self.ToolPositionCollisionStart = [0.08838, -0.46649, 0.24701, -0.3335, 3.11, 0.0202]
        self.ToolPositionTestCollision = [0.08838, -0.76649, 0.24701, -0.3335, 3.11, 0.0202]

        self.initialise()

    def shutdownSafely(self):
        self.initialise()
        shutdownThreads = [Thread(target=self.ModBusReader.shutdownSafely), Thread(target=self.RobotCCO.shutdownSafely)]
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
            except RuntimeError as e:
                self.set_IO_PORT(1, False)
                time.sleep(0.1)
                self.moveTo(start_position, "movel", wait=True, p=p, check_collisions=False)
            time.sleep(0.075)  # To let momentum fade away

    def moveToolTo(self, target_position, move, wait=True, check_collisions=True):
        self.moveTo(target_position, move, wait=wait, check_collisions=check_collisions, p=True)

    def moveJointsTo(self, target_position, move, wait=True, check_collisions=True):
        self.moveTo(target_position, move, wait=wait, check_collisions=check_collisions, p=False)

    @staticmethod
    def spatialDifference(current_position, target_position):
        x1, y1, z1, _, _, _ = current_position
        x2, y2, z2, _, _, _ = target_position
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2) ** 0.5

    def waitUntilTargetReached(self, current_position, target_position, check_collisions):
        difference = [1000.0 for _ in target_position]
        totalDifferenceTolerance = 5e-3
        while sum(difference) >= totalDifferenceTolerance:
            difference = [abs(joint - pos) for joint, pos in zip(current_position(), target_position)]
            if check_collisions and self.detectCollision():
                raise RuntimeError('Bumping in to stuff!')

    @staticmethod
    def waitForParallelTask(function, arguments=None, information=None):
        if information:
            print('Task received:', information)
        thread = Thread(target=function, args=[], daemon=True)
        thread.start()
        thread.join()

    def initialise(self):
        def initialiseInThread():
            currentJointPosition = self.getJointAngles()
            distanceFromAngleInit = sum([abs(i - j) for i, j in zip(currentJointPosition, self.JointAngleInit)])
            currentToolPosition = self.getToolPosition()
            if self.isGripperOpen():
                if currentToolPosition[2] < 0.300:
                    targetToolPosition = currentToolPosition.copy()
                    targetToolPosition[2] = 0.300
                    self.moveToolTo(targetToolPosition, "movel", wait=True)
            else:
                if self.spatialDifference(currentToolPosition, self.ToolPositionBrickDrop) < 0.3:
                    if currentToolPosition[2] < 0.07:
                        targetToolPosition = currentToolPosition.copy()
                        targetToolPosition[2] = 0.07
                        self.moveToolTo(targetToolPosition, "movel", wait=True)
                else:
                    if distanceFromAngleInit > 0.05:
                        self.moveJointsTo(self.JointAngleInit, "movej", wait=True)

                self.moveJointsTo(self.JointAngleBrickDrop, "movej", wait=True)
                self.openGripper()
            if distanceFromAngleInit > 0.05:
                self.moveJointsTo(self.JointAngleInit, "movej", wait=True)
            print("Initialisation Done")
        self.waitForParallelTask(function=initialiseInThread, arguments=None, information="Initialising")

    def test(self):
        print('Testing the gripper')
        self.closeGripper()
        self.openGripper()

    def testCollision(self):
        self.moveToolTo(robot.ToolPositionCollisionStart, "movel", wait=True, check_collisions=False)
        self.moveToolTo(robot.ToolPositionTestCollision, "movel", wait=True)
        time.sleep(1)
        self.moveToolTo(robot.ToolPositionCollisionStart, "movel", wait=True)

    @staticmethod
    def beep():
        winsound.PlaySound("SystemHand", winsound.SND_NOSTOP)


if __name__ == '__main__':
    robot = Robot()
    # robot.moveToolTo(robot.ToolPositionLightBox, "movel", wait=False)
    robot.beep()
    # time.sleep(200)

