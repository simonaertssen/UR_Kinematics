import time
import threading

from Kinematics import ForwardKinematics
from Readers import ModBusReader, RobotChiefCommunicationOfficer


class Robot:
    def __init__(self):
        super(Robot, self).__init__()
        self.ModBusReader = ModBusReader()
        self.RobotCCO = RobotChiefCommunicationOfficer()

        # Save some important positions as attributes:
        pi180 = 3.14159265359/180
        self.jointAngleInit = [a * pi180 for a in [61.42, -93.00, 94.65, -91.59, -90.0, 0.0]]
        self.jointAngleBrickDrop = [a * pi180 for a in [87.28, -74.56, 113.86, -129.29, -89.91, -2.73]]

    def shutdownSafely(self):
        self.ModBusReader.shutdownSafely()
        self.RobotCCO.shutdownSafely()

    def send(self, message):
        self.RobotCCO.send(message)

    def getToolBitInfo(self):
        return self.ModBusReader.getToolBitInfo()

    def getToolPosition(self):
        return self.ModBusReader.getToolInfo()

    def getJointAngles(self):
        return self.ModBusReader.getJointAngles()

    def getJointPositions(self):
        return ForwardKinematics(self.getJointAngles())

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

    def moveTo(self, target_position, move, wait=True, p=True):
        """
        DESCRIPTION: Moves the robot to the target
        :param move: movej (find best move) or movel (move in a line)
        :param current_position: current joint angles (p=False) or tool position (given by p)
        :param target_position: target joint angles (p=False) or tool position (given by p)
        :param wait: wait for the program to reach the required position (blocking or not)
        :param p: defines weather the target is a set of joint angles (p=False) or a tool position (p=True).
        """
        if p:
            current_position = self.getJointAngles()
        else:
            current_position = self.getToolPosition()

        p = "p" if p is True else ""
        command = "{}({}{}) \n".format(move, p, target_position)
        print(command)

        # Send command
        self.send(str.encode(command))
        # Wait for the robot arm to reach the position
        if wait:
            self.waitUntilTargetReached(current_position, target_position)

    @staticmethod
    def waitUntilTargetReached(current_position, target_position):
        difference = [1000.0 for _ in target_position]
        totalDifferenceTolerance = 5e-3
        while sum(difference) >= totalDifferenceTolerance:
            difference = [abs(joint.value - pos) for joint, pos in zip(current_position, target_position)]
            time.sleep(0.001)

    @staticmethod
    def waitForParallelTask(function, arguments=None):
        thread = threading.Thread(target=function, args=[], daemon=True)
        thread.start()
        thread.join()

    def initialise(self):
        def initialiseInThread():
            # If the tool is lower than 350 mm, move the tool straight up:
            currentToolPosition = self.getToolPosition()
            if currentToolPosition[2] < 350:
                targetToolPosition = currentToolPosition.copy()
                targetToolPosition[2] = 350
                self.moveTo(currentToolPosition, targetToolPosition, "movel", wait=True, p=True)

            if not self.isGripperOpen():
                self.moveTo(currentToolPosition, targetToolPosition, "movel", wait=True, p=True)

            # self.moveJointsToAngle("movej", self.jointAngleBrickReady, wait=True, p=False)
            # self.moveJointsToAngle("movej", self.jointAngleBrickUp, wait=True, p=False)
            # self.moveJointsToAngle("movej", self.jointAngleBrickReady, wait=True, p=False)
            # self.moveJointsToAngle("movej", self.jointAngleInit, wait=True, p=False)
            print("Initialisation Done")

        self.waitForParallelTask(function=initialiseInThread(), arguments=None)

    def test(self):
        self.closeGripper()
        time.sleep(2)
        self.openGripper()
        time.sleep(2)


if __name__ == '__main__':
    print('Testing the connectivity of the gripper')
    robot = Robot()
    # time.sleep(100)

    # for _ in range(5):
    #     time.sleep(1)
    #     robot.closeGripper()
    #     robot.openGripper()
