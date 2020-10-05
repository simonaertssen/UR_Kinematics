import time

from Readers import ModBusReader, RobotChiefCommunicationOfficer
from Kinematics import ForwardKinematics


class Robot:
    def __init__(self):
        super(Robot, self).__init__()
        self.ModBusReader = ModBusReader()
        self.RobotCCO = RobotChiefCommunicationOfficer()

    def shutdownSafely(self):
        self.ModBusReader.shutdownSafely()
        self.RobotCCO.shutdownSafely()

    def send(self, message):
        self.RobotCCO.send(message)

    def getToolBitInfo(self):
        return self.ModBusReader.getToolInfo()

    def getToolPositionInfo(self):
        return self.RobotCCO.getToolInfo()

    def getJointAngles(self):
        return self.RobotCCO.getJointInfo()

    def getJointPositions(self):
        return ForwardKinematics(self.getJointAngles())

    def openGripper(self):  # Equals a tool bit of 0
        print('Open Gripper')
        self.send(b'set_digital_out(8, False)' + b"\n")
        self.waitForGripperToRead(0)
        print("Gripper Opened")

    def closeGripper(self):
        print('Close Gripper')
        self.send(b'set_digital_out(8, True)' + b"\n")
        self.waitForGripperToRead(1)
        print("Gripper Closed")

    def waitForGripperToRead(self, bit_value):
        while True:
            tool_bit, settled = self.getToolBitInfo()
            if tool_bit is bit_value and settled:
                break


if __name__ == '__main__':
    print('Testing the connectivity of the gripper')
    robot = Robot()
    for _ in range(5):
        time.sleep(2)
        robot.closeGripper()
        robot.openGripper()
