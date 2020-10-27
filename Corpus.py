import time
from RobotClass import Robot
from CameraManagement import TopCamera, DetailCamera
from threading import Thread


class MainManager(Thread):
    def __init__(self):
        super(MainManager, self).__init__(daemon=True)
        self.start()
        self.robot = None
        self.topCam = None
        self.detailCam = None

    def tryConnect(self):
        print('Starting connection process')
        self.robot = Robot()

        print('Starting connection process')

    def run(self):
        self.tryConnect()
        while True:
            1

    def checkComponentsAreConnected(self):
        return self.isRobotConnected() and self.isModBusConnected() and self.isTopCameraConnected() and self.isDetailCameraConnected()

    def isRobotConnected(self):
        try:
            return self.robot.RobotCCO.Connected
        except:
            return False

    def isModBusConnected(self):
        try:
            return self.robot.ModBusReader.Connected
        except:
            return False

    def isTopCameraConnected(self):
        try:
            return self.topCam.Connected
        except:
            return False

    def isDetailCameraConnected(self):
        try:
            return self.detailCam.Connected
        except:
            return False


if __name__ == '__main__':
    print('Before')
    c = MainManager()
    time.sleep(10)
    print(c.checkComponentsAreConnected())
    time.sleep(10)
    print('After')
