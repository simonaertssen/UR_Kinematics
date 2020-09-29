import sys
import matplotlib
import numpy as np
import time

import RobotSocket
from Kinematics import ForwardKinematics

from PyQt5 import QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

matplotlib.use('Qt5Agg')


def printwrapper(function):
    def inner(*args, **kwargs):
        print("Calling", function.__name__)
        function(*args, **kwargs)
        print(function.__name__, "is done.")
    return inner


class ThreeDimCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        super(ThreeDimCanvas, self).__init__(fig)
        self.axes = fig.gca(projection='3d')  # generates 3D Axes object

        self.axes.set_xlim3d(-0.5, 0.5)
        self.axes.set_ylim3d(-0.5, 0.5)
        self.axes.set_zlim3d(0, 1)

        initPos = np.zeros((6,))
        self.arms   = self.axes.plot3D(initPos, initPos, initPos, 'black')[0]
        self.joints = self.axes.scatter3D(initPos, initPos, initPos, c='r')

    @printwrapper
    def updatePlot(self, positions):
        # self.axes.clear()
        X, Y, Z = positions
        self.joints._offsets3d = (X, Y, Z)
        self.arms.set_data_3d(X, Y, Z)
        self.draw_idle()


class RobotJointReader(QtCore.QThread):
    def __init__(self, read_joints, update_plot, print_me=None):
        super(RobotJointReader, self).__init__()
        self.readJoints = read_joints
        self.updatePlot = update_plot
        self.printMe = print_me

    @printwrapper
    def run(self):
        while True:
            self.updatePlot(self.readJoints())
            if self.printMe is not None:
                print([value*180/np.pi for value in self.printMe])


class Viewer(QtWidgets.QMainWindow):
    def __init__(self, robot):
        super(Viewer, self).__init__()
        self.setWindowTitle("Viewing the robot position")
        self.resize(1000, 500)

        self.shutdownRobot = robot.shutdownSafely

        self.canvas = ThreeDimCanvas(self, width=6, height=6, dpi=50)
        self.setCentralWidget(self.canvas)

        self.jointReader = RobotJointReader(robot.jointPositions, self.canvas.updatePlot, robot.jointAngles)
        self.jointReader.start()
        time.sleep(0.1)

        self.show()

    @printwrapper
    def closeEvent(self, event):
        self.shutdownRobot()
        self.close()


class RobotRotationEmulator:
    # Emulate movements of the robot to use the viewer without a robot
    def __init__(self):
        super(RobotRotationEmulator, self).__init__()
        # Initial angles of the robot:
        self.angles = [0, -np.pi/2, 0, -np.pi/2, 0, 0]

    @printwrapper
    def step(self):
        self.angles[0] += 0.001

    @printwrapper
    def jointPositions(self):
        pos = ForwardKinematics(self.angles)
        self.step()
        return pos

    @printwrapper
    def shutdownSafely(self):
        pass

    @printwrapper
    def jointAngles(self):
        return self.angles

@printwrapper
def seeViewerAtWork(robot):
    app = QtWidgets.QApplication(sys.argv)
    w = Viewer(robot)
    sys.exit(app.exec_())


def seeViewerAtWorkWithEmulator():
    r = RobotRotationEmulator()
    seeViewerAtWork(r)


def seeViewerAtWorkWithRobot():
    r = RobotSocket.Robot()
    seeViewerAtWork(r)


if __name__ == '__main__':
    seeViewerAtWorkWithEmulator()



