import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import time

import RobotSocket
from Kinematics import ForwardKinematics

from PyQt5 import QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

matplotlib.use('Qt5Agg')


class ThreeDimCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        super(ThreeDimCanvas, self).__init__(fig)
        self.axes = fig.gca(projection='3d')  # generates 3D Axes object
        # self.axes.hold(False)

        self.axes.set_xlim3d(-0.5, 0.5)
        self.axes.set_ylim3d(-0.5, 0.5)
        self.axes.set_zlim3d(0, 1)

        initPos = np.zeros((6,))
        self.lines = self.axes.plot3D(initPos, initPos, initPos, 'black')[0]
        self.joint = self.axes.scatter3D(initPos, initPos, initPos, c='r')

    def updatePlot(self, positions):
        # self.axes.clear()
        X, Y, Z = positions
        self.lines.set_data_3d(X, Y, Z)
        #self.joint._offsets3d = (X, Y, Z)
        # print(X[2], Y[2], Z[2])
        # self.axes.clear()
        # self.axes.plot3D(X, Y, Z, 'black')
        self.draw_idle()


class RobotJointReader(QtCore.QThread):
    def __init__(self, read_joints, update_plot):
        super(RobotJointReader, self).__init__()
        self.readJoints = read_joints
        self.updatePlot = update_plot

    def run(self):
        while True:
            self.updatePlot(self.readJoints())


class Viewer(QtWidgets.QMainWindow):
    def __init__(self, robot):
        super(Viewer, self).__init__()
        self.setWindowTitle("Viewing the robot position")

        self.canvas = ThreeDimCanvas(self, width=6, height=4, dpi=50)
        self.jointReader = RobotJointReader(robot.jointPositions, self.canvas.updatePlot)
        self.jointReader.start()

        self.setCentralWidget(self.canvas)
        self.show()


class RobotRotationEmulator:
    # Emulate movements of the robot to use the viewer without a robot
    def __init__(self):
        super(RobotRotationEmulator, self).__init__()
        # Initial angles of the robot:
        self.angles = [0, -np.pi/2, 0, -np.pi/2, 0, 0]
        self.fk = ForwardKinematics()

    def step(self):
        self.angles[0] += 0.01

    def jointPositions(self):
        self.fk.forward(self.angles)
        pos = self.fk.positions()
        x, y, z = pos
        print(x)
        print(y)
        print(z)
        self.step()
        return pos

    def jointAngles(self):
        pass


def seeViewerAtWork(robot):
    app = QtWidgets.QApplication(sys.argv)
    w = Viewer(robot)
    sys.exit(app.exec_())


def seeViewerAtWorkWithEmulator():
    seeViewerAtWork(RobotRotationEmulator())


def seeViewerAtWorkWithRobot():
    seeViewerAtWork(RobotSocket.RobotClass())


if __name__ == '__main__':
    seeViewerAtWorkWithEmulator()



