import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import queue

import RobotSocket

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
        # self.axes = plt.axes(projection='3d')
        self.axes.set_xlim3d(-0.5, 0.5)
        self.axes.set_ylim3d(-0.5, 0.5)
        self.axes.set_zlim3d(0, 1)

        initPos = np.zeros((6,))
        self.lines = self.axes.plot3D(initPos, initPos, initPos, 'black')[0]
        self.joint = self.axes.scatter3D(initPos, initPos, initPos, c='r')

    def updatePlot(self, positions):
        X, Y, Z = positions
        self.lines.set_data_3d(X, Y, Z)
        self.joint._offsets3d = (X, Y, Z)
        # print(X[2], Y[2], Z[2])


class RobotJointReader(QtCore.QThread):
    def __init__(self, read_joints, update_plot, print_me=None):
        super(RobotJointReader, self).__init__()
        self.readJoints = read_joints
        self.updatePlot = update_plot
        self.printMe = print_me

    def run(self):
        while True:
            # print(self.printMe())
            self.updatePlot(self.readJoints())


class Viewer(QtWidgets.QMainWindow):
    def __init__(self, robot):
        super(Viewer, self).__init__()
        self.setWindowTitle("Viewing the robot position")

        self.canvas = ThreeDimCanvas(self, width=6, height=4, dpi=100)
        readJoints = getattr(robot, 'jointPositions')
        updatePlot = getattr(self.canvas, 'updatePlot')
        printMe = getattr(robot, 'jointAngles')
        self.updateThread = RobotJointReader(readJoints, updatePlot, printMe)
        self.updateThread.start()

        self.setCentralWidget(self.canvas)
        self.show()


Robot = RobotSocket.RobotClass()
app = QtWidgets.QApplication(sys.argv)
w = Viewer(Robot)
sys.exit(app.exec_())

