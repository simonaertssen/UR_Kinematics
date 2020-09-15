import os
import cv2
import sys
from PyQt5 import QtCore, QtWidgets
import Image_module as Im
from PyQt5.QtGui import QIcon, QPixmap, QImage,  QIntValidator
from PyQt5.QtCore import pyqtSignal, Qt, QThreadPool, QRunnable, pyqtSlot, QThread
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel,
                             QSpinBox, QComboBox, QGridLayout, QVBoxLayout,
                             QSplitter, QPushButton, QLineEdit, QRadioButton,  QCheckBox)
import Object_learning_GUI  as Ol_gui
import Robot_lego_system as RLS
import Robot_module as Rm
import math
from sip import setapi
from win32api import GetSystemMetrics
import time
from matplotlib import pyplot as plt
setapi("QVariant", 2)
setapi("QString", 2)


rootDir = r"C:/Prj/Robopick/Depot.svn/"
rootDir = r"C:/Users/simon/Documents/JLI Projects/ROBOPICK (Mads L)/Current working version (tortoiseSVN)/depot.svn/"

#Robot plade calibration




#Threads
class Worker(QThread):
    '''
    Worker thread

    Inherits from QRunnable to handler worker thread setup, signals and wrap-up.

    :param callback: The function callback to run on this worker thread. Supplied args and
                     kwargs will be passed through to the runner.
    :type callback: function
    :param args: Arguments to pass to the callback function
    :param kwargs: Keywords to pass to the callback function

    '''
    # Thread for running the robot independently of the main window.

    signal_robot_done = pyqtSignal()

    def __init__(self, fn, fn_get_data, robo_info):
        QtCore.QThread.__init__(self)
        # Store constructor arguments (re-used for processing)
        self.reset = False
        self.fn = fn
        self.get_data = fn_get_data
        self.robo_info = robo_info
        self.next_robo_info = robo_info
        print("Establishing robot connection")
        self.robo_connection = Rm.robot_init( [61.42 * math.pi / 180, -93 * math.pi / 180, 94.65 * math.pi / 180, -91.59 * math.pi / 180, -90 * math.pi / 180, 0 * math.pi / 180])
        print("Robot connection established")

    @pyqtSlot()
    def run(self):
        while(1):
            if self.next_robo_info:
                self.robo_info = self.next_robo_info.copy()
                self.robo_connection, pick_complete = self.fn(self.robo_info, self.robo_connection, self.reset)
                self.reset = False
            self.next_robo_info = self.get_data(True)

    def set_robo_info(self, robo_info):
        self.next_robo_info = robo_info

    def stop(self):
        self.terminate()
        return self.robo_connection


class getImageWorker(QThread):
    # Get image thread, so the image update is independent of the main window.
    signal_frame_received = pyqtSignal()

    def __init__(self, Lookup, callback):
        QtCore.QThread.__init__(self)
        self.frame = Im.get_image()
        self.img_width = self.frame.shape[0]
        self.img_height = self.frame.shape[1]
        # SIMON
        self.frame = cv2.cvtColor(self.frame.astype('uint8'), cv2.COLOR_GRAY2RGB)
        self.Lookup = Lookup
        self.robo_info = None
        self.all_robo_info = None
        self.capture = True
        self.cam_serial = "22290932"
        self.next_contour = -2
        self.object_pos = [-1,-1]
        self.pre_contour_amount = 0
        if callable(callback):
            self.callback = callback
        else:
            raise Exception("Callback function is not callable")

    def run(self):
        while True:
            if self.capture:
                self.frame = Im.get_image()
                print("inside thread:", self.frame.shape, self.frame.strides)
                # SIMON
                # self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
                # self.frame = cv2.cvtColor(self.frame.astype('uint8'), cv2.COLOR_GRAY2RGB)
                self.robo_info, not_object_contours, self.all_robo_info = Im.findObject(self.frame, self.Lookup, self.next_contour)
                #print("INDEX", self.next_contour)
                if self.cam_serial == "22290932":
                    # Color explanation
                    #cv2.putText(self.robo_info[5], 'Target object', (10, self.robo_info[5].shape[0]-70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    #cv2.putText(self.robo_info[5], 'Other objects', (10, self.robo_info[5].shape[0]-40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255) , 2, cv2.LINE_AA)
                    #cv2.putText(self.robo_info[5], 'Objects of other types', (10, self.robo_info[5].shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                    # Other objects
                    cv2.drawContours(self.robo_info[5], self.robo_info[4], -1, (0, 255, 255), 3)
                    if self.robo_info[0] is not None and not self.next_contour == -2:
                            cv2.drawContours(self.robo_info[5], [self.robo_info[4][self.next_contour]], -1, (0,255,0), 3)

                    # Not objects
                    #cv2.drawContours(self.robo_info[5], not_object_contours, -1, (255, 0, 0), 3)
                else:
                    print("switching to other camera")
                    self.pre_contour_amount = 0
                    self.next_contour = -1
                    self.frame = Im.get_image(self.cam_serial)
                    print("inside thread:", self.frame.shape, self.frame.strides)
                    # self.frame = cv2.cvtColor(self.frame.astype('uint8'), cv2.COLOR_GRAY2RGB)
                    if self.frame.shape[0] > self.frame.shape[1]:
                        height = self.img_height
                        width = int(self.img_width * self.frame.shape[1]/self.frame.shape[0])
                    else:
                        height = int(self.img_height * self.frame.shape[0]/self.frame.shape[1])
                        width = self.img_width
                    self.robo_info[5] = cv2.resize(self.frame, (self.img_height, self.img_width))
                    self.robo_info[6] = False

                self.callback()

    def get_data(self):
        return self.robo_info, self.all_robo_info

    def set_lookup(self, lookup):
        self.Lookup = lookup

    def select_cam(self, cam_serial):
        self.cam_serial= cam_serial

    def set_next_contour(self, next_contour, position = [-1,-1]):
        self.pre_contour_amount = len(self.robo_info[4])
        self.next_contour = next_contour
        self.object_pos = position


# User interface
class MainObjectWidget(QWidget):
    """
    Widget for editing OBJECT parameters
    """
    signal_start = pyqtSignal(name='start_robot')
    signal_stop = pyqtSignal(name='stop_robot')
    signal_stop_capture = pyqtSignal()
    signal_start_capture = pyqtSignal()
    signal_exit = pyqtSignal()
    signal_pick = pyqtSignal()

    def __init__(self, screen_width, screen_height, parent=None):
        super(MainObjectWidget, self).__init__(parent)

        self.screen_width = screen_width
        self.screen_height = screen_height

        #Scale text to screen size
        self.text_size1 = str(int(self.screen_width * 0.01 * 1.5)) + 'px'
        self.text_size2 = str(int(self.screen_width * 0.0125 * 1.5)) + 'px'
        self.text_size3 = str(int(self.screen_width * 0.0075 * 1.5)) + 'px'

        self.windows = list()
        self.libraryWin = 0

        self.l_cmap1 = QLabel("Select type:")

        self.dirs = sorted(os.listdir(rootDir +'Library'))
        self.combo1 = QComboBox(self)
        self.combo1.addItems(self.dirs)
        #self.combo1.currentIndexChanged.connect()

        self.save_images = False
        self.show_images = True
        self.manual_pick = True
        self.optimize_view = False

        path = os.path.join(rootDir + "Library", self.combo1.currentText())

        if os.path.exists(path) and not (self.combo1.currentText() == ""):
            self.files = sorted([file for file in os.listdir(rootDir +'Library//' + self.dirs[0] + "/src") if file.endswith('.png')])
            self.files = [x.split('.')[0] for x in self.files]
        else:
            self.files = []

        self.button_type = QPushButton("&Open library", self)
        self.button_type.clicked.connect(self.library)
        label_save_options = QLabel("\nSave options")
        label_view_options = QLabel("\nSave & View options")
        label_view_options.setAlignment(Qt.AlignCenter)
        self.button_show = QPushButton("&Show views", self)
        self.button_show.clicked.connect(self.show)
        label_data_type = QLabel("Save image:")
        self.data_type = QComboBox(self)
        label_watch = QLabel("Select Camera:")
        self.camera = QComboBox(self)
        self.camera.addItems(['Pick view','Cam 1'])
        self.data_type.addItems(("Don't save (0)","Unknown (1)","Error (2)", "Correct (3)", "Manual select"))
        self.button_grip_open = QPushButton("&Open gripper", self)
        self.button_grip_open.clicked.connect(self.open_gripper)
        self.button_grip_close = QPushButton("&Close gripper", self)
        self.button_grip_close.clicked.connect(self.close_gripper)

        self.button_auto_pick = QPushButton("&Auto pick", self)
        self.button_auto_pick.clicked.connect(self.auto_pick)

        self.button_optimize_view = QPushButton("&Optimize view", self)
        self.button_optimize_view.clicked.connect(self.opt_view)

        self.button_start = QPushButton("&Start Robot", self)
        self.button_start.clicked.connect(self.start_robot)
        self.button_stop = QPushButton("&Stop Robot", self)
        self.button_stop.clicked.connect(self.stop_robot)
        self.button_exit = QPushButton("&Exit program", self)
        self.button_exit.clicked.connect(self.exit)
        self.button_start.setStyleSheet("background-color: green")
        self.button_stop.setStyleSheet("background-color: red")
        self.Robot_status1 = QLabel("Robot status: ")
        self.Robot_status2 = QLabel("Not running")
        self.Robot_status2.setStyleSheet("color: red")
        self.Robot_panel = QLabel("\nRobot control panel")
        self.Robot_panel.setAlignment(Qt.AlignCenter)
        self.Camera_status1 = QLabel("Camera status: ")
        self.Camera_status2 = QLabel("Running")
        self.Camera_status2.setStyleSheet("color: red")
        self.Objects_status1 = QLabel("Type " + self.combo1.currentText() + " objects found:")
        self.Objects_status2 = QLabel("0")
        self.User_message = QLabel(" ", self)
        self.Program_label = QLabel("Picking Program")
        self.Program_label.setAlignment(Qt.AlignCenter)
        self.JLI_logo = QLabel()
        pixmap = QPixmap("C:/Prj/Robopick/Depot.svn/Doc/JLI_logo.png")
        self.JLI_logo.setPixmap(pixmap.scaled(float(self.JLI_logo.width()*0.8), float(self.JLI_logo.height()*0.8), Qt.KeepAspectRatio))
        self.JLI_logo.setAlignment(Qt.AlignCenter)
        space_label = QLabel(" ")

        # Position of widgets
        gbox = QGridLayout()
        gbox.setColumnMinimumWidth(0, int(self.screen_width * 0.1))
        gbox.setColumnMinimumWidth(1, int(self.screen_width * 0.1))
        gbox.addWidget(self.Program_label,0, 0, 1, 2)
        gbox.addWidget(self.l_cmap1, 1, 0)
        gbox.addWidget(self.combo1, 1, 1)
        gbox.addWidget(self.button_type, 2, 1)

        gbox.addWidget(self.Robot_panel, 3, 0, 1, 2)
        gbox.addWidget(self.button_auto_pick, 4, 0, 1, 2)
        gbox.addWidget(self.button_grip_open, 5, 1)
        gbox.addWidget(self.button_grip_close, 5, 0)

        gbox.addWidget(self.button_start, 6, 1)
        gbox.addWidget(self.button_stop, 6, 0)
        gbox.addWidget(self.Robot_status1, 7, 0)
        gbox.addWidget(self.Robot_status2, 7, 1)

        #gbox.addWidget(label_save_options,8,0,1, 2)
        #gbox.addWidget(self.button_save, 9, 0, 1, 2)
        gbox.addWidget(label_view_options, 8, 0, 1, 2)
        gbox.addWidget(self.button_show, 9, 0)
        gbox.addWidget(self.button_optimize_view, 9, 1)
        gbox.addWidget(label_data_type, 10, 0)
        gbox.addWidget(self.data_type, 10, 1, Qt.AlignBottom)
        gbox.addWidget(label_watch, 11, 0)
        gbox.addWidget(self.camera, 11, 1)

        gbox.addWidget(self.Camera_status1, 12, 0)
        gbox.addWidget(self.Camera_status2, 12, 1)
        gbox.addWidget(self.Objects_status1, 13, 0)
        gbox.addWidget(self.Objects_status2, 13, 1)
        #gbox.addWidget(space_label, 16, 0, 1, 2)
        gbox.addWidget(self.JLI_logo, 14, 0, 1, 2)


        vbox = QVBoxLayout()
        vbox.addLayout(gbox)
        vbox.addStretch(1.0)

        self.setLayout(vbox)

        #Layout
        self.combo1.setStyleSheet('QComboBox{font-size: '+ self.text_size1 +'}')
        self.button_show.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; background-color: None}')
        self.button_optimize_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; background-color: None}')
        label_watch.setStyleSheet('font-size: ' + self.text_size1)
        self.camera.setStyleSheet('QComboBox{font-size: ' + self.text_size1 + '}')
        self.data_type.setStyleSheet('QComboBox{font-size: ' + self.text_size1 + '}')
        label_data_type.setStyleSheet('font-size: ' + self.text_size1)
        label_view_options.setStyleSheet('font-size: ' + self.text_size3 + '; font-weight: bold')
        self.button_type.setStyleSheet('QPushButton{font-size: '+ self.text_size1 +'}')
        self.button_grip_open.setStyleSheet('QPushButton{font-size: '+ self.text_size1 +'}')
        self.button_grip_close.setStyleSheet('QPushButton{font-size: '+ self.text_size1 +'}')
        self.button_auto_pick.setStyleSheet('QPushButton{font-size: '+ self.text_size1 +'; background-color: None}')
        self.button_start.setStyleSheet('QPushButton{font-size: '+ self.text_size1 +'; background-color: None}')
        self.button_stop.setStyleSheet('QPushButton{font-size: '+ self.text_size1 +'; font-weight: bold; background-color: darkred}')
        self.button_exit.setStyleSheet('QPushButton{font-size: '+ self.text_size1 +'; font-weight: bold}')
        self.Program_label.setStyleSheet('font-size: '+ self.text_size2 +'; font-weight: bold')
        self.l_cmap1.setStyleSheet('font-size: '+ self.text_size1)
        self.Robot_panel.setStyleSheet("font-size: "+ self.text_size3 + '; font-weight: bold')
        self.Robot_status2.setStyleSheet("font-size: "+ self.text_size3 +"; color: red")
        self.Robot_status1.setStyleSheet("font-size: "+ self.text_size3 +";")
        self.Camera_status2.setStyleSheet("font-size: "+ self.text_size3 +"; color: green")
        self.Camera_status1.setStyleSheet("font-size: "+ self.text_size3 +";")
        self.Objects_status2.setStyleSheet("font-size: "+ self.text_size3 +";")
        self.Objects_status1.setStyleSheet("font-size: "+ self.text_size3 +";")
        self.User_message.setStyleSheet("font-size: "+ self.text_size3 +";")


        self.User_message.move(gbox.horizontalSpacing(),int(screen_height-2.2*(gbox.itemAtPosition(2,1).sizeHint().height()*2+gbox.horizontalSpacing())))
        self.User_message.resize(gbox.sizeHint().width(), gbox.itemAtPosition(2, 1).sizeHint().height()*4)
        self.User_message.show()
        self.button_exit.move(gbox.horizontalSpacing(), int(screen_height-(gbox.itemAtPosition(2,1).sizeHint().height()+gbox.horizontalSpacing())))
        self.button_exit.resize(gbox.sizeHint().width()*1.03, gbox.itemAtPosition(2,1).sizeHint().height())
        self.button_exit.show()

        self.show()

    def update_param(self,):
        self.dirs = sorted(os.listdir(rootDir  +'Library'))
        current_type= self.combo1.currentText()
        self.combo1.clear()
        self.combo1.addItems(self.dirs)

        #Check if pervious selected item still exist and if so selct it
        index = self.combo1.findText(current_type)
        if index > 0:
            self.combo1.setCurrentIndex(index)

        self.signal_start_capture.emit()

    def libray_closed(self):
        #self.Robot_status2.setStyleSheet('font-size: '+ self.text_size3 +'; color: green')
        #self.Robot_status2.setText("Running")
        #self.Camera_status2.setStyleSheet('font-size: '+ self.text_size3 +'; color: green')
        #self.Camera_status2.setText("Running")
        #self.button_start.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + ';  background-color: darkgreen}')
        #self.button_stop.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; font-weight: bold; background-color: None}')
        print(1)
        self.signal_start_capture.emit()
        print(2)
        self.update_param()
        print(3)

    def library(self, option):
        self.Robot_status2.setStyleSheet('font-size: '+ self.text_size3 +'; color: red')
        self.Robot_status2.setText("Not running")
        self.Camera_status2.setStyleSheet('font-size: '+ self.text_size3 +'; color: red')
        self.Camera_status2.setText("Not running")
        self.button_start.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + ';  background-color: None}')
        self.button_stop.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; font-weight: bold; background-color: darkred}')
        self.signal_stop_capture.emit()
        self.libraryWin = Ol_gui.MainWindow(self.screen_width, self.screen_height)
        self.libraryWin.signal_exit_library.connect(self.libray_closed)
        self.libraryWin.show()

    def save(self):
        if self.save_images:
            self.save_images = False
            self.button_save.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + ';  background-color: None}')
        else:
            self.save_images = True
            self.button_save.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; font-weight: bold; background-color: gray}')

    def show(self):
        if self.show_images:
            self.show_images = False
            self.button_show.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + ';  background-color: None}')
        else:
            self.show_images = True
            self.button_show.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; background-color: gray}')

    def auto_pick(self):
        if self.manual_pick:
            self.button_auto_pick.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; background-color: gray}')
            self.manual_pick = False
        else:
            self.button_auto_pick.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; background-color: None}')
            self.manual_pick = True

        self.signal_pick.emit()

    def opt_view(self):
        if self.optimize_view:
            self.button_optimize_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; background-color: None}')
            self.optimize_view = False
        else:
            self.button_optimize_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; background-color: gray}')
            self.optimize_view = True

        #self.signal_pick.emit()

    def start_robot(self, option):
        self.button_start.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; font-weight: bold; background-color: darkGreen}')
        self.button_stop.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; background-color: None}')
        self.Robot_status2.setStyleSheet('font-size: '+ self.text_size3 +'; color: green')
        self.Robot_status2.setText("Running")
        self.Camera_status2.setStyleSheet('font-size: '+ self.text_size3 +'; color: green')
        self.Camera_status2.setText("Running")
        #Wait for updating text
        cv2.waitKey(100)
        self.signal_start.emit()

    def stop_robot(self, option):
        self.button_start.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + ';  background-color: None}')
        self.button_stop.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; font-weight: bold; background-color: darkred}')
        self.Robot_status2.setStyleSheet('font-size: '+ self.text_size3 +'; color: red')
        self.Robot_status2.setText("Not running")
        cv2.waitKey(50)
        self.signal_stop.emit()

    def open_gripper(self):
        if self.Robot_status2.text() == "Not running":
            Rm.open_gripper()

    def close_gripper(self):
        if self.Robot_status2.text() == "Not running":
            Rm.close_gripper()

    def exit(self):
        self.signal_exit.emit()

class MainWindow(QMainWindow):

    def __init__(self, screen_width = None, screen_height = None):
        QMainWindow.__init__(self)
        # self.showFullScreen()
        self.setWindowTitle('Robot GUI')

        #Make sure this file doesn't exist to begin with, otherwise the robot will be stuck
        # if os.path.isfile('C:/Prj/Robopick/Depot.svn/images/temp/temp_save.png'):
        #     os.remove('C:/Prj/Robopick/Depot.svn/images/temp/temp_save.png')
        if os.path.isfile(r"C:/Users/simon/Documents/JLI Projects/ROBOPICK (Mads L)/Current working version (tortoiseSVN)/depot.svn/Images/temp/temp_save.png"):
            os.remove(r"C:/Users/simon/Documents/JLI Projects/ROBOPICK (Mads L)/Current working version (tortoiseSVN)/depot.svn/Images/temp/temp_save.png")

        if screen_width is None or screen_height is None:
            self.screen_width = GetSystemMetrics(0)
            self.screen_height = GetSystemMetrics(1)
        else:
            self.screen_width = screen_width
            self.screen_height = screen_height

        self.props = MainObjectWidget(self.screen_width, self.screen_height)

        self.fs_watcher = QtCore.QFileSystemWatcher([r"C:/Users/simon/Documents/JLI Projects/ROBOPICK (Mads L)/Current working version (tortoiseSVN)/depot.svn/Images/temp/temp.png"])
        self.fs_watcher.fileChanged.connect(self.file_changed)
        self.timer_start_capture = QtCore.QTimer(self)
        self.timer_start_capture.setSingleShot(True)
        self.timer_start_capture.timeout.connect(self.start_capture)
        self.file_change_counter = 0

        self.pick_list = []
        self.control = False

        self.robot = 0
        self.robot_idle = True
        self.robot_connection = None

        self.wait_select_class = False

        self.frame = Im.get_image()
        # SIMON
        self.frame = cv2.cvtColor(self.frame.astype('uint8'), cv2.COLOR_GRAY2RGB)
        # self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
        self.robo_info = []
        self.all_robo_info = []
        self.img_width = int(GetSystemMetrics(0)*0.75)

        self.worker = Worker(RLS.Robot, self.get_robo_info, None)
        self.win_pos = []
        self.win_pos_pre = []
        self.capture_frame = True
        splitter = QSplitter(Qt.Horizontal)

        self.props = MainObjectWidget(self.screen_width, self.screen_height)
        splitter.addWidget(self.props)

        self.img_src_disp = QLabel(self)
        self.pixmap = QPixmap().scaled(self.img_width, 10000, QtCore.Qt.KeepAspectRatio)
        self.img_src_disp.setPixmap(self.pixmap)
        print(self.frame.strides)
        image = QImage(self.frame, self.frame.shape[1], self.frame.shape[0], self.frame.strides[0], QImage.Format_RGB888)
        self.img_src_disp.setPixmap(QPixmap.fromImage(image).scaled(self.img_width, 10000, QtCore.Qt.KeepAspectRatio))
        self.img_src_disp.mousePressEvent = self.mousePress

        splitter.addWidget(self.img_src_disp)

        self.setCentralWidget(splitter)
        self.getimageWorker = getImageWorker(list(self.props.combo1.currentText()[0]), self.update_view)

        self.getimageWorker.start()
        self.update_pick_list()

        self.props.signal_start.connect(self.start)
        self.props.signal_stop.connect(self.stop)
        self.props.signal_stop_capture.connect(self.stop_capture)
        self.props.signal_start_capture.connect(self.start_capture)
        self.props.signal_exit.connect(self.exit)
        self.props.combo1.currentIndexChanged.connect(self.update_lookup)
        self.props.camera.currentIndexChanged.connect(self.update_cam)
        self.props.signal_pick.connect(lambda: self.update_pick_list(True))

    def update_view(self):
        self.robo_info, self.all_robo_info = self.getimageWorker.get_data()
        if self.capture_frame and not self.control:
            if self.robo_info[6]:
                self.props.User_message.setText("\n\n\n\nError: Too big object in image")
                self.props.User_message.setStyleSheet("font-size: "+ self.props.text_size3 +"; color: red")
                print("TOO BIG")
            else:
                self.props.User_message.setText("")
                self.props.User_message.setStyleSheet("font-size: "+ self.props.text_size3 +"; color: black")
                #if not self.robot:
                #    self.robo_info[0] = None

                #self.robo_info.append(self.props.save_images)
                #self.robo_info.append(self.props.show_images)
                #self.robo_info.append(self.props.data_type.currentText())
                #self.robo_info.append(self.getimageWorker.next_contour)
                #self.worker.set_robo_info(self.robo_info)
                if not self.props.manual_pick:
                    self.update_pick_list()

                self.frame = self.robo_info[5]

                #self.update_pick_list()
                image = QImage(self.frame, self.frame.shape[1], self.frame.shape[0], self.frame.strides[0], QImage.Format_RGB888)

                # Prvent a bit of lag of the window movemeant
                self.img_src_disp.setPixmap(QPixmap.fromImage(image).scaled(self.img_width, 10000, QtCore.Qt.KeepAspectRatio))
                self.props.Objects_status1.setText("Type " + self.props.combo1.currentText() + " objects found:")
                self.props.Objects_status2.setText(str(len(self.robo_info[4])))

    def get_robo_info(self, update = False):
        if self.robot:
            index = self.update_pick_list()
        else:
            return []

        if index == -1:
            robo_info = self.robo_info.copy()
        else:
            robo_info = self.all_robo_info[index]


        if not self.robot or self.pick_list == []:
            robo_info[0] = None

        #robo_info.append(self.props.save_images)
        robo_info.append(self.props.data_type.currentText() != "Don't save (0)")

        robo_info.append(self.props.show_images)

        if self.props.data_type.currentText() != "Manual select":
            robo_info.append(self.props.data_type.currentText().split(" ")[0])
            print(1)
        else:
            robo_info.append(self.props.data_type.currentText())
            print(2)

        print("Result 1:", robo_info[-1])

        if index == -1:
            robo_info.append(self.getimageWorker.next_contour)
        else:
            robo_info.append(0)

        robo_info.append(self.props.optimize_view)
        return robo_info

    def runRobot(self):

        if self.robot and not self.worker.isRunning():
            robo_info = self.robo_info.copy()

            if not self.robot or self.pick_list == []:
                robo_info[0] = None

            robo_info.append(self.props.data_type.currentText() != "Don't save (0)")

            robo_info.append(self.props.show_images)
            if self.props.data_type.currentText() != "Manual select":
                print(1)
                robo_info.append(self.props.data_type.currentText().split(" ")[0])
            else:
                print(2)
                robo_info.append(self.props.data_type.currentText())

            print("Result 2:", robo_info[-1])
            robo_info.append(self.getimageWorker.next_contour)
            robo_info.append(self.props.optimize_view)

            self.worker.set_robo_info(robo_info)
            print("Robot is started")
            self.worker.start()

    def start(self):
        #if self.robot_connection is not None:
        #    Rm.robot_init([61.42 * math.pi / 180, -93 * math.pi / 180, 94.65 * math.pi / 180, -91.59 * math.pi / 180,-90 * math.pi / 180, 0 * math.pi / 180], self.robot_connection)
        self.robot = True
        self.capture_frame = True
        self.props.data_type.setEnabled(False)
        self.runRobot()

    def stop(self):
        self.pick_list = []
        self.getimageWorker.set_next_contour(-2)
        self.robot = False
        self.worker.reset = True
        self.robot_connection  = self.worker.stop()

    def stop_capture(self):
        self.robot = False
        self.capture_frame = False

        # Set element, so the robot doesn't contiune to run, after it finishes it's current routine.
        self.robo_info = list(self.robo_info[1:-1])
        self.robo_info.insert(0, None)
        self.robo_info = tuple(self.robo_info)
        robo_info_temp = list(self.robo_info[0:6])
        robo_info_temp.append(self.props.data_type.currentText() != "Don't save (0)")
        if self.props.data_type.currentText() != "Manual select":
            robo_info_temp.append(self.props.data_type.currentText().split(" ")[0])
        else:
            robo_info_temp.append(self.props.data_type.currentText())
        self.worker.set_robo_info(robo_info_temp)
        self.getimageWorker.capture = False

    def start_capture(self):
        self.capture_frame = True
        self.robot = True
        self.props.User_message.setText("")
        self.getimageWorker.capture = True
        self.getimageWorker.start()
        self.props.Camera_status2.setStyleSheet('font-size: '+ self.props.text_size3 +'; color: green')
        self.props.Camera_status2.setText("Running")
        cv2.waitKey(100)

    def update_lookup(self):
        self.getimageWorker.set_lookup(self.props.combo1.currentText())
        self.update_pick_list(True)

    def update_cam(self):
        if self.props.camera.currentText() == 'Cam 1':
            print("Cam 1")
            self.getimageWorker.select_cam(cam_serial = F"21565643")
        else:
            print("Pick View")
            self.getimageWorker.select_cam(cam_serial = F"22290932")

    def file_changed(self, path):
        self.file_change_counter += 1
        if self.file_change_counter == 2:
            self.file_change_counter = 0
            self.timer_start_capture.stop()
            self.stop_capture()
            cv2.waitKey(100)
            new_img = cv2.imread(path)

            new_img = cv2.cvtColor(new_img, cv2.COLOR_BGR2RGB)
            new_img = cv2.resize(new_img, (self.frame.shape[1], self.frame.shape[0]))
            image = QImage(new_img, new_img.shape[1], new_img.shape[0], new_img.strides[0], QImage.Format_RGB888)
            self.img_src_disp.setPixmap(QPixmap.fromImage(image).scaled(self.img_width, 10000, QtCore.Qt.KeepAspectRatio))

            if self.worker.robo_info[9] == self.props.data_type.itemText(self.props.data_type.count()-1):
                text_message = "Select class - Press key number:\n"
                for i in range(self.props.data_type.count()-1):
                    text_message +=  "- " + self.props.data_type.itemText(i) + "\n"
                self.props.User_message.setText(text_message)
                self.props.User_message.setStyleSheet("font-size: "+ self.props.text_size3 +"; color: blue")
                self.wait_select_class = True
                self.props.data_type.setEnabled(False)
            else:
                self.timer_start_capture.start(3000)

    def update_pick_list(self, reset = False):
        current_index = -1
        if self.robo_info:
            if not self.props.manual_pick:
                if self.robo_info[4]:
                    self.pick_list = []
                    for i in reversed(range(len(self.robo_info[4]))):
                        self.pick_list.append([i, 0, 0, 0])
                    self.getimageWorker.set_next_contour(self.pick_list[0][0], self.pick_list[0][1:3].copy())
            else:
                if reset or len(self.pick_list) == 1:
                    self.pick_list = []
                    self.getimageWorker.set_next_contour(-2)
                else:
                    if self.pick_list:
                        self.pick_list.pop(0)
                        #print("Contours:", len(self.robo_info[4]))
                        for contour, index in zip(self.robo_info[4], range(len(self.robo_info[4]))):
                            result = cv2.pointPolygonTest(contour, (self.pick_list[0][1], self.pick_list[0][2]), True)
                            #test = cv2.drawContours(self.robo_info[5], self.robo_info[4], -1, (0, 0, 255), 3)
                            #test = cv2.drawContours(test, [contour], -1, (0, 255, 0), 3)
                            #cv2.imshow("Test", test)
                            #print(result, index)
                            #cv2.waitKey()
                            if abs(result) < 30:
                                current_index = index
                                self.getimageWorker.set_next_contour(index, self.pick_list[0][1:3].copy())
                                break
                    else:
                        self.getimageWorker.set_next_contour(-2)
                        self.props.stop_robot(None)
                        self.props.data_type.setEnabled(True)
                        #self.robot = False

        return current_index

    def mousePress(self, event):
        modifiers = QApplication.keyboardModifiers()

        if event.x() < self.img_src_disp.pixmap().width() and event.y() < self.img_src_disp.pixmap().height() and event.x() >= 0 and event.y() >= 0 and self.props.manual_pick and self.capture_frame:
            height, width = self.frame.shape[0], self.frame.shape[1]
            x = int((event.x() / self.img_src_disp.pixmap().width()) * width)
            y = int((event.y() / self.img_src_disp.pixmap().height()) * height)

            for contour, index in zip(self.robo_info[4], range(len(self.robo_info[4]))):
                result = cv2.pointPolygonTest(contour, (x, y), True)
                if abs(result) < 30 and (bool(modifiers == QtCore.Qt.ControlModifier)):

                    new = True

                    for pick, pick_index in zip(self.pick_list,range(len(self.pick_list))):
                        if index == pick[0]:
                            if  len(self.pick_list) > 1:
                                self.pick_list.pop(pick_index)
                            else:
                                self.pick_list = []
                                self.getimageWorker.set_next_contour(-2)
                            new = False
                    if new:
                        self.pick_list.append([index,x,y, 1])

                    if self.pick_list:
                        if self.pick_list[0][3] == 0:
                            self.pick_list.pop(0)

                    frame = self.frame.copy()
                    cv2.drawContours(frame, self.robo_info[4], -1, (0, 255, 255), 3)

                    if self.pick_list:
                        for pick, priority in zip(self.pick_list,range(len(self.pick_list))):
                            cv2.drawContours(frame, [self.robo_info[4][pick[0]]], -1, (0, 255, 0), 3)
                            cv2.putText(frame, str(priority), tuple(self.robo_info[4][pick[0]][self.robo_info[4][pick[0]][:, :, 0].argmax()][0]), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2,  cv2.LINE_AA)
                        self.getimageWorker.set_next_contour(self.pick_list[0][0], self.pick_list[0][1:3].copy())

                    image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
                    # Prvent a bit of lag of the window movemeant
                    self.img_src_disp.setPixmap( QPixmap.fromImage(image).scaled(self.img_width, 10000, QtCore.Qt.KeepAspectRatio))

                    break
                elif abs(result) < 30:
                    frame = self.frame.copy()
                    cv2.drawContours(frame, self.robo_info[4], -1, (0, 255, 255), 3)
                    cv2.drawContours(frame, [self.robo_info[4][index]], -1, (0, 255, 0), 3)
                    image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
                    # Prvent a bit of lag of the window movemeant
                    self.img_src_disp.setPixmap( QPixmap.fromImage(image).scaled(self.img_width, 10000, QtCore.Qt.KeepAspectRatio))
                    self.pick_list = [[index,x,y, 0]]
                    self.getimageWorker.set_next_contour(self.pick_list[0][0], self.pick_list[0][1:3].copy())

                    break

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Control and not self.robo_info[6]:
            self.control = True

            frame = self.frame.copy()
            cv2.drawContours(frame, self.robo_info[4], -1, (0, 255, 255), 3)

            priority_offset = 0
            if self.pick_list:
                for pick, priority in zip(self.pick_list, range(len(self.pick_list))):
                    if len(self.robo_info[4]) > pick[0]:
                        cv2.drawContours(frame, [self.robo_info[4][pick[0]]], -1, (0, 255, 0), 3)
                        cv2.putText(frame, str(priority-priority_offset ), tuple(self.robo_info[4][pick[0]][self.robo_info[4][pick[0]][:, :, 0].argmax()][0]), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2, cv2.LINE_AA)
                    else:
                        priority_offset += 1

            image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)

            # Prvent a bit of lag of the window movement
            self.img_src_disp.setPixmap(QPixmap.fromImage(image).scaled(self.img_width, 10000, QtCore.Qt.KeepAspectRatio))
            if self.pick_list:
                self.getimageWorker.set_next_contour(self.pick_list[0][0], self.pick_list[0][1:3].copy())

        if self.wait_select_class:
            path = None
            if event.key() == Qt.Key_1:
                self.wait_select_class = False
                path = 'C:/Prj/Robopick/Depot.svn/images/' + self.worker.robo_info[0][1] +'/' + self.props.data_type.itemText(1).split(" ")[0]
            elif event.key() == Qt.Key_2:
                self.wait_select_class = False
                path = 'C:/Prj/Robopick/Depot.svn/images/' + self.worker.robo_info[0][1] +'/' + self.props.data_type.itemText(2).split(" ")[0]
            elif event.key() == Qt.Key_3:
                self.wait_select_class = False
                path = 'C:/Prj/Robopick/Depot.svn/images/' + self.worker.robo_info[0][1] +'/' + self.props.data_type.itemText(3).split(" ")[0]
            elif event.key() == Qt.Key_0:
                self.wait_select_class = False


            if not self.wait_select_class:
                if path is not None:
                    self.props.User_message.setText("\n\n\n\nSelect class: " + path.split("/")[-1])
                    if not os.path.exists(path):
                        os.mkdir(path)
                    img_temp = cv2.imread('C:/Prj/Robopick/Depot.svn/images/temp/temp_save.png')
                    cv2.imwrite(path +'/img_' + time.strftime("%Y%m%d-%H%M%S") + ".png", img_temp)
                else:
                    self.props.User_message.setText("\n\n\n\nSelect class: Don't saved")

                self.props.User_message.setStyleSheet("font-size: " + self.props.text_size3 + "; color: green")

                os.remove('C:/Prj/Robopick/Depot.svn/images/temp/temp_save.png')
                #self.props.data_type.setEnabled(True)
                self.timer_start_capture.start(3000)



    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_Control:
            self.control = False
            self.update_view()

    def exit(self):
        self.robot_connection = self.worker.stop()
        self.props.User_message.setText("Exiting....")
        cv2.waitKey(50)
        Rm.robot_init([61.42 * math.pi / 180, -93 * math.pi / 180, 94.65 * math.pi / 180, -91.59 * math.pi / 180,-90 * math.pi / 180, 0 * math.pi / 180], self.robot_connection )
        self.robot_connection.close()
        QApplication.quit()



# -----------------------------------------------------------------------------
if __name__ == '__main__':
    appQt = QApplication(sys.argv)
    RectScreen0 = appQt.desktop().screenGeometry(0)
    win = MainWindow(RectScreen0.width(), RectScreen0.height())
    win.show()
    win.move(RectScreen0.left(), RectScreen0.top())
    win.resize(RectScreen0.width(), RectScreen0.height())
    appQt.exec_()