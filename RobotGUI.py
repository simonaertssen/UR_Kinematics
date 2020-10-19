# This script contains the pyQt application that is the front end of communication with the robot.
# It has been reworked from Mads' previous work.

import os
import cv2
import sys
from PyQt5 import QtCore, QtWidgets
import Image_module as Im
from PyQt5.QtGui import QIcon, QPixmap, QImage,  QIntValidator, QPixmap, QPainter, QPen, QImage, QBrush, QKeySequence
from PyQt5.QtCore import pyqtSignal, Qt, QThreadPool, QRunnable, pyqtSlot, QThread, QRect, QObject
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel,
                             QSpinBox, QComboBox, QGridLayout, QVBoxLayout,
                             QSplitter, QPushButton, QLineEdit, QRadioButton,  QCheckBox, QShortcut)

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
        print(self.dirs)
        self.combo1 = QComboBox(self)
        self.combo1.addItems(self.dirs)
        #self.combo1.currentIndexChanged.connect()

        self.save_images = False
        self.show_images = False
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
        self.button_show.clicked.connect(self.show_view)
        label_data_type = QLabel("Save image:")
        self.data_type = QComboBox(self)
        #label_watch = QLabel("Select Camera:")
        #self.camera = QComboBox(self)
        #self.camera.addItems(['Pick view','Cam 1'])
        self.data_type.addItems(("Don't save","Unknown","Error", "Correct", "Manual select"))
        self.button_grip_open = QPushButton("&Open gripper", self)
        self.button_grip_open.clicked.connect(self.open_gripper)
        self.button_grip_close = QPushButton("&Close gripper", self)
        self.button_grip_close.clicked.connect(self.close_gripper)

        self.button_auto_pick = QPushButton("&Auto pick", self)
        self.button_auto_pick.clicked.connect(self.auto_pick)
        self.button_replace = QPushButton("&Replace")

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
        self.JLI_logo.setPixmap(pixmap.scaled(float(self.JLI_logo.width()*0.5), float(self.JLI_logo.height()*0.5), Qt.KeepAspectRatio))
        self.JLI_logo.setAlignment(Qt.AlignCenter)
        space_label = QLabel(" ")

        label_detector = QLabel("Surface detector:")
        self.detector = QComboBox(self)
        self.detector.addItems(['None','Scratch'])


        # Position of widgets
        gbox = QGridLayout()
        gbox.setColumnMinimumWidth(0, int(self.screen_width * 0.1))
        gbox.setColumnMinimumWidth(1, int(self.screen_width * 0.1))
        gbox.addWidget(self.Program_label,0, 0, 1, 2)
        gbox.addWidget(self.l_cmap1, 1, 0)
        gbox.addWidget(self.combo1, 1, 1)
        gbox.addWidget(self.button_type, 2, 1)

        gbox.addWidget(self.Robot_panel, 3, 0, 1, 2)
        gbox.addWidget(self.button_auto_pick, 4, 0 )
        gbox.addWidget(self.button_replace, 4, 1)
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
        gbox.addWidget(label_detector, 10, 0)
        gbox.addWidget(self.detector, 10, 1, Qt.AlignBottom)
        gbox.addWidget(label_data_type, 11, 0)
        gbox.addWidget(self.data_type, 11, 1, Qt.AlignBottom)
        #gbox.addWidget(label_watch, 12, 0)
        #gbox.addWidget(self.camera, 12, 1)

        gbox.addWidget(self.Camera_status1, 13, 0)
        gbox.addWidget(self.Camera_status2, 13, 1)
        gbox.addWidget(self.Objects_status1, 14, 0)
        gbox.addWidget(self.Objects_status2, 14, 1)
        #gbox.addWidget(space_label, 16, 0, 1, 2)
        gbox.addWidget(self.JLI_logo, 15, 0, 1, 2)


        vbox = QVBoxLayout()
        vbox.addLayout(gbox)
        vbox.addStretch(1.0)

        self.setLayout(vbox)

        #Layout
        self.combo1.setStyleSheet('QComboBox{font-size: '+ self.text_size1 +'}')
        self.button_show.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; background-color: None}')
        self.button_optimize_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; background-color: None}')
        #label_watch.setStyleSheet('font-size: ' + self.text_size1)
        #self.camera.setStyleSheet('QComboBox{font-size: ' + self.text_size1 + '}')
        self.data_type.setStyleSheet('QComboBox{font-size: ' + self.text_size1 + '}')
        label_data_type.setStyleSheet('font-size: ' + self.text_size1)
        label_view_options.setStyleSheet('font-size: ' + self.text_size3 + '; font-weight: bold')
        self.button_type.setStyleSheet('QPushButton{font-size: '+ self.text_size1 +'}')
        self.button_grip_open.setStyleSheet('QPushButton{font-size: '+ self.text_size1 +'}')
        self.button_grip_close.setStyleSheet('QPushButton{font-size: '+ self.text_size1 +'}')
        self.button_auto_pick.setStyleSheet('QPushButton{font-size: '+ self.text_size1 +'; background-color: None}')
        self.button_replace.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; background-color: None}')
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
        label_detector.setStyleSheet('font-size: ' + self.text_size1)
        self.detector.setStyleSheet('QComboBox{font-size: ' + self.text_size1 + '}')


        self.User_message.move(gbox.horizontalSpacing(),int(screen_height-2.2*(gbox.itemAtPosition(2,1).sizeHint().height()*2+gbox.horizontalSpacing())))
        self.User_message.resize(gbox.sizeHint().width(), gbox.itemAtPosition(2, 1).sizeHint().height()*4)
        self.User_message.show()
        self.button_exit.move(gbox.horizontalSpacing(),int(screen_height-(gbox.itemAtPosition(2,1).sizeHint().height()+gbox.horizontalSpacing())))
        self.button_exit.resize(gbox.sizeHint().width()*1.03, gbox.itemAtPosition(2,1).sizeHint().height())
        self.button_exit.show()

        #Initialize setup
        self.show_view()
        self.auto_pick()
        self.opt_view()