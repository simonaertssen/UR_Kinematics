
import os
import cv2
import sys
import math
import shutil
import numpy as np
import Image_module as Im
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel,
                             QSpinBox, QComboBox, QGridLayout, QVBoxLayout,
                             QSplitter, QPushButton, QLineEdit,  QCheckBox, QMessageBox)
from PyQt5.QtGui import QPixmap, QImage,  QIntValidator
from PyQt5 import QtCore
from win32api import GetSystemMetrics
from sip import setapi
setapi("QVariant", 2)
setapi("QString", 2)


rootDir = r"C:/Prj/Robopick/Depot.svn/"
rootDir = r"C:/Users/simon/Documents/JLI Projects/ROBOPICK (Mads L)/Current working version (tortoiseSVN)/depot.svn/"

#This classe contains the control panel in the object learning GUI
class MainObjectWidget(QWidget):

    #Signals definition
    signal_object_changed = pyqtSignal(name='objectChanged')
    signal_close = pyqtSignal()
    signal_get_image = pyqtSignal()
    signal_pick = pyqtSignal()
    signal_view = pyqtSignal()
    signal_save_view = pyqtSignal()
    signal_next_view = pyqtSignal()
    signal_pre_view = pyqtSignal()
    signal_add_view = pyqtSignal()
    signal_del_view = pyqtSignal()
    signal_notSaved = pyqtSignal()

    def __init__(self,  screen_width, screen_height, parent=None,):
        super(MainObjectWidget, self).__init__(parent)

        self.screen_width = screen_width
        self.screen_height = screen_height

        #Scale text to screen size
        self.text_size1 = str(int(self.screen_width * 0.01*1.5)) + 'px'
        self.text_size2 = str(int(self.screen_width * 0.0125*1.5)) + 'px'
        self.text_size3 = str(int(self.screen_width * 0.0075*1.5)) + 'px'

        self.windows = list()
        self.adwin = 0

        self.origina_data = []

        l_blank = QLabel(" ")
        l_cmap1 = QLabel("Select type:")

        self.dirs = sorted(os.listdir(rootDir + 'Library'))
        self.combo1 = QComboBox(self)
        self.combo1.addItems(self.dirs)
        self.combo1.currentIndexChanged.connect(self.update_param)

        l_cmap2 = QLabel("Select view: ")

        path = os.path.join(rootDir + "Library", self.combo1.currentText())

        if os.path.exists(path) and not (self.combo1.currentText() == ""):
            self.files = sorted([file for file in os.listdir(rootDir + 'Library//' + self.dirs[0] + "/src") if file.endswith('.png')])
            self.files = [x.split('.')[0] for x in self.files]
        else:
            self.files = []

        self.combo2 = QComboBox(self)
        self.combo2.addItems(self.files)
        self.combo2.currentIndexChanged.connect(self.update_param)


        self.button_type_delete = QPushButton("&Delete type", self)
        self.button_type_delete.clicked.connect(self.delete_type)
        self.button_view_delete = QPushButton("&Delete view", self)
        self.button_view_delete.clicked.connect(self.delete_view)
        self.button_close = QPushButton("&Close Library", self)
        self.button_close.clicked.connect(self.close)


        # Add view
        self.button_get_image = QPushButton("&Get image", self)
        self.button_get_image.clicked.connect(self.get_image)

        TN_label = QLabel("Type Name:")
        self.type_name = QLineEdit()
        self.type_name.setText(self.combo1.currentText())
        self.type_name.textChanged.connect(self.notSaved)

        VN_label = QLabel("View Name:")
        self.view_name = QLineEdit()
        self.view_name.setText(self.combo2.currentText())
        self.view_name.textChanged.connect(self.notSaved)

        self.button_grip_view = QPushButton("View grip points", self)
        self.button_grip_view.clicked.connect(lambda: self.btnstate("grip"))
        self.button_view_view = QPushButton("View viewing points", self)
        self.button_view_view.clicked.connect(lambda: self.btnstate("view"))
        self.view_indicator = 0

        Cam_label = QLabel("Select camera:")
        self.camera = QComboBox(self)
        self.camera.addItems(['1','2'])
        self.camera.currentIndexChanged.connect(self.cam_update)

        View_type_label = QLabel("Select View type:")
        self.view_type = QComboBox(self)
        self.view_type.addItems(['Point','Line sweep', 'Point rotation'])
        self.view_type.currentIndexChanged.connect(self.view_type_change)

        angel_int_label = QLabel("Angle interval:")
        self.angle_int = QSpinBox()
        self.angle_int.setMinimum(0)
        self.angle_int.setMaximum(90)
        self.angle_int.setSuffix('°')
        self.angle_int.valueChanged.connect(self.notSaved)

        height_offset_label = QLabel("Height offset:")
        self.height_offset= QSpinBox()
        self.height_offset.setMinimum(0)
        self.height_offset.setMaximum(90)
        self.height_offset.setSuffix('mm°')
        self.height_offset.valueChanged.connect(self.notSaved)



        pic_amount_label = QLabel("Number of pictures:")
        self.pic_amount = QSpinBox()
        self.pic_amount.setMinimum(0)
        self.pic_amount.setMaximum(100)
        self.pic_amount.valueChanged.connect(self.notSaved)

        self.cam_update()


        angle_label = QLabel("View angle:")
        self.angle = QSpinBox()
        self.angle.setMinimum(0)
        self.angle.setMaximum(90)
        self.angle.setSuffix('°')
        self.angle.valueChanged.connect(self.notSaved)
        self.label_view_number = QLabel("1 of 1 view(s)")
        self.label_view_number.setAlignment(Qt.AlignCenter)
        self.button_pre_view = QPushButton("&<-", self)
        self.button_pre_view.clicked.connect(self.pre_view_angle)
        self.button_next_view = QPushButton("&->", self)
        self.button_next_view.clicked.connect(self.next_view_angle)
        self.button_del_view = QPushButton("&Delete view", self)
        self.button_del_view.clicked.connect(self.delete_view_angle)
        self.button_del_view.setEnabled(False)
        self.delete_error_label = QLabel("")
        self.button_add_view = QPushButton("&Add view", self)
        self.button_add_view.clicked.connect(self.add_view_angle)
        self.button_save = QPushButton("&Save view", self)
        self.button_save.clicked.connect(self.save_view)
        self.label_save = QLabel("")
        #self.label_save.setAlignment(Qt.AlignCenter)
        self.label_save.setAlignment(Qt.AlignHCenter)
        self.get_image_label = QLabel("")
        self.get_image_label.setAlignment(Qt.AlignCenter)

        Program_label = QLabel("Library")
        Program_label.setAlignment(Qt.AlignCenter)
        aProgram_label = QLabel(self)
        pixmap = QPixmap("C:/Prj/Robopick/Depot.svn/Doc/JLI_logo.png")
        pixmap = pixmap.scaledToWidth(200)
        aProgram_label.setPixmap(pixmap)
        aProgram_label.move(40, 200)
        aProgram_label.lower()
        aProgram_label.show()

        Add_label = QLabel("Add/Change View")
        Add_label.setAlignment(Qt.AlignCenter)

        # Position of widgets
        gbox = QGridLayout()
        gbox.setColumnMinimumWidth(0, int(self.screen_width * 0.1))
        gbox.setColumnMinimumWidth(1, int(self.screen_width * 0.1))
        gbox.addWidget(Program_label, 0, 0, 1, 2)
        gbox.addWidget(l_cmap1, 1, 0)
        gbox.addWidget(self.combo1, 1, 1)
        gbox.addWidget(self.button_type_delete, 2, 1)
        gbox.addWidget(l_blank, 3, 0)
        gbox.addWidget(l_cmap2, 4, 0)
        gbox.addWidget(self.combo2, 4, 1)
        gbox.addWidget(self.button_view_delete, 5, 1)
        gbox.addWidget(Add_label, 6, 0, 1, 2)
        gbox.addWidget(self.button_get_image, 7, 0, 1, 2)
        gbox.addWidget(self.get_image_label, 8, 0, 1, 2)
        gbox.addWidget(TN_label, 9, 0)
        gbox.addWidget(self.type_name, 9, 1)
        gbox.addWidget(VN_label, 10, 0)
        gbox.addWidget(self.view_name, 10, 1, Qt.AlignTop)
        gbox.addWidget(self.button_grip_view, 11, 0)
        gbox.addWidget(self.button_view_view, 11, 1)
        gbox.addWidget(Cam_label, 12, 0)
        gbox.addWidget(self.camera,12, 1)
        gbox.addWidget(View_type_label, 13, 0)
        gbox.addWidget(self.view_type, 13, 1)
        gbox.addWidget(angle_label, 14, 0)
        gbox.addWidget(self.angle, 14, 1)
        gbox.addWidget(angel_int_label, 15, 0)
        gbox.addWidget(self.angle_int, 15, 1)
        gbox.addWidget(height_offset_label, 16, 0)
        gbox.addWidget(self.height_offset, 16, 1)
        gbox.addWidget(pic_amount_label, 17, 0)
        gbox.addWidget(self.pic_amount, 17, 1)
        gbox.addWidget(self.label_view_number, 18, 0, 1, 2)
        gbox.addWidget(self.button_pre_view, 19, 0)
        gbox.addWidget(self.button_next_view, 19, 1)
        gbox.addWidget(self.button_del_view, 20, 0)
        gbox.addWidget(self.button_add_view, 20, 1)
        gbox.addWidget(self.delete_error_label, 21, 1)
        gbox.addWidget(self.button_save, 21, 0)
        gbox.addWidget(self.label_save, 21, 1)


        vbox = QVBoxLayout()
        vbox.addLayout(gbox)
        vbox.addStretch(1.0)
        self.setLayout(vbox)

        # Layout
        Program_label.setStyleSheet('font-size: ' + self.text_size2 + '; font-weight: bold')
        Add_label.setStyleSheet('font-size: ' + self.text_size1 + '; font-weight: bold')
        l_cmap1.setStyleSheet('font-size: ' + self.text_size1)
        self.delete_error_label.setStyleSheet('font-size: ' + self.text_size1+'}')
        self.combo1.setStyleSheet('QComboBox{font-size: ' + self.text_size1+'}')
        l_blank.setStyleSheet('font-size: ' + self.text_size1+'}')
        l_cmap2.setStyleSheet('font-size: ' + self.text_size1+'}')
        self.combo2.setStyleSheet('QComboBox{font-size: ' + self.text_size1+'}')
        self.button_type_delete.setStyleSheet('QPushButton{font-size: ' + self.text_size1+'}')
        self.button_view_delete.setStyleSheet('QPushButton{font-size: ' + self.text_size1+'}')
        self.button_close.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; font-weight: bold}')
        self.button_get_image.setStyleSheet('QPushButton{font-size: ' + self.text_size1+'}')
        TN_label.setStyleSheet('font-size: ' + self.text_size1+'}')
        self.type_name.setStyleSheet('QLineEdit{font-size: ' + self.text_size1+'}')
        VN_label.setStyleSheet('font-size: ' + self.text_size1+'}')
        self.view_name.setStyleSheet('QLineEdit{font-size: ' + self.text_size1+'}')
        self.button_grip_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1+'; background-color: None}')
        self.button_view_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; background-color: None}')
        Cam_label.setStyleSheet('font-size: ' + self.text_size1 + '}')
        self.camera.setStyleSheet('QComboBox{font-size: ' + self.text_size1 + '}')
        View_type_label.setStyleSheet('font-size: ' + self.text_size1 + '}')
        self.view_type.setStyleSheet('QComboBox{font-size: ' + self.text_size1 + '}')
        angle_label.setStyleSheet('font-size: ' + self.text_size1+'}')
        self.angle.setStyleSheet('QSpinBox { background-color: white; }' 'QSpinBox::up-button { width: ' + self.text_size2 + '; }' 'QSpinBox::down-button { width: ' + self.text_size2 + '; }' 'QSpinBox{font-size: ' + self.text_size1 + '; color: black}')
        self.pic_amount.setStyleSheet('QSpinBox { background-color: None; }' 'QSpinBox::up-button { width: ' + self.text_size2 + '; }' 'QSpinBox::down-button { width: ' + self.text_size2 + '; }' 'QSpinBox{font-size: ' + self.text_size1 + '; color: None}')
        self.angle_int.setStyleSheet('QSpinBox { background-color: None; }' 'QSpinBox::up-button { width: ' + self.text_size2 + '; }' 'QSpinBox::down-button { width: ' + self.text_size2 + '; }' 'QSpinBox{font-size: ' + self.text_size1 + '; color: None}')
        angel_int_label.setStyleSheet('font-size: ' + self.text_size1 + '}')
        self.height_offset.setStyleSheet('QSpinBox { background-color: None; }' 'QSpinBox::up-button { width: ' + self.text_size2 + '; }' 'QSpinBox::down-button { width: ' + self.text_size2 + '; }' 'QSpinBox{font-size: ' + self.text_size1 + '; color: None}')
        height_offset_label.setStyleSheet('font-size: ' + self.text_size1 + '}')
        pic_amount_label.setStyleSheet('font-size: ' + self.text_size1 + '}')
        self.button_save.setStyleSheet('QPushButton{font-size: ' + self.text_size1+'}')
        self.label_save.setStyleSheet("font-size: " + self.text_size3 + "; color: green")
        self.get_image_label.setStyleSheet("font-size: " + self.text_size3 + "; color: red")
        self.button_pre_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1+'}')
        self.button_next_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1+'}')
        self.button_del_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; color: lightGray}')
        self.button_add_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1+'}')
        self.label_view_number.setStyleSheet('font-size: ' + self.text_size3 + '}')

        self.button_close.move(gbox.horizontalSpacing(),int(screen_height-(gbox.itemAtPosition(20,1).sizeHint().height()+gbox.horizontalSpacing())))
        self.button_close.resize(gbox.sizeHint().width(), gbox.itemAtPosition(20,1).sizeHint().height())
        self.button_close.show()

        self.update_param()

    def update_param(self, clear=True):
        #Update parameters, so it fits to the current image.
        self.delete_error_label.setText("")
        #if self.view_indicator != 0:
        #    self.btnstate("grip" if self.view_indicator == 2 else "view")
        self.label_save.setText("Saved\n")
        self.label_save.setStyleSheet("font-size: "+ self.text_size3+"; color: green")
        self.get_image_label.setText("")
        self.dirs = sorted(os.listdir(rootDir + 'Library'))
        self.combo1.currentIndexChanged.disconnect()
        cur_text = self.combo1.currentText()
        self.combo1.clear()
        self.combo1.addItems(self.dirs)
        index = self.combo1.findText(cur_text)
        if index == -1:
            index = 0;
        self.combo1.setCurrentIndex(index)
        self.combo1.currentIndexChanged.connect(self.update_param)
        path = os.path.join(rootDir + "Library", self.combo1.currentText()  + "/src")

        if os.path.exists(path):
            self.files = sorted([file for file in os.listdir(path) if file.endswith('.png')])
            self.files = [x.split('.')[0] for x in self.files]
            self.combo2.currentIndexChanged.disconnect()
            cur_text = self.combo2.currentText()
            self.combo2.clear()
            self.combo2.addItems(self.files)
            index = self.combo2.findText(cur_text)
            if index == -1:
                index = 0;
            self.combo2.setCurrentIndex(index)
            self.combo2.currentIndexChanged.connect(self.update_param)
            self.type_name.textChanged.disconnect()
            self.view_name.textChanged.disconnect()
            self.type_name.setText(self.combo1.currentText())
            self.view_name.setText(self.combo2.currentText())
            self.type_name.textChanged.connect(self.notSaved)
            self.view_name.textChanged.connect(self.notSaved)

        self.signal_object_changed.emit()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Left:
            self.pre_view_angle()
        elif event.key() == Qt.Key_Right:
            self.next_view_angle()
        elif event.key() == Qt.Key_Escape:
            self.close()
        elif event.key() == Qt.Key_Delete:
            if self.view_indicator == 1 or self.view_indicator == 3:
                self.delete_view_angle()



    def view_type_change(self, saved = False):
        # Activate/deactivate certain options depending on the view_tye
        if self.view_type.currentText() == 'Point':
            self.pic_amount.setReadOnly(True)
            self.pic_amount.setStyleSheet('QSpinBox {background-color: Lightgray;}' 'QSpinBox::up-button { width: ' + self.text_size2 + '; }' 'QSpinBox::down-button { width: ' + self.text_size2 + '; }' 'QSpinBox{font-size: ' + self.text_size1 + '; color: Lightgray}')
            self.angle_int.setReadOnly(True)
            self.angle_int.setStyleSheet('QLabel{color: Lightgray}' 'QSpinBox { background-color: Lightgray; }' 'QSpinBox::up-button { width: ' + self.text_size2 + '; }' 'QSpinBox::down-button { width: ' + self.text_size2 + '; }' 'QSpinBox{font-size: ' + self.text_size1 + '; color: Lightgray}')
            self.pic_amount.update()
        elif self.view_type.currentText() == 'Line sweep':
            self.pic_amount.setReadOnly(False)
            self.pic_amount.setStyleSheet('QSpinBox { background-color: white; }' 'QSpinBox::up-button { width: ' + self.text_size2 + '; }' 'QSpinBox::down-button { width: ' + self.text_size2 + '; }' 'QSpinBox{font-size: ' + self.text_size1 + '; color: black}')
            self.angle_int.setReadOnly(True)
            self.angle_int.setStyleSheet('QSpinBox { background-color: Lightgray; }' 'QSpinBox::up-button { width: ' + self.text_size2 + '; }' 'QSpinBox::down-button { width: ' + self.text_size2 + '; }' 'QSpinBox{font-size: ' + self.text_size1 + '; color: Lightgray}')
        elif self.view_type.currentText() == 'Point rotation':
            self.pic_amount.setReadOnly(False)
            self.pic_amount.setStyleSheet('QSpinBox { background-color: white; }' 'QSpinBox::up-button { width: ' + self.text_size2 + '; }' 'QSpinBox::down-button { width: ' + self.text_size2 + '; }' 'QSpinBox{font-size: ' + self.text_size1 + '; color: black}')
            self.angle_int.setReadOnly(False)
            self.angle_int.setStyleSheet('QSpinBox { background-color: white; }' 'QSpinBox::up-button { width: ' + self.text_size2 + '; }' 'QSpinBox::down-button { width: ' + self.text_size2 + '; }' 'QSpinBox{font-size: ' + self.text_size1 + '; color: black}')

    def delete_type(self):
        ret = QMessageBox.question(self, 'MessageBox', "Are you sure you want to delete type '" + self.combo1.currentText() + "'?", QMessageBox.Yes | QMessageBox.No,QMessageBox.No)

        if ret == QMessageBox.Yes:
            # Delete the type view
            path = os.path.join(rootDir + "Library", self.combo1.currentText())
            shutil.rmtree(path)
            self.update_param(self)

    def delete_view(self):
        ret = QMessageBox.question(self, 'MessageBox', "Are you sure you want to delete view '" + self.combo2.currentText() + "'?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if ret == QMessageBox.Yes:
            #Delete the currernt view
            path_src = os.path.join(rootDir + "Library", self.combo1.currentText() + "\src\\" + self.combo2.currentText() + ".png")
            path_mask = os.path.join(rootDir + "Library", self.combo1.currentText() + "\mask\\" + self.combo2.currentText() + ".png")
            path_points = os.path.join(rootDir + "Library", self.combo1.currentText() + "\points\\" + self.combo2.currentText() + "_pickpoints.npy")

            os.remove(path_src)
            os.remove(path_mask)
            os.remove(path_points)

            self.update_param(self)

    def get_image(self):
        # Signal main window to grab new image
        self.signal_get_image.emit()

    def btnstate(self, signal):
        #Set checkbox, so only one shows up
        if signal == "grip":
            if self.view_indicator == 0:
                self.button_grip_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1+'; background-color: gray}')
                self.view_indicator = 2
            elif self.view_indicator == 1:
                self.button_grip_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1+'; background-color: gray}')
                self.view_indicator = 3
            else:
                self.button_grip_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1+'; background-color: None}')
                if self.view_indicator == 3:
                    self.view_indicator = 1
                else:
                    self.view_indicator = 0

            self.signal_pick.emit()
        elif signal == "view":
            if self.view_indicator == 0:
                self.button_view_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; background-color: gray}')
                self.button_del_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; color: black}')
                self.button_del_view.setEnabled(True)
                self.view_indicator = 1
            elif self.view_indicator == 2:
                self.button_view_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; background-color: gray}')
                self.button_del_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; color: black}')
                self.button_del_view.setEnabled(True)
                self.view_indicator = 3
            else:
                self.button_view_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; background-color: None}')
                self.button_del_view.setEnabled(False)
                self.button_del_view.setStyleSheet('QPushButton{font-size: ' + self.text_size1 + '; color: lightGray}')
                if self.view_indicator == 3:
                    self.view_indicator = 2
                else:
                    self.view_indicator = 0

            self.delete_error_label.setText("")
            self.signal_view.emit()

    def cam_update(self):
        print("props cam:", self.camera.currentText())
        if self.camera.currentText() == '1':
            self.view_type.disconnect()
            cur_text = self.view_type.currentText()
            self.view_type.clear()
            self.view_type.addItems(['Point','Line sweep'])
            try:
                self.angle.setMinimum(0)
            except:
                pass
            index = self.view_type.findText(cur_text)
            if index == -1:
                index = 0;
            self.view_type.currentIndexChanged.connect(self.view_type_change)
            self.view_type.setCurrentIndex(index)
        else:
            self.view_type.disconnect()
            cur_text = self.view_type.currentText()
            self.view_type.clear()
            self.view_type.addItems(['Point','Line sweep', 'Point rotation'])
            self.angle.setMinimum(-90)
            index = self.view_type.findText(cur_text)
            if index == -1:
                index = 0;
            self.view_type.currentIndexChanged.connect(self.view_type_change)
            self.view_type.setCurrentIndex(index)

    def pre_view_angle(self):
        if self.view_indicator == 0 or self.view_indicator == 2:
            self.btnstate('view')
        self.signal_pre_view.emit()

    def next_view_angle(self):
        if self.view_indicator == 0 or self.view_indicator == 2:
            self.btnstate('view')
        self.signal_next_view.emit()

    def add_view_angle(self):
        if self.view_indicator == 0 or self.view_indicator == 2:
            self.btnstate("view")
        self.signal_add_view.emit()
        self.notSaved()

    def delete_view_angle(self):
        ret = QMessageBox.question(self, 'MessageBox',  "Are you sure you want to delete view this view angle?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if ret == QMessageBox.Yes:
            if self.view_indicator == 1 or self.view_indicator == 3:
                self.delete_error_label.setText("")
                self.signal_del_view.emit()
                self.notSaved()
            else:
                self.delete_error_label.setText("Set to show view")

    def save_view(self):
        print("HEJ")
        if not self.type_name.text().strip():
            print("name string missing")
            self.label_save.setText("Type name missing")
            return
        #Save current view
        path = os.path.join(rootDir + "Library", self.type_name.text())
        if not os.path.exists(path):
            os.mkdir(path)
            os.mkdir(path + "/src")
            os.mkdir(path + "/mask")
            os.mkdir(path + "/points")
        self.signal_save_view.emit()
        self.label_save.setText("Saved\n")
        self.label_save.setStyleSheet("font-size: "+ self.text_size3+"; color: green")

    def notSaved(self):
        #Set label_saved text
        self.signal_notSaved.emit()

    def close(self):
        if self.label_save.text() == "Not Saved\n":
            ret = QMessageBox.question(self, 'MessageBox', "You haven't saved. Unsaved templates will be lost. Do you still want to exit Library?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if ret == QMessageBox.No:
                return

        #Signal close program
        self.signal_close.emit()

# This class contains the main window for the object_learning_gui, where all the pictures are being set.
class MainWindow(QMainWindow):
    signal_exit_library = pyqtSignal()
    def __init__(self, screen_width = None, screen_height = None):
        QMainWindow.__init__(self)
        # self.showFullScreen()
        self.setWindowTitle('Library')

        splitter = QSplitter(Qt.Horizontal)
        if screen_width is None or screen_height is None:
            self.screen_width = GetSystemMetrics(0)
            self.screen_height = GetSystemMetrics(1)
        else:
            self.screen_width = screen_width
            self.screen_height = screen_height


        self.props = MainObjectWidget(self.screen_width, self.screen_height)
        splitter.addWidget(self.props)

        self.img_width = int((self.screen_width-self.props.sizeHint().width())/2)

        self.label_src = QLabel(self)
        self.pixmap_src = QPixmap().scaled(self.img_width, 10000, QtCore.Qt.KeepAspectRatio)
        self.label_src.setPixmap(self.pixmap_src)
        self.label_src.setAlignment(Qt.AlignAbsolute)
        self.label_src.mouseReleaseEvent = self.mouseEvent
        self.label_src.mouseMoveEvent = self.mouseMoveEvent
        self.label_src.mouseDoubleClickEvent = self.mouseAdd
        self.label_src.mousePressEvent = self.mousePress
        splitter.addWidget(self.label_src)

        self.label_mask = QLabel(self)
        self.pixmap_mask = QPixmap().scaled(self.img_width, 10000, QtCore.Qt.KeepAspectRatio)
        self.label_mask.setPixmap(self.pixmap_mask)
        self.label_mask.setAlignment(Qt.AlignAbsolute)
        self.label_mask.mouseReleaseEvent = self.mouseEvent
        self.label_mask.mouseMoveEvent = self.mouseMoveEvent
        self.label_mask.mouseDoubleClickEvent = self.mouseAdd
        self.label_mask.mousePressEvent = self.mousePress
        splitter.addWidget(self.label_mask)

        splitter.setSizes([5, 600, 600])
        splitter.setStretchFactor(0, 1)

        self.pickpoints = np.ones((8),np.intc)*(-1)
        self.pickpointsCounter = -1
        self.pick_value = False

        self.viewpoints_all = np.ones((1,8),np.intc)*(-1)
        self.cam_viewpoints = [self.viewpoints_all.copy(), self.viewpoints_all.copy()]
        self.viewpointsCounter = 2
        self.viewpointsIndex = 0
        self.viewpointsIndextemp = 0
        self.view_value = False
        self.angles = 0
        self.viewpoints_place_holder = None

        self.img_src = cv2.imread('../../../Test_Images/4_blocks_7.bmp')
        self.img_mask = cv2.imread('../../../Test_Images/4_blocks_7.bmp')
        self.img_src_draw = cv2.imread('../../../Test_Images/4_blocks_7.bmp')
        self.img_mask_draw = cv2.imread('../../../Test_Images/4_blocks_7.bmp')

        self.new_image = True
        self.saved_points = []
        self.mouseRelease = True
        self.clickCount = 0

        self.setCentralWidget(splitter)
        self.props.signal_object_changed.connect(self.update_view)
        self.props.signal_close.connect(self.close_library)
        self.props.signal_get_image.connect(self.get_view)
        self.props.signal_pick.connect(self.signal_pick_handle)
        self.props.signal_view.connect(self.signal_view_handle)
        self.props.signal_save_view.connect(self.save_view)
        self.props.signal_pre_view.connect(self.pre_view_angle)
        self.props.signal_next_view.connect(self.next_view_angle)
        self.props.signal_add_view.connect(self.add_view_angle)
        self.props.signal_del_view.connect(self.delete_view_angle)
        self.props.signal_notSaved.connect(self.notSaved)
        self.props.camera.currentIndexChanged.connect(self.cam_change)
        self.props.view_type.currentIndexChanged.connect(self.view_type_change)
        self.update_view()

    def update_view(self):
        self.img_src = cv2.imread(rootDir + 'Library/' + self.props.combo1.currentText() + '/src/' + self.props.combo2.currentText() + '.png')
        src = np.ones((self.img_src.shape[0] + 100, self.img_src.shape[1] + 100, 3), np.uint8) * 255
        src[50:-50, 50:-50] = self.img_src
        src = cv2.resize(src,(src.shape[1]*10,src.shape[0]*10))
        self.img_src = src
        image = QImage(src, src.shape[1], src.shape[0], src.strides[0], QImage.Format_RGB888)
        self.pixmap_src = QPixmap(image).scaled(self.img_width, 10000, QtCore.Qt.KeepAspectRatio)
        self.label_src.setPixmap(self.pixmap_src)
        self.img_mask = cv2.imread(rootDir + 'Library/' + self.props.combo1.currentText() + '/mask/' + self.props.combo2.currentText() + '.png')
        mask = np.zeros((self.img_mask.shape[0] + 100, self.img_mask.shape[1] + 100, 3), np.uint8)
        mask[50:-50, 50:-50] = self.img_mask
        mask = cv2.resize(mask, (mask.shape[1] * 10, mask.shape[0] * 10))
        self.img_mask = mask
        image = QImage(mask, mask.shape[1], mask.shape[0], mask.strides[0], QImage.Format_RGB888)
        self.pixmap_mask = QPixmap(image).scaled(self.img_width, 10000, QtCore.Qt.KeepAspectRatio)
        self.label_mask.setPixmap(self.pixmap_mask)
        path_points = os.path.join(rootDir + "Library", self.props.combo1.currentText() + "\points\\" + self.props.combo2.currentText() + "_pickpoints.npy")
        if os.path.exists(path_points):
            points = np.load(path_points)
            print("Load", points)
            temp = points.copy()
            points[:, 0:4] = (points[:, 0:4] + 50)* 10
            points[temp==-1] = -1
            #Tempoary - used to convert to new format
            if points.shape[1] == 5:
                temp = np.ones((points.shape[0]+1,7), dtype=np.int)*(-1)
                temp[0:points.shape[0],0:points.shape[1]] = points.copy()
                temp[0,4] = points.shape[0]-1
                points = temp.copy()
            if points.shape[1]  == 7:
                temp = np.ones((points.shape[0], 8), dtype=np.int) * (-1)
                temp[0:points.shape[0], 0:points.shape[1]] = points.copy()
                for i in range(1, points.shape[0]):
                    if i < points[0,4]+1:
                        temp[i,7] = 0
                    else:
                        temp[i,7] = 2
                points = temp.copy()

            self.saved_points = points.copy()
            self.pickpoints = points[0,:]
            self.pickpointsCounter = -1
            self.cam_viewpoints[0] = points[1: int(points[0,4]+1),:]
            self.cam_viewpoints[1] = points[int(points[0,4]+1):,:]
            if self.props.camera.currentText() == '1':
                self.viewpoints_all = self.cam_viewpoints[0]
            else:
                self.viewpoints_all = self.cam_viewpoints[1]

            self.viewpointsCounter = 2
            self.props.angle.valueChanged.disconnect()
            self.props.angle.setValue(self.viewpoints_all[self.viewpointsIndex, 4])
            self.props.angle.valueChanged.connect(self.props.notSaved)

            if self.viewpoints_all[self.viewpointsIndex, 7] == -1:
                self.viewpoints_all[self.viewpointsIndex, 7] = 0

            self.props.view_type.setCurrentIndex(self.viewpoints_all[self.viewpointsIndex, 7])

        else:
            print("in 2")
            self.pickpoints = np.ones((8),np.intc)*(-1)
            self.pickpointsCounter = 0
            self.viewpointsCounter = 2
            self.viewpoints_all = np.ones((1, 8), np.intc) * (-1)
            self.viewpoints_all[1:,7] = 0
            self.cam_viewpoints = [self.viewpoints_all.copy(), self.viewpoints_all.copy()]
            self.viewpointsIndex = 0
            self.viewpointsIndextemp = 0
            self.props.angle.setValue(0)
            self.props.view_type.setCurrentIndex(self.viewpoints_all[self.viewpointsIndex, 7])

        if self.viewpoints_all[0, 0] != -1:
            self.props.label_view_number.setText(str(int(self.viewpointsIndex) + int(self.viewpoints_all[-1, 0] != -1)) + " of " + str(self.viewpoints_all.shape[0] - int(self.viewpoints_all[-1, 0] == -1)) + " view(s)")
        else:
            self.props.label_view_number.setText("No views")

        self.generataDrawimages()
        self.pick_view()
        self.props.view_type_change()

    def get_view(self):
        img = Im.get_image()
        # Convert image format
        if len(img.shape) == 2:
            s_img = np.zeros((img.shape[0]-20, img.shape[1]-20, 3), dtype=np.uint8)
            s_img[:, :, 0] = img[10:-10,10:-10]
            s_img[:, :, 1] = img[10:-10,10:-10]
            s_img[:, :, 2] = img[10:-10,10:-10]
            img = s_img
        imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find contours
        kernel = np.ones((7, 7), np.uint8)  # avoid colapsing contours, when objects are close together
        ret, thresh = cv2.threshold(imgray, 75, 255, cv2.THRESH_BINARY_INV)
        _, contours, hierarchy = cv2.findContours(cv2.erode(thresh, kernel, iterations=1), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        mask = np.zeros(thresh.shape, np.uint8)

        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area > 500 and area < 100000:
                self.props.get_image_label.setText("")
                self.props.label_save.setText("Not Saved\n")
                self.props.label_save.setStyleSheet("font-size: "+ self.props.text_size3+"; color: red")
                self.props.type_name.setText("")
                self.props.view_name.setText("")
                self.props.angle.setValue(0)
                self.props.button_view_view.setStyleSheet('QPushButton{font-size: ' + self.props.text_size1 + '; background-color: None}')
                self.props.button_grip_view.setStyleSheet('QPushButton{font-size: ' + self.props.text_size1+'; background-color: None}')
                self.props.view_indicator = 0
                self.pickpoints = np.ones((8),np.intc)*(-1)
                self.pickpointsCounter = -1
                self.viewpoints_all = np.ones((1, 8), np.intc) * (-1)
                self.cam_viewpoints = [self.viewpoints_all.copy(), self.viewpoints_all.copy()]
                self.viewpointsCounter = 2
                self.viewpointsIndex = 0

                rect = cv2.minAreaRect(contours[i])
                cv2.drawContours(mask, [contours[i]], -1, 255, cv2.FILLED)

                self.img_mask, self.img_src, top_ori = Im.getSubImage(rect, imgray, mask)

                final_mask = np.zeros((self.img_mask.shape[0] + 100,self.img_mask.shape[1] + 100), np.uint8)
                final_src = np.ones((self.img_mask.shape[0] + 100, self.img_mask.shape[1] + 100), np.uint8) * 255
                final_mask[50:-50, 50:-50] = self.img_mask
                final_src[50:-50, 50:-50] = self.img_src
                final_mask = cv2.merge((final_mask,final_mask,final_mask))
                final_src = cv2.merge((final_src, final_src, final_src))
                self.img_mask = cv2.resize(final_mask,(final_mask.shape[1]*10, final_mask.shape[0]*10))
                self.img_src = cv2.resize(final_src,(final_src.shape[1]*10, final_mask.shape[0]*10))
                frame = self.img_src

                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                image = QImage(frame, frame.shape[1], frame.shape[0],frame.strides[0], QImage.Format_RGB888)
                self.label_src.setPixmap(QPixmap.fromImage(image).scaled(self.img_width, 10000, QtCore.Qt.KeepAspectRatio))

                frame = self.img_mask
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)

                self.label_mask.setPixmap(QPixmap.fromImage(image).scaled(self.img_width, 10000, QtCore.Qt.KeepAspectRatio))

                self.new_image = True
                self.view_value = False
                self.pick_value = False
                self.props.label_view_number.setText(str(int(self.viewpointsIndex) + 1) + " of " + str(self.viewpoints_all.shape[0]) + " view(s)")
                return

        self.props.get_image_label.setText("No object found")

    def cam_change(self):

        self.viewpoints_all[self.viewpointsIndex, 4] = int(self.props.angle.value())

        #print(self.cam_viewpoints)
        #print(self.viewpoints_all)
        print("Before")
        print( self.cam_viewpoints[0])
        print( self.cam_viewpoints[1])
        print("Cam:", self.props.camera.currentText())
        if self.props.camera.currentText() == '1':
            self.props.angle_int.setMinimum(0)
            self.props.angle_int.setMaximum(90)
            self.viewpoints_all[self.viewpointsIndex, 5] = int(self.props.angle_int.value())
            self.viewpoints_all[self.viewpointsIndex, 6] = int(self.props.pic_amount.value())
            self.cam_viewpoints[1] = self.viewpoints_all.copy()
            self.viewpoints_all = self.cam_viewpoints[0].copy()
            self.viewpointsCounter = 2
            self.viewpointsIndex, self.viewpointsIndextemp = self.viewpointsIndextemp, self.viewpointsIndex
        else:
            self.props.angle_int.setMinimum(-90)
            self.props.angle_int.setMaximum(90)
            self.cam_viewpoints[0] = self.viewpoints_all.copy()
            self.viewpoints_all = self.cam_viewpoints[1].copy()
            self.viewpointsCounter = 2
            self.viewpointsIndex, self.viewpointsIndextemp = self.viewpointsIndextemp, self.viewpointsIndex

        print("After")
        print( self.cam_viewpoints[0])
        print( self.cam_viewpoints[1])

        self.props.view_type.setCurrentIndex(self.viewpoints_all[self.viewpointsIndex, 7])
        self.props.angle.valueChanged.disconnect()
        self.props.angle_int.valueChanged.disconnect()
        self.props.pic_amount.valueChanged.disconnect()
        self.props.angle.setValue(self.viewpoints_all[self.viewpointsIndex, 4])
        self.props.angle_int.setValue(self.viewpoints_all[self.viewpointsIndex, 5])
        self.props.pic_amount.setValue(self.viewpoints_all[self.viewpointsIndex, 6])
        self.props.angle.valueChanged.connect(self.props.notSaved)
        self.props.angle_int.valueChanged.connect(self.props.notSaved)
        self.props.pic_amount.valueChanged.connect(self.props.notSaved)
        self.generataDrawimages()
        self.pick_view()
        if self.viewpoints_all[0, 0] != -1:
            self.props.label_view_number.setText( str(int(self.viewpointsIndex) + int(self.viewpoints_all[-1, 0] != -1)) + " of " + str(self.viewpoints_all.shape[0] - int(self.viewpoints_all[-1, 0] == -1)) + " view(s)")
        else:
            self.props.label_view_number.setText("No views")


        print("Cam 1")
        print(self.cam_viewpoints[0])

        print("Cam 2")
        print(self.cam_viewpoints[1])

    def view_type_change(self):
        self.viewpoints_all[self.viewpointsIndex,7] = self.props.view_type.currentIndex()
        self.notSaved()

    def signal_pick_handle(self):
        self.pick_value = not self.pick_value
        self.generataDrawimages()
        #self.view_value = False
        self.pick_view()

    def signal_view_handle(self):
        self.view_value = not self.view_value
        self.generataDrawimages()
        #self.pick_value = False
        self.pick_view()

    def mouseEvent(self, event):
        self.mouseRelease = True
        self.pickpointsCounter = -1
        self.generataDrawimages()
        self.pick_view()

    def mouseAdd(self, event):
        if self.view_value and not ( event.x() > self.label_mask.pixmap().width() or event.y() > self.label_mask.pixmap().height()  or event.x() < 0 or event.y() < 0):
            self.props.label_save.setText("Not Saved\n")
            self.props.label_save.setStyleSheet("font-size: " + self.props.text_size3 + "; color: red")
            height, width = self.img_mask.shape[0], self.img_mask.shape[1]
            self.add_view_angle()
            self.props.label_view_number.setText(str(int(self.viewpointsIndex) + 1) + " of " + str(self.viewpoints_all.shape[0] ) + " view(s)")
            self.viewpointsCounter = 2
            self.viewpoints_all[self.viewpointsIndex, self.viewpointsCounter] = int((event.x() / self.label_mask.pixmap().width()) * width)
            self.viewpoints_all[self.viewpointsIndex, self.viewpointsCounter + 1] = int((event.y() / self.label_mask.pixmap().height()) * height)
            self.viewpointsCounter = 0
            self.mouseRelease = False

    def mousePress(self, event):
        if (self.view_value or self.pick_value) and not (event.x() > self.label_mask.pixmap().width() or event.y() > self.label_mask.pixmap().height() or event.x() < 0 or event.y() < 0):

            height, width = self.img_mask.shape[0], self.img_mask.shape[1]

            x = int((event.x() / self.label_mask.pixmap().width()) * width)
            y = int((event.y() / self.label_mask.pixmap().height()) * height)

            if self.pick_value:
                if self.pickpoints[0] != -1 and self.pickpoints[2] != -1:
                    if self.mouseRelease and 50 > math.sqrt(math.pow(self.pickpoints[0] - x,2) + math.pow(self.pickpoints[1] - y, 2)):
                        self.pickpointsCounter = 0
                        self.mouseRelease = False
                        self.generataDrawimages()
                        self.pick_view()
                        return
                    elif self.mouseRelease and 50 > math.sqrt(math.pow(self.pickpoints[2] - x,  2) + math.pow( self.pickpoints[3] - y, 2)):
                        self.pickpointsCounter = 2
                        self.mouseRelease = False
                        self.generataDrawimages()
                        self.pick_view()
                        return
                else:
                    self.pickpointsCounter = 0 if self.pickpoints[0] == -1 else 2
                    self.pickpoints[self.pickpointsCounter] = x
                    self.pickpoints[self.pickpointsCounter + 1] = y
                    self.generataDrawimages()
                    self.pick_view()
                    return

            #Check if the point is on the line
            dst = []
            for i in range(self.viewpoints_all.shape[0]):
                if self.viewpoints_all[i, 0] == -1:
                    dst.append(100)
                else:
                    dst.append(np.linalg.norm(np.cross(np.array(self.viewpoints_all[i, 2:4]) - np.array(self.viewpoints_all[i, 0:2]), np.array(self.viewpoints_all[i, 0:2]) - np.array((x,y)))) / np.linalg.norm(np.array(self.viewpoints_all[i, 2:4]) - np.array(self.viewpoints_all[i, 0:2])))
                    #Check if the click is in between the 2 end points
                    if not( x+50 > min(self.viewpoints_all[i, 0],self.viewpoints_all[i, 2]) and x-50 < max(self.viewpoints_all[i, 0],self.viewpoints_all[i, 2]) and y+50 > min(self.viewpoints_all[i, 1],self.viewpoints_all[i, 3]) and y-50 < max(self.viewpoints_all[i, 1],self.viewpoints_all[i, 3])):
                        dst[-1] = 100
            if min(dst) < 50:
                self.viewpoints_all[self.viewpointsIndex, 4] = int(self.props.angle.value())
                self.viewpoints_all[self.viewpointsIndex, 7] = int(self.props.view_type.currentIndex())
                if self.props.camera.currentText() == '2':
                    self.viewpoints_all[self.viewpointsIndex, 5] = int(self.props.angle_int.value())
                    self.viewpoints_all[self.viewpointsIndex, 6] = int(self.props.pic_amount.value())

                self.viewpointsIndex = np.argmin(dst) - 1
                self.next_view_angle(False)

    def mouseMoveEvent(self, event):
        if event.x() > self.label_mask.pixmap().width() or event.y() > self.label_mask.pixmap().height()  or event.x() < 0 or event.y() < 0:
            return

        height, width = self.img_mask.shape[0], self.img_mask.shape[1]

        if self.pick_value:
            if self.mouseRelease and 50 > math.sqrt(math.pow(self.pickpoints[0] - int((event.x() / self.label_mask.pixmap().width()) * width),2) + math.pow(self.pickpoints[1] - int((event.y() / self.label_mask.pixmap().height()) * height), 2)):
                self.pickpointsCounter = 0
                self.mouseRelease = False
                self.generataDrawimages()
                self.props.label_save.setText("Not Saved\n")
                self.props.label_save.setStyleSheet("font-size: " + self.props.text_size3 + "; color: red")
            elif self.mouseRelease and 50 > math.sqrt( math.pow(self.pickpoints[2] - int((event.x() / self.label_mask.pixmap().width()) * width),2) + math.pow(self.pickpoints[3] - int((event.y() / self.label_mask.pixmap().height()) * height),2)):
                self.pickpointsCounter = 2
                self.mouseRelease = False
                self.generataDrawimages()
                self.props.label_save.setText("Not Saved\n")
                self.props.label_save.setStyleSheet("font-size: " + self.props.text_size3 + "; color: red")
            elif self.mouseRelease:
                self.pickpointsCounter = -1


            if self.pickpointsCounter != -1:
                self.pickpoints[self.pickpointsCounter] = int((event.x() / self.label_mask.pixmap().width()) * width)
                self.pickpoints[self.pickpointsCounter + 1] = int((event.y() / self.label_mask.pixmap().height()) * height)

        if self.view_value and self.pickpointsCounter == -1:
            if self.viewpoints_all[self.viewpointsIndex, 0] != -1 and self.viewpoints_all[self.viewpointsIndex, 2] != -1:
                if self.mouseRelease and 50 > math.sqrt(math.pow(self.viewpoints_all[self.viewpointsIndex,0] - int((event.x() / self.label_mask.pixmap().width()) * width),2) + math.pow(self.viewpoints_all[self.viewpointsIndex,1] - int((event.y() / self.label_mask.pixmap().height()) * height),2)):
                    self.viewpointsCounter = 0
                    self.mouseRelease = False
                    self.props.label_save.setText("Not Saved\n")
                    self.props.label_save.setStyleSheet("font-size: " + self.props.text_size3 + "; color: red")
                elif self.mouseRelease and 50 > math.sqrt(math.pow(self.viewpoints_all[self.viewpointsIndex,2] - int((event.x() / self.label_mask.pixmap().width()) * width),2) + math.pow(self.viewpoints_all[self.viewpointsIndex,3] - int((event.y() / self.label_mask.pixmap().height()) * height),2)):
                    self.viewpointsCounter = 2
                    self.mouseRelease = False
                    self.props.label_save.setText("Not Saved\n")
                    self.props.label_save.setStyleSheet("font-size: " + self.props.text_size3 + "; color: red")
                elif self.mouseRelease:
                    return

            elif self.mouseRelease:
                self.viewpointsCounter = 2 if self.viewpointsCounter == 0 else 0
                self.mouseRelease = False
                self.props.label_view_number.setText(str(int(self.viewpointsIndex) + 1) + " of " + str(self.viewpoints_all.shape[0]) + " view(s)")

            if self.viewpoints_all[self.viewpointsIndex, 0] == -1:
                self.viewpoints_all[self.viewpointsIndex, 0] = int((event.x() / self.label_mask.pixmap().width()) * width)
                self.viewpoints_all[self.viewpointsIndex, 1] = int((event.y() / self.label_mask.pixmap().height()) * height)
                self.viewpoints_all[self.viewpointsIndex, 2] = int((event.x() / self.label_mask.pixmap().width()) * width)
                self.viewpoints_all[self.viewpointsIndex, 3] = int((event.y() / self.label_mask.pixmap().height()) * height)
            else:
                self.viewpoints_all[self.viewpointsIndex, self.viewpointsCounter] = int((event.x() / self.label_mask.pixmap().width()) * width)
                self.viewpoints_all[self.viewpointsIndex, self.viewpointsCounter + 1] = int((event.y() / self.label_mask.pixmap().height()) * height)

        self.pick_view()

    def pick_view(self, img = None):
        if self.new_image:
            if self.img_mask_draw is None:
                self.generataDrawimages()


            frame1 = self.img_src_draw.copy()
            frame2 = self.img_mask_draw.copy()

            if self.view_value and self.viewpoints_all[self.viewpointsIndex,0] != -1 and self.viewpoints_all[self.viewpointsIndex,2] != -1:
                length = math.sqrt(math.pow(self.viewpoints_all[self.viewpointsIndex,0]-self.viewpoints_all[self.viewpointsIndex,2],2)+math.pow(self.viewpoints_all[self.viewpointsIndex,1]-self.viewpoints_all[self.viewpointsIndex,3],2))
                length = length if length > 200 else 200
                frame1 = cv2.arrowedLine(frame1, (self.viewpoints_all[self.viewpointsIndex,0], self.viewpoints_all[self.viewpointsIndex,1]),(self.viewpoints_all[self.viewpointsIndex,2], self.viewpoints_all[self.viewpointsIndex,3]), 2555, 20, tipLength=100/length )
                frame2 = cv2.arrowedLine(frame2, (self.viewpoints_all[self.viewpointsIndex,0], self.viewpoints_all[self.viewpointsIndex,1]), (self.viewpoints_all[self.viewpointsIndex,2], self.viewpoints_all[self.viewpointsIndex,3]), 255, 20, tipLength=100/length)

            if self.pick_value and self.pickpointsCounter >= 0 and self.pickpoints[self.pickpointsCounter] >= 0:
                frame1 = cv2.circle(frame1, (self.pickpoints[self.pickpointsCounter], self.pickpoints[self.pickpointsCounter+1]), 50, (0,255,0), 20)
                frame2 = cv2.circle(frame2, (self.pickpoints[self.pickpointsCounter], self.pickpoints[self.pickpointsCounter+1]), 50, (0,255,0), 20)



            frame = frame1
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
            self.label_src.setPixmap(QPixmap.fromImage(image).scaled(self.img_width, 10000, QtCore.Qt.KeepAspectRatio))
            frame = frame2
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
            self.label_mask.setPixmap(QPixmap.fromImage(image).scaled(self.img_width, 10000, QtCore.Qt.KeepAspectRatio))

    def pre_view_angle(self, update=True):
        if update:
            self.viewpoints_all[self.viewpointsIndex, 4] = int(self.props.angle.value())
            self.viewpoints_all[self.viewpointsIndex, 7] = int(self.props.view_type.currentIndex())
            if self.props.camera.currentText() == '2':
                self.viewpoints_all[self.viewpointsIndex, 5] = int(self.props.angle_int.value())
                self.viewpoints_all[self.viewpointsIndex, 6] = int(self.props.pic_amount.value())
        self.viewpointsIndex -= 1

        if self.viewpointsIndex < 0:
            self.viewpointsIndex = self.viewpoints_all.shape[0]-1

        if self.viewpoints_all[0,0] != -1:
            self.props.label_view_number.setText(str(int(self.viewpointsIndex)+int(self.viewpoints_all[-1,0] != -1)) + " of " + str(self.viewpoints_all.shape[0]-int(self.viewpoints_all[-1,0] == -1)) + " view(s)")
        else:
            self.props.label_view_number.setText("No views")
        self.props.angle.valueChanged.disconnect()
        self.props.angle_int.valueChanged.disconnect()
        self.props.pic_amount.valueChanged.disconnect()
        self.props.angle.setValue(self.viewpoints_all[self.viewpointsIndex, 4])
        self.props.angle_int.setValue(self.viewpoints_all[self.viewpointsIndex, 5])
        self.props.pic_amount.setValue(self.viewpoints_all[self.viewpointsIndex, 6])
        self.props.view_type.setCurrentIndex(self.viewpoints_all[self.viewpointsIndex, 7])
        self.props.angle.valueChanged.connect(self.props.notSaved)
        self.props.angle_int.valueChanged.connect(self.props.notSaved)
        self.props.pic_amount.valueChanged.connect(self.props.notSaved)
        self.generataDrawimages()
        self.pick_view()

    def next_view_angle(self, update = True):
        if update:
            self.viewpoints_all[self.viewpointsIndex, 4] = int(self.props.angle.value())
            self.viewpoints_all[self.viewpointsIndex, 7] = int(self.props.view_type.currentIndex())
            if self.props.camera.currentText() == '2':
                self.viewpoints_all[self.viewpointsIndex, 5] = int(self.props.angle_int.value())
                self.viewpoints_all[self.viewpointsIndex, 6] = int(self.props.pic_amount.value())
        self.viewpointsIndex += 1
        if self.viewpointsIndex >= self.viewpoints_all.shape[0]:
            self.viewpointsIndex = 0

        if self.viewpoints_all[0,0] != -1:
            self.props.label_view_number.setText(str(int(self.viewpointsIndex)+int(self.viewpoints_all[-1,0] != -1)) + " of " + str(self.viewpoints_all.shape[0]-int(self.viewpoints_all[-1,0] == -1)) + " view(s)")
        else:
            self.props.label_view_number.setText("No views")

        self.props.angle.valueChanged.disconnect()
        self.props.angle_int.valueChanged.disconnect()
        self.props.pic_amount.valueChanged.disconnect()
        self.props.angle.setValue(self.viewpoints_all[self.viewpointsIndex, 4])
        self.props.angle_int.setValue(self.viewpoints_all[self.viewpointsIndex, 5])
        self.props.pic_amount.setValue(self.viewpoints_all[self.viewpointsIndex, 6])
        self.props.view_type.setCurrentIndex(self.viewpoints_all[self.viewpointsIndex, 7])
        self.props.angle.valueChanged.connect(self.props.notSaved)
        self.props.angle_int.valueChanged.connect(self.props.notSaved)
        self.props.pic_amount.valueChanged.connect(self.props.notSaved)
        self.generataDrawimages()
        self.pick_view()

    def generataDrawimages(self):

        self.img_mask_draw = self.img_mask.copy()
        self.img_src_draw = self.img_src.copy()
        if self.view_value:
            for i in range(self.viewpoints_all.shape[0]):
                if i != self.viewpointsIndex:
                    length = math.sqrt(math.pow(
                        self.viewpoints_all[i, 0] - self.viewpoints_all[i, 2], 2) + math.pow(self.viewpoints_all[i, 1] - self.viewpoints_all[i, 3], 2))
                    length = length if length > 200 else 200
                    self.img_mask_draw = cv2.arrowedLine(self.img_mask_draw, (self.viewpoints_all[i, 0], self.viewpoints_all[i, 1]), (self.viewpoints_all[i, 2], self.viewpoints_all[i, 3]), (255,255,0), 20, tipLength=100/length)
                    self.img_src_draw  = cv2.arrowedLine(self.img_src_draw , (self.viewpoints_all[i, 0], self.viewpoints_all[i, 1]), (self.viewpoints_all[i, 2], self.viewpoints_all[i, 3]), (255,255,0), 20, tipLength=100/length)

        if self.pick_value:
            if self.pickpointsCounter != 0 and self.pickpoints[0] >= 0:
                        self.img_mask_draw= cv2.circle(self.img_mask_draw, (self.pickpoints[0], self.pickpoints[1]), 50, (0,255,0), 20)
                        self.img_src_draw = cv2.circle(self.img_src_draw, (self.pickpoints[0], self.pickpoints[1]), 50, (0, 255, 0),20)

            if self.pickpointsCounter != 2 and self.pickpoints[2] >= 0:
                        self.img_mask_draw= cv2.circle(self.img_mask_draw, (self.pickpoints[2], self.pickpoints[3]), 50, (0,255,0), 20)
                        self.img_src_draw = cv2.circle(self.img_src_draw, (self.pickpoints[2], self.pickpoints[3]), 50, (0, 255, 0),20)

        self.img_mask_draw = cv2.addWeighted(self.img_mask_draw, 0.3, self.img_mask, 0.7, 0)
        self.img_src_draw = cv2.addWeighted(self.img_src_draw, 0.3, self.img_src, 0.7, 0)

    def add_view_angle(self):
        if self.viewpoints_all[-1,0] != -1:
            self.viewpointsCounter = 2
            self.viewpoints_all = np.concatenate((self.viewpoints_all, np.ones((1,8),np.intc)*(-1)))
            self.viewpoints_all[-1,7] = 0
            self.viewpointsIndex = int(self.viewpoints_all.shape[0]-2)
            self.viewpoints_all[-1,4] = 0
            self.next_view_angle()

    def delete_view_angle(self):
        if self.viewpoints_all.shape[0] > 1:
            self.viewpoints_all= np.delete(self.viewpoints_all,[self.viewpointsIndex], 0)
        else:
            self.viewpoints_all = np.ones((1, 8), np.intc) * (-1)

        if self.props.camera.currentText() == '1':
            self.cam_viewpoints[0] = self.viewpoints_all.copy()
        else:
            self.cam_viewpoints[1] = self.viewpoints_all.copy()

        self.pre_view_angle(False)

    def notSaved(self):
        self.viewpoints_all[self.viewpointsIndex, 4] = int(self.props.angle.value())
        cam_index = self.props.camera.currentIndex()

        if self.props.view_type.currentText() == 'Point':
            self.cam_viewpoints[cam_index] = self.viewpoints_all.copy()
        elif self.props.view_type.currentText() == 'Line sweep':
            self.viewpoints_all[self.viewpointsIndex, 6] = int(self.props.pic_amount.value())
            self.cam_viewpoints[cam_index] = self.viewpoints_all.copy()
        elif self.props.view_type.currentText() == 'Point rotation':
            self.viewpoints_all[self.viewpointsIndex, 5] = int(self.props.angle_int.value())
            self.viewpoints_all[self.viewpointsIndex, 6] = int(self.props.pic_amount.value())
            self.cam_viewpoints[cam_index] = self.viewpoints_all.copy()

        self.pickpoints[4] = self.cam_viewpoints[0].shape[0]
        points = np.concatenate((np.reshape(self.pickpoints, [1, 8]), self.cam_viewpoints[0], self.cam_viewpoints[1]))
        if not np.array_equal(points,self.saved_points):
            self.props.label_save.setText("Not Saved\n")
            self.props.label_save.setStyleSheet("font-size: "+ self.props.text_size3+"; color: red")
        else:
            self.props.label_save.setText(" \n")

    def save_view(self):
        img_src = cv2.resize(self.img_src,(int(self.img_src.shape[1]/10), int(self.img_src.shape[0]/10)))
        img_mask = cv2.resize(self.img_mask, (int(self.img_mask.shape[1] / 10), int(self.img_mask.shape[0] / 10)))
        cv2.imwrite(rootDir + 'Library\\' + self.props.type_name.text() + '\\src\\'  +self.props.view_name.text() + ".png",  img_src[50:-50, 50:-50]);
        cv2.imwrite(rootDir + 'Library\\' + self.props.type_name.text() + '\\mask\\' + self.props.view_name.text() + ".png", img_mask[50:-50, 50:-50]);
        view_angle = np.expand_dims(np.array((int(self.props.angle.value()),0)),0)

        self.viewpoints_all[self.viewpointsIndex,4] = int(self.props.angle.value())
        self.viewpoints_all[self.viewpointsIndex, 7] = int(self.props.view_type.currentIndex())

        if self.props.camera.currentText() == '1':
            self.cam_viewpoints[0] = self.viewpoints_all.copy()
        else:
            self.cam_viewpoints[1] = self.viewpoints_all.copy()

        if self.props.view_type.currentText() == 'Line sweep':
            self.viewpoints_all[self.viewpointsIndex, 6] = int(self.props.pic_amount.value())
        elif self.props.view_type.currentText() == 'Point rotation':
            self.viewpoints_all[self.viewpointsIndex, 5] = int(self.props.angle_int.value())
            self.viewpoints_all[self.viewpointsIndex, 6] = int(self.props.pic_amount.value())

        self.pickpoints[4] = self.cam_viewpoints[0].shape[0]
        points = np.concatenate((np.reshape(self.pickpoints, [1,8]), self.cam_viewpoints[0],self.cam_viewpoints[1]))
        self.saved_points = points.copy()


        temp = points.copy()
        print(temp)
        points[:, 0:4,] = points[:, 0:4,]/10 - 50
        points[temp== -1] = -1
        print("SAVED", points)

        np.save(rootDir + 'Library\\'+ self.props.type_name.text() + '\points\\'  +self.props.view_name.text() + "_pickpoints", points)
        self.props.origina_data =  points.copy()
        self.props.combo1.addItem(self.props.type_name.text())
        index = self.props.combo1.findText(self.props.type_name.text())
        if index == -1:
            index = 0;
        # FIND BUG!
        self.props.combo1.setCurrentIndex(index)
        self.props.combo2.addItem(self.props.view_name.text())
        index = self.props.combo2.findText(self.props.view_name.text())
        if index == -1:
            index = 0;
        self.props.combo2.setCurrentIndex(index)
        self.props.update_param(self.props)

    def close_library(self):
        self.signal_exit_library.emit()
        self.close()


# This class contains the control panel for the externael add window. (not used)
class AddObjectWidget(QWidget):
    """
    Widget for editing OBJECT parameters
    """
    signal_object_changed = pyqtSignal(name='objectChanged')
    signal_save_image = pyqtSignal()
    signal_pick = pyqtSignal()
    signal_view = pyqtSignal()

    def __init__(self, parent=None):
        super(AddObjectWidget, self).__init__(parent)

        self.windows = list()


        self.button_image = QPushButton("&Get image", self)
        self.button_image.clicked.connect(self.get_image)

        self.button_save = QPushButton("&Save view", self)
        self.button_save.clicked.connect(self.save_image)
        grip_label = QLabel("Select grip points: ")
        self.button_grip =  QCheckBox("")
        self.button_grip.setChecked(False)
        self.button_grip.toggled.connect(lambda: self.btnstate(self.button_grip, "grip"))
        view_label = QLabel("Select view point: ")
        self.button_view = QCheckBox("")
        self.button_view.setChecked(False)
        self.button_view.toggled.connect(lambda: self.btnstate(self.button_view, "view"))


        tp_label = QLabel("Name:")
        self.type_name = QLineEdit()

        angle_label = QLabel("View angle:")
        self.angle = QLineEdit()
        self.angle.setText('0')
        self.angle.setValue(0)
        self.onlyInt = QIntValidator()
        self.angle.setValidator(self.onlyInt)

        gbox = QGridLayout()
        gbox.addWidget(tp_label, 1, 0)
        gbox.addWidget(self.type_name, 1, 1)
        gbox.addWidget(self.button_image, 0, 0)
        gbox.addWidget(grip_label, 2, 0)
        gbox.addWidget(self.button_grip, 2, 1)
        gbox.addWidget(view_label, 3, 0)
        gbox.addWidget(self.button_view, 3, 1)
        gbox.addWidget(angle_label, 4, 0)
        gbox.addWidget(self.angle, 4, 1)

        gbox.addWidget(self.button_save, 5, 1)

        vbox = QVBoxLayout()
        vbox.addLayout(gbox)
        vbox.addStretch(1.0)

        self.setLayout(vbox)

    def save_image(self, option):
        self.signal_save_image.emit()



    def get_image(self, option):
        self.signal_object_changed.emit()
        self.button_grip.setChecked(False)
        self.button_view.setChecked(False)

    def btnstate(self, b, signal):
        if signal == "grip":
            if b.isChecked() == True:
                b.text() + " is selected"
                self.button_view.setChecked(False)
            else:
                b.text() + " is deselected"
            self.signal_pick.emit()
        elif signal == "view":
            if b.isChecked() == True:
                b.text() + " is selected"
                self.button_grip.setChecked(False)
            else:
                b.text() + " is deselected"
            self.signal_view.emit()
# This class contains the main window for theexternael add window, where all the pictures are being set. (not used)
class AddWindow(QMainWindow):

    def __init__(self, type_name):
        QMainWindow.__init__(self)
        self.type_name = type_name
        self.resize(100, 100)
        self.setWindowTitle("Type: " + type_name)

        splitter = QSplitter(Qt.Horizontal)

        self.img_src = cv2.imread('../../../Test_Images/4_blocks_7.bmp')
        self.img_mask = cv2.imread('../../../Test_Images/4_blocks_7.bmp')
        self.img_src_draw = cv2.imread('../../../Test_Images/4_blocks_7.bmp')
        self.img_mask_draw = cv2.imread('../../../Test_Images/4_blocks_7.bmp')

        self.pickpoints = np.ones((8),np.intc)*(-1)
        self.pickpointsCounter = 0
        self.pick_value = False

        self.viewpoints = np.ones((1,8),np.intc)*(-1)
        self.viewpointsCounter = 2
        self.view_value = False

        self.new_image = False

        self.props = AddObjectWidget()
        splitter.addWidget(self.props)


        self.img_src_disp = QLabel(self)
        self.pixmap = QPixmap().scaled(256, 256, QtCore.Qt.KeepAspectRatio)
        self.img_src_disp.setPixmap(self.pixmap)
        self.img_src_disp.setAlignment(Qt.AlignAbsolute)
        self.img_src_disp.mousePressEvent = self.mouseEvent
        splitter.addWidget(self.img_src_disp)

        self.img_mask_disp = QLabel(self)
        self.pixmap = QPixmap().scaled(256, 256, QtCore.Qt.KeepAspectRatio)
        self.img_mask_disp.setPixmap(self.pixmap)
        self.img_mask_disp.setAlignment(Qt.AlignAbsolute)
        self.img_mask_disp.mousePressEvent = self.mouseEvent
        splitter.addWidget(self.img_mask_disp)



        self.setCentralWidget(splitter)

        self.props.signal_object_changed.connect(self.update_view)
        self.props.signal_save_image.connect(self.save_view)
        self.props.signal_pick.connect(self.signal_pick_handle)
        self.props.signal_view.connect(self.signal_view_handle)


    def update_view(self):
        self.pickpoints = np.ones((8),np.intc)*(-1)
        self.pickpointsCounter = 0


        self.viewpoints = np.ones((1,8),np.intc)*(-1)
        self.viewpointsCounter = 2

        img = Im.get_image()

        # Convert image format
        s_img = np.zeros((img.shape[0]-20, img.shape[1]-20, 3), dtype=np.uint8)
        s_img[:, :, 0] = img[10:-10,10:-10]
        s_img[:, :, 1] = img[10:-10,10:-10]
        s_img[:, :, 2] = img[10:-10,10:-10]
        imgray = cv2.cvtColor(s_img, cv2.COLOR_BGR2GRAY)


        # Find contours
        kernel = np.ones((7, 7), np.uint8)  # avoid colapsing contours, when objects are close together
        ret, thresh = cv2.threshold(imgray, 25, 255, cv2.THRESH_BINARY_INV)
        contours, hierarchy = cv2.findContours(cv2.erode(thresh, kernel, iterations=1), cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)
        mask = np.zeros(thresh.shape, np.uint8)


        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area > 500 and area < 100000:
                ret, thresh = cv2.threshold(imgray, 25, 255, cv2.THRESH_BINARY_INV)
                rect = cv2.minAreaRect(contours[i])
                cv2.drawContours(mask, [contours[i]], -1, 255, cv2.FILLED)

                self.img_mask, self.img_src, top_ori = Im.getSubImage(rect, imgray, mask)



                frame = self.img_src

                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)



                image = QImage(frame, frame.shape[1], frame.shape[0],frame.strides[0], QImage.Format_RGB888)
                self.img_src_disp.setPixmap(QPixmap.fromImage(image).scaled(256, 256, QtCore.Qt.KeepAspectRatio))

                frame = self.img_mask
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                image = QImage(frame, frame.shape[1], frame.shape[0],
                               frame.strides[0], QImage.Format_RGB888)

                self.img_mask_disp.setPixmap(QPixmap.fromImage(image).scaled(256, 256, QtCore.Qt.KeepAspectRatio))

                self.new_image = True
                break

    def save_view(self):
        cv2.imwrite(rootDir + 'Library/' + self.type_name + '/src/'  +self.props.type_name.text() + ".png",  self.img_src);
        cv2.imwrite(rootDir + 'Library/' + self.type_name + '/mask/' + self.props.type_name.text() + ".png",self.img_mask);
        view_angle = np.expand_dims(np.array((int(self.props.angle.value()),0)),0)
        np.save(rootDir + 'Library/'+ self.type_name + '/points/'  +self.props.type_name.text() + "_pickpoints", np.concatenate((self.viewpoints, self.pickpoints, view_angle)))
        self.props.type_name.setText("")
        #self.props.angle.setText("0")
        self.angle.setValue(0)
        print(np.concatenate((self.viewpoints, self.pickpoints, view_angle)))

    def signal_pick_handle(self):
        self.pick_value = not self.pick_value
        self.view_value = False
        self.pick_view()

    def signal_view_handle(self):
        self.view_value = not self.view_value
        self.pick_value = False
        self.pick_view()

    def mouseEvent(self, event):
        if self.pick_value:
            if(event.x() < self.img_mask_disp.pixmap().width() and event.y() < self.img_mask_disp.pixmap().height()):
                height, width = self.img_mask.shape
                self.pickpoints[self.pickpointsCounter,0] = int(( event.x() / self.img_mask_disp.pixmap().width())*width)
                self.pickpoints[self.pickpointsCounter,1] = int((event.y() / self.img_mask_disp.pixmap().height()) * height)
                if self.pickpointsCounter == 0:
                    self.pickpointsCounter = 1
                else:
                    self.pickpointsCounter = 0
        elif self.view_value:
            if(event.x() < self.img_mask_disp.pixmap().width() and event.y() < self.img_mask_disp.pixmap().height()):
                height, width = self.img_mask.shape
                self.viewpoints[self.viewpointsCounter ] = int((event.x() / self.img_mask_disp.pixmap().width()) * width)
                self.viewpoints[self.viewpointsCounter + 1] = int((event.y() / self.img_mask_disp.pixmap().height()) * height)
                if self.viewpointsCounter == 0:
                    self.viewpointsCounter = 2
                else:
                    self.viewpointsCounter = 0


        self.pick_view()

    def pick_view(self):
        if self.new_image:
            self.img_mask_draw = self.img_mask.copy()
            self.img_src_draw = self.img_src.copy()
            if self.pick_value:
                if self.pickpoints[0, 0] >= 0:
                    self.img_mask_draw = cv2.circle(self.img_mask_draw, (self.pickpoints[0, 0], self.pickpoints[0, 1]), 5, (125), 2)
                    self.img_src_draw = cv2.circle(self.img_src_draw, (self.pickpoints[0, 0], self.pickpoints[0, 1]), 5, (125), 2)
                if self.pickpoints[1, 0] >= 0:
                    self.img_mask_draw = cv2.circle(self.img_mask_draw, (self.pickpoints[1, 0], self.pickpoints[1, 1]), 5,(125), 2)
                    self.img_src_draw = cv2.circle(self.img_src_draw, (self.pickpoints[1, 0], self.pickpoints[1, 1]), 5, (125),2)
            elif self.view_value:
                if self.viewpoints[0] >= 0:
                    if self.viewpointsCounter == 0:
                        self.img_mask_draw= cv2.arrowedLine(self.img_mask_draw, (self.viewpoints[0], self.viewpoints[1]),(self.viewpoints[2], self.viewpoints[3]), 125, 2 )
                        self.img_src_draw = cv2.arrowedLine(self.img_src_draw, (self.viewpoints[0], self.viewpoints[1]), (self.viewpoints[2], self.viewpoints[3]), 125, 2)
                    else:
                        self.img_mask_draw = cv2.circle(self.img_mask_draw, (self.viewpoints[0], self.viewpoints[1]), 1, (125), 2)
                        self.img_src_draw = cv2.circle(self.img_src_draw, (self.viewpoints[0], self.viewpoints[1]), 1, (125), 2)


            frame = self.img_src_draw
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            image = QImage(frame, frame.shape[1], frame.shape[0],
                           frame.strides[0], QImage.Format_RGB888)

            self.img_src_disp.setPixmap(QPixmap.fromImage(image).scaled(256, 256, QtCore.Qt.KeepAspectRatio))

            frame = self.img_mask_draw
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            image = QImage(frame, frame.shape[1], frame.shape[0],
                           frame.strides[0], QImage.Format_RGB888)

            self.img_mask_disp.setPixmap(QPixmap.fromImage(image).scaled(256, 256, QtCore.Qt.KeepAspectRatio))


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    appQt = QApplication(sys.argv)
    RectScreen0 = appQt.desktop().screenGeometry(0)
    win = MainWindow(RectScreen0.width(),RectScreen0.height())
    win.show()
    win.move(RectScreen0.left(), RectScreen0.top())
    win.resize(RectScreen0.width(), RectScreen0.height())
    appQt.exec_()
