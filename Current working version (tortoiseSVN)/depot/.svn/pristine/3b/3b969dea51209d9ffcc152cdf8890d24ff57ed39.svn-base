import sys
from PyQt5.QtCore import Qt, QPoint, QObject, QTimer, QThread, pyqtSlot, QRect
from PyQt5.QtWidgets import QMainWindow, QApplication, QLabel, QSplitter, QWidget, QShortcut, QMessageBox, QVBoxLayout, QGridLayout, QInputDialog, QPushButton, QComboBox, QHBoxLayout
from PyQt5.QtGui import QPixmap, QPainter, QPen, QImage, QBrush, QKeySequence, QCursor
import cv2
import numpy as np
import predict
from win32api import GetSystemMetrics
import train
import os
import time
import glob




class Worker(QThread):
    def __init__(self, model_path, fn_train, epochs, callback, dir, update_model):
        QThread.__init__(self)
        # Store constructor arguments (re-used for processing)
        self.train = fn_train
        self.epochs =epochs
        self.Update_Net = callback
        self.dir =dir
        self.dir_mask =dir + 'mask/'
        #self.dir_save = os.path.split(model_path)[0] + '/'
        self.dir_save = dir + 'Unet/'
        self.update_model_callback = update_model

        if not os.path.isdir(self.dir_save):
            os.mkdir(self.dir_save)
        time_stamp = time.strftime("%Y%m%d-%H%M%S")
        self.save_path = self.dir_save + time_stamp + '_UPDATED_MODEL.pth'
        print("Save path:", self.save_path)
        print(self.dir_save, os.path.isdir(self.dir_save))
        files = os.listdir(self.dir_save)
        print(files,len(files))
        if files:
            paths = [os.path.join(self.dir_save, basename) for basename in files]
            model_path = max(paths, key=os.path.getctime)
        else:
            model_path = self.dir_save + "NONE"
        print("Model path:", model_path)

        self.UnetModel, self.device = predict.getModel(model_path)
        print(self.device)

    @pyqtSlot()
    def run(self):
        self.train(self.UnetModel,
                   self.device,
                   self.dir,
                   self.dir_mask,
                   self.save_path,
                   self.epochs,
                   batch_size=1,
                   lr=0.0003,
                   val_percent=0.1,
                   img_scale=0.98)

        self.update_model_callback(self.save_path,cpu=True)



class MainObjectWidget(QWidget):
    """
    Widget for editing OBJECT parameters
    """

    def __init__(self, screen_width, screen_height, parent=None):
        super(MainObjectWidget, self).__init__(parent)


        self.screen_width = screen_width
        self.screen_height = screen_height


        self.text_size1 = str(int(self.screen_width * 0.0075 * 1.5)) + 'px'
        self.text_size2 = str(int(self.screen_width * 0.005 * 1.5)) + 'px'


        self.Program_label = QLabel("JLI\n"
                                    "Annotation tool")
        self.Program_label.setAlignment(Qt.AlignCenter)
        self.button_folder = QPushButton("Set image folder/file", self)
        self.view_label = QLabel("Select view:")
        self.view_type = QComboBox(self)
        self.view_type.addItems(("Image + Mask", "Image", "Mask", "Image crop", "Reverser image crop"))
        self.button_auto_save = QPushButton("Auto save", self)
        self.button_help = QPushButton("Help", self)
        self.button_help.clicked.connect(self.Help)
        self.button_exit = QPushButton("Exit", self)
        self.button_exit.clicked.connect(self.Exit)

        gbox = QGridLayout()
        i = 0
        gbox.addWidget(self.Program_label, i, 0, 1, 2); i+= 1
        gbox.addWidget(self.button_folder, i, 0, 1, 2); i+= 1
        gbox.addWidget(self.view_label, i, 0);
        gbox.addWidget(self.view_type, i, 1);           i += 1
        gbox.addWidget(self.button_auto_save, i, 0, 1, 2);   i+= 1
        gbox.addWidget(self.button_help, i, 0, 1, 2);   i+= 1
        gbox.addWidget(self.button_exit, i, 0, 1, 2);   i+= 1

        self.Program_label.setStyleSheet('font-size: ' + self.text_size1 + '; font-weight: bold')
        self.button_folder.setStyleSheet('QPushButton{font-size: ' + self.text_size2 + '}')
        self.view_label.setStyleSheet('font-size: ' + self.text_size2 + '; font-weight: bold')
        self.view_type.setStyleSheet('QComboBox{font-size: ' + self.text_size2 + '}')
        self.button_auto_save.setStyleSheet('QPushButton{font-size: ' + self.text_size2 + '; font-weight: bold; background-color: gray}')
        self.button_help.setStyleSheet('QPushButton{font-size: '+ self.text_size2 +'}')
        self.button_exit.setStyleSheet('QPushButton{font-size: '+ self.text_size2 +'}')


        vbox = QVBoxLayout()
        vbox.addLayout(gbox)
        vbox.addStretch(1.0)

        self.setLayout(vbox)

    def Help(self):
        if False:
            message = QMessageBox.question(self, 'Help box', "Left mouse click: \t\t\tDraw mask\n"
                                                   "Right mouse click: \t\t\tErase mask\n"
                                                   "Scroll: \t\t\t\t\t\tChange brush size\n"
                                                   "Control + Scroll: \t\t\t\tZoom\n"
                                                   "Control + Right mouse click: \tReset Zoom\n"
                                                   "Control + Z: \t\t\t\t\tReverse last action\n"
                                                   "Control + Alt + Z: \t\t\tRedo last action\n"
                                                   "Left Arrow: \t\t\t\t\tPrevious image\n"
                                                   "Right Arrow: \t\t\t\tNext image\n"
                                                   "Alt + D: \t\t\t\t\t\tClear mask\n"
                                                   "Alt + Left mouse click: \t\tSegmentation draw helper\n"
                                                   "Alt + P: \t\t\t\t\t\tPredict mask\n a" #Limit of text field
                                                   "Shif + Left mouse click: \t\tFill contour",
                                           QMessageBox.Ok)

        message = QMessageBox()
        message.setIcon(QMessageBox.Information)
        message.setWindowTitle("Help Box")
        message.setText("Left mouse click: \t\t\tDraw mask          \n"
                        "Right mouse click: \t\t\tErase mask          \n"
                        "Scroll: \t\t\t\t\t\tChange brush size          \n"
                        "Control + Scroll: \t\t\t\tZoom          \n"
                        "Control + Right mouse click: \tReset Zoom          \n"
                        "Control + Z: \t\t\t\t\tReverse last action          \n"
                        "Control + Alt + Z: \t\t\tRedo last action          \n"
                        "Control + S: \t\t\t\t\tSave current Mask\n"
                        "Left Arrow: \t\t\t\t\tPrevious image          \n"
                        "Right Arrow: \t\t\t\tNext image          \n"
                        "Alt + D: \t\t\t\t\t\tClear mask          \n"
                        "Alt + Left mouse click: \t\tSegmentation draw helper          \n"
                        "Alt + P: \t\t\t\t\t\tPredict mask          \n",)

        message.setStandardButtons(QMessageBox.Ok)

        retval = message.exec_()

    def Exit(self):
        QApplication.quit()


class MainWindow(QMainWindow):

    def __init__(self, screen_width = None, screen_height = None):
        QMainWindow.__init__(self)

        self.setWindowTitle("JLI Annotation tool")
        self.auto_save = True

        #img_path = simpledialog.askstring(title="Get image path", prompt="Enter image path or folder path:")

        if screen_width is None or screen_height is None:
            self.screen_width = GetSystemMetrics(0)
            self.screen_height = GetSystemMetrics(1)
        else:
            self.screen_width = screen_width
            self.screen_height = screen_height


        self.props = MainObjectWidget(self.screen_width, self.screen_height)
        self.props.setMaximumWidth(500)



        self.splitter = QSplitter(Qt.Horizontal)
        self.splitter.addWidget(self.props)
        self.setCentralWidget(self.splitter)

        #self.setStyleSheet("QWidget {background-color:  rgb(0,0,255)}")

        self.props.button_folder.clicked.connect(self.setFolder)
        self.props.view_type.currentIndexChanged.connect(self.view_change)
        self.props.button_auto_save.clicked.connect(self.set_auto_save)

        self.setGeometry(50,50,500,500)



    def setFolder(self):
        img_path, ok = QInputDialog.getText(self, 'Get image path', 'Enter image path or folder path:')

        while True:
            if not ok:
                return

            if (os.path.isfile(img_path) or os.path.isdir(img_path)):
                break

            img_path, ok = QInputDialog.getText(self, 'Get image path', 'The entered file/folder path does not exist: Enter new image path or folder path:')

        if os.path.isdir(img_path):
            img_path += '/'

        # img_path = "T:/Usr/ML/Rail images/"

        print(img_path)

        if hasattr(self, 'smartAnnotationWidget'):
            self.smartAnnotationWidget.set_new_folder(img_path)
        else:
            self.smartAnnotationWidget = Menu(img_path,)
            self.splitter.addWidget(self.smartAnnotationWidget)
            #self.smartAnnotationWidget.setGeometry(0,300,self.smartAnnotationWidget.width(),self.smartAnnotationWidget.height())




            self.keyPressEvent= self.smartAnnotationWidget.keyPressEvent
            self.keyReleaseEvent = self.smartAnnotationWidget.keyReleaseEvent
            self.props.button_folder.keyPressEvent = self.smartAnnotationWidget.keyPressEvent
            self.props.button_help.keyPressEvent = self.smartAnnotationWidget.keyPressEvent
            self.props.button_exit.keyPressEvent = self.smartAnnotationWidget.keyPressEvent


            #self.setFixedHeight(self.smartAnnotationWidget.height())
        if self.props.geometry().height() > self.smartAnnotationWidget.height():
            print("CHANGE POSITION", )
            print(self.smartAnnotationWidget.geometry().x(),self.smartAnnotationWidget.geometry().y())
            self.smartAnnotationWidget.setGeometry(self.smartAnnotationWidget.geometry().x(),int((self.props.geometry().height()-self.smartAnnotationWidget.geometry().height())/2),self.smartAnnotationWidget.geometry().width(),self.smartAnnotationWidget.geometry().height())


        self.setGeometry(self.geometry().x(), self.geometry().y(), 500, 500)
        self.setFixedWidth(500+self.smartAnnotationWidget.ori_image.shape[1])
        self.update()



    def view_change(self):
        if hasattr(self, 'smartAnnotationWidget'):
            self.smartAnnotationWidget.change_view(self.props.view_type.currentText())

    def set_auto_save(self):
        if self.auto_save:
            self.auto_save = False
            self.props.button_auto_save.setStyleSheet('QPushButton{font-size: ' + self.props.text_size2 + ';  background-color: None}')
            if hasattr(self, 'smartAnnotationWidget'):
                self.smartAnnotationWidget.auto_save = False
        else:
            self.auto_save = True
            self.props.button_auto_save.setStyleSheet('QPushButton{font-size: ' + self.props.text_size2 + '; font-weight: bold; background-color: gray}')
            if hasattr(self, 'smartAnnotationWidget'):
                self.smartAnnotationWidget.auto_save = True


class Menu(QWidget):
    def __init__(self, img_path, width = None , height = None, callback = None, parent = None, offset = None, img_save_path = None ):
        super(Menu, self).__init__(parent)
        #self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        if img_save_path is None:
            self.img_save_path = os.path.dirname(img_path) +"/mask/"
        else:
            self.img_save_path = img_save_path
        self.img_path = img_path

        file = None

        print("Start", img_path)

        if os.path.isfile(self.img_path):
            file = os.path.basename(self.img_path)
            self.img_path = os.path.dirname(self.img_path) +"/"

        print("File", file)

        self.files = os.listdir(self.img_path)
        self.files = [file for file in self.files if file.endswith('.png') or file.endswith('.jpg')]
        self.img_folder_path = self.img_path
        self.files.sort()
        files = os.listdir(self.img_save_path)
        self.file_index = 0

        print("Path", self.img_folder_path)
        #Start with the newest mask
        if file is None:
            if len(files) > 0:
                self.file_index = len(files) - 1
        else:
            self.file_index = self.files.index(file)



        self.img_path = self.img_folder_path + self.files[self.file_index]

        print(self.img_path)



        self.callback = callback
        self.drawing = False
        self.erasing = False
        self.lock_drawing = False
        self.auto_save = True
        self.draw_index = 0
        self.zoom_level = 0
        self.Lock_drawing = False
        self.start_pos = (0,0)
        self.lastPoint = (0,0)
        self.lastClickPoint = (0,0)
        self.ori_image = cv2.imread(self.img_path)
        print("Load image:", self.img_path)
        self.draw_helper = False
        self.drawing_process_flag = False
        self.C_level = 15
        self.mouse_click = False
        self.view_type = "Image + Mask"

        # Retrain parameters
        self.auto_retrain = True
        self.retrain_frequncy = 10
        self.retrain_epoch = 5
        self.min_samples = 50
        self.last_file_amount = len(os.listdir(self.img_save_path))

        self.TrainWorker = Worker('C:/Prj/Robopick/Depot.svn/Src/unet/MODEL.pth', train.retrain_net, self.retrain_epoch, self.update_model, self.img_folder_path, self.update_model)
        #self.TrainWorker.start()

        files = os.listdir(self.img_folder_path + 'Unet/')
        if files:
            paths = [os.path.join(self.img_folder_path + 'Unet/', basename) for basename in files]
            model_path = max(paths, key=os.path.getctime)
        else:
            model_path = self.img_folder_path + 'Unet/NONE'


        self.UnetModel, self.device = predict.getModel(model_path , cpu=True)
        #self.UnetModel, self.device = predict.getModel('C:\Prj\C&C\Defects\Defects/Unet/20201014-161251_UPDATED_MODEL.pth', cpu=True)


        #self.setCursor(Qt.CrossCursor)
        self.setCursor(Qt.BlankCursor)

        if height is None or width is None:
            self.img_width = self.ori_image.shape[1]
            self.img_height = self.ori_image.shape[0]
        else:
            self.img_width = width
            self.img_height = height


        self.ori_mask = np.zeros(self.ori_image.shape, dtype=np.uint8)
        self.new_mask = True


        if os.path.isfile(self.img_save_path+self.files[self.file_index]):
            temp = cv2.imread(self.img_save_path+self.files[self.file_index])
            print("Load image:", self.img_save_path+self.files[self.file_index])
            self.ori_mask[:,:,1] = temp[:,:,1]
            self.new_mask = False

        self.ori_mask_init = self.ori_mask.copy()

        self.src_img = cv2.resize(self.ori_image,(self.img_width, self.img_height))
        self.mask = cv2.resize(self.ori_mask,(self.img_width, self.img_height))
        self.mask_changes = [self.ori_mask.copy()]
        self.img_show = QPixmap(self.img_path).scaled(int(self.img_width), int(self.img_height),Qt.KeepAspectRatio)
        self.img_temp = self.img_show.copy()
        self.image = QLabel()
        self.image.setPixmap(self.img_show)
        self.image.setMouseTracking(True)
        self.radius = 100
        self.zoom_rect = [0, 0, self.ori_image.shape[0], self.ori_image.shape[1]]
        #self.resize(self.image.pixmap().width(), self.image.pixmap().height())

        self.setFixedSize(int(self.mask.shape[1]), int(self.mask.shape[0]))

        self.show()
        self.setMouseTracking(True)

        self.shortcut1 = QShortcut(QKeySequence("Ctrl+Z"), self)
        self.shortcut1.activated.connect(self.reverse_draw)
        self.shortcut1.setEnabled(True)

        self.shortcut2 = QShortcut(QKeySequence("Ctrl+Shift+Z"), self)
        self.shortcut2.activated.connect(self.redo_draw)
        self.shortcut2.setEnabled(True)

        self.shortcut3 = QShortcut(QKeySequence("Alt+D"), self)
        self.shortcut3.activated.connect(self.reset_draw)
        self.shortcut3.setEnabled(True)

        self.shortcut3 = QShortcut(QKeySequence("Alt+P"), self)
        self.shortcut3.activated.connect(self.predict_mask)
        self.shortcut3.setEnabled(True)

        self.shortcut4 = QShortcut(QKeySequence("Ctrl+S"), self)
        self.shortcut4.activated.connect(self.save)
        self.shortcut4.setEnabled(True)

        self.shortcut4 = QShortcut(QKeySequence("Shift+S"), self)
        self.shortcut4.activated.connect(self.save_special)
        self.shortcut4.setEnabled(True)

        self.timer = QTimer(self)
        self.timer.setInterval(10)          # Throw event timeout with an interval of 1000 milliseconds
        self.timer.timeout.connect(self.move_periodic)

        #frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
        #image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        image = self.get_Image()

        self.image.setPixmap(QPixmap.fromImage(image))
        self.img_temp = self.image.pixmap().copy()

        painter = QPainter(self.image.pixmap())
        painter.setPen(QPen(Qt.blue, 3, Qt.SolidLine))
        if offset is not None:
            point = QPoint(QCursor().pos().x()-offset[0],  QCursor().pos().y()-offset[1])
        else:
            point = QPoint(QCursor().pos().x(), QCursor().pos().y())
        self.start_point= point
        painter.drawEllipse(point, self.radius, self.radius)
        self.lastPoint = (point.x(),point.y())

        self.label_img = QLabel(self)
        self.label_img.setText("TEST")
        self.label_img.show()

        #self.show()
        self.update()

    def setMouseTracking(self, flag):
        def recursive_set(parent):
            for child in parent.findChildren(QObject):
                try:
                    child.setMouseTracking(flag)
                except:
                    pass
                recursive_set(child)

        QWidget.setMouseTracking(self, flag)
        recursive_set(self)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.drawPixmap(self.rect(), self.image.pixmap())
        self.update()

    def mousePressEvent(self, event):
        x = int((event.x() / self.image.pixmap().width()) * self.mask.shape[1])
        y = int((event.y() / self.image.pixmap().height()) * self.mask.shape[0])
        modifiers = QApplication.keyboardModifiers()
        self.mouse_click = True
        if modifiers == Qt.ControlModifier and event.button() == Qt.LeftButton and self.zoom_rect[2] > self.ori_image.shape[0]*0.05:
            if x < int(self.image.pixmap().width() * 0.25):
                x = int(self.image.pixmap().width() * 0.25)
            elif x > self.src_img.shape[1] - (self.image.pixmap().width() * 0.25):
                x = self.src_img.shape[1] - int(self.image.pixmap().width() * 0.25)

            if y < int(self.image.pixmap().height() * 0.25):
                y = int(self.image.pixmap().height() * 0.25)
            elif y > self.src_img.shape[0] - (self.image.pixmap().height() * 0.25):
                y = self.src_img.shape[0] - int(self.image.pixmap().height() * 0.25)

            self.zoom(True, (x,y))
            self.zoom_level += 1
            self.mouseReleaseEvent(event)

        elif modifiers == Qt.ControlModifier and (event.button() == Qt.RightButton or self.zoom_rect[2] <= self.ori_image.shape[0]*0.05):
            self.src_img = cv2.resize(self.ori_image,(self.img_width, self.img_height))
            self.zoom_level = 0
            temp = cv2.resize(self.mask, (int(self.zoom_rect[3]), int(self.zoom_rect[2])))
            temp[temp > 100] = 255
            temp[temp < 200] = 0
            self.ori_mask[int(self.zoom_rect[0]):int(self.zoom_rect[0]) + int(self.zoom_rect[2]),int(self.zoom_rect[1]):int(self.zoom_rect[1]) + int(self.zoom_rect[3]), :] = temp
            self.zoom_rect = [0, 0, self.ori_image.shape[0], self.ori_image.shape[1]]
            self.mask = self.ori_mask[int(self.zoom_rect[0]):int(self.zoom_rect[0] + self.zoom_rect[2]), int(self.zoom_rect[1]):int(self.zoom_rect[1] + self.zoom_rect[3]), :]
            self.mask = cv2.resize(self.mask, (self.src_img.shape[1], self.src_img.shape[0]))
            self.mask[self.mask > 100] = 255
            self.mask[self.mask < 200] = 0
            self.mouseReleaseEvent(event)
        elif event.button() == Qt.LeftButton:
            self.drawing = True
            if modifiers == Qt.ShiftModifier:
                self.fill_contour(event)
            else:
                self.mouseMoveEvent(event)
        elif event.button() == Qt.RightButton:
            self.erasing = True
            if modifiers == Qt.ShiftModifier:
                self.fill_contour(event, erase = True)
            else:
                self.mouseMoveEvent(event)

        self.mouse_click = False
        self.lastPoint = (x, y)
        self.lastClickPoint = (x,y)
        self.update()

    def mouseMoveEvent(self, event):
        if self.drawing_process_flag:
            return

        modifiers = QApplication.keyboardModifiers()
        if modifiers == Qt.ControlModifier:
            x = int((event.x() / self.image.pixmap().width()) *  self.mask.shape[1])
            y = int((event.y() / self.image.pixmap().height()) * self.mask.shape[0])
            check = True
            if x < int(self.image.pixmap().width() * 0.25):
                x = int(self.image.pixmap().width() * 0.25)
                self.timer.start()
                check = False
            elif x > self.src_img.shape[1] - (self.image.pixmap().width() * 0.25):
                x = self.src_img.shape[1] - int(self.image.pixmap().width() * 0.25)
                check = False
                self.timer.start()


            if y < int(self.image.pixmap().height() * 0.25):
                y = int(self.image.pixmap().height() * 0.25)
                check = False
                self.timer.start()
            elif y > self.src_img.shape[0] - (self.image.pixmap().height() * 0.25):
                y = self.src_img.shape[0] - int(self.image.pixmap().height() * 0.25)
                check = False
                self.timer.start()

            if check:
                self.timer.stop()

            self.image.setPixmap(self.img_temp)
            painter = QPainter(self.image.pixmap())
            painter.setPen(QPen(Qt.yellow, 3, Qt.SolidLine))
            # painter.drawLine(self.lastPoint, event.pos())
            #painter.drawEllipse(event.pos(), self.radius, self.radius)
            width = int(self.image.pixmap().width()*0.5)
            height = int(self.image.pixmap().height() * 0.5)
            painter.drawRect(x-int(width/2),y-int(height/2), width, height)

        elif event.buttons() and Qt.LeftButton and self.drawing:
            x = int((event.x() / self.image.pixmap().width()) *  self.mask.shape[1])
            y = int((event.y() / self.image.pixmap().height()) * self.mask.shape[0])

            in_image = x > 0 and x < self.mask.shape[1] and y > 0 and y < self.mask.shape[0]

            if self.draw_helper and in_image:

                if self.lastClickPoint != (x, y):
                    self.C_level = 15

                new_mask = self.make_patch((x, y))
                pre_mask = self.mask.copy()
                self.mask = cv2.bitwise_or(self.mask, new_mask)


                if self.mouse_click and  (np.sum(pre_mask) == np.sum(self.mask) or self.lastClickPoint == (x, y)):
                        self.C_level -= 1


            elif not self.draw_helper:
                self.mask = cv2.line(self.mask, (x, y), self.lastPoint, (0,255,0), self.radius*2, -1)


            #frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
            #image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
            image = self.get_Image()

            self.image.setPixmap(QPixmap.fromImage(image))
            painter = QPainter(self.image.pixmap())
            painter.setPen(QPen(Qt.green, 3, Qt.SolidLine))
            painter.drawEllipse(event.pos(), self.radius, self.radius)

            self.img_temp = self.image.pixmap().copy()
            self.lastPoint = (x, y)
        elif event.buttons() and Qt.RightButton and self.erasing:
            x = int((event.x() / self.image.pixmap().width()) * self.mask.shape[1])
            y = int((event.y() / self.image.pixmap().height()) * self.mask.shape[0])

            self.mask = cv2.line(self.mask, (x, y), self.lastPoint, (0, 0, 0), self.radius * 2, -1)
            #frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
            #image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
            image = self.get_Image()


            self.image.setPixmap(QPixmap.fromImage(image))
            painter = QPainter(self.image.pixmap())
            painter.setPen(QPen(Qt.red, 3, Qt.SolidLine))
            # painter.drawLine(self.lastPoint, event.pos())
            painter.drawEllipse(event.pos(), self.radius, self.radius)

            self.img_temp = self.image.pixmap().copy()
            self.lastPoint = (x, y)
        else:
            image = self.get_Image()
            self.image.setPixmap(QPixmap.fromImage(image))
            #self.image.setPixmap(self.img_temp)
            painter = QPainter(self.image.pixmap())
            painter.setPen(QPen(Qt.blue, 3, Qt.SolidLine))
            painter.drawEllipse(event.pos(), self.radius, self.radius)


        self.lastPoint = (event.x(), event.y())
        self.update()

    def wheelEvent(self, event):
        modifiers = QApplication.keyboardModifiers()
        if modifiers == Qt.ControlModifier:
            #temp = cv2.resize(self.mask, (int(self.zoom_rect[3]), int(self.zoom_rect[2])))
            #temp[temp > 100] = 255
            #temp[temp < 200] = 0
            #self.ori_mask[int(self.zoom_rect[0]):int(self.zoom_rect[0]) + int(self.zoom_rect[2]), int(self.zoom_rect[1]):int(self.zoom_rect[1]) + int(self.zoom_rect[3]), :] = temp

            if event.angleDelta().y() > 0 and self.zoom_rect[2] > self.ori_image.shape[0]*0.04:
                self.zoom_rect[0] +=  self.zoom_rect[2]*0.01#(self.zoom_rect[2]*self.lastPoint[1]/self.src_img.shape[0] - self.zoom_rect[2]/2)
                self.zoom_rect[1] +=  self.zoom_rect[2]*0.01#(self.zoom_rect[3]*self.lastPoint[0]/self.src_img.shape[1] - self.zoom_rect[3]/2)
                self.zoom_rect[2] -= self.zoom_rect[2]*0.02
                self.zoom_rect[3] -= self.zoom_rect[3]*0.02
            elif event.angleDelta().y() < 0 :
                if self.zoom_rect[0] - self.zoom_rect[2] * 0.01 >= 0:
                    self.zoom_rect[0] -= self.zoom_rect[2] * 0.01
                else:
                    self.zoom_rect[0] = 0

                if self.zoom_rect[1] - self.zoom_rect[3] * 0.01 >= 0:
                    self.zoom_rect[1] -= self.zoom_rect[3] * 0.01
                else:
                    self.zoom_rect[1] = 0

                if self.zoom_rect[2] + self.zoom_rect[2] * 0.02 < self.ori_image.shape[0]:
                    self.zoom_rect[2] += self.zoom_rect[2] * 0.02
                    self.zoom_rect[3] += self.zoom_rect[3] * 0.02
                    if self.zoom_rect[0] + self.zoom_rect[2]  > self.ori_image.shape[0]:
                        self.zoom_rect[0] = self.ori_image.shape[0] - self.zoom_rect[2]
                    if self.zoom_rect[1] + self.zoom_rect[3] > self.ori_image.shape[1]:
                        self.zoom_rect[1] = self.ori_image.shape[1] - self.zoom_rect[3]
                else:
                    self.zoom_rect[2] = self.ori_image.shape[0]
                    self.zoom_rect[3] = self.ori_image.shape[1]


            self.zoom(False, None)
        else:
            if event.angleDelta().y() > 0:
                self.radius += 5
            elif event.angleDelta().y() < 0:
                if self.radius > 6:
                    self.radius -= 5
                else:
                    self.radius = 1

        #frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
        #image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        image = self.get_Image()
        self.image.setPixmap(QPixmap.fromImage(image))
        self.img_temp = self.image.pixmap().copy()

        painter = QPainter(self.image.pixmap())
        if modifiers == Qt.ControlModifier:
            x = int((self.lastPoint[0] / self.image.pixmap().width()) * self.mask.shape[1])
            y = int((self.lastPoint[1] / self.image.pixmap().height()) * self.mask.shape[0])
            if x < int(self.image.pixmap().width() * 0.25):
                x = int(self.image.pixmap().width() * 0.25)
            elif x > self.src_img.shape[1] - (self.image.pixmap().width() * 0.25):
                x = self.src_img.shape[1] - int(self.image.pixmap().width() * 0.25)

            if y < int(self.image.pixmap().height() * 0.25):
                y = int(self.image.pixmap().height() * 0.25)
            elif y > self.src_img.shape[0] - (self.image.pixmap().height() * 0.25):
                y = self.src_img.shape[0] - int(self.image.pixmap().height() * 0.25)

            painter.setPen(QPen(Qt.yellow, 3, Qt.SolidLine))
            width = int(self.image.pixmap().width() * 0.5)
            height = int(self.image.pixmap().height() * 0.5)
            painter.drawRect(x - int(width / 2), y - int(height / 2), width, height)
            self.update()
        else:
            painter.setPen(QPen(Qt.blue, 3, Qt.SolidLine))
            painter.drawEllipse(event.pos(), self.radius, self.radius)

        #self.show()
        self.update()

    def mouseReleaseEvent(self, event):
        temp = cv2.resize(self.mask, (int(self.zoom_rect[3]), int(self.zoom_rect[2])))
        temp[temp > 100] = 255
        temp[temp < 200] = 0
        self.ori_mask[int(self.zoom_rect[0]):int(self.zoom_rect[0]) + int(self.zoom_rect[2]),int(self.zoom_rect[1]):int(self.zoom_rect[1]) + int(self.zoom_rect[3]), :] = temp
        modifiers = QApplication.keyboardModifiers()
        if modifiers != Qt.ControlModifier:
            #if self.draw_index < len(self.mask_changes)-1:
            #    del self.mask_changes[self.draw_index+1:]
            if event.button() == Qt.LeftButton:
                if self.draw_index < len(self.mask_changes) - 1:
                    del self.mask_changes[self.draw_index + 1:]
                self.drawing = False
                self.draw_index += 1
                if len(self.mask_changes) - 1 >= self.draw_index and self.draw_index < 5:
                    self.mask_changes[self.draw_index] = self.ori_mask.copy()
                else:
                    self.mask_changes.append(self.ori_mask.copy())

                    if len(self.mask_changes) > 5:
                        self.draw_index = 4
                        self.mask_changes.pop(0)
            elif event.button() == Qt.RightButton:
                if self.draw_index < len(self.mask_changes) - 1:
                    del self.mask_changes[self.draw_index + 1:]

                self.erasing = False
                self.draw_index += 1
                if len(self.mask_changes)-1 >= self.draw_index and self.draw_index < 5:
                    self.mask_changes[self.draw_index] = self.ori_mask.copy()
                else:
                    self.mask_changes.append(self.mask.copy())
                    if len(self.mask_changes) > 5:
                        self.draw_index = 4
                        self.mask_changes.pop(0)


        #frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
        #image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        image = self.get_Image()

        self.image.setPixmap(QPixmap.fromImage(image))
        self.img_temp = self.image.pixmap().copy()
        #self.image.setPixmap(self.img_temp)
        painter = QPainter(self.image.pixmap())
        modifiers = QApplication.keyboardModifiers()
        if modifiers == Qt.ControlModifier and event is not None:
            painter.setPen(QPen(Qt.yellow, 3, Qt.SolidLine))
            width = int(self.image.pixmap().width()*0.5)
            height = int(self.image.pixmap().height() * 0.5)
            painter.drawRect(event.x()-int(width/2),event.y()-int(height/2), width, height)
        else:
            painter.setPen(QPen(Qt.blue, 3, Qt.SolidLine))
            painter.drawEllipse(event.pos(), self.radius, self.radius)
        self.update()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Control:
            x = int((self.lastPoint[0] / self.image.pixmap().width()) * self.mask.shape[1])
            y = int((self.lastPoint[1] / self.image.pixmap().height()) * self.mask.shape[0])
            check = True
            if x < int(self.image.pixmap().width() * 0.25):
                x = int(self.image.pixmap().width() * 0.25)
                self.timer.start()
                check = False
            elif x > self.src_img.shape[1] - (self.image.pixmap().width() * 0.25):
                x = self.src_img.shape[1] - int(self.image.pixmap().width() * 0.25)
                check = False
                self.timer.start()

            if y < int(self.image.pixmap().height() * 0.25):
                y = int(self.image.pixmap().height() * 0.25)
                check = False
                self.timer.start()
            elif y > self.src_img.shape[0] - (self.image.pixmap().height() * 0.25):
                y = self.src_img.shape[0] - int(self.image.pixmap().height() * 0.25)
                check = False
                self.timer.start()

            if check:
                self.timer.stop()

            self.image.setPixmap(self.img_temp)
            painter = QPainter(self.image.pixmap())
            painter.setPen(QPen(Qt.yellow, 3, Qt.SolidLine))
            # painter.drawLine(self.lastPoint, event.pos())
            # painter.drawEllipse(event.pos(), self.radius, self.radius)
            width = int(self.image.pixmap().width() * 0.5)
            height = int(self.image.pixmap().height() * 0.5)
            painter.drawRect(x - int(width / 2), y - int(height / 2), width, height)
            self.update()

        elif False and (event.key() == Qt.Key_Right or event.key() == Qt.Key_Left or event.key() == Qt.Key_Up or event.key() == Qt.Key_Down):

            temp = cv2.resize(self.mask, (int(self.zoom_rect[3]), int(self.zoom_rect[2])))
            temp[temp > 100] = 255
            temp[temp < 200] = 0
            self.ori_mask[int(self.zoom_rect[0]):int(self.zoom_rect[0]) + int(self.zoom_rect[2]),int(self.zoom_rect[1]):int(self.zoom_rect[1]) + int(self.zoom_rect[3]), :] = temp = temp

            if event.key() == Qt.Key_Right and int(self.zoom_rect[1] + self.zoom_rect[3] + self.zoom_rect[2]*0.01) < self.ori_image.shape[1]:
                self.zoom_rect[1] += self.zoom_rect[2]*0.01
            elif event.key() == Qt.Key_Left and int(self.zoom_rect[1] - self.zoom_rect[2]*0.01) >= 0:
                self.zoom_rect[1] -= self.zoom_rect[2] * 0.01
            elif event.key() == Qt.Key_Up and int(self.zoom_rect[0] - self.zoom_rect[2]*0.01) >= 0:
                self.zoom_rect[0] -= self.zoom_rect[2] * 0.01
            elif event.key() == Qt.Key_Down and int(self.zoom_rect[0] +  self.zoom_rect[2] + self.zoom_rect[2]*0.01) < self.ori_image.shape[0]:
                self.zoom_rect[0] += self.zoom_rect[2] * 0.01

            self.zoom(False, None)

            #frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
            #image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
            image = self.get_Image()
            self.image.setPixmap(QPixmap.fromImage(image))
            self.img_temp = self.image.pixmap().copy()
            self.image.setPixmap(self.img_temp)
            painter = QPainter(self.image.pixmap())
            painter.setPen(QPen(Qt.blue, 3, Qt.SolidLine))
            painter.drawEllipse(self.lastPoint[0] - self.radius, self.lastPoint[1] - self.radius, self.radius * 2, self.radius * 2)
        elif event.key() == Qt.Key_Escape:
            self.close()
        elif event.key() == Qt.Key_Alt:
            self.draw_helper = True
        elif self.callback is not None:
            self.callback(event)

        elif event.key() == Qt.Key_Right and self.files is not None:
            if self.auto_save and (not np.array_equal(self.ori_mask, self.ori_mask_init) or self.new_mask):
                self.save()
            elif not np.array_equal(self.ori_mask, self.ori_mask_init) or self.new_mask:
                ret = QMessageBox.question(self, 'MessageBox', "You haven't saved. Do you want to save current mask?",
                                           QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
                if ret == QMessageBox.Yes:
                    self.save()


            self.file_index += 1
            if self.file_index == len(self.files):
                self.file_index = 0

            self.img_path = self.img_folder_path + self.files[self.file_index]

            self.ori_image = cv2.imread(self.img_path)
            print("Load image:", self.img_path)
            self.ori_mask = np.zeros(self.ori_image.shape, dtype=np.uint8)
            self.new_mask = True

            if os.path.isfile(self.img_save_path + self.files[self.file_index]):
                temp = cv2.imread(self.img_save_path + self.files[self.file_index])
                self.ori_mask[:, :, 1] = temp[:, :, 1]
                self.new_mask = False

            self.ori_mask_init = self.ori_mask.copy()


            self.mask_changes = [self.ori_mask.copy()]
            self.draw_index = 0
            self.zoom_rect = [0, 0, self.ori_image.shape[0], self.ori_image.shape[1]]


            self.src_img = cv2.resize(self.ori_image, (self.img_width, self.img_height))
            self.mask = cv2.resize(self.ori_mask, (self.img_width, self.img_height))
            self.img_show = QPixmap(self.img_path).scaled(int(self.img_width), int(self.img_height), Qt.KeepAspectRatio)
            self.img_temp = self.get_Image()#self.img_show.copy()
            self.image.setPixmap(self.img_show)

            #frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
            #image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
            image = self.get_Image()

            self.image.setPixmap(QPixmap.fromImage(image))
            self.img_temp = self.image.pixmap().copy()


        elif event.key() == Qt.Key_Left and self.files is not None:
            if self.auto_save and (not np.array_equal(self.ori_mask,self.ori_mask_init) or self.new_mask):
                self.save()
            elif not np.array_equal(self.ori_mask,self.ori_mask_init) or self.new_mask:
                ret = QMessageBox.question(self, 'MessageBox', "You haven't saved. Do you want to save current mask?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
                if ret == QMessageBox.Yes:
                    self.save()

            self.file_index -= 1
            if self.file_index == -1:
                self.file_index = len(self.files)-1

            self.img_path = self.img_folder_path + self.files[self.file_index]

            self.ori_image = cv2.imread(self.img_path)
            print("Load image:", self.img_path)
            self.ori_mask = np.zeros(self.ori_image.shape, dtype=np.uint8)
            self.new_mask = True

            if os.path.isfile(self.img_save_path + self.files[self.file_index]):
                temp = cv2.imread(self.img_save_path + self.files[self.file_index])
                self.ori_mask[:,:,1]= temp[:, :, 1]
                self.new_mask = False

            self.ori_mask_init = self.ori_mask.copy()

            self.mask_changes = [self.ori_mask.copy()]
            self.draw_index = 0
            self.zoom_rect = [0, 0, self.ori_image.shape[0], self.ori_image.shape[1]]

            self.src_img = cv2.resize(self.ori_image, (self.img_width, self.img_height))
            self.mask = cv2.resize(self.ori_mask, (self.img_width, self.img_height))

            #self.img_show = QPixmap(self.img_path).scaled(int(self.img_width), int(self.img_height), Qt.KeepAspectRatio)
            self.img_show = QPixmap(self.get_Image())
            self.img_temp = self.img_show.copy()
            self.image.setPixmap(self.img_show)

            #frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
            #image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
            image = self.get_Image()

            self.image.setPixmap(QPixmap.fromImage(image))
            self.img_temp = self.image.pixmap().copy()

        elif event.key() == Qt.Key_Delete and self.files is not None:
            print("Delete image")
            ret = QMessageBox.question(self, 'MessageBox', "Are you sure you want to delete image '" + self.files[self.file_index] + "'?",  QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

            if ret == QMessageBox.Yes:
                os.remove(self.img_path)

                if os.path.isfile(self.img_save_path + self.files[self.file_index]):
                    os.remove(self.img_save_path + self.files[self.file_index])

                self.files = os.listdir(self.img_folder_path)
                self.files = [file for file in self.files if file.endswith('.png')]
                self.files.sort()

                self.file_index -= 1
                if self.file_index == -1:
                    self.file_index = len(self.files)-1

                self.img_path = self.img_folder_path + self.files[self.file_index]


                self.ori_image = cv2.imread(self.img_path)
                print("Load image:", self.img_path)
                self.ori_mask = np.zeros(self.ori_image.shape, dtype=np.uint8)
                self.new_mask = True

                if os.path.isfile(self.img_save_path + self.files[self.file_index]):
                    temp = cv2.imread(self.img_save_path + self.files[self.file_index])
                    self.ori_mask[:,:,1]= temp[:, :, 1]
                    self.new_mask = False

                self.ori_mask_init = self.ori_mask.copy()

                self.mask_changes = [self.ori_mask.copy()]
                self.draw_index = 0
                self.zoom_rect = [0, 0, self.ori_image.shape[0], self.ori_image.shape[1]]

                self.src_img = cv2.resize(self.ori_image, (self.img_width, self.img_height))
                self.mask = cv2.resize(self.ori_mask, (self.img_width, self.img_height))
                #self.img_show = QPixmap(self.img_path).scaled(int(self.img_width), int(self.img_height), Qt.KeepAspectRatio)
                self.img_show = QPixmap(self.get_Image())
                self.img_temp = self.img_show.copy()
                self.image.setPixmap(self.img_show)

                #frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
                #image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
                image = self.get_Image()

                self.image.setPixmap(QPixmap.fromImage(image))
                self.img_temp = self.image.pixmap().copy()


    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_Control:
            self.image.setPixmap(self.img_temp)
            painter = QPainter(self.image.pixmap())
            painter.setPen(QPen(Qt.blue, 3, Qt.SolidLine))
            # painter.drawLine(self.lastPoint, event.pos())
            painter.drawEllipse(self.lastPoint[0]-self.radius, self.lastPoint[1]-self.radius,self.radius*2, self.radius*2)
            self.update()
        elif event.key() == Qt.Key_Alt:
            self.draw_helper = False
            self.C_level = 15

    def reverse_draw(self):
        if self.draw_index > 0:
            self.draw_index -= 1

        self.ori_mask = self.mask_changes[self.draw_index].copy()
        self.mask = self.mask_changes[self.draw_index][int(self.zoom_rect[0]):int(self.zoom_rect[0] + self.zoom_rect[2]),int(self.zoom_rect[1]):int(self.zoom_rect[1] + self.zoom_rect[3]), :].copy()
        self.mask = cv2.resize(self.mask, (self.src_img.shape[1], self.src_img.shape[0]),interpolation=cv2.INTER_AREA)

        #frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
        #image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        image = self.get_Image()

        self.image.setPixmap(QPixmap.fromImage(image))
        self.img_temp = self.image.pixmap().copy()

    def redo_draw(self):
        if self.draw_index < 4 and self.draw_index < len(self.mask_changes)-1:
            self.draw_index += 1

        self.ori_mask = self.mask_changes[self.draw_index].copy()
        self.mask = self.mask_changes[self.draw_index][int(self.zoom_rect[0]):int(self.zoom_rect[0] + self.zoom_rect[2]),int(self.zoom_rect[1]):int(self.zoom_rect[1] + self.zoom_rect[3]), :].copy()
        self.mask = cv2.resize(self.mask, (self.src_img.shape[1], self.src_img.shape[0]))
        self.mask[self.mask > 100] = 255
        self.mask[self.mask < 200] = 0

        #frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
        #image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        image = self.get_Image()


        self.image.setPixmap(QPixmap.fromImage(image))
        self.img_temp = self.image.pixmap().copy()

    def save(self):
        if self.callback is None:
            if not os.path.exists(self.img_save_path):
                os.mkdir(self.img_save_path)

            self.ori_mask_init = self.ori_mask.copy()
            self.new_mask = False
            cv2.imwrite(self.img_save_path+self.img_path.split("/")[-1], self.ori_mask[:,:,1])

            file_amout = len(os.listdir(self.img_save_path))
            #self.TrainWorker.start()
            if self.auto_retrain and not self.TrainWorker.isRunning() and file_amout > self.min_samples and (file_amout%self.retrain_frequncy == 0 or file_amout-self.last_file_amount >= self.retrain_frequncy):
                #Load newest network
                #files = os.listdir(self.img_folder_path + 'Unet/')
                #paths = [os.path.join(self.img_folder_path + 'Unet/', basename) for basename in files]
                #model_path = max(paths, key=os.path.getctime)
                #self.UnetModel, self.device = predict.getModel(model_path, cpu=True)

                print("Train Unet")
                self.last_file_amount = file_amout
                self.TrainWorker.start()

    def save_special(self):
        if self.callback is None:
            if not os.path.exists(self.img_folder_path + "/Special cases"):
                os.mkdir(self.img_folder_path + "/Special cases")

            self.ori_mask_init = self.ori_mask.copy()
            self.new_mask = False
            cv2.imwrite(self.img_folder_path + "/Special cases/"+self.img_path.split("/")[-1], self.ori_image)

    def make_patch(self, point, invert = False):
        self.drawing_process_flag = True

        #in_image = x > self.radius and x < self.mask.shape[1] and y > self.radius and y < self.mask.shape[0]

        circle_mask = np.zeros((self.radius*2,self.radius*2),dtype=np.uint8)

        cv2.circle(circle_mask,(self.radius+1,self.radius+1),self.radius, 255, -1)

        if point[1] < self.radius:
            x0 = 0

            circle_mask = circle_mask[self.radius-point[1]:,:]
        else:

            x0 = point[1] - self.radius


        if point[1] > self.mask.shape[0] - self.radius:
            x1 = self.mask.shape[0] - self.radius

            circle_mask = circle_mask[:self.radius+(self.mask.shape[0]-point[1]), :]
        else:
            x1 = point[1] + self.radius


        if point[0] < self.radius:
            y0 = 0
            circle_mask = circle_mask[:,self.radius-point[0]:]
        else:
            y0 = point[0] - self.radius

        if point[0] > self.mask.shape[1] - self.radius:
            y1 = self.mask.shape[1] - self.radius

            circle_mask = circle_mask[:, :self.radius+(self.mask.shape[1]-point[0])]
        else:
            y1 = point[0] + self.radius


        mask = self.src_img[x0:x1,y0:y1]#[point[1] - self.radius:point[1] + self.radius + 1, point[0] - self.radius:point[0] + self.radius + 1]


        if invert:
            mask = 255-mask

        #current_mask = cv2.bitwise_not(self.mask[x0:x1,y0:y1,1])#[point[1] - self.radius:point[1] + self.radius + 1, point[0] - self.radius:point[0] + self.radius + 1, 1])
        current_mask = cv2.bitwise_and(self.mask[x0:x1,y0:y1,1],self.mask[x0:x1,y0:y1,1],mask=circle_mask)



        if mask.shape[0] == 0 or mask.shape[1] == 0:
            return np.zeros(self.src_img.shape, np.uint8)

        if len(mask.shape) == 3:
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

        mask_blur = cv2.GaussianBlur(mask,(9,9),0)

        th1 = cv2.adaptiveThreshold(mask, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 35, self.C_level)

        th2 = cv2.adaptiveThreshold(mask_blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 35, 5)

        th1 = cv2.bitwise_and(th1, th2)

        th2 = cv2.bitwise_or(th1, current_mask)

        #if not invert:
        #    cv2.imshow("T1", th1)
        #    cv2.imshow("T2", th2)

        th2 = cv2.bitwise_and(th1, th2)
        th2 = cv2.bitwise_and(th2, th2, mask=circle_mask)
        #th2_test = cv2.bitwise_and(current_mask, th2)

        contours, _ = cv2.findContours(th2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        th2 = np.zeros(th2.shape,np.uint8)

        contours.sort(key=cv2.contourArea)

        if len(contours) > 0:
            c = max(contours, key = cv2.contourArea)
            th2 = cv2.drawContours(th2, contours, len(contours)-1, 255, -1).astype(np.uint8)

        mask = np.zeros(self.src_img.shape, np.uint8)


        mask[x0:x1,y0:y1,1] =th2#[point[1] - self.radius:point[1] + self.radius + 1, point[0] - self.radius:point[0] + self.radius + 1, 1] = th2

        self.drawing_process_flag = False


        src = self.src_img[x0:x1,y0:y1]#[point[1] - self.radius:point[1] + self.radius + 1, point[0] - self.radius:point[0] + self.radius + 1]
        if len(src.shape) == 3:

            src = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

        src = cv2.bitwise_and(src,cv2.bitwise_not(th2))

        #if not invert:
        #    cv2.imshow("a",th2)
        #    cv2.imshow("",src)

        if np.mean(src)*2 < np.max(src) and not invert and np.max(src) > 200:
            mask2 = self.make_patch(point, invert = True)
        #    cv2.imshow("ASD",mask2)
        #    cv2.imshow("ASDa", mask)
            mask = cv2.bitwise_or(mask,mask2)


        return mask

    def predict_mask(self):
        #return

        if len(self.ori_image) > 2:
            img = cv2.cvtColor(self.ori_image,cv2.COLOR_RGB2GRAY)
        else:
            img = self.ori_image.copy()

        #print(type(img), img.dtype)
        #img = Image.fromarray(img)
        img = np.expand_dims(img, axis = 0)

        mask = predict.predict_img(self.UnetModel, self.img_path , scale_factor=1, out_threshold=0.1, device=self.device)
        mask = (mask * 255).astype(np.uint8)#predict.mask_to_image(mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours = [contour for contour in contours if cv2.contourArea(contour) > 500]
        self.mask = np.zeros(self.ori_mask.shape,dtype=np.uint8)
        cv2.drawContours(self.mask, contours, -1, (0,255,0),-1)


        self.ori_mask = self.mask

        #self.mask = np.zeros(self.ori_mask.shape,dtype=np.uint8)
        #self.mask[:,:,1] = mask

        self.mask_changes.append(self.ori_mask.copy())
        self.zoom_rect = [0, 0, self.ori_image.shape[0], self.ori_image.shape[1]]

        self.zoom(False, None)

        #Save changes
        if self.draw_index < len(self.mask_changes) - 1:
            del self.mask_changes[self.draw_index + 1:]
        self.drawing = False
        self.draw_index += 1
        if len(self.mask_changes) - 1 >= self.draw_index and self.draw_index < 5:
            self.mask_changes[self.draw_index] = self.ori_mask.copy()
        else:
            self.mask_changes.append(self.ori_mask.copy())

            if len(self.mask_changes) > 5:
                self.draw_index = 4
                self.mask_changes.pop(0)

    def reset_draw(self):
        self.mask = np.zeros(self.src_img.shape, dtype=np.uint8)
        self.draw_index += 1
        if len(self.mask_changes) - 1 >= self.draw_index and self.draw_index < 4:
            self.mask_changes[self.draw_index] = self.mask.copy()
        else:
            self.mask_changes.append(self.mask.copy())
            if len(self.mask_changes) > 5:
                self.draw_index = 4
                self.mask_changes.pop(0)

        #image = QImage(self.src_img, self.src_img.shape[1], self.src_img.shape[0], self.src_img.strides[0], QImage.Format_RGB888)
        image = self.get_Image()

        self.image.setPixmap(QPixmap.fromImage(image))
        self.img_temp = self.image.pixmap().copy()

        painter = QPainter(self.image.pixmap())
        painter.setPen(QPen(Qt.blue, 3, Qt.SolidLine))
        point = QPoint(self.lastPoint[0], self.lastPoint[1])
        painter.drawEllipse(point, self.radius, self.radius)

    def zoom(self, Zoom, pos):
        if Zoom:
            temp = cv2.resize(self.mask, (int(self.zoom_rect[3]), int(self.zoom_rect[2])))
            temp[temp > 100] = 255
            temp[temp < 200] = 0
            self.ori_mask[int(self.zoom_rect[0]):int(self.zoom_rect[0]) + int(self.zoom_rect[2]),int(self.zoom_rect[1]):int(self.zoom_rect[1]) + int(self.zoom_rect[3]), :] = temp

            self.zoom_rect[0] += self.zoom_rect[2]*pos[1]/self.image.pixmap().height()-self.zoom_rect[2]/4
            self.zoom_rect[1] += self.zoom_rect[3]*pos[0]/self.image.pixmap().width()-self.zoom_rect[3]/4
            self.zoom_rect[2] = self.zoom_rect[2]/2
            self.zoom_rect[3] = self.zoom_rect[3]/2

        if self.zoom_rect[0] < 0:
            self.zoom_rect[0] = 0
        elif self.zoom_rect[0] + self.zoom_rect[2] > self.ori_image.shape[0]:
            self.zoom_rect[0] =  self.ori_image.shape[0]- self.zoom_rect[2]

        if self.zoom_rect[1] < 0:
            self.zoom_rect[1] = 0
        elif self.zoom_rect[1] + self.zoom_rect[3] > self.ori_image.shape[1]:
            self.zoom_rect[1] =  self.ori_image.shape[1]- self.zoom_rect[3]

        frame = self.ori_image[int(self.zoom_rect[0]):int(self.zoom_rect[0]+self.zoom_rect[2]),int(self.zoom_rect[1]):int(self.zoom_rect[1]+self.zoom_rect[3]),:]
        frame = cv2.resize(frame,(self.src_img.shape[1],self.src_img.shape[0]))

        self.mask = self.ori_mask[int(self.zoom_rect[0]):int(self.zoom_rect[0] + self.zoom_rect[2]),int(self.zoom_rect[1]):int(self.zoom_rect[1] + self.zoom_rect[3]), :]
        self.mask = cv2.resize(self.mask,(self.src_img.shape[1],self.src_img.shape[0]))
        self.src_img = frame

        #frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
        #image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        image = self.get_Image()

        self.image.setPixmap(QPixmap.fromImage(image))
        self.img_temp = self.image.pixmap().copy()

    def move_periodic(self):
        modifiers = QApplication.keyboardModifiers()
        if modifiers == Qt.ControlModifier:
            x = int((self.lastPoint[0] / self.image.pixmap().width()) * self.mask.shape[1])
            y = int((self.lastPoint[1] / self.image.pixmap().height()) * self.mask.shape[0])
            if x < int(self.image.pixmap().width() * 0.25):
                x = int(self.image.pixmap().width() * 0.25)
                self.zoom_rect[1] -= self.zoom_rect[2] * 0.01
            elif x > self.src_img.shape[1] - (self.image.pixmap().width() * 0.25):
                x = self.src_img.shape[1] - int(self.image.pixmap().width() * 0.25)
                self.zoom_rect[1] += self.zoom_rect[2] * 0.01

            if y < int(self.image.pixmap().height() * 0.25):
                self.zoom_rect[0] -= self.zoom_rect[2] * 0.01
                y = int(self.image.pixmap().height() * 0.25)
            elif y > self.src_img.shape[0] - (self.image.pixmap().height() * 0.25):
                self.zoom_rect[0] += self.zoom_rect[2] * 0.01
                y = self.src_img.shape[0] - int(self.image.pixmap().height() * 0.25)

            self.zoom(False, None)

            self.zoom(False, None)

            #frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
            #image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
            image = self.get_Image()
            self.image.setPixmap(QPixmap.fromImage(image))
            self.img_temp = self.image.pixmap().copy()
            self.image.setPixmap(self.img_temp)
            painter = QPainter(self.image.pixmap())
            painter.setPen(QPen(Qt.yellow, 3, Qt.SolidLine))
            width = int(self.image.pixmap().width() * 0.5)
            height = int(self.image.pixmap().height() * 0.5)
            painter.drawRect(x - int(width / 2), y - int(height / 2), width, height)
            self.update()
        else:
            self.timer.stop()

    def set_mask(self, mask):
        self.ori_mask = mask
        self.mask_changes[self.draw_index] = self.ori_mask.copy()
        self.mask = cv2.resize(self.ori_mask, (self.src_img.shape[1], self.src_img.shape[0]))
        #frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
        #image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)

        image = self.get_Image()
        self.image.setPixmap(QPixmap.fromImage(image))
        self.img_temp = self.image.pixmap().copy()

        painter = QPainter(self.image.pixmap())
        painter.setPen(QPen(Qt.blue, 3, Qt.SolidLine))
        painter.drawEllipse(self.start_point, self.radius, self.radius)

        self.update()

    def set_new_folder(self,img_path, img_save_path = None):
        if img_save_path is None:
            self.img_save_path = os.path.dirname(img_path) +"/mask/"
        else:
            self.img_save_path = img_save_path
        self.img_path = img_path

        file = None

        if os.path.isfile(self.img_path):
            file = os.path.basename(self.img_path)
            self.img_path = os.path.dirname(self.img_path) + "/"


        self.files = os.listdir(self.img_path)
        self.files = [file for file in self.files if file.endswith('.png') or file.endswith('.jpg')]
        self.img_folder_path = self.img_path
        self.files.sort()
        files = os.listdir(self.img_save_path)
        self.file_index = 0

        # Start with the newest mask
        if file is None:
            if len(files) > 0:
                self.file_index = len(files) - 1
        else:
            self.file_index = self.files.index(file)

        self.img_path = self.img_folder_path + self.files[self.file_index]

        self.drawing = False
        self.erasing = False
        self.draw_index = 0
        self.zoom_level = 0
        self.ori_image = cv2.imread(self.img_path)

        self.img_height = self.ori_image.shape[0]
        self.img_width = self.ori_image.shape[1]


        self.draw_helper = False
        self.drawing_process_flag = False
        self.C_level = 15
        self.mouse_click = False

        self.ori_image = cv2.imread(self.img_path)
        self.ori_mask = np.zeros(self.ori_image.shape, dtype=np.uint8)


        if os.path.isfile(self.img_save_path + self.files[self.file_index]):
            temp = cv2.imread(self.img_save_path + self.files[self.file_index])
            self.ori_mask[:,:,1]= temp[:, :, 1]

        self.mask_changes = [self.ori_mask.copy()]
        self.draw_index = 0
        self.zoom_rect = [0, 0, self.ori_image.shape[0], self.ori_image.shape[1]]

        self.src_img = cv2.resize(self.ori_image, (self.img_width, self.img_height))
        self.mask = cv2.resize(self.ori_mask, (self.img_width, self.img_height))
#        self.img_show = QPixmap(self.img_path).scaled(int(self.img_width), int(self.img_height), Qt.KeepAspectRatio)
#        self.img_temp = self.img_show.copy()
#        self.image.setPixmap(self.img_show)

        #frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
        #image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)


        self.img_show = QPixmap(self.get_Image())
        self.img_temp = self.img_show.copy()
        self.image.setPixmap(self.img_show)
        self.img_temp = self.image.pixmap().copy()

        self.setFixedSize(int(self.mask.shape[1]), int(self.mask.shape[0]))



    def fill_contour(self,event, erase = False):

        contours, hierarchy = cv2.findContours(self.mask[:,:,1], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        test= np.zeros(self.mask.shape,dtype=np.uint8)
        cv2.drawContours(test,contours, -1, [0,255,0], -1)
        for index in range(len(contours)):
            result = cv2.pointPolygonTest(contours[index], (event.x(), event.y()), True)
            if result >= 0:
                if erase and self.ori_mask[event.y(), event.x(),1] == 255:
                    cv2.drawContours(self.mask, contours, index, (0, 0, 0), -1)
                elif not erase:
                    cv2.drawContours(self.mask,contours, index, (0,255,0), -1)

                break


        #frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
        #image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        image = self.get_Image()

        self.image.setPixmap(QPixmap.fromImage(image))
        painter = QPainter(self.image.pixmap())
        painter.setPen(QPen(Qt.green, 3, Qt.SolidLine))
        painter.drawEllipse(event.pos(), self.radius, self.radius)

        self.img_temp = self.image.pixmap().copy()


        #cv2.imshow("test",test)

    def update_model(self, path, cpu = False):
        #del self.UnetModel
        #self.UnetModel = self.TrainWorker.model
        print("Update network")
        self.UnetModel, self.device = predict.getModel(path , cpu=cpu)

    def change_view(self,view_type):
        print("Change view", view_type)
        self.view_type = view_type
        self.lock_drawing = True
        image = self.get_Image()
        self.image.setPixmap(QPixmap.fromImage(image))

    def get_Image(self):

        if self.view_type == "Image":
            image = QImage(self.src_img, self.src_img.shape[1], self.src_img.shape[0], self.src_img.strides[0], QImage.Format_RGB888)
        elif self.view_type == "Mask":
            mask = self.mask.copy()
            mask[:,:,0] = self.mask[:,:,1]
            mask[:, :,2] = self.mask[:, :,1]
            image = QImage(mask, mask.shape[1], mask.shape[0], mask.strides[0], QImage.Format_RGB888)
        elif self.view_type == "Image crop":
            mask = self.mask.copy()
            mask[:,:,0] = self.mask[:,:,1]
            mask[:, :,2] = self.mask[:, :,1]
            frame = cv2.bitwise_and(self.ori_image, mask)
            image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        elif self.view_type == "Reverser image crop":
            mask = self.mask.copy()
            mask[:,:,0] = self.mask[:,:,1]
            mask[:, :,2] = self.mask[:, :,1]
            frame = cv2.bitwise_and(self.ori_image, cv2.bitwise_not(mask))
            image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        else:
            self.lock_drawing = False
            frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
            image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)

        return image


if __name__ == '__main__':
    # the input dialog
    if False:
        img_path = simpledialog.askstring(title="Get image path", prompt="Enter image path or folder path:")

        while True :
            if os.path.isfile(img_path) or os.path.isdir(img_path):
                break

            img_path = simpledialog.askstring(title="Get image path", prompt="The entered file/folder path does not exist: Enter new image path or folder path:")


        img_path+='/'


    #img_path = "T:/Usr/ML/Rail images/"
    #print(img_path)

    app = QApplication(sys.argv)
    RectScreen0 = app.desktop().screenGeometry(0)
    win = MainWindow(RectScreen0.width(), RectScreen0.height())
    win.show()
    win.move(RectScreen0.left(), RectScreen0.top())
    #win.resize(RectScreen0.width(), RectScreen0.height())
    sys.exit(app.exec_())