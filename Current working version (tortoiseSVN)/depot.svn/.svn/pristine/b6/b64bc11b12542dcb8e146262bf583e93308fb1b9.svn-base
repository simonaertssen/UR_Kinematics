import sys
from PyQt5.QtCore import Qt, QPoint, QObject, QTimer
from PyQt5.QtWidgets import QMainWindow, QApplication, QLabel, QSplitter, QWidget, QShortcut
from PyQt5.QtGui import QPixmap, QPainter, QPen, QImage, QBrush, QKeySequence, QCursor
import cv2
import numpy as np

class Menu(QWidget):
    def __init__(self, img_path, width , height, callback = None, parent = None, offset = None ):
        super(Menu, self).__init__(parent)
        #self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        self.img_width = width
        self.img_height = height
        self.img_path = img_path
        self.callback = callback
        self.drawing = False
        self.erasing = False
        self.draw_index = 0
        self.zoom_level = 0
        self.start_pos = (0,0)
        self.lastPoint = (0,0)
        self.ori_image = cv2.imread(self.img_path)
        self.ori_mask = np.zeros(self.ori_image.shape, dtype=np.uint8)
        self.src_img = cv2.resize(self.ori_image,(self.img_width, self.img_height))
        self.mask = np.zeros(self.src_img.shape,dtype= np.uint8)
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

        self.timer = QTimer(self)
        self.timer.setInterval(10)          # Throw event timeout with an interval of 1000 milliseconds
        self.timer.timeout.connect(self.move_periodic)

        frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
        image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)

        self.image.setPixmap(QPixmap.fromImage(image))
        self.img_temp = self.image.pixmap().copy()

        painter = QPainter(self.image.pixmap())
        painter.setPen(QPen(Qt.blue, 3, Qt.SolidLine))
        point = QPoint(QCursor().pos().x()-offset[0],  QCursor().pos().y()-offset[1])
        self.start_point= point
        painter.drawEllipse(point, self.radius, self.radius)
        self.lastPoint = (point.x(),point.y())

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
            self.mouseMoveEvent(event)
        elif event.button() == Qt.RightButton:
            self.erasing = True
            self.mouseMoveEvent(event)
        self.lastPoint = (x, y)
        self.update()

    def mouseMoveEvent(self, event):
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

            self.mask = cv2.line(self.mask, (x, y), self.lastPoint, (0,255,0), self.radius*2, -1)
            frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
            image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)

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
            frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
            image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)

            self.image.setPixmap(QPixmap.fromImage(image))
            painter = QPainter(self.image.pixmap())
            painter.setPen(QPen(Qt.red, 3, Qt.SolidLine))
            # painter.drawLine(self.lastPoint, event.pos())
            painter.drawEllipse(event.pos(), self.radius, self.radius)

            self.img_temp = self.image.pixmap().copy()
            self.lastPoint = (x, y)
        else:
            self.image.setPixmap(self.img_temp)
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
                self.radius += 10
            elif event.angleDelta().y() < 0 and self.radius > 1:
                self.radius -= 10

        frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
        image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
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
        self.ori_mask[int(self.zoom_rect[0]):int(self.zoom_rect[0]) + int(self.zoom_rect[2]),int(self.zoom_rect[1]):int(self.zoom_rect[1]) + int(self.zoom_rect[3]), :] = temp = temp
        modifiers = QApplication.keyboardModifiers()
        if modifiers != Qt.ControlModifier:
            if self.draw_index < len(self.mask_changes)-1:
                del self.mask_changes[self.draw_index+1:]
            if event.button() == Qt.LeftButton:
                self.drawing = False
                self.draw_index += 1
                if len(self.mask_changes) - 1 >= self.draw_index and self.draw_index < 4:
                    self.mask_changes[self.draw_index] = self.ori_mask.copy()
                else:
                    self.mask_changes.append(self.ori_mask.copy())
                    if len(self.mask_changes) > 5:
                        self.draw_index = 4
                        self.mask_changes.pop(0)
            elif event.button() == Qt.RightButton:
                self.erasing = False
                self.draw_index += 1
                if len(self.mask_changes)-1 >= self.draw_index:
                    self.mask_changes[self.draw_index] = self.ori_mask.copy()
                else:
                    self.mask_changes.append(self.mask.copy())
                    if len(self.mask_changes) > 5:
                        self.draw_index = 4
                        self.mask_changes.pop(0)


        frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
        image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)

        self.image.setPixmap(QPixmap.fromImage(image))
        self.img_temp = self.image.pixmap().copy()
        self.image.setPixmap(self.img_temp)
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
        print("KEYPRESS", 1)
        if event.key() == Qt.Key_Control:
            print("KEYPRESS", 2)
            x = int((self.lastPoint[0] / self.image.pixmap().width()) *  self.mask.shape[1])
            y = int((self.lastPoint[1] / self.image.pixmap().height()) * self.mask.shape[0])
            print("KEYPRESS", 3)
            if x < int(self.image.pixmap().width() * 0.25):
                x = int(self.image.pixmap().width() * 0.25)
                self.zoom_rect[1] -= self.zoom_rect[2] * 0.01
            elif x > self.src_img.shape[1] - (self.image.pixmap().width() * 0.25):
                x = self.src_img.shape[1] - int(self.image.pixmap().width() * 0.25)
                self.zoom_rect[1] += self.zoom_rect[2] * 0.01
            print("KEYPRESS", 4)
            if y < int(self.image.pixmap().height() * 0.25):
                y = int(self.image.pixmap().height() * 0.25)
                self.zoom_rect[0] -= self.zoom_rect[2] * 0.01
            elif y > self.src_img.shape[0] - (self.image.pixmap().height() * 0.25):
                y = self.src_img.shape[0] - int(self.image.pixmap().height() * 0.25)
                self.zoom_rect[1] += self.zoom_rect[2] * 0.01
            print("KEYPRESS", 5)
            self.image.setPixmap(self.img_temp)
            painter = QPainter(self.image.pixmap())
            painter.setPen(QPen(Qt.yellow, 3, Qt.SolidLine))
            width = int(self.image.pixmap().width()*0.5)
            height = int(self.image.pixmap().height() * 0.5)
            painter.drawRect(x-int(width/2),y-int(height/2), width, height)
            self.update()
            print("KEYPRESS", 6)
        elif event.key() == Qt.Key_Right or event.key() == Qt.Key_Left or event.key() == Qt.Key_Up or event.key() == Qt.Key_Down:
            print("KEYPRESS", 7)
            temp = cv2.resize(self.mask, (int(self.zoom_rect[3]), int(self.zoom_rect[2])))
            temp[temp > 100] = 255
            temp[temp < 200] = 0
            self.ori_mask[int(self.zoom_rect[0]):int(self.zoom_rect[0]) + int(self.zoom_rect[2]),int(self.zoom_rect[1]):int(self.zoom_rect[1]) + int(self.zoom_rect[3]), :] = temp = temp
            print("KEYPRESS", 8)
            if event.key() == Qt.Key_Right and int(self.zoom_rect[1] + self.zoom_rect[3] + self.zoom_rect[2]*0.01) < self.ori_image.shape[1]:
                self.zoom_rect[1] += self.zoom_rect[2]*0.01
            elif event.key() == Qt.Key_Left and int(self.zoom_rect[1] - self.zoom_rect[2]*0.01) >= 0:
                self.zoom_rect[1] -= self.zoom_rect[2] * 0.01
            elif event.key() == Qt.Key_Up and int(self.zoom_rect[0] - self.zoom_rect[2]*0.01) >= 0:
                self.zoom_rect[0] -= self.zoom_rect[2] * 0.01
            elif event.key() == Qt.Key_Down and int(self.zoom_rect[0] +  self.zoom_rect[2] + self.zoom_rect[2]*0.01) < self.ori_image.shape[0]:
                self.zoom_rect[0] += self.zoom_rect[2] * 0.01
            print("KEYPRESS", 9)
            self.zoom(False, None)
            print("KEYPRESS", 10)
            frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
            image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
            self.image.setPixmap(QPixmap.fromImage(image))
            self.img_temp = self.image.pixmap().copy()
            self.image.setPixmap(self.img_temp)
            painter = QPainter(self.image.pixmap())
            painter.setPen(QPen(Qt.blue, 3, Qt.SolidLine))
            painter.drawEllipse(self.lastPoint[0] - self.radius, self.lastPoint[1] - self.radius, self.radius * 2, self.radius * 2)
        elif event.key() == Qt.Key_Escape:
            print("KEYPRESS", 11)
            self.close()
        else:
            print("KEYPRESS", 12)
            self.callback(event)
        print("KEYPRESS", 13)

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_Control:
            self.image.setPixmap(self.img_temp)
            painter = QPainter(self.image.pixmap())
            painter.setPen(QPen(Qt.blue, 3, Qt.SolidLine))
            # painter.drawLine(self.lastPoint, event.pos())
            painter.drawEllipse(self.lastPoint[0]-self.radius, self.lastPoint[1]-self.radius,self.radius*2, self.radius*2)
            self.update()

    def reverse_draw(self):
        if self.draw_index > 0:
            self.draw_index -= 1

        self.mask = self.mask_changes[self.draw_index][int(self.zoom_rect[0]):int(self.zoom_rect[0] + self.zoom_rect[2]),int(self.zoom_rect[1]):int(self.zoom_rect[1] + self.zoom_rect[3]), :]

        self.mask = cv2.resize(self.mask, (self.src_img.shape[1], self.src_img.shape[0]),interpolation=cv2.INTER_AREA)

        frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
        image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)

        self.image.setPixmap(QPixmap.fromImage(image))
        self.img_temp = self.image.pixmap().copy()

    def redo_draw(self):
        if self.draw_index < 4 and self.draw_index < len(self.mask_changes)-1:
            self.draw_index += 1

        self.mask = self.mask_changes[self.draw_index][int(self.zoom_rect[0]):int(self.zoom_rect[0] + self.zoom_rect[2]),int(self.zoom_rect[1]):int(self.zoom_rect[1] + self.zoom_rect[3]), :]
        self.mask = cv2.resize(self.mask, (self.src_img.shape[1], self.src_img.shape[0]))
        self.mask[self.mask > 100] = 255
        self.mask[self.mask < 200] = 0
        frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
        image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)

        self.image.setPixmap(QPixmap.fromImage(image))
        self.img_temp = self.image.pixmap().copy()

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

        image = QImage(self.src_img, self.src_img.shape[1], self.src_img.shape[0], self.src_img.strides[0], QImage.Format_RGB888)

        self.image.setPixmap(QPixmap.fromImage(image))
        self.img_temp = self.image.pixmap().copy()

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
        image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)

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

            frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)
            image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
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
        frame = cv2.addWeighted(self.src_img, 1, self.mask, 0.2, 0)

        image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        self.image.setPixmap(QPixmap.fromImage(image))
        self.img_temp = self.image.pixmap().copy()

        painter = QPainter(self.image.pixmap())
        painter.setPen(QPen(Qt.blue, 3, Qt.SolidLine))
        painter.drawEllipse(self.start_point, self.radius, self.radius)

        self.update()



if __name__ == '__main__':
    app = QApplication(sys.argv)
    mainMenu = Menu()
    mainMenu.show()
    sys.exit(app.exec_())