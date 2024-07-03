import sys
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5 import uic
import cv2

class MainWindow(QWidget):
    def __init__(self):
        super(MainWindow, self).__init__()
        uic.loadUi('form.ui', self)
        self.show()

        #initializing parameters
        self.feed1IPAdd = "0"
        self.feed2IPAdd = "0"
        self.feed3IPAdd = "0"

        #if ui used, do action
        self.stopstart_1.clicked.connect(self.checkToggle1)
        self.comboBox_1.currentTextChanged.connect(self.text_changed1)

        self.stopstart_2.clicked.connect(self.checkToggle2)
        self.comboBox_2.currentTextChanged.connect(self.text_changed2)

        self.stopstart_3.clicked.connect(self.checkToggle3)
        self.comboBox_3.currentTextChanged.connect(self.text_changed3)

    def checkToggle1(self):
        if(self.stopstart_1.text() == "Start"):
            self.stopstart_1.setText("Stop")
            self.Worker1 = Worker(self.feed1IPAdd)
            self.Worker1.start()
            self.Worker1.ImageUpdate.connect(self.ImageUpdateSlot1)
        else:
            self.stopstart_1.setText("Start")
            self.Worker1.stop()

    def checkToggle2(self):
        if(self.stopstart_2.text() == "Start"):
            self.stopstart_2.setText("Stop")
            self.Worker2 = Worker(self.feed2IPAdd)
            self.Worker2.start()
            self.Worker2.ImageUpdate.connect(self.ImageUpdateSlot2)
        else:
            self.stopstart_2.setText("Start")
            self.Worker2.stop()

    def checkToggle3(self):
        if(self.stopstart_3.text() == "Start"):
            self.stopstart_3.setText("Stop")
            self.Worker3 = Worker(self.feed3IPAdd)
            self.Worker3.start()
            self.Worker3.ImageUpdate.connect(self.ImageUpdateSlot3)
        else:
            self.stopstart_3.setText("Start")
            self.Worker3.stop()

    def ImageUpdateSlot1(self, Image):
        self.FeedLabel_1.setPixmap(QPixmap.fromImage(Image))
    
    def ImageUpdateSlot2(self, Image):
        self.FeedLabel_2.setPixmap(QPixmap.fromImage(Image))

    def ImageUpdateSlot3(self, Image):
        self.FeedLabel_3.setPixmap(QPixmap.fromImage(Image))

    def text_changed1(self, s):
        if(s == "Web-Cam"):
            self.feed1IPAdd = "0"
        elif(s == "Example 1"):
            self.feed1IPAdd = "1"
        elif(s == "Example 2"):
            self.feed1IPAdd = "2"

        self.IP_edit_1.setText(self.feed1IPAdd)

    def text_changed2(self, s):
        if(s == "Web-Cam"):
            self.feed2IPAdd = "0"
        elif(s == "Example 1"):
            self.feed2IPAdd = "1"
        elif(s == "Example 2"):
            self.feed2IPAdd = "2"

        self.IP_edit_2.setText(self.feed2IPAdd)

    def text_changed3(self, s):
        if(s == "Web-Cam"):
            self.feed3IPAdd = "0"
        elif(s == "Example 1"):
            self.feed3IPAdd = "1"
        elif(s == "Example 2"):
            self.feed3IPAdd = "2"

        self.IP_edit_3.setText(self.feed3IPAdd)


#thread Class
class Worker(QThread):
    ImageUpdate = pyqtSignal(QImage)
    def __init__(self, text, parent=None):
        super().__init__(parent)
        if (text == "0"):
            self.getfeedipAdd = 0
        else:
            self.getfeedipAdd = text

    def run(self):
        self.ThreadActive = True
        Capture = cv2.VideoCapture(self.getfeedipAdd)
        while self.ThreadActive:
            ret, frame = Capture.read()
            if ret:
                Image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                FlippedImage = cv2.flip(Image, 1)
                ConvertToQtFormat = QImage(FlippedImage.data, FlippedImage.shape[1], FlippedImage.shape[0], QImage.Format_RGB888)
                Pic = ConvertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
                self.ImageUpdate.emit(Pic)
    def stop(self):
        self.ThreadActive = False
        self.quit()

if __name__ == "__main__":
    App = QApplication(sys.argv)
    Root = MainWindow()
    Root.show()
    sys.exit(App.exec())