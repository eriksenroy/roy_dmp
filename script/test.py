# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'test.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.frame = QtWidgets.QFrame(self.centralwidget)
        self.frame.setGeometry(QtCore.QRect(0, 0, 791, 561))
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.frame_2 = QtWidgets.QFrame(self.frame)
        self.frame_2.setGeometry(QtCore.QRect(20, 10, 751, 511))
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.gridLayout = QtWidgets.QGridLayout(self.frame_2)
        self.gridLayout.setObjectName("gridLayout")
        self.sourceButton = QtWidgets.QPushButton(self.frame_2)
        self.sourceButton.setObjectName("sourceButton")
        self.gridLayout.addWidget(self.sourceButton, 0, 0, 1, 1)
        self.roslaunch = QtWidgets.QPushButton(self.frame_2)
        self.roslaunch.setObjectName("roslaunch")
        self.gridLayout.addWidget(self.roslaunch, 1, 0, 1, 1)
        self.dmp_learn = QtWidgets.QPushButton(self.frame_2)
        self.dmp_learn.setObjectName("dmp_learn")
        self.gridLayout.addWidget(self.dmp_learn, 2, 0, 1, 1)
        self.dmp_execute = QtWidgets.QPushButton(self.frame_2)
        self.dmp_execute.setObjectName("dmp_execute")
        self.gridLayout.addWidget(self.dmp_execute, 3, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.sourceButton.setText(_translate("MainWindow", "Source ws"))
        self.roslaunch.setText(_translate("MainWindow", "roslaunch"))
        self.dmp_learn.setText(_translate("MainWindow", "dmp_learn"))
        self.dmp_execute.setText(_translate("MainWindow", "dmp_execute"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

