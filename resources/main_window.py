# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main_window.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(823, 635)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.widget = QtWidgets.QWidget(self.centralwidget)
        self.widget.setGeometry(QtCore.QRect(40, 0, 761, 561))
        self.widget.setObjectName("widget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.widget)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.main_frame = QtWidgets.QFrame(self.widget)
        self.main_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.main_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.main_frame.setObjectName("main_frame")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.main_frame)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame = QtWidgets.QFrame(self.main_frame)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.frame)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.splitter = QtWidgets.QSplitter(self.frame)
        self.splitter.setOrientation(QtCore.Qt.Horizontal)
        self.splitter.setObjectName("splitter")
        self.RobotIPLabel = QtWidgets.QLabel(self.splitter)
        self.RobotIPLabel.setObjectName("RobotIPLabel")
        self.IPLineEdit = QtWidgets.QLineEdit(self.splitter)
        self.IPLineEdit.setMaximumSize(QtCore.QSize(16777215, 26))
        self.IPLineEdit.setObjectName("IPLineEdit")
        self.verticalLayout_2.addWidget(self.splitter)
        self.RobotComboBox = QtWidgets.QComboBox(self.frame)
        self.RobotComboBox.setObjectName("RobotComboBox")
        self.RobotComboBox.addItem("")
        self.RobotComboBox.addItem("")
        self.RobotComboBox.addItem("")
        self.RobotComboBox.addItem("")
        self.RobotComboBox.addItem("")
        self.RobotComboBox.addItem("")
        self.verticalLayout_2.addWidget(self.RobotComboBox)
        self.splitter_2 = QtWidgets.QSplitter(self.frame)
        self.splitter_2.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_2.setObjectName("splitter_2")
        self.simulationCheckBox = QtWidgets.QCheckBox(self.splitter_2)
        self.simulationCheckBox.setMaximumSize(QtCore.QSize(16777215, 25))
        self.simulationCheckBox.setObjectName("simulationCheckBox")
        self.verticalLayout_2.addWidget(self.splitter_2)
        self.startButton = QtWidgets.QPushButton(self.frame)
        self.startButton.setMinimumSize(QtCore.QSize(0, 70))
        self.startButton.setObjectName("startButton")
        self.verticalLayout_2.addWidget(self.startButton)
        self.shutDown = QtWidgets.QPushButton(self.frame)
        self.shutDown.setObjectName("shutDown")
        self.verticalLayout_2.addWidget(self.shutDown)
        self.verticalLayout.addWidget(self.frame)
        self.gridLayout_2.addWidget(self.main_frame, 0, 0, 1, 1)
        self.frame_2 = QtWidgets.QFrame(self.widget)
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.gridLayout = QtWidgets.QGridLayout(self.frame_2)
        self.gridLayout.setObjectName("gridLayout")
        self.TerminalPlainTextEdit = QtWidgets.QTextEdit(self.frame_2)
        self.TerminalPlainTextEdit.setObjectName("TerminalPlainTextEdit")
        self.gridLayout.addWidget(self.TerminalPlainTextEdit, 0, 0, 1, 1)
        self.gridLayout_2.addWidget(self.frame_2, 0, 1, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 823, 22))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.menubar.addAction(self.menuFile.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.RobotIPLabel.setText(_translate("MainWindow", "Robot IP:"))
        self.IPLineEdit.setPlaceholderText(_translate("MainWindow", "192.168.1.128"))
        self.RobotComboBox.setItemText(0, _translate("MainWindow", "UR3"))
        self.RobotComboBox.setItemText(1, _translate("MainWindow", "UR5"))
        self.RobotComboBox.setItemText(2, _translate("MainWindow", "Ur10"))
        self.RobotComboBox.setItemText(3, _translate("MainWindow", "UR3e"))
        self.RobotComboBox.setItemText(4, _translate("MainWindow", "UR5e"))
        self.RobotComboBox.setItemText(5, _translate("MainWindow", "UR10e"))
        self.simulationCheckBox.setText(_translate("MainWindow", "Simulation"))
        self.startButton.setText(_translate("MainWindow", "Start"))
        self.shutDown.setText(_translate("MainWindow", "Shutdown"))
        self.menuFile.setTitle(_translate("MainWindow", "File"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
