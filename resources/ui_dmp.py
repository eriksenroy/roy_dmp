# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui_dmp.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(951, 654)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.stackedWidget = QtWidgets.QStackedWidget(self.centralwidget)
        self.stackedWidget.setObjectName("stackedWidget")
        self.page_1 = QtWidgets.QWidget()
        self.page_1.setObjectName("page_1")
        self.formLayout = QtWidgets.QFormLayout(self.page_1)
        self.formLayout.setObjectName("formLayout")
        self.frame_3 = QtWidgets.QFrame(self.page_1)
        self.frame_3.setMinimumSize(QtCore.QSize(0, 200))
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.frame_3)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.frame_2 = QtWidgets.QFrame(self.frame_3)
        self.frame_2.setMinimumSize(QtCore.QSize(100, 100))
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.frame_2)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.splitter = QtWidgets.QSplitter(self.frame_2)
        self.splitter.setOrientation(QtCore.Qt.Horizontal)
        self.splitter.setObjectName("splitter")
        self.RobotIPLabel = QtWidgets.QLabel(self.splitter)
        self.RobotIPLabel.setObjectName("RobotIPLabel")
        self.IPLineEdit = QtWidgets.QLineEdit(self.splitter)
        self.IPLineEdit.setMaximumSize(QtCore.QSize(16777215, 26))
        self.IPLineEdit.setObjectName("IPLineEdit")
        self.gridLayout_4.addWidget(self.splitter, 0, 0, 1, 1)
        self.splitter_2 = QtWidgets.QSplitter(self.frame_2)
        self.splitter_2.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_2.setObjectName("splitter_2")
        self.simulationCheckBox = QtWidgets.QCheckBox(self.splitter_2)
        self.simulationCheckBox.setMaximumSize(QtCore.QSize(16777215, 25))
        self.simulationCheckBox.setObjectName("simulationCheckBox")
        self.gridLayout_4.addWidget(self.splitter_2, 1, 0, 1, 1)
        self.gridLayout_5.addWidget(self.frame_2, 0, 0, 1, 1)
        self.frame = QtWidgets.QFrame(self.frame_3)
        self.frame.setMinimumSize(QtCore.QSize(100, 100))
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.frame)
        self.verticalLayout.setObjectName("verticalLayout")
        self.startButton = QtWidgets.QPushButton(self.frame)
        self.startButton.setMinimumSize(QtCore.QSize(0, 70))
        self.startButton.setObjectName("startButton")
        self.verticalLayout.addWidget(self.startButton)
        self.shutDown = QtWidgets.QPushButton(self.frame)
        self.shutDown.setObjectName("shutDown")
        self.verticalLayout.addWidget(self.shutDown)
        self.gridLayout_5.addWidget(self.frame, 1, 0, 1, 1)
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.frame_3)
        self.stackedWidget.addWidget(self.page_1)
        self.page_2 = QtWidgets.QWidget()
        self.page_2.setObjectName("page_2")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.page_2)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.frame_4 = QtWidgets.QFrame(self.page_2)
        self.frame_4.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_4.setObjectName("frame_4")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.frame_4)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.radioButton = QtWidgets.QRadioButton(self.frame_4)
        self.radioButton.setObjectName("radioButton")
        self.verticalLayout_2.addWidget(self.radioButton)
        self.radioButton_2 = QtWidgets.QRadioButton(self.frame_4)
        self.radioButton_2.setObjectName("radioButton_2")
        self.verticalLayout_2.addWidget(self.radioButton_2)
        self.radioButton_3 = QtWidgets.QRadioButton(self.frame_4)
        self.radioButton_3.setObjectName("radioButton_3")
        self.verticalLayout_2.addWidget(self.radioButton_3)
        self.gridLayout_3.addWidget(self.frame_4, 0, 0, 1, 1)
        self.frame_5 = QtWidgets.QFrame(self.page_2)
        self.frame_5.setMaximumSize(QtCore.QSize(16777215, 40))
        self.frame_5.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_5.setObjectName("frame_5")
        self.verticalLayout_8 = QtWidgets.QVBoxLayout(self.frame_5)
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.splitter_3 = QtWidgets.QSplitter(self.frame_5)
        self.splitter_3.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_3.setObjectName("splitter_3")
        self.label = QtWidgets.QLabel(self.splitter_3)
        self.label.setObjectName("label")
        self.lineEdit = QtWidgets.QLineEdit(self.splitter_3)
        self.lineEdit.setMaximumSize(QtCore.QSize(16777215, 25))
        self.lineEdit.setText("")
        self.lineEdit.setObjectName("lineEdit")
        self.verticalLayout_8.addWidget(self.splitter_3)
        self.gridLayout_3.addWidget(self.frame_5, 1, 0, 1, 1)
        self.frame_6 = QtWidgets.QFrame(self.page_2)
        self.frame_6.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_6.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_6.setObjectName("frame_6")
        self.verticalLayout_9 = QtWidgets.QVBoxLayout(self.frame_6)
        self.verticalLayout_9.setObjectName("verticalLayout_9")
        self.pushButton = QtWidgets.QPushButton(self.frame_6)
        self.pushButton.setMinimumSize(QtCore.QSize(0, 50))
        self.pushButton.setObjectName("pushButton")
        self.verticalLayout_9.addWidget(self.pushButton)
        self.pushButton_2 = QtWidgets.QPushButton(self.frame_6)
        self.pushButton_2.setMinimumSize(QtCore.QSize(0, 50))
        self.pushButton_2.setObjectName("pushButton_2")
        self.verticalLayout_9.addWidget(self.pushButton_2)
        self.gridLayout_3.addWidget(self.frame_6, 2, 0, 1, 1)
        self.stackedWidget.addWidget(self.page_2)
        self.page_3 = QtWidgets.QWidget()
        self.page_3.setObjectName("page_3")
        self.gridLayout_9 = QtWidgets.QGridLayout(self.page_3)
        self.gridLayout_9.setObjectName("gridLayout_9")
        self.frame_10 = QtWidgets.QFrame(self.page_3)
        self.frame_10.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_10.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_10.setObjectName("frame_10")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.frame_10)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.frame_7 = QtWidgets.QFrame(self.frame_10)
        self.frame_7.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_7.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_7.setObjectName("frame_7")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.frame_7)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_2 = QtWidgets.QLabel(self.frame_7)
        self.label_2.setObjectName("label_2")
        self.verticalLayout_4.addWidget(self.label_2)
        self.radioButton_4 = QtWidgets.QRadioButton(self.frame_7)
        self.radioButton_4.setObjectName("radioButton_4")
        self.verticalLayout_4.addWidget(self.radioButton_4)
        self.radioButton_5 = QtWidgets.QRadioButton(self.frame_7)
        self.radioButton_5.setObjectName("radioButton_5")
        self.verticalLayout_4.addWidget(self.radioButton_5)
        self.verticalLayout_3.addWidget(self.frame_7)
        self.frame_9 = QtWidgets.QFrame(self.frame_10)
        self.frame_9.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_9.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_9.setObjectName("frame_9")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.frame_9)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.splitter_9 = QtWidgets.QSplitter(self.frame_9)
        self.splitter_9.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_9.setObjectName("splitter_9")
        self.label_5 = QtWidgets.QLabel(self.splitter_9)
        self.label_5.setObjectName("label_5")
        self.lineEdit_4 = QtWidgets.QLineEdit(self.splitter_9)
        self.lineEdit_4.setObjectName("lineEdit_4")
        self.verticalLayout_5.addWidget(self.splitter_9)
        self.splitter_8 = QtWidgets.QSplitter(self.frame_9)
        self.splitter_8.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_8.setObjectName("splitter_8")
        self.label_6 = QtWidgets.QLabel(self.splitter_8)
        self.label_6.setObjectName("label_6")
        self.lineEdit_5 = QtWidgets.QLineEdit(self.splitter_8)
        self.lineEdit_5.setObjectName("lineEdit_5")
        self.verticalLayout_5.addWidget(self.splitter_8)
        self.splitter_7 = QtWidgets.QSplitter(self.frame_9)
        self.splitter_7.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_7.setObjectName("splitter_7")
        self.label_7 = QtWidgets.QLabel(self.splitter_7)
        self.label_7.setObjectName("label_7")
        self.lineEdit_6 = QtWidgets.QLineEdit(self.splitter_7)
        self.lineEdit_6.setObjectName("lineEdit_6")
        self.verticalLayout_5.addWidget(self.splitter_7)
        self.splitter_6 = QtWidgets.QSplitter(self.frame_9)
        self.splitter_6.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_6.setObjectName("splitter_6")
        self.label_8 = QtWidgets.QLabel(self.splitter_6)
        self.label_8.setObjectName("label_8")
        self.lineEdit_7 = QtWidgets.QLineEdit(self.splitter_6)
        self.lineEdit_7.setObjectName("lineEdit_7")
        self.verticalLayout_5.addWidget(self.splitter_6)
        self.verticalLayout_3.addWidget(self.frame_9)
        self.frame_8 = QtWidgets.QFrame(self.frame_10)
        self.frame_8.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_8.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_8.setObjectName("frame_8")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.frame_8)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.splitter_5 = QtWidgets.QSplitter(self.frame_8)
        self.splitter_5.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_5.setObjectName("splitter_5")
        self.label_3 = QtWidgets.QLabel(self.splitter_5)
        self.label_3.setObjectName("label_3")
        self.lineEdit_2 = QtWidgets.QLineEdit(self.splitter_5)
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.verticalLayout_6.addWidget(self.splitter_5)
        self.splitter_4 = QtWidgets.QSplitter(self.frame_8)
        self.splitter_4.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_4.setObjectName("splitter_4")
        self.label_4 = QtWidgets.QLabel(self.splitter_4)
        self.label_4.setObjectName("label_4")
        self.lineEdit_3 = QtWidgets.QLineEdit(self.splitter_4)
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.verticalLayout_6.addWidget(self.splitter_4)
        self.verticalLayout_3.addWidget(self.frame_8)
        self.frame_11 = QtWidgets.QFrame(self.frame_10)
        self.frame_11.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_11.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_11.setObjectName("frame_11")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(self.frame_11)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.pushButton_3 = QtWidgets.QPushButton(self.frame_11)
        self.pushButton_3.setObjectName("pushButton_3")
        self.verticalLayout_7.addWidget(self.pushButton_3)
        self.verticalLayout_3.addWidget(self.frame_11)
        self.gridLayout_9.addWidget(self.frame_10, 1, 0, 1, 1)
        self.stackedWidget.addWidget(self.page_3)
        self.page_4 = QtWidgets.QWidget()
        self.page_4.setObjectName("page_4")
        self.frame_12 = QtWidgets.QFrame(self.page_4)
        self.frame_12.setGeometry(QtCore.QRect(50, 20, 675, 458))
        self.frame_12.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_12.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_12.setObjectName("frame_12")
        self.gridLayout_17 = QtWidgets.QGridLayout(self.frame_12)
        self.gridLayout_17.setObjectName("gridLayout_17")
        self.frame_18 = QtWidgets.QFrame(self.frame_12)
        self.frame_18.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_18.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_18.setObjectName("frame_18")
        self.verticalLayout_10 = QtWidgets.QVBoxLayout(self.frame_18)
        self.verticalLayout_10.setObjectName("verticalLayout_10")
        self.splitter_11 = QtWidgets.QSplitter(self.frame_18)
        self.splitter_11.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_11.setObjectName("splitter_11")
        self.label_13 = QtWidgets.QLabel(self.splitter_11)
        self.label_13.setObjectName("label_13")
        self.lineEdit_34 = QtWidgets.QLineEdit(self.splitter_11)
        self.lineEdit_34.setObjectName("lineEdit_34")
        self.verticalLayout_10.addWidget(self.splitter_11)
        self.pushButton_10 = QtWidgets.QPushButton(self.frame_18)
        self.pushButton_10.setObjectName("pushButton_10")
        self.verticalLayout_10.addWidget(self.pushButton_10)
        self.gridLayout_17.addWidget(self.frame_18, 0, 0, 1, 1)
        self.frame_19 = QtWidgets.QFrame(self.frame_12)
        self.frame_19.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_19.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_19.setObjectName("frame_19")
        self.gridLayout_14 = QtWidgets.QGridLayout(self.frame_19)
        self.gridLayout_14.setObjectName("gridLayout_14")
        self.splitter_10 = QtWidgets.QSplitter(self.frame_19)
        self.splitter_10.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_10.setObjectName("splitter_10")
        self.label_14 = QtWidgets.QLabel(self.splitter_10)
        self.label_14.setObjectName("label_14")
        self.lineEdit_35 = QtWidgets.QLineEdit(self.splitter_10)
        self.lineEdit_35.setMaximumSize(QtCore.QSize(16777215, 28))
        self.lineEdit_35.setObjectName("lineEdit_35")
        self.gridLayout_14.addWidget(self.splitter_10, 0, 0, 1, 1)
        self.pushButton_8 = QtWidgets.QPushButton(self.frame_19)
        self.pushButton_8.setObjectName("pushButton_8")
        self.gridLayout_14.addWidget(self.pushButton_8, 1, 0, 1, 1)
        self.gridLayout_17.addWidget(self.frame_19, 1, 0, 1, 1)
        self.frame_13 = QtWidgets.QFrame(self.frame_12)
        self.frame_13.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_13.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_13.setObjectName("frame_13")
        self.gridLayout_16 = QtWidgets.QGridLayout(self.frame_13)
        self.gridLayout_16.setObjectName("gridLayout_16")
        self.tabWidget = QtWidgets.QTabWidget(self.frame_13)
        self.tabWidget.setObjectName("tabWidget")
        self.jointspace = QtWidgets.QWidget()
        self.jointspace.setObjectName("jointspace")
        self.gridLayout_7 = QtWidgets.QGridLayout(self.jointspace)
        self.gridLayout_7.setObjectName("gridLayout_7")
        self.frame_14 = QtWidgets.QFrame(self.jointspace)
        self.frame_14.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_14.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_14.setObjectName("frame_14")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.frame_14)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.lineEdit_8 = QtWidgets.QLineEdit(self.frame_14)
        self.lineEdit_8.setObjectName("lineEdit_8")
        self.gridLayout_6.addWidget(self.lineEdit_8, 1, 0, 1, 1)
        self.lineEdit_9 = QtWidgets.QLineEdit(self.frame_14)
        self.lineEdit_9.setObjectName("lineEdit_9")
        self.gridLayout_6.addWidget(self.lineEdit_9, 3, 0, 1, 1)
        self.lineEdit_10 = QtWidgets.QLineEdit(self.frame_14)
        self.lineEdit_10.setObjectName("lineEdit_10")
        self.gridLayout_6.addWidget(self.lineEdit_10, 6, 0, 1, 1)
        self.lineEdit_11 = QtWidgets.QLineEdit(self.frame_14)
        self.lineEdit_11.setObjectName("lineEdit_11")
        self.gridLayout_6.addWidget(self.lineEdit_11, 2, 0, 1, 1)
        self.lineEdit_12 = QtWidgets.QLineEdit(self.frame_14)
        self.lineEdit_12.setObjectName("lineEdit_12")
        self.gridLayout_6.addWidget(self.lineEdit_12, 4, 0, 1, 1)
        self.pushButton_4 = QtWidgets.QPushButton(self.frame_14)
        self.pushButton_4.setObjectName("pushButton_4")
        self.gridLayout_6.addWidget(self.pushButton_4, 7, 0, 1, 1)
        self.lineEdit_13 = QtWidgets.QLineEdit(self.frame_14)
        self.lineEdit_13.setObjectName("lineEdit_13")
        self.gridLayout_6.addWidget(self.lineEdit_13, 5, 0, 1, 1)
        self.label_9 = QtWidgets.QLabel(self.frame_14)
        self.label_9.setObjectName("label_9")
        self.gridLayout_6.addWidget(self.label_9, 0, 0, 1, 1)
        self.gridLayout_7.addWidget(self.frame_14, 0, 0, 1, 1)
        self.frame_15 = QtWidgets.QFrame(self.jointspace)
        self.frame_15.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_15.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_15.setObjectName("frame_15")
        self.gridLayout_8 = QtWidgets.QGridLayout(self.frame_15)
        self.gridLayout_8.setObjectName("gridLayout_8")
        self.lineEdit_14 = QtWidgets.QLineEdit(self.frame_15)
        self.lineEdit_14.setObjectName("lineEdit_14")
        self.gridLayout_8.addWidget(self.lineEdit_14, 2, 0, 1, 1)
        self.pushButton_5 = QtWidgets.QPushButton(self.frame_15)
        self.pushButton_5.setObjectName("pushButton_5")
        self.gridLayout_8.addWidget(self.pushButton_5, 7, 0, 1, 1)
        self.lineEdit_15 = QtWidgets.QLineEdit(self.frame_15)
        self.lineEdit_15.setObjectName("lineEdit_15")
        self.gridLayout_8.addWidget(self.lineEdit_15, 5, 0, 1, 1)
        self.lineEdit_16 = QtWidgets.QLineEdit(self.frame_15)
        self.lineEdit_16.setObjectName("lineEdit_16")
        self.gridLayout_8.addWidget(self.lineEdit_16, 4, 0, 1, 1)
        self.lineEdit_17 = QtWidgets.QLineEdit(self.frame_15)
        self.lineEdit_17.setObjectName("lineEdit_17")
        self.gridLayout_8.addWidget(self.lineEdit_17, 1, 0, 1, 1)
        self.lineEdit_18 = QtWidgets.QLineEdit(self.frame_15)
        self.lineEdit_18.setObjectName("lineEdit_18")
        self.gridLayout_8.addWidget(self.lineEdit_18, 6, 0, 1, 1)
        self.lineEdit_19 = QtWidgets.QLineEdit(self.frame_15)
        self.lineEdit_19.setObjectName("lineEdit_19")
        self.gridLayout_8.addWidget(self.lineEdit_19, 3, 0, 1, 1)
        self.label_10 = QtWidgets.QLabel(self.frame_15)
        self.label_10.setObjectName("label_10")
        self.gridLayout_8.addWidget(self.label_10, 0, 0, 1, 1)
        self.gridLayout_7.addWidget(self.frame_15, 0, 1, 1, 1)
        self.tabWidget.addTab(self.jointspace, "")
        self.cartesianSpace = QtWidgets.QWidget()
        self.cartesianSpace.setObjectName("cartesianSpace")
        self.gridLayout_10 = QtWidgets.QGridLayout(self.cartesianSpace)
        self.gridLayout_10.setObjectName("gridLayout_10")
        self.frame_16 = QtWidgets.QFrame(self.cartesianSpace)
        self.frame_16.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_16.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_16.setObjectName("frame_16")
        self.gridLayout_11 = QtWidgets.QGridLayout(self.frame_16)
        self.gridLayout_11.setObjectName("gridLayout_11")
        self.lineEdit_20 = QtWidgets.QLineEdit(self.frame_16)
        self.lineEdit_20.setObjectName("lineEdit_20")
        self.gridLayout_11.addWidget(self.lineEdit_20, 3, 0, 1, 1)
        self.lineEdit_21 = QtWidgets.QLineEdit(self.frame_16)
        self.lineEdit_21.setObjectName("lineEdit_21")
        self.gridLayout_11.addWidget(self.lineEdit_21, 7, 0, 1, 1)
        self.lineEdit_22 = QtWidgets.QLineEdit(self.frame_16)
        self.lineEdit_22.setObjectName("lineEdit_22")
        self.gridLayout_11.addWidget(self.lineEdit_22, 2, 0, 1, 1)
        self.lineEdit_23 = QtWidgets.QLineEdit(self.frame_16)
        self.lineEdit_23.setObjectName("lineEdit_23")
        self.gridLayout_11.addWidget(self.lineEdit_23, 4, 0, 1, 1)
        self.pushButton_6 = QtWidgets.QPushButton(self.frame_16)
        self.pushButton_6.setObjectName("pushButton_6")
        self.gridLayout_11.addWidget(self.pushButton_6, 8, 0, 1, 1)
        self.lineEdit_24 = QtWidgets.QLineEdit(self.frame_16)
        self.lineEdit_24.setObjectName("lineEdit_24")
        self.gridLayout_11.addWidget(self.lineEdit_24, 6, 0, 1, 1)
        self.label_11 = QtWidgets.QLabel(self.frame_16)
        self.label_11.setObjectName("label_11")
        self.gridLayout_11.addWidget(self.label_11, 0, 0, 1, 1)
        self.lineEdit_25 = QtWidgets.QLineEdit(self.frame_16)
        self.lineEdit_25.setObjectName("lineEdit_25")
        self.gridLayout_11.addWidget(self.lineEdit_25, 1, 0, 1, 1)
        self.lineEdit_26 = QtWidgets.QLineEdit(self.frame_16)
        self.lineEdit_26.setObjectName("lineEdit_26")
        self.gridLayout_11.addWidget(self.lineEdit_26, 5, 0, 1, 1)
        self.gridLayout_10.addWidget(self.frame_16, 0, 0, 1, 1)
        self.frame_17 = QtWidgets.QFrame(self.cartesianSpace)
        self.frame_17.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_17.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_17.setObjectName("frame_17")
        self.gridLayout_12 = QtWidgets.QGridLayout(self.frame_17)
        self.gridLayout_12.setObjectName("gridLayout_12")
        self.lineEdit_27 = QtWidgets.QLineEdit(self.frame_17)
        self.lineEdit_27.setObjectName("lineEdit_27")
        self.gridLayout_12.addWidget(self.lineEdit_27, 1, 0, 1, 1)
        self.lineEdit_28 = QtWidgets.QLineEdit(self.frame_17)
        self.lineEdit_28.setObjectName("lineEdit_28")
        self.gridLayout_12.addWidget(self.lineEdit_28, 2, 0, 1, 1)
        self.lineEdit_29 = QtWidgets.QLineEdit(self.frame_17)
        self.lineEdit_29.setObjectName("lineEdit_29")
        self.gridLayout_12.addWidget(self.lineEdit_29, 4, 0, 1, 1)
        self.pushButton_7 = QtWidgets.QPushButton(self.frame_17)
        self.pushButton_7.setObjectName("pushButton_7")
        self.gridLayout_12.addWidget(self.pushButton_7, 8, 0, 1, 1)
        self.lineEdit_30 = QtWidgets.QLineEdit(self.frame_17)
        self.lineEdit_30.setObjectName("lineEdit_30")
        self.gridLayout_12.addWidget(self.lineEdit_30, 6, 0, 1, 1)
        self.label_12 = QtWidgets.QLabel(self.frame_17)
        self.label_12.setObjectName("label_12")
        self.gridLayout_12.addWidget(self.label_12, 0, 0, 1, 1)
        self.lineEdit_31 = QtWidgets.QLineEdit(self.frame_17)
        self.lineEdit_31.setObjectName("lineEdit_31")
        self.gridLayout_12.addWidget(self.lineEdit_31, 7, 0, 1, 1)
        self.lineEdit_32 = QtWidgets.QLineEdit(self.frame_17)
        self.lineEdit_32.setObjectName("lineEdit_32")
        self.gridLayout_12.addWidget(self.lineEdit_32, 3, 0, 1, 1)
        self.lineEdit_33 = QtWidgets.QLineEdit(self.frame_17)
        self.lineEdit_33.setObjectName("lineEdit_33")
        self.gridLayout_12.addWidget(self.lineEdit_33, 5, 0, 1, 1)
        self.gridLayout_10.addWidget(self.frame_17, 0, 1, 1, 1)
        self.tabWidget.addTab(self.cartesianSpace, "")
        self.gridLayout_16.addWidget(self.tabWidget, 0, 0, 1, 1)
        self.gridLayout_17.addWidget(self.frame_13, 1, 1, 2, 1)
        self.frame_20 = QtWidgets.QFrame(self.frame_12)
        self.frame_20.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_20.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_20.setObjectName("frame_20")
        self.gridLayout_15 = QtWidgets.QGridLayout(self.frame_20)
        self.gridLayout_15.setObjectName("gridLayout_15")
        self.pushButton_9 = QtWidgets.QPushButton(self.frame_20)
        self.pushButton_9.setObjectName("pushButton_9")
        self.gridLayout_15.addWidget(self.pushButton_9, 0, 0, 1, 1)
        self.gridLayout_17.addWidget(self.frame_20, 2, 0, 1, 1)
        self.stackedWidget.addWidget(self.page_4)
        self.gridLayout_2.addWidget(self.stackedWidget, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 951, 22))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.menubar.addAction(self.menuFile.menuAction())

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.RobotIPLabel.setText(_translate("MainWindow", "Robot IP:"))
        self.IPLineEdit.setPlaceholderText(_translate("MainWindow", "192.168.1.128"))
        self.simulationCheckBox.setText(_translate("MainWindow", "Simulation"))
        self.startButton.setText(_translate("MainWindow", "Start"))
        self.shutDown.setText(_translate("MainWindow", "Shutdown"))
        self.radioButton.setText(_translate("MainWindow", "End-Effector"))
        self.radioButton_2.setText(_translate("MainWindow", "Joint_States"))
        self.radioButton_3.setText(_translate("MainWindow", "Joint_States with filtering"))
        self.label.setText(_translate("MainWindow", "Name:"))
        self.pushButton.setText(_translate("MainWindow", "Start"))
        self.pushButton_2.setText(_translate("MainWindow", "Stop"))
        self.label_2.setText(_translate("MainWindow", "Type of recording to load"))
        self.radioButton_4.setText(_translate("MainWindow", "End-Effector rec"))
        self.radioButton_5.setText(_translate("MainWindow", "Joint_state rec"))
        self.label_5.setText(_translate("MainWindow", "dt"))
        self.label_6.setText(_translate("MainWindow", "K"))
        self.label_7.setText(_translate("MainWindow", "D"))
        self.label_8.setText(_translate("MainWindow", "# basis_func"))
        self.label_3.setText(_translate("MainWindow", "name of recording"))
        self.label_4.setText(_translate("MainWindow", "name of output file"))
        self.pushButton_3.setText(_translate("MainWindow", "Generate DMP"))
        self.label_13.setText(_translate("MainWindow", "Set active DMP"))
        self.pushButton_10.setText(_translate("MainWindow", "PushButton"))
        self.label_14.setText(_translate("MainWindow", "tau"))
        self.pushButton_8.setText(_translate("MainWindow", "Retrieve DMP plan"))
        self.pushButton_4.setText(_translate("MainWindow", "Retrieve from Robot"))
        self.label_9.setText(_translate("MainWindow", "Initial position"))
        self.pushButton_5.setText(_translate("MainWindow", "Retrieve from robot"))
        self.label_10.setText(_translate("MainWindow", "Goal Position"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.jointspace), _translate("MainWindow", "Joint space"))
        self.pushButton_6.setText(_translate("MainWindow", "Retrieve from Robot"))
        self.label_11.setText(_translate("MainWindow", "TextLabel"))
        self.pushButton_7.setText(_translate("MainWindow", "Retrieve from Robot"))
        self.label_12.setText(_translate("MainWindow", "TextLabel"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.cartesianSpace), _translate("MainWindow", "Cartesian Space"))
        self.pushButton_9.setText(_translate("MainWindow", "EXECUTE PLAN"))
        self.menuFile.setTitle(_translate("MainWindow", "File"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
