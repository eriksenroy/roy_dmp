#!/usr/bin/python

import sys
import os
import rospy
from PyQt5 import QtWidgets
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from ui_dmp import Ui_MainWindow
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(ApplicationWindow, self).__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        # self.
        self.ui.stackedWidget.setCurrentIndex(0)
        self.ui.startButton.clicked.connect(self.start_click)
        self.ui.start_recording_button.clicked.connect(lambda : self.ui.stackedWidget.setCurrentIndex(2))
        self.ui.generate_DMP_button.clicked.connect(self.browse_folder)

        

        self.robot = 'roslaunch roy_dmp ur3_gazebo_with_dmp.launch'
        self.IP = '192.168.2.192'
        self.sim = False

        self.process = QProcess(self)


    def browse_folder(self):
        self.ui.load_name_rec_comboBox.clear()
        directory = QtWidgets.QFileDialog.getExistingDirectory(self,"Choose a directory")
        if directory:
            for file_name in os.listdir(directory):
                print(file_name)
                self.ui.load_name_rec_comboBox.addItem(str(file_name))

    def setSimulation(self,state):
        if state == Qt.Checked:
            self.sim = True
        else:
            self.sim = False

    def setSimulation(self):
        pass
    def setIPRobot(self,text):
        self.IP = text
    def chooseRobot(self,text):
        if text == 'UR3':
            if self.sim:
               self.robot = 'roslaunch roy_dmp ur3_gazebo_with_dmp.launch' 
            else:
                self.robot = 'roslaunch roy_dmp ur3_with_dmp.launch'
        elif text == 'UR5':
            if self.sim:
                self.robot = 'roslaunch ur_gazebo ur5.launch'
            else:
                self.robot = 'roslaunch ur_gazebo ur5.launch'
        elif text == 'UR10':
            if self.sim:
                self.robot = 'roslaunch ur_gazebo ur10.launch'
            else:
                self.robot = 'roslaunch ur_gazebo ur10.launch'
        elif text == 'UR3e':
            pass
        elif text == 'UR5e':
            pass
        elif text == 'UR10e':
            pass

    def start_click(self):
        self.ui.stackedWidget.setCurrentIndex(1)
        program = self.robot
        self.process.start(program)
        print("Finished")

    def shutdown_click(self):
        rospy.signal_shutdown("Shutting down ros")
        sys.exit(0)

    def setRecEE(self):
        pass
    def setRecJS(self):
        pass
    def setRecJSFilter(self):
        pass
    def setNameRecording(self):
        pass
    def startRecording(self):
        pass
    def stopRecording(self):
        pass
    def loadEERec(self):
        pass
    def loadJSRec(self):
        pass
    def setdt(self):
        pass
    def setK(self):
        pass
    def setD(self):
        pass
    def setBasisfunc(self):
        pass
    def loadNamerec(self):
        pass
    def saveWeights(self):
        pass
    def generateDMP(self):
        pass
    def setActiveDMP(self):
        pass
    def sendActiveDMP(self):
        pass
    def setJS_init_0(self):
        pass
    def setJS_init_1(self):
        pass
    def setJS_init_2(self):
        pass
    def setJS_init_3(self):
        pass
    def setJS_init_4(self):
        pass
    def setJS_init_5(self):
        pass
    def setJS_goal_0(self):
        pass
    def setJS_goal_1(self):
        pass
    def setJS_goal_2(self):
        pass
    def setJS_goal_3(self):
        pass
    def setJS_goal_4(self):
        pass
    def setJS_goal_5(self):
        pass
    def getJS_init_robot(self):
        pass
    def getJS_goal_robot(self):
        pass
    def setTau(self):
        pass
    def getDMP_plan(self):
        pass
    def executeDMP_plan(self):
        pass




def main():
    rospy.init_node("DMP_GUI", anonymous=False)
    app = QtWidgets.QApplication(sys.argv)
    application = ApplicationWindow()
    application.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()