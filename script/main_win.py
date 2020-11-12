#!/usr/bin/python

import sys
import rospy
from PyQt5 import QtWidgets
from PyQt5.QtGui import *
from PyQt5.QtCore import *
sys.path.append("/home/roy/catkin_ws/src/roy_dmp/resources")
from main_window import Ui_MainWindow
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(ApplicationWindow, self).__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.startButton.clicked.connect(self.start_click)
        self.ui.IPLineEdit.textChanged[str].connect(self.setIP)
        self.ui.simulationCheckBox.stateChanged.connect(self.setSimulation)
        self.ui.RobotComboBox.activated[str].connect(self.choosing_robot)
        self.ui.shutDown.clicked.connect(self.shutdown)

        

        self.robot = 'roslaunch roy_dmp ur3_gazebo_with_dmp.launch'
        self.IP = '192.168.2.192'
        self.sim = False

        self.process = QProcess(self)
        self.process.readyReadStandardOutput.connect(self.handleStdOut)
        self.process.readyReadStandardError.connect(self.handleStdErr)


    def handleStdOut(self):
        data = self.process.readAllStandardOutput().data()
        self.ui.TerminalPlainTextEdit.append(data.decode('utf-8'))

    def handleStdErr(self):
        data = self.process.readAllStandardError().data()
        self.ui.TerminalPlainTextEdit.append(data.decode('utf-8'))   

    def shutdown(self):
        rospy.signal_shutdown("Shutting down ros")
        sys.exit(0)

    def start_click(self):
        
        print("learn...")
        # source ~/catkin_ws/devel/setup.bash
        program = self.robot
        self.process.start(program)
        print("Finished")

    def choosing_robot(self,text):
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

    def setIP(self,text):
        self.IP = text

    def setSimulation(self,state):
        if state == Qt.Checked:
            self.sim = True
        else:
            self.sim = False




def main():
    app = QtWidgets.QApplication(sys.argv)
    application = ApplicationWindow()
    application.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()