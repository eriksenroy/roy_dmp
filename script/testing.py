#!/usr/bin/python


from PyQt5 import QtWidgets
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from test import Ui_MainWindow
import sys

class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(ApplicationWindow, self).__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.sourceButton.clicked.connect(self.onClick1)
        self.ui.roslaunch.clicked.connect(self.onClick2)
        self.ui.dmp_learn.clicked.connect(self.onClick3)
        self.ui.dmp_execute.clicked.connect(self.onClick4)

    def onClick1(self):
        ROS_PROGRAM = QProcess(self)
        print("learn...")
        # source ~/catkin_ws/devel/setup.bash
        program = 'source ~/devel/setup.bash'
        ROS_PROGRAM.start(program)
        print("Finished")

    def onClick2(self):
        ROS_PROGRAM = QProcess(self)
        print("Launching...")
        program2 = 'roslaunch roy_dmp ur3_gazebo_with_dmp.launch'
        ROS_PROGRAM.start(program2)
    def onClick3(self):
        ROS_PROGRAM = QProcess(self)
        print("execute...")
        program3 = 'rosrun roy_dmp dmp_learn.py'
        ROS_PROGRAM.start(program3) 
    def onClick4(self):
        ROS_PROGRAM = QProcess(self)
        print("execute...")
        program3 = 'rosrun roy_dmp dmp_execute.py'
        ROS_PROGRAM.start(program3) 



def main():
    app = QtWidgets.QApplication(sys.argv)
    application = ApplicationWindow()
    application.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()