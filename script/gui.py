#!/usr/bin/python

from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
import sys

class MainLauncher(QWidget):

    def __init__(self):
        QWidget.__init__(self)
        mainLayout = QGridLayout()
        self.launchButton = QPushButton( "learn" )
        self.launchButton.clicked.connect( self.onClick1 )
        self.launchButton2 = QPushButton( "LAUNCH" )
        self.launchButton2.clicked.connect( self.onClick2 )
        self.launchButton3 = QPushButton( "execute" )
        self.launchButton3.clicked.connect( self.onClick3 )         
        mainLayout.addWidget(self.launchButton, 1, 1)
        mainLayout.addWidget(self.launchButton2, 2, 1)
        mainLayout.addWidget(self.launchButton3, 3, 1)
        self.setLayout( mainLayout )

    def onClick1(self):
        ROS_PROGRAM = QProcess(self)
        print("sourcing...")
        # source ~/catkin_ws/devel/setup.bash
        program = 'rosrun roy_dmp dmp_learn.py'
        ROS_PROGRAM.start(program)
        print("Finished")

    def onClick2(self):
        ROS_PROGRAM = QProcess(self)
        print("Launching...")
        program2 = 'roslaunch roy_dmp ur3_gazebo_with_dmp.launch'
        ROS_PROGRAM.start(program2)
    def onClick3(self):
        ROS_PROGRAM = QProcess(self)
        print("Launching...")
        program3 = 'rosrun roy_dmp dmp_execute.py'
        ROS_PROGRAM.start(program3)         

if __name__ == '__main__':

    app = QApplication( sys.argv )
    mainLauncher = MainLauncher()
    mainLauncher.show()
    sys.exit(app.exec_())
