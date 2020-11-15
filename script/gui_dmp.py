#!/usr/bin/python

# from roy_dmp.src.dmp_record import RecordFromJointState
# from roy_dmp.src.dmp_learn import motionGeneration
# from roy_dmp.src.dmp_execute import motionExecution
import sys
import os
import rospy
from sensor_msgs.msg import JointState
from PyQt5 import QtWidgets
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from ui_dmp import Ui_MainWindow
sys.path.append("/home/roy/catkin_ws/src/roy_dmp/src")
from dmp_record import *
from dmp_learn import *
from dmp_execute import *
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

def get_joints():
    # rospy.init_node("test_extrinsics_node")
    js = rospy.wait_for_message("joint_states",JointState)
    return js

class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(ApplicationWindow, self).__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        # self.
        self.ui.stackedWidget.setCurrentIndex(0)
        self.ui.startButton.clicked.connect(self.start_click)


    
        # Recording buttons connect
        self.ui.rec_EE_radio_button.clicked.connect(self.setRecEE)
        self.ui.rec_JS_radio_button.clicked.connect(self.setRecJS)
        self.ui.name_recording_line_edit.textChanged[str].connect(self.setNameRecording)
        self.ui.start_recording_button.clicked.connect(self.startRecording)
        self.ui.stop_recording_button.clicked.connect(self.stopRecording)

        # Learning buttons connect
        self.ui.load_EE_rec_radio_button.clicked.connect(self.loadEERec)
        self.ui.load_JS_rec_radio_button.clicked.connect(self.loadJSRec)
        self.ui.dmp_dt_param_lineedit.textChanged[str].connect(self.setdt)
        self.ui.dmp_K_param_lineedit.textChanged[str].connect(self.setK)
        self.ui.dmp_D_param_lineedit.textChanged[str].connect(self.setD)
        self.ui.dmp_basisfunc_param_lineedit.textChanged[str].connect(self.setBasisfunc)
        self.ui.Get_recordings_pushButton.clicked.connect(self.browse_folder)
        self.ui.load_name_rec_comboBox.currentIndexChanged.connect(self.loadNamerec)
        self.ui.generate_DMP_button.clicked.connect(self.generateDMP)
        
        # Executing buttons connect
        self.ui.get_active_DMP_pushButton.clicked.connect(self.browse_weights)
        self.ui.set_active_DMP_pushButton.clicked.connect(self.sendActiveDMP)
        self.ui.set_DMP_comboBox.currentIndexChanged.connect(self.setActiveDMP)
        self.ui.get_JS_init_robot_pushButton.clicked.connect(self.getJS_init_robot)
        #Robot variables
        self.arm = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
       
       #Initialize DMP classes from ROS
        self.dmp_record_JS = RecordFromJointState()
        self.dmp_mg = motionGeneration()
        # self.dmp_me = motionExecution()



        #Param Recording parge
        self.dmp_record = 0  # 0 for EE, 1 for JS, 2 for JS with Filtering
        self.dmp_record_name = "No_name"
        
        #Param Learning page
        self.dmp_record_name_load = "No_name"
        self.dmp_weight_name = "No_name"
        self.dmp_load_type = 1 # 0 for EE, 1 for JS, 2 for JS with filtering
        self.dmp_param_dt = 0.008
        self.dmp_param_K = 100
        self.dmp_param_D = 1
        self.dmp_param_basisfunc = 150

        # Param executing page
        self.active_DMP = "No_name"
        self.initial_JS = [-0.2273033300982874, -2.298889462147848, -1.0177272001849573, -1.3976243177997034,  1.5502419471740723, 9.261386219655172]
        self.goal_JS = [-2.3324595133410853, -2.2434170881854456, -1.1172669569598597, -1.3543337027179163, 1.5941375494003296, 7.169057373200552]
        # Front page params
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
        self.dmp_record = 0
    def setRecJS(self):
        self.dmp_record = 1
        print(self.dmp_record)
    def setRecJSFilter(self):
        self.dmp_record = 2
    def setNameRecording(self, text):
        self.dmp_record_name = text
        # print(self.dmp_record_name)

    def startRecording(self):
        if self.dmp_record ==1:
            self.dmp_record_JS.start_record("Testing",self.arm,self.dmp_record_name)
        else:
            pass
        
    def stopRecording(self):
        self.dmp_record_JS.stop_record()
    def loadEERec(self):
        self.dmp_load_type = 0
    def loadJSRec(self):
        # self.dmp_mg.loadMotionFromJointStates(self.dmp_record_name_load,self.arm)
        self.dmp_load_type = 1

    def setdt(self,text):
        self.dmp_param_dt = float(text)
    def setK(self,text):
        self.dmp_param_K = float(text)
    def setD(self,text):
        self.dmp_param_D = float(text)
    def setBasisfunc(self,text):
        self.dmp_param_basisfunc = int(text)
    def loadNamerec(self):
        self.dmp_record_name_load = self.ui.load_name_rec_comboBox.currentText()
    def saveWeights(self,text):
        self.dmp_weight_name = str(text)
    def generateDMP(self):
        self.dmp_mg.loadMotionFromJointStates(self.dmp_record_name_load,self.arm)
    def browse_weights(self):
        self.ui.set_DMP_comboBox.clear()
        directory = QtWidgets.QFileDialog.getExistingDirectory(self,"Choose a directory")
        if directory:
            for file_name in os.listdir(directory):
                print(file_name)
                self.ui.set_DMP_comboBox.addItem(str(file_name))      
    def setActiveDMP(self):
        self.active_DMP = self.ui.set_DMP_comboBox.currentText()
    def sendActiveDMP(self):
        self.dmp_mg.loadMotionYAML(self.active_DMP)
    def setJS_init_0(self,text):
        self.initial_JS[0] = float(text)
    def setJS_init_1(self,text):
        self.initial_JS[1] = float(text)
    def setJS_init_2(self,text):
        self.initial_JS[2] = float(text)
    def setJS_init_3(self,text):
        self.initial_JS[3] = float(text)
    def setJS_init_4(self,text):
        self.initial_JS[4] = float(text)
    def setJS_init_5(self,text):
        self.initial_JS[5] = float(text)
    def setJS_goal_0(self,text):
        self.goal_JS[0] = float(text)
    def setJS_goal_1(self,text):
        self.goal_JS[1] = float(text)
    def setJS_goal_2(self,text):
        self.goal_JS[2] = float(text)
    def setJS_goal_3(self,text):
        self.goal_JS[3] = float(text)
    def setJS_goal_4(self,text):
        self.goal_JS[4] = float(text)
    def setJS_goal_5(self,text):
        self.goal_JS[5] = float(text)
    def getJS_init_robot(self):
        print("waiting for message")
        # joint_states = rospy.wait_for_message("/joint_states", JointState)
        # print(joint_states)
        joint_states = get_joints()
        self.initial_JS = [joint_states.position[2],joint_states.position[1],joint_states.position[0],joint_states.position[3],joint_states.position[4],joint_states.position[5]]
        self.ui.set_JointState_init_0_lineEdit.setText(str(self.initial_JS[0]))
        print(self.initial_JS[0])
    def getJS_goal_robot(self):
        joint_states = rospy.wait_for_message("/joint_states", JointState)
        self.goal_JS = [joint_states.position[2],joint_states.position[1],joint_states.position[0],joint_states.position[3],joint_states.position[4],joint_states.position[5]]
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