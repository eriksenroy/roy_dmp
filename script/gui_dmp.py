#!/usr/bin/python

# from roy_dmp.src.dmp_record import RecordFromJointState
# from roy_dmp.src.dmp_learn import motionGeneration
# from roy_dmp.src.dmp_execute import motionExecution
import sys
import os
import rospy
from sensor_msgs.msg import JointState
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QWidget,QHBoxLayout, QMainWindow, QPushButton, QMessageBox, QBoxLayout,QVBoxLayout
from PyQt5.QtGui import *
from PyQt5.QtCore import *
# from python_qt_binding.QtGui import *
# from python_qt_binding.QtCore import *
import rviz
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
        self.ui.simulationCheckBox.stateChanged.connect(self.setSimulation)
        self.ui.stackedWidget.setCurrentIndex(0)
        self.ui.startButton.clicked.connect(self.start_click)
        self.ui.start_loadClasses_PB.clicked.connect(self.load_classes)
        self.RviZ = RViz()
        self.ui.gridLayout_6.addWidget(self.RviZ)

    
        # Recording buttons connect
        self.ui.rec_EE_radio_button.clicked.connect(self.setRecEE)
        self.ui.rec_JS_radio_button.clicked.connect(self.setRecJS)
        self.ui.name_recording_line_edit.textChanged[str].connect(self.setNameRecording)
        self.ui.start_recording_button.clicked.connect(self.startRecording)
        self.ui.stop_recording_button.clicked.connect(self.stopRecording)

        # Learning buttons connect
        self.ui.load_EE_rec_radio_button.clicked.connect(self.loadEERec)
        self.ui.load_JS_rec_radio_button.clicked.connect(self.loadJSRec)
        # self.ui.dmp_dt_param_lineedit.setValidator(QDoubleValidator(0.0,99,5))
        self.ui.dmp_dt_param_lineedit.textChanged[str].connect(self.setdt)
        self.ui.dmp_K_param_lineedit.setValidator(QIntValidator(0,1000))
        self.ui.dmp_K_param_lineedit.textChanged[str].connect(self.setK)
        self.ui.dmp_D_param_lineedit.setValidator(QIntValidator(0,1000))
        self.ui.dmp_D_param_lineedit.textChanged[str].connect(self.setD)
        self.ui.dmp_basisfunc_param_lineedit.setValidator(QIntValidator(0,9999))
        self.ui.dmp_basisfunc_param_lineedit.textChanged[str].connect(self.setBasisfunc)
        self.ui.Get_recordings_pushButton.clicked.connect(self.browse_folder)
        self.ui.load_name_rec_comboBox.currentIndexChanged.connect(self.loadNamerec)
        self.ui.generate_DMP_button.clicked.connect(self.generateDMP)
        
        # Executing buttons connect
        self.ui.get_active_DMP_pushButton.clicked.connect(self.browse_weights)
        self.ui.set_active_DMP_pushButton.clicked.connect(self.sendActiveDMP)
        self.ui.set_DMP_comboBox.currentIndexChanged.connect(self.setActiveDMP)
        self.ui.get_JS_init_robot_pushButton.clicked.connect(self.getJS_init_robot)
        self.ui.execute_collisionCheck_PB.clicked.connect(self.collision_check)
        self.ui.execute_plan_pushButton.clicked.connect(self.execute_plan)

        #line edit connections
        self.ui.set_JointState_init_0_lineEdit.textChanged[str].connect(self.setJS_init_0)
        self.ui.set_JointState_init_1_lineEdit.textChanged[str].connect(self.setJS_init_1)
        self.ui.set_JointState_init_2_lineEdit.textChanged[str].connect(self.setJS_init_2)
        self.ui.set_JointState_init_3_lineEdit.textChanged[str].connect(self.setJS_init_3)
        self.ui.set_JointState_init_4_lineEdit.textChanged[str].connect(self.setJS_init_4)
        self.ui.set_JointState_init_5_lineEdit.textChanged[str].connect(self.setJS_init_5)

        self.ui.execute_plan_pushButton.setEnabled(False)
        #Robot variables
        self.arm = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
       
       #Initialize DMP classes from ROS
        self.dmp_record_JS =[]
        self.dmp_mg = []
        self.dmp_me = []



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
        self.path_plan = []
        # Front page params
        self.robot = 'roslaunch roy_dmp ur3_gazebo_with_dmp.launch'
        # self.robot = 'roslaunch ur_gazebo ur3.launch'
        self.IP = '192.168.2.192'
        self.sim = True
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
            print("True")
        else:
            self.sim = False
            print("False")

    def setIPRobot(self,text):
        self.IP = text
    def chooseRobot(self,text):
        if text == 'UR3':
            if self.sim:
               self.robot = 'roslaunch roy_dmp ur3_gazebo_with_dmp.launch' 
            else:
                self.robot = 'roslaunch ur_gazebo ur3.launch'
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
        self.dmp_mg.loadMotionFromJointStates(self.dmp_record_name_load,self.arm)
        self.dmp_load_type = 1

    def setdt(self,text):
        try:
            self.dmp_param_dt = float(text)
        except:
            print("Wrong Type")
    def setK(self,text):
        try:
            self.dmp_param_K = float(text)
        except:
            print("Wrong Type")
    def setD(self,text):
        try:
            self.dmp_param_D = float(text)
        except:
            print("Wrong Type")
    def setBasisfunc(self,text):
        try:
            self.dmp_param_basisfunc = int(text)
        except:
            print("Wrong Type")
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
        self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
        self.active_DMP = self.ui.set_DMP_comboBox.currentText()
    def sendActiveDMP(self):
        self.dmp_mg.loadMotionYAML(self.active_DMP)
    def setJS_init_0(self,text):
        try:
            self.initial_JS[0] = float(text)
        except:
            print("Wrong Type")
    def setJS_init_1(self,text):
        try:
            self.initial_JS[1] = float(text)
        except:
            print("Wrong Type")
    def setJS_init_2(self,text):
        try:
            self.initial_JS[2] = float(text)
        except:
            print("Wrong Type")
    def setJS_init_3(self,text):
        try:
            self.initial_JS[3] = float(text)
        except:
            print("Wrong Type")
    def setJS_init_4(self,text):
        try:
            self.initial_JS[4] = float(text)
        except:
            print("Wrong Type")
    def setJS_init_5(self,text):
        try:
            self.initial_JS[5] = float(text)
        except:
            print("Wrong Type")
    def setJS_goal_0(self,text):
        try:
            self.goal_JS[0] = float(text)
        except:
            print("Wrong Type")
    def setJS_goal_1(self,text):
        try:
            self.goal_JS[1] = float(text)
        except:
            print("Wrong Type")
    def setJS_goal_2(self,text):
        try:
            self.goal_JS[2] = float(text)
        except:
            print("Wrong Type")
    def setJS_goal_3(self,text):
        try:
            self.goal_JS[3] = float(text)
        except:
            print("Wrong Type")
    def setJS_goal_4(self,text):
        try:
            self.goal_JS[4] = float(text)
        except:
            print("Wrong Type")
    def setJS_goal_5(self,text):
        try:
            self.goal_JS[5] = float(text)
        except:
            print("Wrong Type")
    def getJS_init_robot(self):
        print("waiting for message")
        # joint_states = rospy.wait_for_message("/joint_states", JointState)
        # print(joint_states)
        
        joint_states = get_joints()
        self.initial_JS = [joint_states.position[2],joint_states.position[1],joint_states.position[0],joint_states.position[3],joint_states.position[4],joint_states.position[5]]
        self.ui.set_JointState_init_0_lineEdit.setText(str(self.initial_JS[0]))
        self.ui.set_JointState_init_1_lineEdit.setText(str(self.initial_JS[1]))
        self.ui.set_JointState_init_2_lineEdit.setText(str(self.initial_JS[2]))
        self.ui.set_JointState_init_3_lineEdit.setText(str(self.initial_JS[3]))
        self.ui.set_JointState_init_4_lineEdit.setText(str(self.initial_JS[4]))
        self.ui.set_JointState_init_5_lineEdit.setText(str(self.initial_JS[5]))
        print(self.initial_JS[0])
    def getJS_goal_robot(self):
        joint_states = rospy.wait_for_message("/joint_states", JointState)
        self.goal_JS = [joint_states.position[2],joint_states.position[1],joint_states.position[0],joint_states.position[3],joint_states.position[4],joint_states.position[5]]
        self.ui.set_JointState_goal_0_lineEdit.setText(str(self.goal_JS[0]))
        self.ui.set_JointState_goal_1_lineEdit.setText(str(self.goal_JS[1]))
        self.ui.set_JointState_goal_2_lineEdit.setText(str(self.goal_JS[2]))
        self.ui.set_JointState_goal_3_lineEdit.setText(str(self.goal_JS[3]))
        self.ui.set_JointState_goal_4_lineEdit.setText(str(self.goal_JS[4]))
        self.ui.set_JointState_goal_5_lineEdit.setText(str(self.goal_JS[5]))
    def setTau(self):
        pass
    def getDMP_plan(self):
        pass
    def load_classes(self):
        self.dmp_record_JS = RecordFromJointState()
        self.dmp_mg = motionGeneration()
        self.dmp_me = motionExecution()
        self.ui.stackedWidget.setCurrentIndex(1)

    def execute_plan(self):
        st = self.dmp_me.sendTrajectoryAction(self.path_plan,self.initial_JS)
        print(st)
    def collision_check(self):
        
        initial_pose = self.initial_JS
        final_pose = self.goal_JS
        print("start path plan")
        self.path_plan = self.dmp_mg.getPlan(initial_pose,final_pose,-1,[],None,tau=5,dt=0.008)
        print(self.path_plan.plan.points[0].positions)
        robot_traj = self.dmp_me.robotTrajectoryFromPlan(self.path_plan,self.dmp_me.arm)
        print("checking collisions")
        validity = self.dmp_me.checkTrajectoryValidity(robot_traj)
        self.dmp_me.pathPublish(self.path_plan)
        print("OK")
        print(validity)
        if validity:
            print("OK")
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: green")
            self.ui.execute_plan_pushButton.setEnabled(True)
        else:
            print("not ok")
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: red")
            self.ui.execute_plan_pushButton.setEnabled(False)
        
class RViz(QWidget ):

    def __init__(self):
        QWidget.__init__(self)
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath( "" )
        self.frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile( config, "/home/roy/catkin_ws/src/roy_dmp/resources/dmp_config.rviz" )
        self.frame.load( config )
        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )
        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( True)
        self.manager = self.frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
        layout = QVBoxLayout()
        layout.addWidget( self.frame )
        h_layout = QHBoxLayout()
        layout.addLayout( h_layout )
        self.setLayout( layout )

        


def main():
    rospy.init_node("DMP_GUI", anonymous=False)
    app = QtWidgets.QApplication(sys.argv)
    application = ApplicationWindow()
    application.show()
    # rviz = RViz()
    # # rviz.resize( 500, 500 )
    # rviz.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()