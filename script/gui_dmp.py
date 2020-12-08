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
        # self.ui.
        self.ui.gridLayout_6.addWidget(self.RviZ)
        # self.ui.gridLayout_6.addWidget(self.RviZ)
    
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
        self.ui.execute_collisionCheck_PB.clicked.connect(self.collision_check)
        self.ui.execute_plan_pushButton.clicked.connect(self.execute_plan)

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

    # def setSimulation(self):
        # pass
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
        self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
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
        self.ui.set_JointState_init_1_lineEdit.setText(str(self.initial_JS[1]))
        self.ui.set_JointState_init_2_lineEdit.setText(str(self.initial_JS[2]))
        self.ui.set_JointState_init_3_lineEdit.setText(str(self.initial_JS[3]))
        self.ui.set_JointState_init_4_lineEdit.setText(str(self.initial_JS[4]))
        self.ui.set_JointState_init_5_lineEdit.setText(str(self.initial_JS[5]))
        print(self.initial_JS[0])
    def getJS_goal_robot(self):
        joint_states = rospy.wait_for_message("/joint_states", JointState)
        self.goal_JS = [joint_states.position[2],joint_states.position[1],joint_states.position[0],joint_states.position[3],joint_states.position[4],joint_states.position[5]]
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
        
        initial_pose =[-0.2273033300982874, -2.298889462147848, -1.0177272001849573, -1.3976243177997034,  1.5502419471740723, 9.261386219655172]
        final_pose = [-2.3324595133410853, -2.2434170881854456, -1.1172669569598597, -1.3543337027179163, 1.5941375494003296, 7.169057373200552]
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

    ## MyViz Constructor
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## Its constructor creates and configures all the component widgets:
    ## frame, thickness_slider, top_button, and side_button, and adds them
    ## to layouts.
    def __init__(self):
        QWidget.__init__(self)

        ## rviz.VisualizationFrame is the main container widget of the
        ## regular RViz application, with menus, a toolbar, a status
        ## bar, and many docked subpanels.  In this example, we
        ## disable everything so that the only thing visible is the 3D
        ## render window.
        self.frame = rviz.VisualizationFrame()

        ## The "splash path" is the full path of an image file which
        ## gets shown during loading.  Setting it to the empty string
        ## suppresses that behavior.
        self.frame.setSplashPath( "" )

        ## VisualizationFrame.initialize() must be called before
        ## VisualizationFrame.load().  In fact it must be called
        ## before most interactions with RViz classes because it
        ## instantiates and initializes the VisualizationManager,
        ## which is the central class of RViz.
        self.frame.initialize()

        ## The reader reads config file data into the config object.
        ## VisualizationFrame reads its data from the config object.
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile( config, "/home/roy/catkin_ws/src/roy_dmp/resources/dmp_config.rviz" )
        self.frame.load( config )

        ## You can also store any other application data you like in
        ## the config object.  Here we read the window title from the
        ## map key called "Title", which has been added by hand to the
        ## config file.
        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )

        ## Here we disable the menu bar (from the top), status bar
        ## (from the bottom), and the "hide-docks" buttons, which are
        ## the tall skinny buttons on the left and right sides of the
        ## main render window.
        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( True)

        ## frame.getManager() returns the VisualizationManager
        ## instance, which is a very central class.  It has pointers
        ## to other manager objects and is generally required to make
        ## any changes in an rviz instance.
        self.manager = self.frame.getManager()

        ## Since the config file is part of the source code for this
        ## example, we know that the first display in the list is the
        ## grid we want to control.  Here we just save a reference to
        ## it for later.
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
        
        ## Here we create the layout and other widgets in the usual Qt way.
        layout = QVBoxLayout()
        layout.addWidget( self.frame )
        
        # thickness_slider = QSlider( Qt.Horizontal )
        # thickness_slider.setTracking( True )
        # thickness_slider.setMinimum( 1 )
        # thickness_slider.setMaximum( 1000 )
        # thickness_slider.valueChanged.connect( self.onThicknessSliderChanged )
        # layout.addWidget( thickness_slider )
        
        h_layout = QHBoxLayout()
        
        top_button = QPushButton( "Top View" )
        top_button.clicked.connect( self.onTopButtonClick )
        h_layout.addWidget( top_button )
        
        side_button = QPushButton( "Side View" )
        side_button.clicked.connect( self.onSideButtonClick )
        h_layout.addWidget( side_button )
        
        layout.addLayout( h_layout )
        
        self.setLayout( layout )

    ## Handle GUI events
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## After the constructor, for this example the class just needs to
    ## respond to GUI events.  Here is the slider callback.
    ## rviz.Display is a subclass of rviz.Property.  Each Property can
    ## have sub-properties, forming a tree.  To change a Property of a
    ## Display, use the subProp() function to walk down the tree to
    ## find the child you need.
    def onThicknessSliderChanged( self, new_value ):
        if self.grid_display != None:
            self.grid_display.subProp( "Line Style" ).subProp( "Line Width" ).setValue( new_value / 1000.0 )

    ## The view buttons just call switchToView() with the name of a saved view.
    def onTopButtonClick( self ):
        self.switchToView( "Top View" );
        
    def onSideButtonClick( self ):
        self.switchToView( "Side View" );
        
    ## switchToView() works by looping over the views saved in the
    ## ViewManager and looking for one with a matching name.
    ##
    ## view_man.setCurrentFrom() takes the saved view
    ## instance and copies it to set the current view
    ## controller.
    def switchToView( self, view_name ):
        view_man = self.manager.getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == view_name:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
        print( "Did not find view named %s." % view_name )

## Start the Application
        


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