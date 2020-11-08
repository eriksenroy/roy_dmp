import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import math
import actionlib
# from ros_dmp.msg import *
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from math import pi
# from ur3ik.srv import *

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

client = None

def move_robot(_joint_value):
    g = FollowJointTrajectoryActionGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    try:
        joint_states = rospy.wait_for_message("joint_states",JointState)
        joints_pos = joint_states.position
        joint_value = []
        velocity = []
        times = []
        d= 2.00
        g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
        for i in range(len(joint_states)):
            Q = [joint_value[0],joint_value[1],joint_value[2],joint_value[3],joint_value[4],joint_value[5]]
            V = [velocity[0],velocity[1],velocity[2],velocity[3],velocity[4],velocity[5]]
            T = times[i]
            g.trajectory.points = [JointTrajectoryPoint(positions=Q, velocities=V, time_from_start=rospy.Duration(T))]
        client.send_goal(g)
        client.wait_for_result()

    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def main():
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        # client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print("Waiting for server...")
        client.wait_for_server()
        print ("Connected to server")
        inp = raw_input("Continue? y/n: ")[0]
        if (inp == 'y'):
            move_robot(cartesian_trajectory)
        else:
            print("Halting program")
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise