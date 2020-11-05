#!/usr/bin/env python

import rospy
import subprocess, yaml
import math
import numpy as np
import rosbag
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion
from dmp.srv import GetDMPPlan, GetDMPPlanRequest,LearnDMPFromDemo, LearnDMPFromDemoRequest, SetActiveDMP, SetActiveDMPRequest
from dmp.msg import DMPTraj, DMPData, DMPPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse, GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from scipy.interpolate import interp1d
import time

DEFAULT_JOINT_STATES = "/joint_states"
DEFAULT_FK_SERVICE = "/compute_fk"
DEFAULT_IK_SERVICE = "/compute_ik"

class motionGeneration():

    def __init__(self):
        self.arm = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.motion_difference = []

    def getNamesAndMsgList(self, joints, joint_state_msg):
        """ Get the joints for the specified group and return this name list and a list of it's values in joint_states
        Note: the names and values are correlated in position """
        list_to_iterate = joints
        curr_j_s = joint_state_msg
        ids_list = []
        msg_list = []
        rospy.logdebug("Current message: " + str(curr_j_s))
        for joint in list_to_iterate:
            idx_in_message = curr_j_s.name.index(joint)
            ids_list.append(idx_in_message)
            msg_list.append(curr_j_s.position[idx_in_message])
        rospy.logdebug("Current position of joints in message: " + str(ids_list))
        rospy.logdebug("Current msg:" + str(msg_list))

        return list_to_iterate, msg_list        


    def printNamesAndValues():
        """Given a group, print in screen in a pretty way joint names and it's values"""
        pass

    def loadMotionFromEndEffector():
        """Load motion from the bag name given """
        pass

    def loadMotionFromJointStates(self, bagname, joints):
        """Load motion from the bag name given """
        # Get bag info
        self.info_bag = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bagname],stdout=subprocess.PIPE).communicate()[0])
        bases_rel_to_time = math.ceil(self.info_bag['duration'] * 20) # empirically for every second 20 bases it's ok

        # Create a DMP from the number of joints in the trajectory
        dims = len(joints)
        dt = 0.02
        K = 100
        D = 2.0 * np.sqrt(K)
        num_bases = bases_rel_to_time

        traj = []
        bag = rosbag.Bag(bagname)
        first_point = True
        for topic, msg, t in bag.read_messages(topics=[DEFAULT_JOINT_STATES]):
            # Get the joint and its values
            js = msg
            # Process interesting joints
            names, positions = self.getNamesAndMsgList(joints,msg)
            # Append interesting joints here
            traj.append(positions)
            if first_point:
                # Store the first point
                self.motion_x0 = positions
                first_point = False
        bag.close()
        # Store the final point
        self.motion_goal = positions

        # Compute the difference between initial and final point
        for val1, val2 in zip(self.motion_x0, self.motion_goal):
            self.motion_difference.append(val2-val1)
        print(str(len(traj)) + " points in example traj. Using " + str(num_bases) + " num_bases")

        resp = self.makeLFDRequest(dims,traj,dt,K,D,num_bases)
        # Set it as the active DMP on the DMP server( ros_DMP)
        self.makeSetActiveRequest(resp.dmp_list)
        self.resp_from_makeLFDRequest = resp

        rospy.loginfo("Joints:" + str(joints))
        rospy.loginfo("Initial pose:" + str(self.motion_x0))
        rospy.loginfo("Final pose: " + str(self.motion_goal))
        time = self.info_bag['duration']
        rospy.loginfo("Time: " + str(time))
        #rospy.loginfo("DMP result: " + str(self.resp_from_makeLFDRequest))
        motion_dict = self.saveMotionYAML(bagname + ".yaml", bagname, joints, self.motion_x0, self.motion_goal, self.resp_from_makeLFDRequest, time)
        return motion_dict        
        



    def loadMotionFromBagJointStatesAndRemoveJerkiness(self, bagname, joints, frequency_to_downsample=15):
        """Load motion from the bag name given and remove jerkiness by downsampling and
        interpolating with a cubic spline """        
        pass

    def saveMotionYAML(self, yamlname, name, joints, initial_pose, final_pose, computed_dmp, time):
        """Given the info of the motion computed with the DMP server save it into a yaml file.
        @yamlname string name of the yaml file
        @name string name of the motion
        @joints list of strings joints that are included in this motion
        @initial_pose list of double initial pose for the motion as it was recorded in the training
        @final_pose list of double final pose of the motion as it was recorded in the training
        @computed_dmp python/object/new:dmp.srv._LearnDMPFromDemo.LearnDMPFromDemoResponse with the response of the DMP server
        @time double how long the motion took"""
        motion_dict = {"name": name,
                        "joints" : joints,
                        "initial_pose" : initial_pose,
                        "final_pose" : final_pose,
                        "computed_dmp" : computed_dmp,
                        "duration" : time}
        #rospy.loginfo("motion_dict:\n" + str(motion_dict))
        stream = file(yamlname, "w")
        yaml.dump(motion_dict, stream)
        self.resp_from_makeLFDRequest = motion_dict["computed_dmp"]
        return motion_dict        

    def loadMotionYAML(self, yamlname):
        """Given a yamlname which has a motion saved load it and set it as active in the DMP server"""
        try:
            stream = file(yamlname, "r")
        except:
            print("Can not find file: " + yamlname)
            return None
        motion_dict = yaml.load(stream)
        # set it as the active DMP on the dmp server
        self.makeSetActiveRequest(motion_dict['computed_dmp'].dmp_list)
        self.resp_from_makeLFDRequest = motion_dict['computed_dmp']
        return motion_dict

    def makeLFDRequest(self, dims, traj, dt, K_gain, D_gain, num_bases):
        """Learn a DMP from demonstration data """
        demotraj = DMPTraj()

        for i in range(len(traj)):
            pt = DMPPoint()
            pt.positions = traj[i]
            demotraj.points.append(pt)
            demotraj.times.append(dt*i)
        k_gains = [K_gain]*dims
        d_gains = [D_gain]*dims

        print("Starting ...")
        init_time = time.time()
        rospy.wait_for_service('learn_dmp_from_demo')
        try:
            lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
            resp = lfd(demotraj,k_gains,d_gains,num_bases)
        except rospy.ServiceException as e:
            print("Service call failes: %s"%e)
        fin_time = time.time()
        print("LfD done, took: " + str(fin_time - init_time))

        return resp

    def makeSetActiveRequest(self, dmp_list):
        """ Set a DMP as active on the DMP server"""
        try:
            sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
            sad(dmp_list)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)


if __name__ == "__main__":
    rospy.init_node("test_generation_classes")
    rospy.loginfo("Initializing dmp_generation test.")
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    mg = motionGeneration()
    mg.loadMotionFromJointStates("no_bag_name_set.bag",JOINT_NAMES)