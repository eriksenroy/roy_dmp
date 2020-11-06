#!/usr/bin/python


import rospy
from dmp.srv import GetDMPPlan, GetDMPPlanRequest, GetDMPPlanResponse,LearnDMPFromDemo, LearnDMPFromDemoRequest, SetActiveDMP, SetActiveDMPRequest
from moveit_msgs.msg import RobotState, RobotTrajectory, DisplayTrajectory, DisplayRobotState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.srv import ExecuteKnownTrajectory, ExecuteKnownTrajectoryRequest, ExecuteKnownTrajectoryResponse
from sensor_msgs.msg import JointState
from moveit_msgs.msg._DisplayRobotState import DisplayRobotState
import time
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIKResponse, GetPositionIK
from moveit_msgs.msg._MoveItErrorCodes import MoveItErrorCodes
import copy
from tf.transformations import *
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf
import sys
sys.path.append("/home/roy/catkin_ws/src/roy_dmp/include/roy_dmp")
from kinematics_interface import *
# from helper_functions import moveit_error_dict


DEFAULT_JOINT_STATES = '/joint_states'
EXECUTE_KNOWN_TRAJ_SRV = '/execute_kinematic_path'
DEFAULT_IK_SERVICE = "/compute_ik"


DEBUG_MODE =  True

def createExecuteKnownTrajectoryRequest(trajectory, wait_for_execution=True):
    """Create a ExecuteKnownTrajectoryRequest from the given data,
    trajectory must be a RobotTrajectory probably filled from a GetCartesianPath call"""
    ektr = ExecuteKnownTrajectoryRequest()
    ektr.trajectory = trajectory
    ektr.wait_for_execution = wait_for_execution
    return ektr

class motionExecution():

    def __init__(self):
        rospy.loginfo("Initializing motionExecution")
        rospy.loginfo("Connecting to MoveIt! known trajectory executor server '" + EXECUTE_KNOWN_TRAJ_SRV + "'...")
        # self.execute_known_traj_service = rospy.ServiceProxy(EXECUTE_KNOWN_TRAJ_SRV, ExecuteKnownTrajectory)
        # self.execute_known_traj_service.wait_for_service()
        rospy.loginfo("Connected.")
        self.sv = StateValidity()
        self.robot_state_collision_pub = rospy.Publisher('/robot_collision_state', DisplayRobotState,queue_size=1)
        rospy.sleep(0.1) # Give time to the publisher to register
        #TODO: make ik_service_name a param to load from a yaml
        self.ik_service_name = DEFAULT_IK_SERVICE
        # Get a ServiceProxy for the IK service
        rospy.loginfo("Waiting for service '" + self.ik_service_name + "'...")
        rospy.wait_for_service(self.ik_service_name)
        self.ik_serv = rospy.ServiceProxy(self.ik_service_name, GetPositionIK)
        rospy.loginfo("Successful connection  to '" + self.ik_service_name + "'.")        
        
        self.arm = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        if DEBUG_MODE:
            self.pub_ok_markers = rospy.Publisher('ik_ok_marker_list', MarkerArray, latch=True,queue_size=1)
            self.ok_markers = MarkerArray()
        
            self.pub_fail_markers = rospy.Publisher('ik_fail_marker_list', MarkerArray, latch=True,queue_size=1)
            self.fail_markers = MarkerArray()
            self.markers_id = 5 

    def robotTrajectoryFromPlanPoseBased(self, plan, groups, downsample_freq=None):
        """Given a dmp plan (GetDMPPlanResponse) create a RobotTrajectory doing IK calls for the PoseStampeds
        provided so we can visualize and execute the motion"""
        rt = RobotTrajectory()
        jt = JointTrajectory()

        jt.points
        
        fjtg = self.computeJointTrajFromCartesian(plan.plan.points, plan.plan.times, downsample_freq=downsample_freq)
        rt.joint_trajectory = fjtg.trajectory
        if len(rt.joint_trajectory.points) != len(plan.plan.times):
            rospy.logerr(" Number of trajectory points is not equal to number of timestamps. This is a result of failed IKs")
            rospy.logerr("points: " + str(len(rt.joint_trajectory.points)) + " times: " + str(len(plan.plan.times)))

        return rt
    
    def robotTrajectoryFromPlanPoseBasedDownSampledAndWithDMP(self, plan, groups):
        """Given a dmp plan (GetDMPPlanResponse) create a RobotTrajectory doing IK calls for the PoseStampeds
        provided so we can visualize and execute the motion"""
        rt = RobotTrajectory()
        jt = JointTrajectory()

        jt.points
        
        fjtg = self.computeJointTrajFromCartesian(plan.plan.points, plan.plan.times)
        rt.joint_trajectory = fjtg.trajectory
        if len(rt.joint_trajectory.points) != len(plan.plan.times):
            rospy.logerr(" Number of trajectory points is not equal to number of timestamps. This is a result of failed IKs")
            rospy.logerr("points: " + str(len(rt.joint_trajectory.points)) + " times: " + str(len(plan.plan.times)))

        return rt
     
    def robotTrajectoryFromPlan(self, plan, joint_names):
        """Given a dmp plan (GetDMPPlanResponse) create a RobotTrajectory to be able to visualize what it consists and also
        to be able to send it to execution"""
        rt = RobotTrajectory()
        rt.joint_trajectory.joint_names = joint_names
        for point, time in zip(plan.plan.points, plan.plan.times):
            jtp = JointTrajectoryPoint()
            jtp.positions = point.positions
            jtp.velocities = point.velocities
            jtp.time_from_start = rospy.Duration(time)
            rt.joint_trajectory.points.append(jtp)
        return rt

    def checkTrajectoryValidity(self, robot_trajectory, groups=[]):
        """Given a robot trajectory, deduce it's groups and check it's validity on each point of the traj
        returns True if valid, False otherwise
        It's considered not valid if any point is not valid"""

        init_time = time.time()
        if len(groups) > 0:
            groups_to_check = groups
        else:
            groups_to_check = ['both_arms_torso'] # Automagic group deduction... giving a group that includes everything 
        for traj_point in robot_trajectory.joint_trajectory.points:
            rs = RobotState()
            rs.joint_state.name = robot_trajectory.joint_trajectory.joint_names
            rs.joint_state.position = traj_point.positions
            for group in groups_to_check:
                result = self.sv.getStateValidity(rs,group)
                if not result.valid:
                    rospy.logerr("Trajectory is not valid at point (RobotState):" + str(rs) + "with result of StateValidity: " + str(result))
                    rospy.logerr("published in /robot_collision_state the conflicting state")
                    drs = DisplayRobotState()
                    drs.state = rs
                    self.robot_state_collision_pub.publish(drs)
                    return False
            fin_time = time.time()
        rospy.logwarn("Trajectory validity of " + str(len(robot_trajectory.joint_trajectory.points)) + " points took " + str(fin_time - init_time))
        return True

    def displayTrajFromPlan(self, plan, joint_names, initial_state):
        """Given a plan (GetDMPPlanResponse), joint_names list and an initial_state as RobotState
        Create a displayTraj message"""
        dt = DisplayTrajectory()
        dt.trajectory.append( self.robotTrajectoryFromPlan(plan, joint_names) )
        dt.trajectory_start = self.robotStateFromJoints(joint_names, initial_state)
        dt.model_id = "reem"
        return dt

    def displayTrajFromRobotTraj(self, robot_traj):
        """Given a robot trajectory return a displaytrajectory with it"""
        dt = DisplayTrajectory()
        dt.trajectory.append(robot_traj)
        dt.trajectory_start.joint_state.name = robot_traj.joint_trajectory.joint_names
        dt.trajectory_start.joint_state.position = robot_traj.joint_trajectory.points[0].positions
        dt.model_id = "reem"
        return dt

    def robotStateFromJoints(self, joint_names, initial_state):
        """Given joint names and configs return a robotstate message"""
        rs = RobotState()
        rs.joint_state.name = joint_names
        rs.joint_state.position = initial_state
        return rs

    def sendTrajectory(self, robot_trajectory, wait_for_execution=True):
        """Given a RobotTrajectory send it to the controllers firstly checking it's trajectory validity"""
        if self.checkTrajectoryValidity(robot_trajectory):
            ektr = createExecuteKnownTrajectoryRequest(robot_trajectory, wait_for_execution)
            rospy.loginfo("Sending trajectory...")
            tim_init = time.time()
            ros_time_init = rospy.Time.now()
            self.execute_known_traj_service(ektr)
            ros_time_fin = rospy.Time.now()
            time_fin = time.time()
            ros_time_diff = ros_time_fin - ros_time_init

            rospy.loginfo("Finished trajectory! The service call took (realtime): " + str(time_fin - time_init) + " (ros time): " + str(ros_time_diff.to_sec()) + "s " + str(ros_time_diff.to_nsec()) + "ns")
            return True
        else:
            rospy.logerr("Trajectory in collision at some point, won't be sent to controllers.")
            return False

    def sendTrajectoryAndRecordBag(self, robot_trajectory, motion_name, joints, bag_name, wait_for_execution=False):
        from roy_dmp.script.dmp_record import RecordFromJointState
        lfjs = RecordFromJointState()

        if self.checkTrajectoryValidity(robot_trajectory):
            ektr = createExecuteKnownTrajectoryRequest(robot_trajectory, wait_for_execution)
            rospy.loginfo("Sending trajectory...")
            time_init = time.time()
            ros_time_init = rospy.Time.now()
            self.execute_known_traj_service.call(ektr)
            lfjs.start_record(motion_name, joints, bag_name)
            # Wait until completion of the trajectory.... kind of the duration of the traj
            # Get last trajectory time
            trajectory_final_time = robot_trajectory.joint_trajectory.points[-1].time_from_start
            rospy.sleep(trajectory_final_time)
            lfjs.stop_record()
            ros_time_fin = rospy.Time.now()
            time_fin = time.time()
            ros_time_diff = ros_time_fin - ros_time_init
            
            rospy.loginfo("Finished trajectory! The service call took (realtime): " + str(time_fin - time_init) + " (ros time): " + str(ros_time_diff.to_sec()) + "s " + str(ros_time_diff.to_nsec()) + "ns")
            return True
        else:
            rospy.logerr("Trajectory in collision at some point, won't be sent to controllers.")
            return False    

    def getCurrentJointsPose(self, joint_names):
        """ Retrieve joint values from a set of joints """
        current_joint_state = rospy.wait_for_message(DEFAULT_JOINT_STATES, JointState)
        joint_names, values = self.getNamesAndMsgList(joint_names,current_joint_state)
        return values

    def getCurrentEndEffectorPose(self, tf_frame):
        """With a list of joints, retrieve the pose of EE """
        transformer = tf.TransformListener()
        rospy.sleep(0.9)
        if not transformer.frameExists(tf_frame):
            print("Frame " + str(tf_frame) + " does not exist.")
            exit(0)
        transformer.waitForTransform(tf_frame, 'base_link', rospy.Time(), rospy.Duration(20.0))
        ps = PoseStamped()
        pos, quat = transformer.lookupTransform('base_link', tf_frame, rospy.Time())

        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = 'base_link'
        ps.pose.position = Point(*pos)
        ps.pose.orientation = Quaternion(*quat)
        return ps


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

    def getIkPose(self, pose, group="manipulator", previous_state=None):
        """Get IK of the pose specified, for the group specified, optionally using
        the robot_state of previous_state (if not, current robot state will be requested) """
        req = GetPositionIKRequest()
        req.ik_request.avoid_collisions = True
        req.ik_request.group_name = group
        req.ik_request.pose_stamped.header = Header(stamp=rospy.Time.now())
        req.ik_request.pose_stamped.header.frame_id = 'base_link'

        # Give the desired point to check for IK
        req.ik_request.pose_stamped.pose.position = pose.position
        req.ik_request.pose_stamped.pose.orientation = pose.orientation

        if previous_state == None:
            current_joint_state = rospy.wait_for_message(DEFAULT_JOINT_STATES, JointState)
            cs = RobotState()
            cs.joint_state = current_joint_state
            req.ik_request.robot_state = cs
        else:
            req.ik_request.robot_state = previous_state

        ik_answer = GetPositionIKRequest()
        ik_answer = self.ik_serv.call(req)

        return ik_answer
            
    def computeJointTrajFromCartesian(self, points, times, arm="right_arm_torso", downsample_freq=None):
        poselist = []
        for point in points:
            qt = quaternion_from_euler(point.positions[3], point.positions[4], point.positions[5])
            pose = Pose(Point(point.positions[0], point.positions[1], point.positions[2]),
                        Quaternion(*qt.tolist()))
            poselist.append(pose)
        fjt_goal = self.computeIKsPose(poselist, times, arm, downsample_freq)

        return fjt_goal

    def computeIKsPose(self, poselist, times, arm="right_arm", downsample_freq=None):
        """Giving a poselist (list of Pose) and times for every point compute it's iks and add it's times
        if a value is given to downsample_freq then we will only compute IKs and so for the downsampled
        rate of poses"""
        rospy.loginfo("Computing " + str(len(poselist)) + " IKs" )
        fjt_goal = FollowJointTrajectoryGoal()

        fjt_goal.trajectory.joint_names = self.arm

        ik_answer = None
        last_succesfull_ik_answer = None
        if downsample_freq != None:
            num_poses = 0
            num_downsampled_poses = 0
        for pose, time in zip(poselist, times):
            if downsample_freq != None:
                num_poses += 1
                if num_poses % downsample_freq == 0:
                    num_downsampled_poses += 1
                else: # if we are downsampling dont do IK calls if it's not one of the samples we want
                    continue
            if last_succesfull_ik_answer != None:
                ik_answer = self.getIkPose(pose,"right_arm", previous_state=last_succesfull_ik_answer.solution)
            else:
                ik_answer = self.getIkPose(pose)
            if DEBUG_MODE:
                rospy.loginfo("Got error_code: " + str(ik_answer.error_code.val) + " which means: " + moveit_error_dict[ik_answer.error_code.val])
            if ik_answer.error_code.val == 1:
                # We should check if the solution is very far away from joint config
                # if so.. try again... being a generic solution i dont know how to manage this
                last_succesfull_ik_answer = ik_answer
                if DEBUG_MODE:
                    arrow = self.createArrowMarker(pose, ColorRGBA(0,1,0,1))
                    self.ok_markers.markers.append(arrow)
                jtp = JointTrajectoryPoint()
                #ik_answer = GetConstraintAwarePositionIKResponse()
                # sort positions and add only the ones of the joints we are interested in
                positions = self.sortOutJointList(fjt_goal.trajectory.joint_names, ik_answer.solution.joint_state)
                jtp.positions = positions
                jtp.time_from_start = rospy.Duration(time)
                fjt_goal.trajectory.points.append(jtp)
                if DEBUG_MODE:
                    self.pub_ok_markers.publish(self.ok_markers)
                
            else:
                if DEBUG_MODE:
                    arrow = self.createArrowMarker(pose, ColorRGBA(1,0,0,1))
                    self.fail_markers.markers.append(arrow)
                    self.pub_fail_markers.publish(self.fail_markers)
                    # Loop for a while to check if we get a solution on further checks?
                    
        if downsample_freq != None:
            rospy.loginfo("From " + str(num_poses) + " points we downsampled to " + str(num_downsampled_poses) + " and fjt_goal.trajectory.points has " + str(fjt_goal.trajectory.points) + " points")
        return fjt_goal

    def sortOutJointList(self, joint_name_list, joint_state):
        """ Get only the joints we are interested in and it's values and return it in
        joint_state.name and joint_state.points format"""
        if DEBUG_MODE:
            rospy.loginfo("Sorting jointlist...")
        list_to_iterate = joint_name_list      
        curr_j_s = joint_state
        ids_list = []
        position_list = []
        for joint in list_to_iterate:
            idx_in_message = curr_j_s.name.index(joint)
            ids_list.append(idx_in_message)
            position_list.append(curr_j_s.position[idx_in_message])
        return position_list    

    def adaptTimesAndVelocitiesOfMsg(self, trajectory, plan, desired_final_time):
        """Adapt the times and velocities of the message for the controller
        from the times computed in the DMP and velocities 0.0, controller will take care of it"""
        rospy.loginfo("Adapting times and velocities...")
        traj = trajectory
        p = plan
        
        point = JointTrajectoryPoint()
        counter = 0
        for point in traj.trajectory.points:
            counter += 1
            point.velocities.extend(len(point.positions) * [0.0])
            point.time_from_start = rospy.Duration( counter * (desired_final_time / len(traj.trajectory.points)) )
        return traj

    def publish_markers(self, time=10.0):
        init_time = rospy.Time.now()
        time_done = False
        while not time_done:
            self.pub_ok_markers.publish(self.ok_markers)
            self.pub_fail_markers.publish(self.fail_markers)
            rospy.sleep(0.1)
            if rospy.Time.now() - init_time > rospy.Duration(time):
                time_done = True

    def createArrowMarker(self, pose, color):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.type = marker.ARROW
        marker.action = marker.ADD
        general_scale = 0.01
        marker.scale.x = general_scale
        marker.scale.y = general_scale / 3.0
        marker.scale.z = general_scale / 10.0
        marker.color = color
        marker.pose.orientation = pose.orientation
        marker.pose.position = pose.position
        marker.id = self.markers_id
        self.markers_id += 1
        return marker


if __name__ == "__main__":
    rospy.init_node("test_execution_classes")
    rospy.loginfo("Initializing dmp_execution test.")
    me = motionExecution()
    eepos = me.getCurrentEndEffectorPose('ee_link')
    epos = eepos.pose
    print(eepos)
    iksolu = me.getIkPose(epos)
    print(iksolu)
    
          
