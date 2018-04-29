#!/usr/bin/env python

from __future__ import division, print_function
import numpy as np
from math import *
from time import sleep
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import roslib
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
import actionlib
import rospy
from ros_myo.msg import EmgArray
from std_msgs.msg import String, Header
from geometry_msgs.msg import WrenchStamped, Vector3
import tf
from tf.transformations import *
# import roslib
# roslib.load_manifest("ur_kinematics")
# import sys
# #sys.path.append('/home/intuitivecompting/catkin_ws/src/ur5/universal_robot/ur_kinematics/src/ur_kin_py.cpp')
# from .ur_kin_py import forward, inverse
from icl_phri_robotiq_control.robotiq_utils import *

normalize = lambda x: x/np.sqrt(x[0]**2.+x[1]**2.+x[2]**2.)
norm = lambda a:np.sqrt(a.x**2.+a.y**2.+a.z**2.)
to_quat = lambda o: np.array([o.x, o.y, o.z, o.w])
rad_to_ang = lambda x: x / np.pi * 180.
ang_to_rad = lambda x: x / 180. * np.pi
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joi        self.joints_pos_start[3] = self.joints_pos_start[3]+diff_znt', 'wrist_3_joint']
Q_fetch = [1.5279078483581543, -2.149566952382223, -1.1292513052569788, 0.14056265354156494, 1.786577582359314, 1.541101336479187]
Q_wait = [0.6767016649246216, -1.2560237089740198, -1.9550021330462855, -0.019442383443013966, 1.5472047328948975, 1.5413050651550293]
Q_give = [-0.20308286348451787, -2.2909210363971155, -1.2152870337115687, 0.34171295166015625, 1.4318565130233765, 1.5419158935546875]

# def best_sol(sols, q_guess, weights):
#     valid_sols = []
#     for sol in sols:
#         test_sol = np.ones(6)*9999.
#         for i in range(6):
#             for add_ang in [-2.*np.pi, 0, 2.*np.pi]:
#                 test_ang = sol[i] + add_ang
#                 if (abs(test_ang) <= 2.*np.pi and 
#                     abs(test_ang - q_guess[i]) < abs(test_sol[i] - q_guess[i])):
#                     test_sol[i] = test_ang
#         if np.all(test_sol != 9999.):
#             valid_sols.append(test_sol)
#     if len(valid_sols) == 0:
#         return None
#     best_sol_ind = np.argmin(np.sum((weights*(valid_sols - np.array(q_guess)))**2,1))
#     return valid_sols[best_sol_ind]

# class Handover:
#     def __init__(self):
#         #/vel_based_pos_traj_controller/
#         self.client = actionlib.SimpleActionClient('icl_phri_ur5/follow_joint_trajectory', FollowJointTrajectoryAction)
#         self.goal = FollowJointTrajectoryGoal()
#         self.goal.trajectory = JointTrajectory()
#         self.goal.trajectory.joint_names = JOINT_NAMES
#         print ("Waiting for server...")
#         self.client.wait_for_server()
#         print ("Connected to server")
#         joint_states = rospy.wait_for_message("icl_phri_ur5/joint_states", JointState)
#         self.joints_pos_start = np.array(joint_states.position)
#         print ("Init done")
#         self.listener = tf.TransformListener()

#     def move(self):
#         current_m = forward(self.joints_pos_start)
#         hand_base = self._get_transform('/ee_link', '/right_hand_1')
#         gripper_hand = None
#         Desti_m = self._cal_dest(gripper_hand, hand_base)
#         sols = inverseinverse(np.array(Desti_m), float(self.joints_pos_start[5]))
#         qsol = best_sol(sols, self.joints_pos_start, [1.]*6)
#         if qsol is not None:
#             self.goal.trajectory.points = [
#                 JointTrajectoryPoint(positions=self.joints_pos_start.tolist(), velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
#                 JointTrajectoryPoint(positions=qsol.tolist(), velocities=[0]*6, time_from_start=rospy.Duration(1./125.)),
#             ]
#             print('start: ' + str(self.joints_pos_start.tolist()))
#             print('goal: ' + str(qsol.tolist()))
#             try:
#                 self.client.send_goal(self.goal)
#                 self.joints_pos_start = qsol
#             except:
#                 raise
    
#     def _get_transform(self, target_frame, source_frame, is_matrix=True):
#         rate = rospy.Rate(10.0)
#         rate.sleep()        
#         # try:
#         (trans, quat) = self.listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
#         if is_matrix:
#             rot = euler_from_quaternion(quat)            
#             return np.asarray(tf.TransformerROS.fromTranslationRotation(trans, rot))
#         else:
#             return trans, quat
#         # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#         #     print("wait for tf")
    
#     def _cal_dest(self, gripper_hand, hand_base):
#         return np.dot(gripper_hand , hand_base)
        





class MoveGroup:
    def __init__(self):
        print ("============ Starting Moveit setup==============")
        moveit_commander.roscpp_initialize(sys.argv)
<<<<<<< HEAD
        #rospy.init_node('move_group_python_interface',
                  #anonymous=True)
=======
        rospy.init_node('move_group_python_interface',
                  anonymous=True)
>>>>>>> d1f5881878b7ab599b4d756fa91fef8580f5544c
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander("manipulator")
        display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,
                                      queue_size=20)
        
<<<<<<< HEAD
        print ("============ Reference frame: %s" % self._group.get_planning_frame())
        print ("============ End effector: %s" % self._group.get_end_effector_link())
        print ("============ Robot Groups:")
        print (self._robot.get_group_names())
        print ("============ Printing robot state")
        print (self._robot.get_current_state())
=======
        print ("============ Waiting for RVIZ====================")
        rospy.sleep(10)
        print ("============ Reference frame: %s" % group.get_planning_frame())
        print ("============ End effector: %s" % group.get_end_effector_link())
        print ("============ Robot Groups:")
        print (robot.get_group_names())
        print ("============ Printing robot state")
        print (robot.get_current_state())
>>>>>>> d1f5881878b7ab599b4d756fa91fef8580f5544c
        print ("============")
        self.waypoints = []
        self.waypoints.append(self._group.get_current_pose().pose) # start with the current pose
        self.wpose = geometry_msgs.msg.Pose()
<<<<<<< HEAD
=======
        self.wpose.orientation.w = 1.0 # first orient gripper and move forward (+x)
        # self.client = actionlib.SimpleActionClient('icl_phri_ur5/follow_joint_trajectory', FollowJointTrajectoryAction)
        # print ("Waiting for server...")
        # self.client.wait_for_server()
>>>>>>> d1f5881878b7ab599b4d756fa91fef8580f5544c
        
    def append_waypoint(self, Q):
        self.wpose = copy.deepcopy(self._group.get_current_pose().pose)
        #self.wpose.position.z = self._group.get_current_pose().pose.position.z + diff
        self.wpose.position.x = Q[0]
        self.wpose.position.y = Q[1]
        self.wpose.position.z = Q[2]
        self.wpose.orientation.x = Q[3]
        self.wpose.orientation.y = Q[4]
        self.wpose.orientation.z = Q[5]
        self.wpose.orientation.w = Q[6]
        self.waypoints.append(copy.deepcopy(self.wpose))
        print('waypount', self.waypoints)
        (cartesian_plan, fraction) = self._group.compute_cartesian_path(
                                     self.waypoints,   # waypoints to follow
                                     0.005,        # eef_step, 1cm
                                     0.0)         # jump_threshold, disabling
<<<<<<< HEAD
        rospy.sleep(1)
        self._group.execute(cartesian_plan)
        self.waypoints.pop(0)

    def move(self, Q):
        self.append_waypoint(Q)
        
=======
        print ('==========wait rviz to display========')
        rospy.sleep(5)
        self._group.execute(cartesian_plan)
        self.waypoints.pop(0)


    def move(self, Q):
        self.append_waypoint(Q)
        
        # g = FollowJointTrajectoryGoal()
        # g.trajectory = JointTrajectory()
        # g.trajectory.joint_names = JOINT_NAMES
        # move_time = 10.0
        # try:
        #     joint_states = rospy.wait_for_message("icl_phri_ur5/joint_states", JointState)
        #     joints_pos = joint_states.position
        #     #joints_pos = self._group.get_current_joint_values()
        #     g.trajectory.points = []
        #     time_from_start = 0.0
        #     g.trajectory.points.append(JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(time_from_start)))
        #     for q in qs:
        #         time_from_start = time_from_start + move_time
        #         g.trajectory.points.append(JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(time_from_start)))
        #     self.client.send_goal(g)
        #     return self.client.wait_for_result()
        # except KeyboardInterrupt:
        #     self.client.cancel_goal()
        #     raise
        # except:
        #     raise
>>>>>>> d1f5881878b7ab599b4d756fa91fef8580f5544c
        
    def get_pose(self):
        return self._group.get_current_pose().pose

    def shift_pose_target(self, axis, value):
        axis_dict = {'x': 0, 'y': 1, 'z': 2, 'r': 3, 'p': 4, 'y': 5}
        print('ee: %s' % self._group.get_end_effector_link())
        self._group.shift_pose_target(axis_dict[axis], value, self._group.get_end_effector_link())
        #self.pose_target = self.get_pose()

class Handover:
    def __init__(self):
        self._mg = MoveGroup()
        self._gripper_ac = RobotiqActionClient('icl_phri_gripper/gripper_controller')
        self._gripper_ac.wait_for_server()
        print('============Init gripper============')
        self._gripper_ac.initiate()
        self.listener = tf.TransformListener()
        self.fetch = False

#-----------------get two frame transformation------------------#
    def _get_transform(self, target_frame, source_frame):
        rate = rospy.Rate(10.0)
        rate.sleep()        
        try:
<<<<<<< HEAD
            (trans, rot) = self.listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
=======
            (trans, rot) = self.listener.lookupTransform(frame1, frame2, rospy.Time(0))
>>>>>>> d1f5881878b7ab599b4d756fa91fef8580f5544c
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("wait for tf")

   
    def _list_add(self, ls1, ls2):
        return [x + y for x, y in zip(ls1, ls2)]


<<<<<<< HEAD
    def _get_handover_pos(self):
        (trans1, rot1) = self._get_transform('/right_hand_1', '/base_link')
        offset = ([0, 0, 0.4],
                            [0, 0, 0, 0])
        rot_new = tf.transformations.quaternion_multiply(rot1, offset[1])
        #tf.transformations.quaternion_from_euler()
        trans_new = self._list_add(trans1, offset[0])
        # Q = trans_new + list(rot_new)
=======
    def get_handover_pos(self):
        (trans1, rot1) = self.get_transform('/right_hand_1', '/base_link')
        (trans2, rot2) = ([1.2472304749429421, -0.2435223285317073, 0.25414502289255403],
                            [-0.9961315207647974, 0.0028692349443506515, -0.08656085053647994, 0.014865395522738467])
        rot_new = tf.transformations.quaternion_multiply(rot2, rot1)
        #tf.transformations.quaternion_from_euler()
        trans_new = self._list_add(trans1, trans2)
>>>>>>> d1f5881878b7ab599b4d756fa91fef8580f5544c
        Q = trans_new + list(rot_new)
        return Q
        

    def frame_action(self, trans):
<<<<<<< HEAD
        if abs(trans[0]) > 0.4:
            rospy.sleep(1)
            Q_new = self._get_handover_pos()
            print('Q_new', Q_new)
            self._mg.move(Q_new)
            # self.fetch = True
        # if abs(trans[0]) < 0.4 and self.fetch:
        #     print(self._mg.move([Q_wait]))
        #     self.fetch = False

=======
        if abs(trans[0]) > 1 and not self.fetch
            rospy.sleep(5)
            Q_new = self.get_handover_pos()
            print('Q_new', Q_new)
            print(self._mg.move([Q_new]))
            self.fetch = True
        if abs(trans[0]) < 0.4 and self.fetch:
            print(self._mg.move([Q_wait]))
            self.fetch = False
        
>>>>>>> d1f5881878b7ab599b4d756fa91fef8580f5544c
if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    h = Handover()
    while not rospy.is_shutdown():
        try:
            rate = rospy.Rate(10.0)
            rate.sleep()
<<<<<<< HEAD
            trans = h._get_transform('/right_hand_1', '/right_shoulder_1')[0]
            # print("hand_gripper_tf_trans:{}\nhand_gripper_tf_quat:{}".format(
            #     h._get_transform('/ee_link', '/right_hand_1')[0],
            #     h._get_transform('/ee_link', '/right_hand_1')[1]
            # ))
=======
            trans = h.get_transform('/left_hand_1', '/right_hand_1')[0]
            print("hand_gripper_tf_trans:{}\nhand_gripper_tf_quat:{}".format(
                h.get_transform('/ee_link', '/right_hand_1')[0],
                h.get_transform('/ee_link', '/right_hand_1')[1]
            ))
>>>>>>> d1f5881878b7ab599b4d756fa91fef8580f5544c
            h.frame_action(trans)
        except:
            print('no action')