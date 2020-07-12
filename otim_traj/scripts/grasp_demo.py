#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('grasp_ur5', anonymous=True)
robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("manipulator")

#Put the arm in the begin position
arm_group.set_named_target("begin")
plan1 = arm_group.go()

hand_group = moveit_commander.MoveGroupCommander("endeffector")
#OPEN GRIPPER
joint_goal = hand_group.get_current_joint_values()
joint_goal [2] = 0.3
hand_group.go(joint_goal, wait=True)

##Put the arm in the 1s grasp position
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = 0.5
pose_target.orientation.y = -0.5
pose_target.orientation.z = -0.5
pose_target.orientation.w = 0.5
pose_target.position.x = -0.52
pose_target.position.y = 0.00
pose_target.position.z = 0.22
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()

waypoints = []
scale = 1.0
wpose = arm_group.get_current_pose().pose
wpose.position.x -= scale * 0.22  # First move x 
wpose.position.z -= 0
waypoints.append(copy.deepcopy(wpose))



# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
(plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.05,        # eef_step
                                   0.0)         # jump_threshold

arm_group.execute(plan, wait=True)

# #CLOSE GRIPPER
# joint_goal = hand_group.get_current_joint_values()
# joint_goal [2] = 0.4
# hand_group.go(joint_goal, wait=True)

##GO BACK

# waypoints = []
# wpose = arm_group.get_current_pose().pose

# wpose.position.z += scale * 0.01
# waypoints.append(copy.deepcopy(wpose))

# wpose.position.x += scale * 0.2
# waypoints.append(copy.deepcopy(wpose))

# (plan, fraction) = arm_group.compute_cartesian_path(
#                                    waypoints,   # waypoints to follow
#                                    0.05,        # eef_step
#                                    0.0)         # jump_threshold

# arm_group.execute(plan, wait=True)



# pose_target.position.x = -0.58
# arm_group.set_pose_target(pose_target)
# plan1 = arm_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()