#! /usr/bin/env python

import sys
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
#CLOSE GRIPPER
joint_goal = hand_group.get_current_joint_values()
joint_goal [2] = 0.5
hand_group.go(joint_goal, wait=True)

##Put the arm in the 1s grasp position
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = 0.5
pose_target.orientation.y = -0.5
pose_target.orientation.z = -0.5
pose_target.orientation.w = 0.5
pose_target.position.x = -0.52
pose_target.position.y = 0.01
pose_target.position.z = 0.25
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()

pose_target.position.x = -0.58
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()