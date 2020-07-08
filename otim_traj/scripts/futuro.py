#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.mgs
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class Grasp():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('grasp_ur5', anonymous=True)
        robot = moveit_commander.RobotCommander()

        arm_group = moveit_commander.MoveGroupCommander("manipulator")
        arm_group = self.arm_group
        #Put the arm in the begin position
        arm_group.set_named_target("begin")
        plan1 = arm_group.go()

        hand_group = moveit_commander.MoveGroupCommander("endeffector")
        hand_group = self.hand_group
        #Close the gripper

    def close_gripper(self):
        hand_group = self.hand_group
        joint_goal = hand_group.get_current_joint_values()
        joint_goal [2] = 0.66
        plan2 = joint_goal.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()