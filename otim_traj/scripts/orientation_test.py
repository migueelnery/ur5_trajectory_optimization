#! /usr/bin/env python

import sys
import copy
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown, MoveGroupCommander
import moveit_commander
import moveit_msgs.msg
import time 
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint
from sensor_msgs.msg import JointState
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from trac_ik_python.trac_ik import IK



def main():
    roscpp_initialize(sys.argv)
    rospy.init_node('grasp_ur5', anonymous=True)
    robot = RobotCommander()
    scene = PlanningSceneInterface()


    arm_group = MoveGroupCommander("manipulator")


    q = quaternion_from_euler(1.5701, 0.0, -1.5701)
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = q[0] 
    pose_target.orientation.y = q[1]
    pose_target.orientation.z = q[2]
    pose_target.orientation.w = q[3]
    pose_target.position.z = 0.23 #0.23
    pose_target.position.y = 0.11 #0.11
    pose_target.position.x = -0.45  #-0.45
    arm_group.set_pose_target(pose_target)

    plan = arm_group.plan(pose_target)

    while plan[0]!= True:
        plan = arm_group.plan(pose_target)


    if plan[0]:
        traj = plan[1]
        arm_group.execute(traj, wait = True)


    arm_group.stop()
    arm_group.clear_pose_targets()


    # rospy.sleep(1)

    pose = arm_group.get_current_pose()
    constraint = Constraints()
    constraint.name = "restricao"
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header = pose.header
    orientation_constraint.link_name = arm_group.get_end_effector_link()
    orientation_constraint.orientation = pose.pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = 0.5
    orientation_constraint.absolute_y_axis_tolerance = 3.14
    orientation_constraint.absolute_z_axis_tolerance = 3.14
    orientation_constraint.weight = 1
    constraint.orientation_constraints.append(orientation_constraint)
    arm_group.set_path_constraints(constraint)


    state = RobotState()
    arm_group.set_start_state(state)
    pose_target.position.z = 0.77 # 0.77
    # pose_target.position.y = -0.11 # -0.11
    # pose_target.position.x = 0.31  # 0.31
    arm_group.set_pose_target(pose_target)

    plan = arm_group.plan(pose_target)

    while plan[0]!= True:
        plan = arm_group.plan(pose_target)


    if plan[0]:
        traj = plan[1]
        arm_group.execute(traj, wait = True)


    arm_group.stop()
    arm_group.clear_pose_targets()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass