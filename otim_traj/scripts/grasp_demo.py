#! /usr/bin/env python

import sys
import copy
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown, MoveGroupCommander
import moveit_commander
import moveit_msgs.msg
import time 
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from trac_ik_python.trac_ik import IK


roscpp_initialize(sys.argv)
rospy.init_node('grasp_ur5', anonymous=True)
robot = RobotCommander()
scene = PlanningSceneInterface()


arm_group = MoveGroupCommander("manipulator")

#Put the arm in the begin position


# print state
# print arm_group.get_current_pose.pose

# rospy.sleep(1)

# arm_group.set_start_state_to_current_state
# print arm_group.get_current_pose().pose
# rospy.sleep(1)
arm_group.get_current_pose
# arm_group.get_current_state
# arm_group.get_current_joint_values
# arm_group.set_named_target("begin")
# plan = arm_group.plan()
# arm_group.execute(plan, wait = True)



hand_group = moveit_commander.MoveGroupCommander("endeffector")
# #OPEN GRIPPER
# joint_goal = hand_group.get_current_joint_values()
joint_goal = [0.3, -0.3, 0.3, 0.3, -0.3, 0.3]
hand_group.set_joint_value_target(joint_goal)
plan_gripper = hand_group.plan()
hand_group.go()
# while plan_gripper[0]!= True:
#     plan_gripper = hand_group.plan()


# if plan_gripper[0]:
#     traj = plan_gripper[1]
#     hand_group.execute(traj, wait = True)



##Put the arm in the 1s grasp position
# arm_group.set_start_state_to_current_state()
# rospy.sleep(1)
# arm_group.set_goal_position_tolerance(0.001)

q = quaternion_from_euler(1.5701, 0.0, -1.5701)
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = q[0] 
pose_target.orientation.y = q[1]
pose_target.orientation.z = q[2]
pose_target.orientation.w = q[3]
pose_target.position.z = 0.10 #0.25
pose_target.position.y = 0.00 #0.00
pose_target.position.x = -0.45  #-0.45
arm_group.set_pose_target(pose_target)

# # # ## Now, we call the planner to compute the plan and execute it.

plan = arm_group.plan(pose_target)

while plan[0]!= True:
    plan = arm_group.plan(pose_target)


if plan[0]:
    traj = plan[1]
    arm_group.execute(traj, wait = True)


arm_group.stop()
arm_group.clear_pose_targets()

# rospy.sleep(1)

#GO FRONT

state = RobotState()
arm_group.set_start_state(state)
pose_target.position.x -= 0.18
arm_group.set_pose_target(pose_target)
plan1 = arm_group.plan(pose_target)

while plan1[0] != True:
    state = RobotState()
    arm_group.set_start_state(state)
    plan1 = arm_group.plan(pose_target)

if plan1[0]:
    traj = plan1[1]
    arm_group.execute(traj, wait = True)

arm_group.go()
arm_group.stop()
arm_group.clear_pose_targets()

# waypoints = []
# scale = 1.0
# wpose = arm_group.get_current_pose().pose
# wpose.position.x -= scale*0.1
# waypoints.append(copy.deepcopy(wpose))

# (plan1, fraction) = arm_group.compute_cartesian_path(
#                                    waypoints,   # waypoints to follow
#                                    0.02,        # eef_step
#                                    0.0, True)         # jump_threshold

# arm_group.execute(plan1, wait=True)

# rospy.sleep(1)

#CLOSE GRIPPER
state = RobotState()
arm_group.set_start_state(state)
# joint_goal = hand_group.get_current_joint_values()
joint_goal1 = [0.48, -0.48, 0.48, 0.48, -0.48, 0.48]  #0.49
hand_group.set_joint_value_target(joint_goal1)
plan_gripper1 = hand_group.plan()
hand_group.go()

# while plan_gripper1[0]!= True:
#     plan_gripper1 = hand_group.plan()


# if plan_gripper1[0]:
#     traj = plan_gripper[1]
#     hand_group.execute(traj, wait = True)

joint_ver = hand_group.get_current_joint_values()
if (joint_ver[0]-0.47) > 0.1:
    print joint_ver

##GO UP
state = RobotState()
arm_group.set_start_state(state)
pose_target.position.z += 0.02
arm_group.set_pose_target(pose_target)
plan2 = arm_group.plan(pose_target)

while plan2[0] != True:
    plan2 = arm_group.plan(pose_target)

if plan2[0]:
    traj = plan2[1]
    arm_group.execute(traj, wait = True)

arm_group.go()
arm_group.stop()
arm_group.clear_pose_targets()


#GO BACK

state = RobotState()
arm_group.set_start_state(state)
pose_target.position.x += 0.18
arm_group.set_pose_target(pose_target)
plan3 = arm_group.plan(pose_target)

while plan3[0] != True:
    plan3 = arm_group.plan(pose_target)

if plan3[0]:
    traj = plan3[1]
    arm_group.execute(traj, wait = True)
    
arm_group.go()
arm_group.stop()
arm_group.clear_pose_targets()

# waypoints = []
# wpose = arm_group.get_current_pose().pose

# wpose.position.z -= scale * 0.65
# waypoints.append(copy.deepcopy(wpose))

# # wpose.position.x += scale * 0.2
# # waypoints.append(copy.deepcopy(wpose))

# (plan, fraction) = arm_group.compute_cartesian_path(
#                                     waypoints,   # waypoints to follow
#                                     0.01,        # eef_step
#                                     0.0, True)         # jump_threshold

# arm_group.execute(plan, wait=True)



# pose_target.position.x = -0.58
# arm_group.set_pose_target(pose_target)
# plan1 = arm_group.go()


roscpp_shutdown()