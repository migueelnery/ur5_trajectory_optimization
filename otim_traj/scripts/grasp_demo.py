#! /usr/bin/env python

import actionlib
import sys
import copy
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown, MoveGroupCommander
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_commander
import moveit_msgs.msg
import time 
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
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

#GRIPPER
close='close'
open = 'open'
robotiq_joint_name = rospy.get_param("/robotiq_joint_name")
# Action clients
client_gripper = actionlib.SimpleActionClient('gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
print("Waiting for server (gripper_controller)...")
client_gripper.wait_for_server()
print("Connected to server (gripper_controller)")


# print state
# print arm_group.get_current_pose.pose

# rospy.sleep(1)

# arm_group.set_start_state_to_current_state
# print arm_group.get_current_pose().pose
# rospy.sleep(1)
arm_group.get_current_pose
# arm_group.get_current_state
# arm_group.get_current_joint_values

def add_table_colision():
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.
    p.pose.position.y = 0.3
    p.pose.position.z = 0.
    rospy.sleep(2)
    scene.add_box("table", p, (1.0, 1.5, 0.01))

def go_home():
    home_target = arm_group.set_named_target("home")
    plan = arm_group.plan(home_target)
    traj_home = plan[1]
    arm_group.execute(traj_home, wait = True)

def gripper_send_position_goal(action):
    """Send position goal to the gripper"""

    duration = 6
    velocity = 0.08
    if action == 'open':
        position = 0.1
        grasp_status = False
    elif action == 'close':
        position = 0.7
        grasp_status = True

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = robotiq_joint_name
    
    goal.trajectory.points.append(JointTrajectoryPoint(positions=[position]*6,
                                                        velocities=[velocity]*6,
                                                        accelerations=[0.0]*6,
                                                        time_from_start=rospy.Duration(duration)))
    rospy.set_param('/webot_grasp_status', grasp_status)
    client_gripper.send_goal(goal)
    client_gripper.wait_for_result()

def move_to_pose(x, y, z, R, P, Y):
    # arm_group.set_start_state_to_current_state()
    q = quaternion_from_euler(R, P, Y)
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = q[0] 
    pose_target.orientation.y = q[1]
    pose_target.orientation.z = q[2]
    pose_target.orientation.w = q[3]
    pose_target.position.z = z 
    pose_target.position.y = y 
    pose_target.position.x = x  
    arm_group.set_pose_target(pose_target)

    # Now, we call the planner to compute the plan and execute it.
    i = 0
    for i in range(5):
        plan = arm_group.plan(pose_target)
        if plan[0]!= True:
            i = i+1
        else:
            traj = plan[1]
            arm_group.execute(traj, wait = True)
            i=0
            break

    arm_group.stop()
    arm_group.clear_pose_targets()
  
add_table_colision()
go_home()
gripper_send_position_goal(open)
# rospy.sleep(1.0)

#Get planner name
planner = rospy.get_param("/move_group/default_planning_pipeline")

if planner == 'chomp':
    print("still working")
else: 
    ##Put the arm in the 1s grasp position
    move_to_pose(0.0, 0.70, 0.50, -1.571, 0, 1.571)
    

gripper_send_position_goal(close)
    # rospy.sleep(1)

    #GO FRONT

    # state = RobotState()
    # arm_group.set_start_state(state)
    # pose_target.position.x -= 0.18
    # arm_group.set_pose_target(pose_target)
    # plan1 = arm_group.plan(pose_target)

    # while plan1[0] != True:
    #     state = RobotState()
    #     arm_group.set_start_state(state)
    #     plan1 = arm_group.plan(pose_target)

    # if plan1[0]:
    #     traj = plan1[1]
    #     arm_group.execute(traj, wait = True)

    # arm_group.go()
    # arm_group.stop()
    # arm_group.clear_pose_targets()

    # # waypoints = []
    # # scale = 1.0
    # # wpose = arm_group.get_current_pose().pose
    # # wpose.position.x -= scale*0.1
    # # waypoints.append(copy.deepcopy(wpose))

    # # (plan1, fraction) = arm_group.compute_cartesian_path(
    # #                                    waypoints,   # waypoints to follow
    # #                                    0.02,        # eef_step
    # #                                    0.0, True)         # jump_threshold

    # # arm_group.execute(plan1, wait=True)

    # # rospy.sleep(1)

    # #CLOSE GRIPPER
    # state = RobotState()
    # arm_group.set_start_state(state)
    # # joint_goal = hand_group.get_current_joint_values()
    # joint_goal1 = [0.48, -0.48, 0.48, 0.48, -0.48, 0.48]  #0.49
    # hand_group.set_joint_value_target(joint_goal1)
    # plan_gripper1 = hand_group.plan()
    # hand_group.go()

    # # while plan_gripper1[0]!= True:
    # #     plan_gripper1 = hand_group.plan()


    # # if plan_gripper1[0]:
    # #     traj = plan_gripper[1]
    # #     hand_group.execute(traj, wait = True)

    # joint_ver = hand_group.get_current_joint_values()
    # if (joint_ver[0]-0.47) > 0.1:
    #     print joint_ver

    # ##GO UP
    # state = RobotState()
    # arm_group.set_start_state(state)
    # pose_target.position.z += 0.02
    # arm_group.set_pose_target(pose_target)
    # plan2 = arm_group.plan(pose_target)

    # while plan2[0] != True:
    #     plan2 = arm_group.plan(pose_target)

    # if plan2[0]:
    #     traj = plan2[1]
    #     arm_group.execute(traj, wait = True)

    # arm_group.go()
    # arm_group.stop()
    # arm_group.clear_pose_targets()


    # #GO BACK

    # state = RobotState()
    # arm_group.set_start_state(state)
    # pose_target.position.x += 0.18
    # arm_group.set_pose_target(pose_target)
    # plan3 = arm_group.plan(pose_target)

    # while plan3[0] != True:
    #     plan3 = arm_group.plan(pose_target)

    # if plan3[0]:
    #     traj = plan3[1]
    #     arm_group.execute(traj, wait = True)
        
    # arm_group.go()
    # arm_group.stop()
    # arm_group.clear_pose_targets()

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