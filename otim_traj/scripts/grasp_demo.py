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


class ur5_grasp_demo:
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node('grasp_ur5', anonymous=True)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander("manipulator")
        self.robotiq_joint_name = rospy.get_param("/robotiq_joint_name")
        # Action clients
        self.client_gripper = actionlib.SimpleActionClient('gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print("Waiting for server (gripper_controller)...")
        self.client_gripper.wait_for_server()
        print("Connected to server (gripper_controller)")
        # print state
        # print arm_group.get_current_pose.pose
        # rospy.sleep(1)
        # arm_group.set_start_state_to_current_state
        # print arm_group.get_current_pose().pose
        # rospy.sleep(1)
        # arm_group.get_current_pose
        # arm_group.get_current_state
        # arm_group.get_current_joint_values

    def add_table_colision(self):
        self.scene.remove_world_object("table")
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 0.
        p.pose.position.y = 0.4
        p.pose.position.z = 0.
        rospy.sleep(1)
        self.scene.add_box("table", p, (1.0, 2.0, 0.01))

    def add_printer_colision(self):
        self.scene.remove_world_object("printer")
        self.scene.remove_world_object("printer_glass")
        p1 = PoseStamped()
        p2 = PoseStamped()
        p1.header.frame_id = self.robot.get_planning_frame()
        p2.header.frame_id = self.robot.get_planning_frame()
        p1.pose.position.x = 0.
        p1.pose.position.y = 1.17
        p1.pose.position.z = 0.
        p2.pose.orientation.x = -0.9269149
        p2.pose.orientation.y = 0.001037
        p2.pose.orientation.z = -0.0004393
        p2.pose.orientation.w = 0.3752699
        p2.pose.position.x = 0
        p2.pose.position.y = 1.
        p2.pose.position.z = 0.325
        rospy.sleep(1)
        self.scene.add_mesh("printer", p1, '/home/miguel/Downloads/printer1.stl', (1, 1, 1))
        self.scene.add_mesh("printer_glass", p2, '/home/miguel/Downloads/printer.stl', (1, 1, 1))

    def go_home(self):
        home_target = self.arm_group.set_named_target("home")
        plan = self.arm_group.plan(home_target)
        traj_home = plan[1]
        self.arm_group.execute(traj_home, wait = True)

    def gripper_send_position_goal(self, action):
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
        goal.trajectory.joint_names = self.robotiq_joint_name
        
        goal.trajectory.points.append(JointTrajectoryPoint(positions=[position]*6,
                                                            velocities=[velocity]*6,
                                                            accelerations=[0.0]*6,
                                                            time_from_start=rospy.Duration(duration)))
        rospy.set_param('/webot_grasp_status', grasp_status)
        self.client_gripper.send_goal(goal)
        self.client_gripper.wait_for_result()

    def move_to_pose(self, x, y, z, R, P, Y):
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
        self.arm_group.set_pose_target(pose_target)

        # Now, we call the planner to compute the plan and execute it.
        i = 0
        for i in range(5):
            plan = self.arm_group.plan(pose_target)
            if plan[0]!= True:
                i = i+1
            else:
                traj = plan[1]
                self.arm_group.execute(traj, wait = True)
                i=0
                break

        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        rospy.sleep(1)

    def move(self, direction):
        waypoints = []
        scale = 1.0
        wpose = self.arm_group.get_current_pose().pose
        if direction == 'up':
            wpose.position.z += scale*0.1
        elif direction == 'down':
            wpose.position.z -= scale*0.1
        waypoints.append(copy.deepcopy(wpose))

        (plan1, fraction) = self.arm_group.compute_cartesian_path(
                                            waypoints,   # waypoints to follow
                                            0.05,        # eef_step
                                            100.0, True)         # jump_threshold

        self.arm_group.execute(plan1, wait=True)

        rospy.sleep(1)  
    
def main():
    ur5_grasp = ur5_grasp_demo() 
    ur5_grasp.add_table_colision()
    # ur5_grasp.add_printer_colision()
    ur5_grasp.go_home()
    open = 'open'
    close = 'close'
    ur5_grasp.gripper_send_position_goal(open)
    rospy.sleep(1)      
    #Get planner name
    planner = rospy.get_param("/move_group/default_planning_pipeline")
    if planner == 'chomp':
        print("still working")
    else: 
        ##Put the arm in the 1s grasp position
        ur5_grasp.move_to_pose(0.0, 0.70, 0.44, -1.571, 0, 1.571)
        ur5_grasp.move_to_pose(0.0, 0.70, 0.34, -1.571, 0, 1.571)
        #ur5_grasp.move(down)
        ur5_grasp.gripper_send_position_goal(close)
        ur5_grasp.move_to_pose(0.0, 0.70, 0.50, -1.571, 0, 1.571)
        # move(up)
        ur5_grasp.move_to_pose(0.4, 0.70, 0.50, -1.571, 0, 1.571)
        # move_to_pose(0.4, 0.70, 0.34, -1.571, 0, 1.571)
        #move(down)
        ur5_grasp.gripper_send_position_goal(open)
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

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		print("Program interrupted before completion")