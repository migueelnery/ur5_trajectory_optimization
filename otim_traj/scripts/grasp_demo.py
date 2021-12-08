#! /usr/bin/env python

import actionlib
import sys
import copy
import rospy
from operator import itemgetter
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown, MoveGroupCommander
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_commander
import moveit_msgs.msg
import time 
from moveit_msgs.msg import RobotState, PositionIKRequest, Constraints, OrientationConstraint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK
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
        rospy.loginfo("Waiting for get_position_ik...")
        rospy.wait_for_service('compute_ik')
        self.get_position_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        rospy.loginfo("Service available get_position_ik...")
        # print arm_group.get_current_pose.pose
        # rospy.sleep(1)
        # arm_group.set_start_state_to_current_state
        # print arm_group.get_current_pose().pose
        # arm_group.get_current_pose
        # arm_group.get_current_state
        # arm_group.get_current_joint_values

    def add_table_collision(self):
        self.scene.remove_world_object("table")
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 0.
        p.pose.position.y = 0.4
        p.pose.position.z = 0.
        rospy.sleep(1)
        self.scene.add_box("table", p, (1.0, 2.0, 0.01))

    def add_printer_collision(self):
        #not used, actually using camera data in perception to get printer collision
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

    def compute_ik(self, x, y, z, R, P, Y):
        q = quaternion_from_euler(R, P, Y)
        constraints = Constraints()
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.link_name = self.arm_group.get_end_effector_link()
        orientation_constraint.orientation.x = q[0]
        orientation_constraint.orientation.y = q[1]
        orientation_constraint.orientation.z = q[2]
        orientation_constraint.orientation.w = q[3]
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 3.14
        orientation_constraint.weight = 1
        constraints.orientation_constraints.append(orientation_constraint) 
        p_ik = PoseStamped()
        p_ik.pose.orientation.x = q[0] 
        p_ik.pose.orientation.y = q[1]
        p_ik.pose.orientation.z = q[2]
        p_ik.pose.orientation.w = q[3]
        p_ik.header.frame_id = "base_link"
        p_ik.pose.position.x = x
        p_ik.pose.position.y = y
        p_ik.pose.position.z = z
        ik_request = PositionIKRequest()
        ik_request.group_name = self.arm_group.get_name()
        ik_request.pose_stamped = p_ik
        ik_request.timeout.secs= 0.005
        ik_request.avoid_collisions = True
        ik_request.constraints = constraints
        ik_request.robot_state = self.arm_group.get_current_state()
        self.resp = self.get_position_ik(ik_request)
        # if self.resp.error_code.val != 1:
        #     while self.resp.error_code.val != 1:
        #         self.resp = self.get_position_ik(ik_request)
        #         print("calculating ik")  
        while True:
            self.resp = self.get_position_ik(ik_request)
            print("calculating ik")
            if self.resp.error_code.val == 1:
                break
        self.resp = self.resp.solution.joint_state.position
        joint_goal = list(self.resp)
        joint_goal = joint_goal[:-6]
        return joint_goal
                    
    def go_home(self):
        home_target = self.arm_group.set_named_target("home")
        plan = self.arm_group.plan(home_target)
        traj_home = plan[1]
        self.arm_group.execute(traj_home, wait = True)

    def gripper_send_position_goal(self, srt):
        """Send position goal to the gripper"""
        action = srt
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

    def move_to_joint(self, j1, j2, j3, j4, j5, j6):
        joint_goal = self.arm_group.get_current_joint_values()
        joint_goal[0] = j1
        joint_goal[1] = j2
        joint_goal[2] = j3
        joint_goal[3] = j4
        joint_goal[4] = j5
        joint_goal[5] = j6
        self.arm_group.set_joint_value_target(joint_goal)
        # Now, we call the planner to compute the plan and execute it.
        i = 0
        for i in range(5):
            plan = self.arm_group.plan()
            if plan[0]!= True:
                i = i+1
            else:
                traj = plan[1]
                self.arm_group.execute(traj, wait = True)
                i=0
                break
        rospy.sleep(1)

    def move(self, srt):
        direction = srt
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

    def check_planner(self):
        planner = rospy.get_param("/move_group/default_planning_pipeline")
        if planner == 'chomp':
            # print("still working")
            joint_goal = self.compute_ik(0.0, 0.70, 0.33, -1.571, 0, 1.571)
            print("ik1 ok")
            self.move_to_joint(*joint_goal)
            self.gripper_send_position_goal("close")
            joint_goal = self.compute_ik(0.4, 0.70, 0.50, -1.571, 0, 1.571)
            print("ik2 ok")
            self.move_to_joint(*joint_goal)
            joint_goal = self.compute_ik(0.4, 0.70, 0.33, -1.571, 0, 1.571)
            print("ik3 ok")
            self.move_to_joint(*joint_goal)
            self.gripper_send_position_goal("open")
            self.move_to_joint(0, 0, 0, 0, 0, 0)
        else: 
            ##Put the arm in the 1s grasp position
            # self.move_to_pose(0.0, 0.70, 0.44, -1.571, 0, 1.571)
            self.move_to_pose(0.0, 0.70, 0.33, -1.571, 0, 1.571)
            # self.move(down)
            self.gripper_send_position_goal("close")
            self.move_to_pose(0.4, 0.70, 0.50, -1.571, 0, 1.571)
            self.move_to_pose(0.4, 0.70, 0.34, -1.571, 0, 1.571)
            self.move_to_joint(-1.50002926245, -2.70002937017, -1.00003825701, 0.499945316519, 1.56991733561, -1.56990505851)
            self.move_to_joint(-1.49999877654, -2.84275634, -0.477780227084, -0.111686093081, 1.56994707603, -1.56997936866)
            self.gripper_send_position_goal("open")
            self.go_home
            self.move_to_joint(0, 0, 0, 0, 0, 0)
            

def main():
    ur5_grasp = ur5_grasp_demo()
    ur5_grasp.add_table_collision()
    ur5_grasp.go_home() 
    ur5_grasp.gripper_send_position_goal("open")
    ur5_grasp.check_planner()      
    roscpp_shutdown()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		print("Program interrupted before completion")