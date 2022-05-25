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
import os
from moveit_commander.conversions import pose_to_list


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

    def create_file(self, filename):
        current_path = os.path.dirname(os.path.abspath(__file__))
        fullpath = current_path + "/tests"
        filepath = os.path.join(fullpath, filename)
        if not os.path.exists(fullpath):
            os.makedirs(fullpath)
        self._file = open(filepath, "a")


    def add_table_collision(self):
        self.scene.remove_world_object("table")
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 0.
        p.pose.position.y = 0.4
        p.pose.position.z = 0.
        rospy.sleep(1)
        self.scene.add_box("table", p, (1.0, 2.0, 0.01))

    def add_cylinder_collision(self):
        self.scene.remove_world_object("cylinder")
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 0.2
        p.pose.position.y = 0.66
        p.pose.position.z = 0.125
        rospy.sleep(1)
        self.scene.add_cylinder("cylinder", p, 0.25, 0.08)

    def add_box_collision(self, srt, x, y, z, px, py):
        id = srt
        self.scene.remove_world_object(id)
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = px
        p.pose.position.y = py
        p.pose.position.z = (z/2)
        rospy.sleep(1)
        self.scene.add_box(id, p, (x, y, z))

    def add_printer_collision(self):
        #not used, actually using camera data in perception to get printer collision
        # self.scene.remove_world_object("printer")
        # self.scene.remove_world_object("printer_glass")
        # p1 = PoseStamped()
        # p2 = PoseStamped()
        # p1.header.frame_id = self.robot.get_planning_frame()
        # p2.header.frame_id = self.robot.get_planning_frame()
        # p1.pose.position.x = 0.
        # p1.pose.position.y = 1.17
        # p1.pose.position.z = 0.
        # p2.pose.orientation.x = -0.9269149
        # p2.pose.orientation.y = 0.001037
        # p2.pose.orientation.z = -0.0004393
        # p2.pose.orientation.w = 0.3752699
        # p2.pose.position.x = 0
        # p2.pose.position.y = 1.
        # p2.pose.position.z = 0.325
        # rospy.sleep(1)
        # self.scene.add_mesh("printer", p1, '/home/miguel/Downloads/printer1.stl', (1, 1, 1))
        # self.scene.add_mesh("printer_glass", p2, '/home/miguel/Downloads/printer.stl', (1, 1, 1))
        self.add_box_collision("printer1", 0.50, 0.35, 0.15, 0.0, 1.17)
        self.add_box_collision("printer2", 0.50, 0.05, 0.50, 0.0, 1.27)

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
                    
    def go_to(self, srt):
        name = "go_to"
        named_target = srt
        self.arm_group.set_named_target(named_target)
        i = 0
        j = 0
        while i<5:
            plan = self.arm_group.plan()
            if plan[0]!= True:
                i = i+1
            else:
                traj = plan[1]
                planning_time = plan[2]
                self._file.write("SUCCESS \n" + name + " " + named_target + ": " +  str(planning_time) + "\n") 
                self._file.write("Trajectory Duration (s): " + str(float(format(traj.joint_trajectory.points[-1].time_from_start))/1e9) + "\n")
                self.arm_group.execute(traj, wait = True)
                i=5
        if plan[0]!=True:
            self._file.write(name + " " + named_target + ": " +  "PLANNING ERROR "+ "\n") 

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
        name = "move_to_pose"
        previous_pose = pose_to_list(self.arm_group.get_current_pose().pose)
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
        pose_target_ = pose_to_list(pose_target)
        

        # Now, we call the planner to compute the plan and execute it.
        i = 0
        while i<5:
            plan = self.arm_group.plan(pose_target)
            if plan[0]!= True:
                i = i+1
            else:
                traj = plan[1]
                planning_time = plan[2]
                self._file.write("SUCCESS \n" + name + " from " + str(previous_pose) + " to " + str(pose_target_) + ": " +  str(planning_time) + "\n") 
                self._file.write("Trajectory Duration (s): " + str(float(format(traj.joint_trajectory.points[-1].time_from_start))/1e9) + "\n") 
                self.arm_group.execute(traj, wait = True)
                i=5
        if plan[0]!=True:
            self._file.write(name + " from " + str(previous_pose) + " to " + str(pose_target_) + ": " +  "PLANNING ERROR" + "\n")     
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        rospy.sleep(1)

    def move_to_joint(self, j1, j2, j3, j4, j5, j6):
        name = "move_to_joint"
        joint_goal = self.arm_group.get_current_joint_values()
        current_joints = str(joint_goal)
        joint_goal[0] = j1
        joint_goal[1] = j2
        joint_goal[2] = j3
        joint_goal[3] = j4
        joint_goal[4] = j5
        joint_goal[5] = j6
        self.arm_group.set_joint_value_target(joint_goal)
        # Now, we call the planner to compute the plan and execute it.
        i = 0
        while i<5:
            plan = self.arm_group.plan()
            if plan[0]!= True:
                i = i+1
            else:
                traj = plan[1]
                planning_time = plan[2]
                self._file.write("SUCCESS \n" + name + " from " + str(current_joints) + " to " + str(joint_goal) + ": " +  str(planning_time) + "\n") 
                self._file.write("Trajectory Duration (s): " + str(float(format(traj.joint_trajectory.points[-1].time_from_start))/1e9) + "\n")
                self.arm_group.execute(traj, wait = True)
                i=5
        if plan[0]!=True:
            self._file.write(name + " from " + str(current_joints) + " to " + str(joint_goal) + ": " +  "PLANNING ERROR" + "\n") 
        rospy.sleep(1)
    def get_planner(self):
        planner = rospy.get_param("/move_group/default_planning_pipeline")
        return planner
    def move(self, srt):
        direction = srt
        fraction = 0.0
        attempts = 0
        max_tries = 20
        waypoints = []
        scale = 1.0
        wpose = self.arm_group.get_current_pose().pose
        if direction == 'up':
            wpose.position.z += scale*0.03
        elif direction == 'down':
            wpose.position.z -= scale*0.03
        waypoints.append(copy.deepcopy(wpose))
        
        while fraction<1.0 and attempts<max_tries:
            (plan1, fraction) = self.arm_group.compute_cartesian_path(
                                                waypoints,   # waypoints to follow
                                                0.06,        # eef_step
                                                0.0,         # jump_threshold (5.0?)
                                                True)        # avoid collisions 
            attempts += 1
        
        if fraction == 1.0:
            self.arm_group.execute(plan1, wait=True)
            rospy.loginfo("Cartesian path executed")
        else:
            rospy.loginfo("Cartesian path failed")

        rospy.sleep(1)
    
    def add_collisions(self):
        self.add_cylinder_collision()
        self.add_table_collision()
        self.add_printer_collision()
        #2
        self.add_box_collision("box1", 0.13, 0.13, 0.13, -0.19, 0.68)
        #3
        self.add_box_collision("box2", 0.17, 0.17, 0.17, 0.2, 0.88)
        #4
        self.add_box_collision("box2", 0.17, 0.17, 0.34, 0.2, 0.88)

    def check_planner(self):
        planner = rospy.get_param("/move_group/default_planning_pipeline")
        if planner == 'chomp':
            # self.gripper_send_position_goal("open")
            # self.go_to("home")
            self.go_to("ready")
            # joint_goal = self.compute_ik(0.0, 0.75, 0.40, -1.571, 0, 1.571)
            # print("ik1 ok")
            # print(joint_goal)
            # self.move_to_joint(*joint_goal)
            # self.move_to_joint(-1.3781493580974669, -2.172610821908544, -1.4556796010146842, -1.0842527868702971, 1.5707183491332233, -1.3774213803247992) #sim
            # self.gripper_send_position_goal("close")
            # joint_goal = self.compute_ik(0.4, 0.70, 0.50, -1.571, 0, 1.571)
            # print("ik2 ok")
            # self.move_to_joint(*joint_goal)
            # self.move_to_joint(-1.922959853269357, -2.371830785716463, -0.6645558021092874, -1.6760031517492138, 1.5706005352878785, -1.92316833815096) #sim
            # joint_goal = self.compute_ik(0.4, 0.65, 0.34, -1.571, 0, 1.571)
            # print("ik3 ok")
            # self.move_to_joint(*joint_goal)
            # self.move_to_joint(-1.9459636479249252, -2.304801613701348, -1.1981926133427496, -1.2093890077668945, 1.5706105469043279, -1.9461771764045679) # nao
            # #pregrasp
            # self.move_to_joint(-1.50002926245, -2.70002937017, -1.00003825701, 0.499945316519, 1.56991733561, -1.56990505851) #sim
            # #grasp
            # self.move_to_joint(-1.49999877654, -2.84275634, -0.477780227084, -0.111686093081, 1.56994707603, -1.56997936866) #nao
            # self.gripper_send_position_goal("open")
            # self.move_to_joint(-1.50002926245, -2.70002937017, -1.00003825701, 0.499945316519, 1.56991733561, -1.56990505851) #nao
            # self.go_to("home") #nao
            # self.move_to_joint(0, 0, 0, 0, 0, 0) #nao
            # # #object on printer
            # self.go_to("home") #nao
            #pregrasp
            # self.move_to_joint(-1.50002926245, -2.70002937017, -1.00003825701, 0.499945316519, 1.56991733561, -1.56990505851) #sim
            # # #grasp
            # self.move_to_joint(-1.49999877654, -2.84275634, -0.477780227084, -0.111686093081, 1.56994707603, -1.56997936866) #nao
            # self.move("down")
            # self.gripper_send_position_goal("close")
            # self.move("up")
            # self.move_to_joint(-1.50002926245, -2.70002937017, -1.00003825701, 0.499945316519, 1.56991733561, -1.56990505851) #nao
            #place object
            # joint_goal = self.compute_ik(-0.1, 0.70, 0.34, -1.571, 0, 1.571)
            # self.move_to_joint(*joint_goal)
            self.move_to_joint(-1.2382410195362379, -2.1770815903968836, -1.4195257850455416, -1.1159097620131329, 1.5706051297246038, -1.238442495010653) #sim
            # joint_goal = self.compute_ik(0.4, 0.65, 0.34, -1.571, 0, 1.571)
            # self.move_to_joint(*joint_goal)
            self.move_to_joint(-1.9459642312729857, -2.304787234076681, -1.1982143179740632, -1.2093797317569202, 1.5706053241015951, -1.9461775309413003) #sim
            # self.gripper_send_position_goal("open")
            # # #return home
            # self.go_to("home")
            # self.move_to_joint(0, 0, 0, 0, 0, 0)
        else: 
            #Put the arm in the 1s grasp position
            # # object on table
            self.go_to("home")
            self.gripper_send_position_goal("open")
            self.go_to("ready")
            self.move_to_pose(0.0, 0.70, 0.33, -1.571, 0, 1.571) #sim 
            self.gripper_send_position_goal("close")
            self.move_to_pose(0.4, 0.70, 0.50, -1.571, 0, 1.571) #sim 
            self.move_to_pose(0.4, 0.65, 0.34, -1.571, 0, 1.571)
            # #pregrasp
            self.move_to_joint(-1.50002926245, -2.70002937017, -1.00003825701, 0.499945316519, 1.56991733561, -1.56990505851) #sim 
            # #grasp
            self.move_to_joint(-1.49999877654, -2.84275634, -0.477780227084, -0.111686093081, 1.56994707603, -1.56997936866)
            self.gripper_send_position_goal("open")
            self.move_to_joint(-1.50002926245, -2.70002937017, -1.00003825701, 0.499945316519, 1.56991733561, -1.56990505851)
            # self.go_to("ready")
            # self.move_to_joint(0, 0, 0, 0, 0, 0)
            # #object on printer
            # self.go_to("home")
            # #pregrasp
            # self.move_to_joint(-1.50002926245, -2.70002937017, -1.00003825701, 0.499945316519, 1.56991733561, -1.56990505851) #sim
            # #grasp
            # self.move_to_joint(-1.49999877654, -2.84275634, -0.477780227084, -0.111686093081, 1.56994707603, -1.56997936866)
            # self.move("down")
            # self.gripper_send_position_goal("close")
            # self.move("up")
            # self.move_to_joint(-1.50002926245, -2.70002937017, -1.00003825701, 0.499945316519, 1.56991733561, -1.56990505851)
            #place object
            self.move_to_pose(-0.1, 0.75, 0.34, -1.571, 0, 1.571) #sim 
            self.move_to_pose(0.4, 0.65, 0.34, -1.571, 0, 1.571) #sim 
            # self.gripper_send_position_goal("open")
            #return home
            # self.go_to("home")
            # self.move_to_joint(0, 0, 0, 0, 0, 0)
def main():
    ur5_grasp = ur5_grasp_demo()
    timeStr = time.strftime('%m-%d-%Y_%H_%M_%S_%Z')
    planner = ur5_grasp.get_planner()
    filename = "{0}.{1}.{2}".format(planner,timeStr, "txt")
    ur5_grasp.create_file(filename)
    ur5_grasp.add_collisions()
    ur5_grasp.check_planner()      
    roscpp_shutdown()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		print("Program interrupted before completion")