#!/usr/bin/env python
import time
import rospy

from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input

class RFinger:
    def __init__(self):
            self.data=Robotiq2FGripper_robot_output()
            self.status=Robotiq2FGripper_robot_input()
            self.speed=125
            self.force=150
            self.is_subbed=False
            self.pub = rospy.Publisher('/Robotiq2FGripperRobotOutput', Robotiq2FGripper_robot_output, queue_size=10)

    def set_state(self,now):
        self.is_subbed=True
        self.status.gACT=now.gACT
        self.status.gGTO=now.gGTO
        self.status.gSTA=now.gSTA
        self.status.gOBJ=now.gOBJ
        self.status.gFLT=now.gFLT
        self.status.gPR=now.gPR
        self.status.gPO=now.gPO
        self.status.gCU=now.gCU

    def init(self):
        if self.status.gACT == 0:
            self.data.rACT=1
            self.data.rGTO=1
            self.data.rATR=0
            self.data.rPR=0
            self.data.rSP=255
            self.data.rFR=150
            self.speed=255
            self.force=150
            self.pub.publish(self.data)
            rospy.loginfo("Gripper Initialized")
            rospy.sleep(3)

        else:
            
            rospy.loginfo("Gripper already on")

    def reset(self):
        if self.status.gACT == 1:
            self.data.rACT=0
            self.data.rGTO=0
            self.data.rATR=0
            self.data.rPR=0
            self.data.rSP=0
            self.data.rFR=0
            self.speed=0
            self.force=0
            self.pub.publish(self.data)
            rospy.loginfo("Gripper Restarted")
            rospy.sleep(0.5)

        else:
            
            rospy.loginfo("Gripper already off")

    def close(self):
        if self.status.gACT == 1:
            self.data.rACT=1
            self.data.rGTO=1
            self.data.rATR=0
            self.data.rPR=255
            self.data.rSP=self.speed
            self.data.rFR=self.force
            self.pub.publish(self.data)
            rospy.loginfo("Gripper closed")
            rospy.sleep(3)

        else:
            
            rospy.loginfo("Initialize the gripper")

    def open(self):
        if self.status.gACT == 1:
            self.data.rACT=1
            self.data.rGTO=1
            self.data.rATR=0
            self.data.rPR=0
            self.data.rSP=self.speed
            self.data.rFR=self.force
            self.pub.publish(self.data)
            rospy.loginfo("Gripper Open")
            rospy.sleep(3)

        else:
            
            rospy.loginfo("Initialize the gripper")

    def set_position(self,pos):
        if self.status.gACT == 1:
            self.data.rACT=1
            self.data.rGTO=1
            self.data.rATR=0
            self.data.rPR=pos
            self.data.rSP=self.speed
            self.data.rFR=self.force
            self.pub.publish(self.data)
            rospy.loginfo("Gripper In Position")

        else:
            
            rospy.loginfo("Initialize the gripper")
        
