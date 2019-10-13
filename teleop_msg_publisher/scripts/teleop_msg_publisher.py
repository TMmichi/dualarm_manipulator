#! /usr/bin/env python

import roslib; roslib.load_manifest('kinova_demo')
import rospy
import sys
import math
import actionlib
import kinova_msgs.msg
import argparse
from rosgraph_msgs.msg import Clock

finger_maxTurn = 6800
finger_maxDist = 18.9/2/1000

class teleopPub:
    def __init__(self,prefix,test):
        rospy.init_node('teleop_msg_publisher',anonymous=True)
        self.prefix = prefix
        #self.teleop_sub = rospy.Subscriber("/teleop_msgs",data_class=,callback=self.subsCB)
        self.test_sub = rospy.Subscriber("/clock",Clock,self.teleopTest)
        self.joint_action_address = '/'+self.prefix+'_driver/joints_action/joint_angles'
        self.joint_client = actionlib.SimpleActionClient(self.joint_action_address,
                                          kinova_msgs.msg.ArmJointAnglesAction)
        self.joint_client.wait_for_server()
        self.finger_action_address = '/'+self.prefix+'_driver/fingers_action/finger_positions'
        self.finger_client = actionlib.SimpleActionClient(self.finger_action_address,
                                          kinova_msgs.msg.SetFingersPositionAction)
        self.finger_client.wait_for_server()
        #self.teleop_joint_pub = rospy.Publisher("/"+self.prefix+"/effort_joint_trajectory_controller/command",JointTrajectory,queue_size=1)
        #self.teleop_finger_pub = rospy.Publisher("/"+self.prefix+"/effort_finger_trajectory_controller/command",JointTrajectory,queue_size=1)

    def spin(self,rate):
        self.rate = rospy.Rate(rate)
        rospy.spin()
    

    def subsCB(self, msg):
        msg = [] #should be redefined
        self.teleopPub(msg)

    
    def teleopPub(self,msg):
        joint_goal = kinova_msgs.msg.ArmJointAnglesGoal()
        finger_goal = kinova_msgs.msg.SetFingersPositionGoal()
        joint_goal.angles.joint1 = float(msg[0]) * 180 / 3.141592
        joint_goal.angles.joint2 = float(msg[1]) * 180 / 3.141592
        joint_goal.angles.joint3 = float(msg[2]) * 180 / 3.141592
        joint_goal.angles.joint4 = float(msg[3]) * 180 / 3.141592
        joint_goal.angles.joint5 = float(msg[4]) * 180 / 3.141592
        joint_goal.angles.joint6 = float(msg[5]) * 180 / 3.141592
        joint_goal.angles.joint7 = float(0)
        finger_goal.fingers.finger1 = float(msg[6])/100 * finger_maxTurn
        finger_goal.fingers.finger2 = float(msg[7])/100 * finger_maxTurn
        finger_goal.fingers.finger3 = float(msg[8])/100 * finger_maxTurn
        self.finger_client.send_goal(finger_goal)
        self.joint_client.send_goal(joint_goal)
        if self.joint_client.wait_for_result(rospy.Duration(2)) and self.finger_client.wait_for_result(rospy.Duration(2)):
            return self.joint_client.get_result(), self.finger_client.get_result()
        else:
            print('        the action timed-out')
            self.joint_client.cancel_all_goals()
            self.finger_client.cancel_all_goals()
            return [0,0]


    def teleopTest(self,msg):
        while True:
            raw_msg = raw_input("Joint States: 1 2 3 4 5 6 grip1 grip2 grip3>> ")
            if raw_msg == "1":
                msg = ["3", "2", "1", "2", "1.4", "1.32", "50", "50", "50", "0", "0", "0"]
            elif raw_msg == "2":
                msg = ["3.5", "2.1", "1.2", "3", "2", "1.32", "100", "100", "100", "0", "0", "0"]
            elif raw_msg == "3":
                msg = ["4.0", "2.3", "1", "2", "1.4", "1.32", "50", "50", "50", "0", "0", "0"]
            elif raw_msg == "4":
                msg = ["4.0", "2.3", "1", "2", "1.4", "1.32", "50", "50", "50", "0", "0", "0"]
            elif raw_msg == "5":
                msg = ["4.0", "2.3", "1", "2", "1.4", "1.32", "50", "50", "50", "0", "0", "0"]
            elif raw_msg == "6":
                msg = ["4.0", "2.3", "1", "2", "1.4", "1.32", "50", "50", "50", "0", "0", "0"]
            elif raw_msg == "7":
                msg = ["4.0", "2.3", "1", "2", "1.4", "1.32", "50", "50", "50", "0", "0", "0"]
            else:
                msg = raw_msg.split(" ")
            result = self.teleopPub(msg)
            print(result[0])
            print(result[1])


if __name__=='__main__':
    try:
        prefix = rospy.get_param("prefix", default="j2n6s300")
        test = rospy.get_param("test", default="ERROR")
        print(test)
        rospy.sleep(1)

        tp = teleopPub(prefix,test)
        tp.spin(5)
    except rospy.ROSInterruptException:
        rospy.loginfo("note terminated")

        
