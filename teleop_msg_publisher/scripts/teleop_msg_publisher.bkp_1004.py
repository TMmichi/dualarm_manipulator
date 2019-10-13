#! /usr/bin/env python

import argparse
import time
import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rosgraph_msgs.msg import Clock


class teleopPub:
    def __init__(self,prefix,test):
        rospy.init_node('teleop_msg_publisher',anonymous=True)
        self.prefix = prefix
        #self.teleop_sub = rospy.Subscriber("/teleop_msgs",data_class=,callback=self.subsCB)
        self.test_sub = rospy.Subscriber("/clock",Clock,self.teleopTest)
        self.teleop_joint_pub = rospy.Publisher("/"+self.prefix+"/effort_joint_trajectory_controller/command",JointTrajectory,queue_size=1)
        #self.teleop_finger_pub = rospy.Publisher("/"+self.prefix+"/effort_finger_trajectory_controller/command",JointTrajectory,queue_size=1)

    def spin(self,rate):
        self.rate = rospy.Rate(rate)
        rospy.spin()
    

    def subsCB(self, msg):
        msg = [] #should be redefined
        self.teleopPub(msg)

    
    def teleopPub(self,msg):
        jointCmd = JointTrajectory()
        point = JointTrajectoryPoint()
        jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
        point.time_from_start = rospy.Duration.from_sec(5.0)
        for i in range(0, 6):
            jointCmd.joint_names.append(self.prefix +'_joint_'+str(i+1))
            point.positions.append(float(msg[i]))
            point.velocities.append(0)
            point.accelerations.append(0)
            point.effort.append(0)
        '''
        for i in range(0,3):
            jointCmd.joint_names.append(self.prefix +'_joint_finger_'+str(i+1))
            point.positions.append(float(msg[6+i]))
            point.velocities.append(0)
            point.accelerations.append(0)
            point.effort.append(0)
        for i in range(0,3):
            jointCmd.joint_names.append(self.prefix +'_joint_finger_tip_'+str(i+1))
            point.positions.append(float(msg[9+i]))
            point.velocities.append(0)
            point.accelerations.append(0)
            point.effort.append(0)'''
        jointCmd.points.append(point)
        print(jointCmd)
        self.teleop_joint_pub.publish(jointCmd)
        count = 0
        while (count < 50):
            self.teleop_joint_pub.publish(jointCmd)
            count = count + 1
            self.rate.sleep()


    def teleopTest(self,msg):
        while True:
            raw_msg = raw_input("Joint States: 1 2 3 4 5 6 grip1 grip2 grip3>> ")
            if len(raw_msg) < 9:
                #msg = ["4.50", "2.92", "1", "3.2", "1.4", "1.32", "1", "1", "1", "0", "0", "0"]
                msg = ["4.50", "2.92", "1", "3.2", "1.4", "1.32"]
            else:
                msg = raw_msg.split(" ")
            self.teleopPub(msg)


if __name__=='__main__':
    try:
        prefix = rospy.get_param("prefix", default="j2n6s300")
        test = rospy.get_param("test", default="ERROR")
        tp = teleopPub(prefix,test)
        print(test)
        tp.spin(10)
    except rospy.ROSInterruptException:
        rospy.loginfo("note terminated")

        
