#!/usr/bin/env python

import rospy
import time
import subprocess

subprocess.call(
    "rosparam set use_sim_time true",
    shell=True
)

vrep_path = rospy.get_param("/vrep_jaco_bringup_simulator/vrep_path")
gui = rospy.get_param("/vrep_jaco_bringup_simulator/gui")
scene = rospy.get_param("/vrep_jaco_bringup_simulator/scene_file")
paused = rospy.get_param("/vrep_jaco_bringup_simulator/paused")
auto_quit = rospy.get_param("/vrep_jaco_bringup_simulator/auto_quit")

if gui:
    vrep_exec = vrep_path+"/vrep.sh "
    t_val = 5.0
else:
    vrep_exec = vrep_path+"/vrep.sh -h "
    t_val = 1.0
if not paused:
    vrep_exec = vrep_exec+"-s "
if auto_quit:
    vrep_exec = vrep_exec+"-q "
subprocess.call(
    vrep_exec+scene+" &",
    shell=True
)
time.sleep(t_val)      
