#!/usr/bin/env python3
"""
//TODO
source devel/setup.bash

TASK: Sends goal pose every 1ms to the robot drive OR sends a list of all poses, sampled in 1ms steps to the driver (this is the better way to do it, because everything is precalculated)
INPUT: goal pose, current pose
OUTPUT: goal pose in form of angles
"""

from numpy import cos
from robot_kinematics import robot_kinematics
import rospy
import sys
from sensor_msgs.msg import JointState
import numpy as np
import csv
from std_msgs.msg import Float64MultiArray
import os
from trajectory_planer import trajectory_planer

tmp = os.path.dirname(__file__)

class motion_manager:
    def __init__(self, command_topic):
        self.command_topic = command_topic
        self.trajectory_planer = trajectory_planer()
        self.counter = 0
        self.kinematic = robot_kinematics()
        self.max_dist_between_stutzpunkten = 0.01

    def send_commandlist(self):
        # Create the message and send it to the advertised topic
        path = self.get_list()
        angle = path[self.counter]
        msg = Float64MultiArray()
        msg.data = angle
        self.publisher.publish(msg)
        self.counter += 1
        
        #/TODO Error handling nessesary! If list index higher than last length, resend last entrie (hold state)
        #/TODO exit get_list only once
        #/TODO robot join-limit handler
    
    def send_command(self):
        # send single command
        pass

    def get_position_error(self):
        # calcualte position error. If to high, stop execution and wati for reaching target
        #/TODO compare actual state with target state, if error to high do not send new angles, instead send last target angles
        pass

    


    
    def get_flag_list(self):
        # if / else flag, replace existing list
        pass

    def set_flag_newlist(self):
        #set flag if new list avaible
        pass

    def plan_list(self):
        # initialize list planing
        # TODO write listplanner
        path = self.trajectory_planer.create_path()
        pass
        


def main(argv):
    rospy.init_node("motion_executor")
    command_topic = rospy.get_param("~command_topic", "/default_command_topic")

    #rospy.logwarn("Using topic " + str(command_topic) + " and distance " + str(move_dist))
    rospy.logwarn("Using topic " + str(command_topic))
    motion_executor = motion_manager(command_topic)

    # Loop infinitely with a fixed frequency of 1000 hz
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        motion_manager.calculate_path_list_jointspace()

if __name__ == '__main__':
    main(sys.argv)
