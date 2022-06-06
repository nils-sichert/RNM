#!/usr/bin/env python3
"""
//TODO
source devel/setup.bash

TASK: Sends goal pose every 1ms to the robot drive OR sends a list of all poses, sampled in 1ms steps to the driver (this is the better way to do it, because everything is precalculated)
INPUT: goal pose, current pose
OUTPUT: goal pose in form of angles
"""

from math import cos
from kinematic import kinematic
import rospy
import sys
from sensor_msgs.msg import JointState
import numpy as np
import csv
from std_msgs.msg import Float64MultiArray
import os
from trajectory_planer import trajectory_planer

tmp = os.path.dirname(__file__)

class motion_execution:
    def __init__(self, command_topic):
        self.command_topic = command_topic
        self.trajectory_planer = trajectory_planer()
        self.counter = 0

    def send_commandlist(self):
        # Create the message and send it to the advertised topic
        path = self.get_list()
        angle = path[self.counter]
        msg = Float64MultiArray()
        msg.data = angle
        self.publisher.publish(msg)
        self.counter += 1
    
    def send_command(self):
        # send single command
        pass

    def get_position_error(self):
        # calcualte position error. If to high, stop execution and wati for reaching target
        #/TODO Winkel in Liste schreiben und Zwischenschritte interpolieren

        pass

    def get_list(self):
        path = self.trajectory_planer.creat_path()
        return path


def main(argv):
    rospy.init_node("motion_executor")
    # Read parameters specified in the .launch file
    # The "~" in the beginning means that it will use parameters from the node
    # e.g. /node_name/parameter_name can then be
    # read by rospy.get_param("parameter_name", default)
    # move_dist = rospy.get_param("~joint_move_dist", 10)
    command_topic = rospy.get_param("~command_topic", "/default_command_topic")

    #rospy.logwarn("Using topic " + str(command_topic) + " and distance " + str(move_dist))
    rospy.logwarn("Using topic " + str(command_topic))
    motion_executor = motion_execution(command_topic)

    # Loop infinitely with a fixed frequency of 1000 hz
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        motion_executor.send_commandlist()
        rate.sleep()


if __name__ == '__main__':
    main(sys.argv)
