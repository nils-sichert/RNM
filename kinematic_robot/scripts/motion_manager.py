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
    def __init__(self, topic_joint_command, topic_joint_states, topic_goal_pose, debug=False):


        #self.current_angles     = rospy.wait_for_message(self.joint_state_topic, JointState, rospy.Duration(10)).position
        self.current_angles = []

        self.pub_joint_commands = rospy.Publisher(name=topic_joint_command, data_class=Float64MultiArray, queue_size=20)
        self.sub_joint_states   = rospy.Subscriber(name=topic_joint_states, data_class=JointState, callback=self.callback_joint_states, queue_size=10)
        self.sub_goal_pose      = rospy.Subscriber(name=topic_goal_pose, data_class=Float64MultiArray, callback=self.callback_goal_pose, queue_size=10)



        self.command_topic = joint_command_topic
        self.trajectory_planer = trajectory_planer()
        self.counter = 0
        self.kinematic = robot_kinematics()
        self.max_dist_between_stutzpunkten = 0.01

    def callback_joint_states(self, msg_in : JointState):
        ''' Callback function for the tpoic_joint_states. Stores current angles in object variable'''
        self.current_joint_state = msg_in.position

    def callback_goal_pose(self, msg_in : Float64MultiArray):
        ''' Callback function for the topic_goal_pose. Stores current goal pose in object variable'''
        self.current_goal_pose = msg_in.data
        # TODO Input verification -> should be a A matrix

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
    
    def send_joint_command(self):
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
    rospy.init_node("motion_manager")
    operation_mode      = rospy.get_param("/operation_mode")
    topic_joint_command = rospy.get_param("/topic_joint_command")
    topic_joint_states  = rospy.get_param("/topic_joint_states")
    topic_goal_pose     = "/goal_pose"

    rospy.logwarn("Operation Mode " + str(operation_mode))
    manager     = motion_manager(topic_joint_command, topic_joint_states, topic_goal_pose)

    # Loop infinitely with a fixed frequency of 1000 hz
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()
        pass

if __name__ == '__main__':
    main(sys.argv)


