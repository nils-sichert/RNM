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
        self.kinematic = kinematic()
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

    def get_path_list_cartesian(self):
        path_list = []
        with open((os.path.join(os.path.dirname(__file__),"Path/planned_path_cartesian.csv"))) as f:
            reader = csv.reader(f, delimiter=",")
            for row in reader:
                path_list.append([float(row[0]),float(row[1]),float(row[2])])
        return path_list
    
    def calculate_path_list_jointspace(self):
        path_list = self.get_path_list_cartesian()
        # TODO hardcode ersetzten
        current_theta = [0,0,0,0,0,0,0]
        current_pos = self.kinematic.get_A_2(current_theta)
        tmp_A_list = []
       

        for i in range(len(path_list)):
            #tmp_pos_list.append(current_pos)
            next_pose = np.array(path_list[i])
            delta_pose = next_pose - current_pos[9:12]
            tmp_dist = np.linalg.norm(delta_pose)
            counter = int((tmp_dist// self.max_dist_between_stutzpunkten)+1)
            
            for i in range(counter):
                interpol_pos_tmp = current_pos[9:12] + i/counter*delta_pose
                interpol_pos = np.concatenate((current_pos[0:9], interpol_pos_tmp), axis=None)
                tmp_A_list.append(interpol_pos)
            
            current_pos = np.concatenate((current_pos[0:9], next_pose), axis=None)

        with open(os.path.join(os.path.dirname(__file__),"Path/planned_path_jointspace.csv"),'r+') as file:
            file.truncate(0)

        for i in range(len(tmp_A_list)-1):
            # TODO hardcode ersetzten
            with open((os.path.join(os.path.dirname(__file__),"Path/planned_path_jointspace.csv")), mode="a", newline="") as f:
                writer = csv.writer(f, delimiter=",")
                writer.writerow(current_theta)

            A_current = tmp_A_list[i]
            A_target = tmp_A_list [i+1]
            current_theta = self.kinematic.inverse_kinematic(current_theta,A_current,A_target)

        return path_list
        

    def sample_path_list_jointspace(self):
        # method, that interpolates Joint Space into 1ms Step
        # number of steps need to be calculated, e.g. calculating stepwidth with get_A and divide by max. movement per 1ms
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
        path = self.trajectory_planer.creat_path()
        pass
        


def main(argv):
    rospy.init_node("motion_executor")
    command_topic = rospy.get_param("~command_topic", "/default_command_topic")

    #rospy.logwarn("Using topic " + str(command_topic) + " and distance " + str(move_dist))
    rospy.logwarn("Using topic " + str(command_topic))
    motion_executor = motion_execution(command_topic)

    # Loop infinitely with a fixed frequency of 1000 hz
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        motion_executor.calculate_path_list_jointspace()

if __name__ == '__main__':
    main(sys.argv)
