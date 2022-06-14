#!/usr/bin/env python3
"""
//TODO
source devel/setup.bash
"""
from math import cos
from robot_kinematics import robot_kinematics
import rospy
import sys
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Float64MultiArray
from tqdm import tqdm

class trajectory_planer:
    def __init__(self):
        
        self.robot_kinematics = robot_kinematics()
        self.theta_init = self.get_joint_state()
        self.A_init = self.robot_kinematics.get_pose_from_angles(self.theta_init)
        self.A = self.robot_kinematics.get_pose_from_angles(self.theta_init)

        #debugging parameters
        self.stepsize = 0.001
        self.iteration = 100
        
    def create_path(self):
        theta = self.get_joint_state()
        theta_list = []
        for i in tqdm(range(self.iteration), ncols=100):
            # /TODO Testpfad korrigieren, Stuetzstellen einlesen
            A_target = self.A + np.array([0,0,0,0,0,0,0,0,0,1*self.stepsize,1*self.stepsize,1*self.stepsize])
            
            #calculate joint-space 
            theta_target = self.kinematic.inverse_kinematic(theta, self.A, A_target,)
            # Create the message and send it to the advertised topic
            theta_list.append(theta_target)
            self.A = A_target
        return theta_list

    def get_joint_state(self):
        msg = rospy.wait_for_message("/joint_states", JointState, rospy.Duration(10))
       
        joint_state = np.array(msg.position)

        return joint_state

if __name__ == '__main__':
    trajectory_player = trajectory_planer()
    trajectory_player.create_path()