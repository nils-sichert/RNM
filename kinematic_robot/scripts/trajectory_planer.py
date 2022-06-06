#!/usr/bin/env python3
"""
//TODO
source devel/setup.bash
"""
from math import cos
from kinematic import kinematic
import rospy
import sys
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Float64MultiArray
from tqdm import tqdm

class trajectory_planer:
    def __init__(self):
        
        self.q_init = self.get_joint_state()
        self.stepsize = 0.001
        self.iteration = 100
        self.kinematic = kinematic()
        self.A_init = self.kinematic.direct_kinematic(self.q_init)
        self.A = self.kinematic.direct_kinematic(self.q_init)
        # Setup the publisher
        
    def creat_path(self):
        theta = self.get_joint_state()
        theta_list = []
        for i in tqdm(range(self.iteration), ncols=100):
            # /TODO Testpfad korrigieren
            A_target = self.A + np.array([0,0,0,0,0,0,0,0,0,1*self.stepsize,1*self.stepsize,1*self.stepsize])
            theta_target = self.kinematic.inverse_kinematic(theta, self.A, A_target,)
            # Create the message and send it to the advertised topic
            theta_list.append(theta_target)
            self.A = A_target
        return theta_list

    def get_joint_state(self):
        #if "sim" in self.command_topic:
        msg = rospy.wait_for_message("/joint_states", JointState, rospy.Duration(10))
        # else:
        #     # TODO: Why is it called "joint_states_desired"?
        #     msg = rospy.wait_for_message("/franka_state_controller/joint_states_desired", JointState, rospy.Duration(10))

        joint_state = np.array(msg.position)

        return joint_state

