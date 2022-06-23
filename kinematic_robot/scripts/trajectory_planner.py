#!/usr/bin/env python3
"""
//TODO
source devel/setup.bash
"""
import sys
from math import cos

import numpy as np
from genpy import Duration
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tqdm import tqdm

from robot_kinematics import robot_kinematics
import util

class trajectory_planner_v01:
    # /TODO: CG: Please check if this class is still useful, contact me
    def __init__(self):
        
        self.robot_kinematics = robot_kinematics()
        self.theta_init = self.get_joint_state()
        self.A_init = self.robot_kinematics.get_pose_from_angles(self.theta_init)
        self.A = self.robot_kinematics.get_pose_from_angles(self.theta_init)

        #debugging parameters
        self.stepsize = 0.001
        self.iteration = 100
        
    def create_path(self):                  
        # /TODO CG: This is a task for the path planner 
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

class TrajectoryPlanner: 
    ''' Takes in a list of joint positions "waypoints" and returns a list of 1kHz joint commands.


    '''
    def __init__(self, limits_dict, safety_factor=0.1, safety_margin=0.01, debug=False):
        ''' Parameters:
                limits_dict (dict): Joint limits for pos, vel, acc and jrk
                debug (bool): Enables debug behavior
                safety_factor (float): Scales vel, acc and jrk limits
                safety_margin (float): Distance to min/max pos
        '''

        # Our objects
        self.kinematics     = robot_kinematics()

        # Constants and parameters
        self.safety_factor  = safety_factor

        self.qlimit_pos_max = np.array(limits_dict["q_pos_max"]) - safety_margin    # Joint space [rad]
        self.qlimit_pos_min = np.array(limits_dict["q_pos_min"]) + safety_margin
        self.qlimit_vel_max = np.array(limits_dict["q_vel_max"]) * safety_factor
        self.qlimit_acc_max = np.array(limits_dict["q_acc_max"]) * safety_factor
        self.qlimit_jrk_max = np.array(limits_dict["q_jrk_max"]) * safety_factor
        self.plimit_vel_max = np.array(limits_dict["p_vel_max"]) * safety_factor    # Cartesian space [m/s]
        self.plimit_acc_max = np.array(limits_dict["p_acc_max"]) * safety_factor
        self.plimit_jrk_max = np.array(limits_dict["p_jrk_max"]) * safety_factor

        


    def get_trajectory(self, waypoints, timepoints=None):
        ''' Calculate a trajectory given a list of waypoints.

            Parameters:
                waypoints (List): List of Lists with 7 joint angles given from path planning
                timepoints (List): Time points corresponding to waypoints in ms. Will be estimated if not provided

            Returns:
                trajectory_list (List): List of Lists with 7 joint angles describing 1kHz trajectory
                max_vel (float): Maximum velocity experienced by a joint
                max_acc (float): Maximum velocity experienced by a joint
                max_jrk (float): Maximum velocity experienced by a joint
        '''

        # Input verification
        waypoints   = np.array(waypoints)
        assert waypoints.shape[1] == 7, "waypoints not in correct format"

        # If not provided, estimate time points for waypoints using certesian max velocity
        if timepoints == None:
            timepoints = [0]
            last_coord = self.kinematics.get_pose_from_angles(waypoints[0])[9:12]

            for waypoint in waypoints[1:]:
                current_coord   = self.kinematics.get_pose_from_angles(waypoint)[9:12]
                dist            = np.linalg.norm(last_coord - current_coord)
                duration_ms     = np.ceil(1000 * dist/self.plimit_vel_max)
                timepoints.append(duration_ms)
                last_coord      = current_coord

        # Get quintic equation parameters for each segment
        for i in range(waypoints.shape[0]):
            pass

        # Sample trajectory list from quintic expressions


        # Safety check



if __name__ == '__main__':

    # Taken from https://frankaemika.github.io/docs/control_parameters.html
    limits  = { "q_pos_max" : [ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973],
                "q_pos_min" : [-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973],
                "q_vel_max" : [ 2.1750,	2.1750,	2.1750,	2.1750, 2.6100, 2.6100, 2.6100],
                "q_acc_max" : [     15,    7.5,     10,   12.5,     15,     20,     20],
                "q_jrk_max" : [   7500,   3750,   5000,   6250,   7500,  10000,  10000],
                "p_vel_max" :   1.7000,
                "p_acc_max" :  13.0000,
                "p_jrk_max" : 6500.000 }

    trajectory_planner  = TrajectoryPlanner(limits, safety_factor=0.5)
    waypoints           = util.get_list_from_csv("Path", "hardcoded_path_joint.csv")

    trajectory_list     = trajectory_planner.get_trajectory(waypoints)
    