#!/usr/bin/env python3

from math import cos
from re import A
from robot_kinematics import robot_kinematics
import rospy
import sys
from sensor_msgs.msg import JointState
import numpy as np
import csv
from std_msgs.msg import Float64MultiArray
import os
from trajectory_planner_simple import trajectory_planner_simple
from tqdm import tqdm


class path_planner:
    def __init__(self):
        # Creat Object of robot kinematics, which contains direct/ inverse kinematic calculations
        self.robot_kinematics = robot_kinematics()
       
    def calculate_target_path(self, current_joint_state, goal_pose, max_dist_between_waypoints, filename_path_preinsertion_cartesian, filename_path_insertion_cartesian, filename_path_preinsertion_joint_space, filename_path_insertion_joint_space):
        #TODO if/else! 
        intersection_pose = self.calculate_intersection(goal_pose)
        self.clean_path_list_cartesian(filename_path_preinsertion_cartesian, current_joint_state)
        self.clean_path_list_cartesian(filename_path_insertion_cartesian, intersection_pose)
        path_list_preinsertion = self.get_path_list_cartesian(filename_path_preinsertion_cartesian)       
        path_list_insertion = self.get_path_list_cartesian(filename_path_insertion_cartesian)

        self.calculate_path_list_jointspace(current_joint_state, current_joint_pose, goal_pose, max_dist_between_waypoints, path_list_preinsertion, filename_path_preinsertion_joint_space)
        self.calculate_path_list_jointspace(current_joint_state, current_joint_pose, goal_pose, max_dist_between_waypoints, path_list_insertion, filename_path_insertion_joint_space)

    def clean_path_list_cartesian(self, filename, current_joint_state):
        # TODO add IF load path do NOT execute this method else if calculate new path execute this before loading path list cartesian
        with open(os.path.join(os.path.dirname(__file__),filename),'w+') as file:
            file.truncate(0)

        with open((os.path.join(os.path.dirname(__file__),filename)), mode="a", newline="") as f:
                writer = csv.writer(f, delimiter=",")
                writer.writerow(self.robot_kinematics.get_pose_from_angles(current_joint_state))
    
    def get_path_list_cartesian(self, filename):
        """ 
        Load A matrices given in cartesian space from .csv file and safe it in "path_list"
        """
        
        path_list = []
        with open((os.path.join(os.path.dirname(__file__),filename))) as f:
            reader = csv.reader(f, delimiter=",")
            for row in reader:
                path_list.append([float(row[0]),float(row[1]),float(row[2]),float(row[3]),float(row[4]),float(row[5]),float(row[6]),float(row[7]),float(row[8]),float(row[9]),float(row[10]),float(row[11])])
        rospy.logwarn("Got path cartesian space.")

        return path_list

    def calculate_intersection(self, goal_pose):
        #TODO Controll Calculation!
        #FIXME local Parameter from rospy params

        offset_insection = np.array([0,0,0]).reshape((3,1))
        pos_target = np.array(goal_pose[9:]).reshape((3,1))
        rot_mat = np.array(goal_pose[0:9]).reshape((3,3))

        pos_offset = np.matmul(rot_mat, offset_insection) 

        rospy.logwarn("Pathplaner Pos Offset" + str(pos_offset))

        intersection_hight = 0                                      #FIXME local Parameter from rospy params
        a,b,c,d,e,f,g,h,i = goal_pose[0], goal_pose[1], goal_pose[2], goal_pose[3], goal_pose[4], goal_pose[5], goal_pose[6], goal_pose[7], goal_pose[8]
        n = float(intersection_hight/(g+h+i))
        pos_intersection = np.matmul(rot_mat,np.ones((3,1)))*n + pos_target - pos_offset

        rospy.logwarn("Pathplaner Pos Intersection" + str(pos_intersection))

        A_intersection = [a,b,c,d,e,f,g,h,i,pos_intersection[0][0],pos_intersection[1][0], pos_intersection[2][0]]
        
        return A_intersection
    

    def calculate_path_list_jointspace(self, current_joint_state, current_joint_pose, goal_pose, max_dist_between_waypoints, input_path, output_filename):
        """
        method, that interpolates Joint Space into 1ms Step
        number of steps need to be calculated, e.g. calculating stepwidth with get_A and divide by max. movement per 1ms
        """    
        current_pose = self.robot_kinematics.get_pose_from_angles(current_joint_state)
        tmp_A_list = []
        # iterates over given waypoints and calculate nessesary number of additional waypoints to be below max. distance between waypoints
        for i in range(len(input_path)-1):
            next_pose = np.array(input_path[i+1])
            delta_pose = next_pose - current_pose
            tmp_dist = np.linalg.norm(delta_pose)
            counter = int((tmp_dist// max_dist_between_waypoints)+1)

        # intepolate between given waypoints with given max. distance between interpolated waypoints   
            for i in range(counter):
                interpol_pose = current_pose + i/counter*delta_pose
                tmp_A_list.append(interpol_pose)
            
            current_pose = next_pose

        # delete content of existing file
        with open(os.path.join(os.path.dirname(__file__),output_filename),'w+') as file:
            file.truncate(0)

        current_theta = current_joint_state
        # calculate joint space for each A and write it into file
        for i in tqdm(range(len(tmp_A_list)-1), ncols=100 ):
            
            with open((os.path.join(os.path.dirname(__file__),output_filename)), mode="a", newline="") as f:
                writer = csv.writer(f, delimiter=",")
                writer.writerow(current_theta)

            goal_pose = tmp_A_list [i+1]
            current_theta, pos_err = self.robot_kinematics.get_angles_from_pose(current_theta,goal_pose)
        rospy.logwarn("Got path joint space.")
        return

    

    

# for testing purpose  

if __name__ == '__main__':
    
    path_planner = path_planner()    
    current_joint_state                     = [-1.16675,-1.07474, 1.15409,-1.7767, 0.37539, 1.057256, 0.808336]
    goal_pose                               = [0.54942863, -0.78999408,  0.27209839, -0.77634363, -0.36227926,  0.51579483, -0.30889926, -0.4946343,  -0.81235347,  0.34874679,  0.04655161, 0.25860086]
    max_dist_between_waypoints              = 0.01
    filename_path_preinsertion_cartesian    = "Path/calculated_path_preinsertion_cartesian.csv"
    filename_path_insertion_cartesian       = "Path/calculated_path_insertion_cartesian.csv"
    filename_path_preinsertion_joint_space  = "Path/calculated_path_preinsertion_jointspace.csv"
    filename_path_insertion_joint_space     = "Path/calculated_path_insertion_jointspace.csv"
  
    path_planner.calculate_target_path(current_joint_state, goal_pose, max_dist_between_waypoints, filename_path_preinsertion_cartesian, filename_path_insertion_cartesian, filename_path_preinsertion_joint_space, filename_path_insertion_joint_space)

