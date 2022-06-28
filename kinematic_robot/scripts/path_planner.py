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
    def __init__(self, current_theta):
        # Creat instance of robot kinematics, which contains direct/ inverse kinematic calculations
        self.robot_kinematics = robot_kinematics()
        self.current_theta = current_theta
        # maximum distance between each waypoint (|x/y/z|), no rotation is taken into account
        self.max_dist_between_supports = 0.01

    def get_path_list_cartesian(self):
        """ 
        Load A matrices given in cartesian space from .csv file and safe it in "path_list"
        """
        
        path_list = []
        with open((os.path.join(os.path.dirname(__file__),"Path/planned_path_cartesian.csv"))) as f:
            reader = csv.reader(f, delimiter=",")
            for row in reader:
                path_list.append([float(row[0]),float(row[1]),float(row[2]),float(row[3]),float(row[4]),float(row[5]),float(row[6]),float(row[7]),float(row[8]),float(row[9]),float(row[10]),float(row[11])])
        rospy.logwarn("Got path cartesian space.")

        return path_list

    def clean_path_list_cartesian(self):
        # TODO add IF load path do NOT execute this method else if calculate new path execute this before loading path list cartesian
        with open(os.path.join(os.path.dirname(__file__),"Path/planned_path_cartesian.csv"),'r+') as file:
            file.truncate(0)

        with open((os.path.join(os.path.dirname(__file__),"Path/planned_path_cartesian.csv")), mode="a", newline="") as f:
                writer = csv.writer(f, delimiter=",")
                writer.writerow(self.robot_kinematics.get_pose_from_angles(self.current_theta))

    def calculate_path_list_jointspace(self):
        """
        method, that interpolates Joint Space into 1ms Step
        number of steps need to be calculated, e.g. calculating stepwidth with get_A and divide by max. movement per 1ms
        """
        # load waypointlist
        path_list = self.get_path_list_cartesian()
        
        # set start theta
        # TODO hardcode ersetzten
        # FIXME Anfangsposition übernehmen, leichter Denkfehler zwischen move to start und dem hier, bedenke move to start ist nicht gleich current_theta
        current_pose = self.robot_kinematics.get_pose_from_angles(self.current_theta)
        tmp_A_list = []
       
        # iterates over given waypoints and calculate nessesary number of additional waypoints to be below max. distance between waypoints
        for i in range(len(path_list)-1):
            next_pose = np.array(path_list[i+1])
            delta_pose = next_pose - current_pose
            tmp_dist = np.linalg.norm(delta_pose)
            counter = int((tmp_dist// self.max_dist_between_supports)+1)

        # intepolate between given waypoints with given max. distance between interpolated waypoints   
            for i in range(counter):
                interpol_pose = current_pose + i/counter*delta_pose
                tmp_A_list.append(interpol_pose)
            
            current_pose = next_pose

        # delete content of existing file
        with open(os.path.join(os.path.dirname(__file__),"Path/planned_path_jointspace.csv"),'r+') as file:
            file.truncate(0)

        current_theta = self.current_theta
        # calculate joint space for each A and write it into file
        for i in tqdm(range(len(tmp_A_list)-1), ncols=100 ):
            
            with open((os.path.join(os.path.dirname(__file__),"Path/planned_path_jointspace.csv")), mode="a", newline="") as f:
                writer = csv.writer(f, delimiter=",")
                writer.writerow(current_theta)

            A_target = tmp_A_list [i+1]
            current_theta, pos_err = self.robot_kinematics.get_angles_from_pose(current_theta,A_target)
        rospy.logwarn("Got path joint space.")
        return path_list

    def calculate_target_path(self, A_target):
        #TODO clean writing, double entries! First entry current A from joint space
        #TODO if/else! 
        self.clean_path_list_cartesian()
        path_list = self.get_path_list_cartesian()
        A_intersection = self.calculate_intersection(A_target)
        with open(os.path.join(os.path.dirname(__file__),"Path/planned_path_cartesian.csv"),'r+') as file:
            file.truncate(0)

        with open((os.path.join(os.path.dirname(__file__),"Path/planned_path_cartesian.csv")), mode="a", newline="") as f:
                writer = csv.writer(f, delimiter=",")
                for i in range(len(path_list)):
                    writer.writerow(path_list[i])
                writer.writerow(A_intersection)
                writer.writerow(A_target)

    def calculate_intersection(self, A_target):
        #TODO Recalculate entrie when receiving intersection point
        offset_insection = np.array([0,0,0]).reshape((3,1))
        pos_target = np.array(A_target[9:]).reshape((3,1))
        rot_mat = np.array(A_target[0:9]).reshape((3,3))
        pos_offset = np.matmul(rot_mat, offset_insection) #FIXME Offset need to be calculated with A from camera
        rospy.logwarn("Pathplaner Pos Offset" + str(pos_offset))
        intersection_hight = 0 #
        a,b,c,d,e,f,g,h,i = A_target[0], A_target[1], A_target[2], A_target[3], A_target[4], A_target[5], A_target[6], A_target[7], A_target[8]
        n = float(intersection_hight/(g+h+i))
        pos_intersection = np.matmul(rot_mat,np.ones((3,1)))*n + pos_target - pos_offset
        rospy.logwarn("Pathplaner Pos Intersection" + str(pos_intersection))
        A_intersection = [a,b,c,d,e,f,g,h,i,pos_intersection[0][0],pos_intersection[1][0], pos_intersection[2][0]]
        
        return A_intersection

# for testing purpose  

if __name__ == '__main__':
    current_theta = [-1.16675,-1.07474, 1.15409,-1.7767, 0.37539, 1.057256, 0.808336]
    path_planner = path_planner(current_theta)    
    A_target = [0.54942863, -0.78999408,  0.27209839, -0.77634363, -0.36227926,  0.51579483, -0.30889926, -0.4946343,  -0.81235347,  0.34874679,  0.04655161, 0.25860086]
    path_planner.calculate_target_path(A_target)
    path_planner.calculate_path_list_jointspace()
    #path_planner.calculate_path_list_jointspace()
