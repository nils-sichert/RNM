#!/usr/bin/env python3

from math import cos
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

tmp = os.path.dirname(__file__)

class path_planner:
    def __init__(self):
        self.movement_speed = 0.05/1000 #[m/s]
        self.robot_kinematics = robot_kinematics()
        self.max_dist_between_supports = 0.01

    def get_path_list_cartesian(self):
            path_list = []
            with open((os.path.join(os.path.dirname(__file__),"Path/planned_path_cartesian.csv"))) as f:
                reader = csv.reader(f, delimiter=",")
                for row in reader:
                    path_list.append([float(row[0]),float(row[1]),float(row[2]),float(row[3]),float(row[4]),float(row[5]),float(row[6]),float(row[7]),float(row[8]),float(row[9]),float(row[10]),float(row[11])])
            return path_list
    
    def calculate_path_list_jointspace(self):
        path_list = self.get_path_list_cartesian()
        # TODO hardcode ersetzten
        current_theta = [0.0,0.0,0.0,-1.0,0.0,1.0,0.0]
        current_pose = self.robot_kinematics.get_pose_from_angles(current_theta)
        tmp_A_list = []
       

        for i in range(len(path_list)-1):
            #tmp_pos_list.append(current_pos)
            next_pose = np.array(path_list[i+1])
            delta_pose = next_pose - current_pose
            tmp_dist = np.linalg.norm(delta_pose)
            counter = int((tmp_dist// self.max_dist_between_supports)+1)
            
            for i in range(counter):
                interpol_pose = current_pose + i/counter*delta_pose
                tmp_A_list.append(interpol_pose)
            
            current_pose = next_pose

        with open(os.path.join(os.path.dirname(__file__),"Path/planned_path_jointspace.csv"),'r+') as file:
            file.truncate(0)

        A_current = tmp_A_list[0]
        for i in tqdm(range(len(tmp_A_list)-1), ncols=100 ):
            # TODO hardcode ersetzten

            with open((os.path.join(os.path.dirname(__file__),"Path/planned_path_jointspace.csv")), mode="a", newline="") as f:
                writer = csv.writer(f, delimiter=",")
                writer.writerow(current_theta)

            
            A_target = tmp_A_list [i+1]
            current_theta, pos_err = self.robot_kinematics.get_angles_from_pose(current_theta,A_target)
        

        return path_list
        

    
        # method, that interpolates Joint Space into 1ms Step
        # number of steps need to be calculated, e.g. calculating stepwidth with get_A and divide by max. movement per 1ms
        

if __name__ == '__main__':
    path_planner = path_planner()
    #path_planner.calculate_path_list_jointspace()
    path_planner.calculate_path_list_jointspace()