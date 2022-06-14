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
from trajectory_planer import trajectory_planer

tmp = os.path.dirname(__file__)

class path_planner:
    def _init_(self):
        self.movement_speed = 0.05/1000 #[m/s]

    def get_path_list_cartesian(self):
            path_list = []
            with open((os.path.join(os.path.dirname(_file_),"Path/planned_path_cartesian.csv"))) as f:
                reader = csv.reader(f, delimiter=",")
                for row in reader:
                    path_list.append([float(row[0]),float(row[1]),float(row[2]),float(row[3]),float(row[4]),float(row[5]),float(row[6]),float(row[7]),float(row[8]),float(row[9]),float(row[10]),float(row[11])])
            return path_list
    
    def calculate_path_list_jointspace(self):
        path_list = self.get_path_list_cartesian()
        # TODO hardcode ersetzten
        current_theta = [0,0,0,0,0,0,0]
        current_pose = self.robot_kinematic.get_pose_from_angles(current_theta)
        tmp_A_list = []
       

        for i in range(len(path_list)):
            #tmp_pos_list.append(current_pos)
            next_pose = np.array(path_list[i])
            delta_pose = next_pose - current_pose
            tmp_dist = np.linalg.norm(delta_pose)
            counter = int((tmp_dist// self.max_dist_between_stutzpunkten)+1)
            
            for i in range(counter):
                interpol_pose_tmp = current_pose + i/counter*delta_pose
                interpol_pose = np.concatenate((current_pose, interpol_pose_tmp), axis=None)
                tmp_A_list.append(interpol_pose)
            
            current_pose = np.concatenate((current_pose, next_pose), axis=None)

        with open(os.path.join(os.path.dirname(__file__),"Path/planned_path_jointspace.csv"),'r+') as file:
            file.truncate(0)

        for i in range(len(tmp_A_list)-1):
            # TODO hardcode ersetzten
            with open((os.path.join(os.path.dirname(__file___),"Path/planned_path_jointspace.csv")), mode="a", newline="") as f:
                writer = csv.writer(f, delimiter=",")
                writer.writerow(current_theta)

            A_current = tmp_A_list[i]
            A_target = tmp_A_list [i+1]
            current_theta = self.robot_kinematic.get_angles_from_pose(current_theta,A_current,A_target)

        return path_list
        

    def sample_path_list_jointspace(self):
        # TODO replace .append!
        joint_list = []
        with open((os.path.join(os.path.dirname(__file__),"Path/planned_path_cartesian.csv"))) as f:
                reader = csv.reader(f, delimiter=",")
                for row in reader:
                    joint_list.append([float(row[0]),float(row[1]),float(row[2]),float(row[3]),float(row[4]),float(row[5]),float(row[6])])
       

        with open(os.path.join(os.path.dirname(__file__),"Path/planned_path_jointspace_1ms.csv"),'r+') as file:
            file.truncate(0)

        current_pose = self.robot_kinematic.get_pose_from_angles(joint_list[0])

        for i in range(len(joint_list)-1):
            next_pose = joint_list[i+1]
            dist = np.linalg.norm(next_pose[9:12]-current_pose[9:12])
            steps = dist/self.movement_speed

            with open((os.path.join(os.path.dirname(__file__), "")), mode="a", newline="") as f:
                writer = csv.writer(f, delimiter=",")
                delta_joints_per_step = (joint_list[i+1] - joint_list[i])/steps
                for j in range(steps):
                    sample_joint = joint_list + j*delta_joints_per_step
                    writer.writerow(sample_joint)

            current_pose = next_pose

        return 0

        # method, that interpolates Joint Space into 1ms Step
        # number of steps need to be calculated, e.g. calculating stepwidth with get_A and divide by max. movement per 1ms
        

if _name_ == '_main_':
    path_planner = path_planner()
    path_planner.calculate_path_list_jointspace()