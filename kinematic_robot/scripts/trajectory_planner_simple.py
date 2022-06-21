from math import cos
from robot_kinematics import robot_kinematics
import rospy
import sys
from sensor_msgs.msg import JointState
import numpy as np
import csv
import os
# FIXME eine Zeile zu viel, dividieren durch 0 unterbinden, nan bei gleicher Position?

class trajectory_planner_simple:
    def __init__(self):
        self.movement_speed = 0.01/1000 #[m/s]
        self.robot_kinematics = robot_kinematics()
        tmp = os.path.dirname(__file__)

    def create_path(self):
            # TODO replace .append!
            joint_list = []
            with open((os.path.join(os.path.dirname(__file__),"Path/planned_path_jointspace.csv"))) as f:
                    reader = csv.reader(f, delimiter=",")
                    for row in reader:
                        joint_list.append([float(row[0]),float(row[1]),float(row[2]),float(row[3]),float(row[4]),float(row[5]),float(row[6])])
        

            with open(os.path.join(os.path.dirname(__file__),"trajectory/calculated_trajectory_simple_1ms.csv"),'r+') as file:
                file.truncate(0)

            current_pose = self.robot_kinematics.get_pose_from_angles(joint_list[0])

            for i in range(len(joint_list) - 1):
                current_joints = np.array(joint_list[i])
                next_joints = np.array(joint_list[i + 1])
                next_pose = self.robot_kinematics.get_pose_from_angles(next_joints)     # /FIXME confirm correction
                dist = np.linalg.norm(next_pose[9:12] -  current_pose[9:12])
                steps = int(dist/self.movement_speed)
                delta_joints_per_step = (next_joints - current_joints)/steps

                with open((os.path.join(os.path.dirname(__file__), "trajectory/calculated_trajectory_simple_1ms.csv")), mode="a", newline="") as f:    
                    writer = csv.writer(f, delimiter=",")
                    for j in range(steps+1):
                        sample_joint = current_joints + j*delta_joints_per_step
                        writer.writerow(sample_joint)

                current_pose = next_pose

            return 0



if __name__ == '__main__':
    trajectory_calculate_simple = trajectory_planner_simple()
    #path_planner.calculate_path_list_jointspace()
    trajectory_calculate_simple.create_path()