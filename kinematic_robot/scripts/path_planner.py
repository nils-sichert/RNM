#!/usr/bin/env python3
import rospy
import numpy as np
import csv
import os
from tqdm import tqdm
from robot_kinematics import robot_kinematics


class path_planner:
    def __init__(self, robot_kinematics):
        # Creat Object of robot kinematics, which contains direct/ inverse kinematic calculations
        self.robot_kinematics = robot_kinematics
        self.last_joint = []
       
    def calculate_path(self, current_joint_state, goal_pose, max_dist_between_waypoints_preinsertion, max_dist_between_waypoints_insertion, filename_path_preinsertion_cartesian, filename_path_insertion_cartesian, filename_path_preinsertion_joint_space, filename_path_insertion_joint_space):
        #TODO if/else! 
        current_pose = self.robot_kinematics.get_pose_from_angles(current_joint_state)
        intersection_pose = self.calculate_intersection(goal_pose)
        
        self.clean_path_list_cartesian(filename_path_preinsertion_cartesian, current_pose, intersection_pose)
        path_list_preinsertion = self.get_path_list_cartesian(filename_path_preinsertion_cartesian) #FIXME Redundency !
        self.calculate_path_list_jointspace(current_joint_state, max_dist_between_waypoints_preinsertion, path_list_preinsertion, filename_path_preinsertion_joint_space)

        self.clean_path_list_cartesian(filename_path_insertion_cartesian, self.robot_kinematics.get_pose_from_angles(self.last_joint), goal_pose)
        path_list_insertion = self.get_path_list_cartesian(filename_path_insertion_cartesian)
        self.calculate_path_list_jointspace(self.last_joint, max_dist_between_waypoints_insertion, path_list_insertion, filename_path_insertion_joint_space)

    def clean_path_list_cartesian(self, filename, start_pose, goal_pose):
        # TODO add IF load path do NOT execute this method else if calculate new path execute this before loading path list cartesian
        with open(os.path.join(os.path.dirname(__file__),filename),'w+') as file:
            file.truncate(0)

        with open((os.path.join(os.path.dirname(__file__),filename)), mode="a", newline="") as f:
                writer = csv.writer(f, delimiter=",")
                writer.writerow(start_pose)
                writer.writerow(goal_pose)

    def get_path_list_cartesian(self, filename, ):
        """ 
        Load A matrices given in cartesian space from .csv file and safe it in "path_list"
        """
        
        path_list = []
        with open((os.path.join(os.path.dirname(__file__),filename))) as f:
            reader = csv.reader(f, delimiter=",")
            for row in reader:
                path_list.append([float(row[0]),float(row[1]),float(row[2]),float(row[3]),float(row[4]),float(row[5]),float(row[6]),float(row[7]),float(row[8]),float(row[9]),float(row[10]),float(row[11])])
        rospy.logwarn("[PP] Got path cartesian space.")

        return path_list

    def calculate_intersection(self, goal_pose):
        #TODO Controll Calculation!
        #FIXME local Parameter from rospy params

        offset_insection = np.array([0,0,0]).reshape((3,1))
        pos_target = np.array(goal_pose[9:]).reshape((3,1))
        rot_mat = np.array(goal_pose[0:9]).reshape((3,3))

        A_intersection = np.append(rot_mat, np.array([pos_target[0]-0.1, pos_target[1], pos_target[2]+0.2]))

        # diff Vektoren
        # normieren
        # Normalenvektoren bilden 
        # Stackoverflow 
        return A_intersection

        pos_offset = np.matmul(rot_mat, offset_insection) 

        rospy.logwarn("[PP] Pos Offset" + str(pos_offset))

        intersection_height = 0.15                                      #FIXME local Parameter from rospy params

        pos_intersection = np.matmul(rot_mat,np.ones((3,1))) * n + pos_target - pos_offset


        a,b,c,d,e,f,g,h,i = goal_pose[0], goal_pose[1], goal_pose[2], goal_pose[3], goal_pose[4], goal_pose[5], goal_pose[6], goal_pose[7], goal_pose[8]
        n = float(intersection_height/(g+h+i))
        pos_intersection = np.matmul(rot_mat,np.ones((3,1)))*n + pos_target - pos_offset

        rospy.logwarn("[PP] Pos Intersection" + str(pos_intersection))

        A_intersection = np.array([a,b,c,d,e,f,g,h,i,pos_intersection[0][0],pos_intersection[1][0], pos_intersection[2][0]])
        
        return A_intersection

    def calculate_path_list_jointspace(self, current_joint_state, max_dist_between_waypoints, input_path, output_filename):
        """
        method, that interpolates Joint Space into 1ms Step
        number of steps need to be calculated, e.g. calculating stepwidth with get_A and divide by max. movement per 1ms
        """    
        current_pose = self.robot_kinematics.get_pose_from_angles(current_joint_state)
        tmp_A_list = []
        # iterates over given waypoints and calculate nessesary number of additional waypoints to be below max. distance between waypoints
        for i in range(len(input_path)-1):
            #TODO Do not interpolate over roation
            next_pose = np.array(input_path[i+1])
            delta_pose = next_pose - current_pose # TODO cartesian norm between x,y,z
            tmp_dist = np.linalg.norm(delta_pose)
            counter = int((tmp_dist// max_dist_between_waypoints)+1)

        # intepolate between given waypoints with given max. distance between interpolated waypoints   
            for i in range(counter+1):
                interpol_pose = current_pose + i/counter*delta_pose
                tmp_A_list.append(interpol_pose)
            
            current_pose = next_pose

        # delete content of existing file
        with open(os.path.join(os.path.dirname(__file__),output_filename),'w+') as file:
            file.truncate(0)

        current_theta = current_joint_state
        with open((os.path.join(os.path.dirname(__file__),output_filename)), mode="a", newline="") as f:
                writer = csv.writer(f, delimiter=",")
                writer.writerow(current_theta)
        # calculate joint space for each A and write it into file
        for i in tqdm(range(len(tmp_A_list)-1), ncols=100 ):
            goal_pose = tmp_A_list [i+1]
            current_theta, pos_err = self.robot_kinematics.get_angles_from_pose(current_theta,goal_pose)
            with open((os.path.join(os.path.dirname(__file__),output_filename)), mode="a", newline="") as f:
                writer = csv.writer(f, delimiter=",")
                writer.writerow(current_theta)

           
        rospy.logwarn("[PP] Got path joint space.")
        self.last_joint = current_theta

        # FIXME implement large Motion handler
        return

    

    

# for testing purpose  

if __name__ == '__main__':
    
    kinematic   = robot_kinematics()
    path_planner = path_planner(kinematic)    
    
    current_joint_state                     = [-7.455726072969071e-06, -3.5540748690721102e-06, -6.046157276173858e-06, -0.7851757638374179, 4.600804249577095e-06, 1.4001585464384902e-06, 1.013981160369326e-06]
    current_pose                            = [ 7.07267526e-01, -5.96260536e-06 ,-7.06945999e-01 ,-1.09650444e-05, -1.00000000e+00 ,-2.53571628e-06 ,-7.06945999e-01 , 9.54512406e-06 ,-7.07267526e-01  ,2.82213352e-01, -3.40555121e-06,  8.41024897e-01]
    goal_pose                               = [7.07267526e-01, -5.96260536e-06 ,-7.06945999e-01 ,-1.09650444e-05, -1.00000000e+00 ,-2.53571628e-06 ,-7.06945999e-01 , 9.54512406e-06 ,-7.07267526e-01,  0.30874679,  0.24655161, 0.45860086]
    max_dist_between_waypoints              = 0.01
    filename_path_preinsertion_cartesian    = "Path/calculated_path_preinsertion_cartesian.csv"
    filename_path_insertion_cartesian       = "Path/calculated_path_insertion_cartesian.csv"
    filename_path_preinsertion_joint_space  = "Path/calculated_path_preinsertion_jointspace.csv"
    filename_path_insertion_joint_space     = "Path/calculated_path_insertion_jointspace.csv"
  
    path_planner.calculate_intersection(goal_pose)
    #path_planner.calculate_target_path(current_joint_state, goal_pose, max_dist_between_waypoints, filename_path_preinsertion_cartesian, filename_path_insertion_cartesian, filename_path_preinsertion_joint_space, filename_path_insertion_joint_space)