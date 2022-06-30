#!/usr/bin/env python3

from robot_kinematics import robot_kinematics
import rospy
import numpy as np
import csv
import os
from tqdm import tqdm
# FIXME eine Zeile zu viel, dividieren durch 0 unterbinden, nan bei gleicher Position?

class trajectory_planner_simple:
    def __init__(self):
        ''' Parameters:
                movement_speed (float): Movement speed for end effector [m/s] TODO: is that correct?
        '''
        # Instances
        self.robot_kinematics = robot_kinematics()

    def create_point_to_point_traj(self, start_pose, end_pose):
        ''' Wraps create_simple_trajectory with auto waypoint file generation, such that a simple 
            point to point trajectory can easily be realised. A directory and file name of the 
            trajectory is returned for further processing (e.g. for motion execution).
            Parameters:
                start_pose (List): Contains start pose joints
                end_pose (List): Contains end pose joints
            Returns:
                traj_dirname (String): directory and file string for generated trajectory
        '''

        path_dirname    = 'Path/temp_point2point_path.csv'
        traj_dirname    = 'trajectory/temp_point2point_traj.csv'

        # Delete content of existing file or create if not existing
        with open(os.path.join(os.path.dirname(__file__), path_dirname),'w+') as file:
            file.truncate(0)

        # Write start and end pose into path file
        with open((os.path.join(os.path.dirname(__file__), path_dirname)), mode="a", newline="") as f:    
            writer = csv.writer(f, delimiter=",")
            writer.writerow(start_pose)
            writer.writerow(end_pose)

        # Create point 2 point trajectory
        self.create_simple_trajectory(path_dirname, traj_dirname)
        
        return traj_dirname 

    def create_simple_trajectory(self, file_input_waypoints, file_output_trajectory, MOVEMENT_SPEED, initial_waypoint=None):
        ''' Load file with waypoints in joint-space and write file with interpolated joint-angles 
            sampled at 1kHz (Step size of 1ms). An initial waypoint can be set in case the current 
            robot pose is not the same as the first input waypoint. This is useful for hardcoded 
            paths with variable starting point.
            Paramters:
                file_input_waypoints (String): Directory and file name to input waypoints in joint-space
                file_output_trajectory (String): Directory and file name for output trajectory list in joint space
                initial_waypoint (List): An initial waypoint in joint-space can preceed the first waypoint in the input file
        '''
        # TODO replace .append!
        # Load planned path from .csv file
        joint_list = []
        with open((os.path.join(os.path.dirname(__file__), file_input_waypoints))) as f:
                reader = csv.reader(f, delimiter=",")
                for row in reader:
                    joint_list.append([float(row[0]),float(row[1]),float(row[2]),float(row[3]),float(row[4]),float(row[5]),float(row[6])])
    
        # delete content of existing file
        with open(os.path.join(os.path.dirname(__file__), file_output_trajectory),'w+') as file:
            file.truncate(0)

        # Prepend initial waypoint
        if not initial_waypoint == None:
            joint_list.insert(0, initial_waypoint) 

        # inital pose from first joint
        # FIXME evtl. Fehler mit Doppelerfassung
        current_pose = self.robot_kinematics.get_pose_from_angles(joint_list[0])

        # calculate number of interpolations between each given waypoint
        for i in tqdm(range(len(joint_list) - 1), ncols=100):
            current_joints  = np.array(joint_list[i])
            next_joints     = np.array(joint_list[i + 1])
            next_pose       = self.robot_kinematics.get_pose_from_angles(next_joints)     # /FIXME confirm correction
            dist            = np.linalg.norm(next_pose[9:12] -  current_pose[9:12])
            steps           = int(dist / MOVEMENT_SPEED)
            delta_joints_per_step = (next_joints - current_joints) / steps

            # calculate each joint state for given movement speed and 1000Hz publishing rate
            with open((os.path.join(os.path.dirname(__file__), file_output_trajectory)), mode="a", newline="") as f:    
                writer = csv.writer(f, delimiter=",")
                for j in range(steps + 1):
                    sample_joint = current_joints + j * delta_joints_per_step
                    writer.writerow(sample_joint)

            current_pose = next_pose
        
        rospy.logwarn(f'Generated simple trajectory in file {file_output_trajectory}')
        print("Generated simple trajectory")
        return 

# for test purposes
if __name__ == '__main__':
    #trajectory_calculate_simple = trajectory_planner_simple()
    #path_planner.calculate_path_list_jointspace()
    #trajectory_calculate_simple.create_path()

    # New tests
    traj_planner    = trajectory_planner_simple(movement_speed=0.020/1000)

    # Test point 2 point trajectory generation
    p1  = [4.902633183867522e-06,-2.817378035757656e-06,3.7433388122565248e-06,-0.11962521536745907,-2.3976978260620285e-06,7.143186770974808e-07,6.714333409263418e-07]
    p2  = [0.24893705772069105,2.664877200306105,0.07457192893703553,1.9891384910981544,-0.023143228919031037,0.5593613260435555,0.16576735333757406]
    traj_file   = traj_planner.create_point_to_point_traj(p1, p2)
    
    # Test planned path trajectory generation
    file_input_waypoints    = "Path/planned_path_jointspace.csv"
    file_output_trajectory  = "trajectory/simple_trajectory.csv"
    initial_waypoint        = [4.9026e-06,-2.8173e-06,3.7433e-06,-0.1196,-2.3976e-06,7.1431e-07,6.7143e-07]

    traj_planner.create_simple_trajectory(file_input_waypoints, file_output_trajectory, initial_waypoint=initial_waypoint)
        