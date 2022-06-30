#!/usr/bin/env python3
import csv
import os
import sys

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from path_planner import path_planner
from robot_kinematics import robot_kinematics
from trajectory_planner_simple import trajectory_planner_simple
from motion_executor import MotionExecutor

class MotionManager:
    def __init__(self, command_topic, topic_joint_states, topic_goal_pose, err_tol=1e-3, debug=False):

        # ROS communication

        # Subscriber
        if "sim" in command_topic:
            self.current_joint_state_sub    = rospy.Subscriber('/joint_states', JointState, self.callback_joint_states, queue_size=1)
        else:
            self.current_joint_state_sub    = rospy.Subscriber('/franka_state_controller/joint_states_desired', JointState, self.callback_joint_states, queue_size=1)

        # Objects
        self.path_planner       = path_planner(self.current_theta)
        self.trajectory_planner = trajectory_planner_simple()
        self.kinematics         = robot_kinematics()
        self.motion_executor    = MotionExecutor(command_topic)

        # Constants and parameters
        """FIXME Pose2Angle error, can not create path when having different rotation in endefector
        Offset neadle!! Wir bekommen ein Target A, welches zuruckgerechnet werden muss bis zum Joint, dass ist Zielposition
        """

        #self.pos_target         = [0.37982467,  0.35923062,  0.43909587]
        #self.pos_target         = [0.31982467,  0.05923062,  0.23909587]
        
        self.current_theta      = self.curr_joint_states.position
        self.current_A          = self.kinematics.get_pose_from_angles(self.current_theta)
        self.current_rot        = self.current_A[:9]
        self.curr_goal_pose     = [self.current_rot[0], self.current_rot[1], self.current_rot[2], self.current_rot[3], self.current_rot[4], self.current_rot[5], self.current_rot[6], self.current_rot[7], self.current_rot[8], self.pos_target[0], self.pos_target[1], self.pos_target[2]]
        self.err_tolerance      = err_tol

        # Debug
        rospy.logwarn("Current A:" + str(self.current_A))

    def callback_joint_states(self, msg_in):
        ''' Callback function for the topic_joint_states. Stores current angles in object variable'''
        self.current_joint_state = msg_in.position

  
    def move2goal_js(self, GoalPose, MOVEMENT_SPEED):
        """
        Move the robot's endefector to goal pose given in joint space. 
        To do so:
        1. interpolate actual to goal joint angles and sample to 1ms steps
        2. Load trajectory and execute motion
        """
        filename = "calculated_trajectory_to_goal_1ms.csv"
        self.trajectory_planner.create_path(GoalPose, MOVEMENT_SPEED, filename)
        self.motion_executor.run(filename)
        return

    def move_start2preincision(self, needle_goal_pose, MOVEMENT_SPEED):
        self.path_planner.calculate_target_path()
        pass

    def plan_motion(self):
        # initialize list planing
        # TODO write listplanner
        self.path_planner.calculate_target_path(self.curr_goal_pose)
        self.path_planner.calculate_path_list_jointspace()
        self.trajectory_planner.create_path()
        return

    def move2start(self):
        self.motion_executor.move_to_start()
        return

    def motion_execution(self):
        self.motion_executor.run()
        return

def main(argv):

    # Init
    rospy.init_node("motion_manager")
    #operation_mode      = rospy.get_param("/operation_mode")
    topic_joint_states  = "Joint_state" #rospy.get_param("/topic_joint_states")
    topic_goal_pose     = None # "/goal_pose"
    command_topic   = rospy.get_param("~command_topic", "/joint_position_example_controller_sim/joint_command")
    has_a_path_flag = rospy.get_param("~new_plan_flag", False)
    #rospy.logwarn("Operation Mode " + str(operation_mode))
    motion_manager      = MotionManager(command_topic, topic_joint_states, topic_goal_pose, has_a_path_flag)

    # Pre-run
    #   launch launchfile with:
    #       publishers: joint_states
    #       parameter set up für topic names
    #       init_pose (joint space)
    # 
    # Setup MM
    #   get topic names -> als rosparam aus launchfile
    #   get init_post (parameter)
    #   publisher init
    #       goal_pose_reached (True=reached/False=notreached)
    #   subscriber init
    #       joint_state (js list) für aktuelle pose
    #       goal_pose_js (js list) für aktuelle goal pose im joint space
    #       goal_pose_cs (A matrix) für aktuelle goal pose im cartesian space
    #   param
    #       needle_goal_published (bool) from CV when target found

    # Loop
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        # Received new goal pose for CV
        # If old goal pose != new goal pose && !needle_goal_published
        #   then goal_pose_reached = false

        # Received new goal pose for incision
        #   Move to init_pose (for collision avoidance)
        #   Wait for user input (to change needle)
        #   Move to to pre-incision point
        #   Wait
        #   Execute incision


    # Loop

    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        if motion_manager.has_active_goal_pose():
            if motion_manager.has_a_plan:
                motion_manager.plan_motion()
                rospy.logwarn("Has a planned motion.")
                motion_manager.motion_execution()
                # /TODO Detect divergence from planned joints to actual joints
            else:
                # /TODO add last path to goal
                motion_manager.plan_motion()
                rospy.logwarn("Planned motion.")
                motion_manager.motion_execution()
        else:
            if motion_manager.has_a_plan:
                rospy.logwarn("Has a planned motion.")
                motion_manager.motion_execution()
                # /TODO Detect divergence from planned joints to actual joints
            else:
                motion_manager.plan_motion()
                rospy.logwarn("Planned motion.")
                motion_manager.motion_execution()
        rate.sleep()

if __name__ == '__main__':
    
    main(sys.argv)


