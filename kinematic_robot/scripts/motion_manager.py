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
    def __init__(self, command_topic, topic_joint_states, topic_goal_pose, new_plan_flag, err_tol=1e-3, debug=False):

        # ROS communication
        if "sim" in command_topic:
            self.curr_joint_states = rospy.wait_for_message("/joint_states", JointState, rospy.Duration(10))
        else:
            self.curr_joint_states = rospy.wait_for_message("/franka_state_controller/joint_states_desired", JointState, rospy.Duration(10))
  
        #self.pub_joint_commands = rospy.Publisher(name=topic_joint_command, data_class=Float64MultiArray, queue_size=20)
        self.sub_joint_states   = rospy.Subscriber(name=topic_joint_states, data_class=JointState, callback=self.callback_joint_states, queue_size=10)
        #self.sub_goal_pose      = rospy.Subscriber(name=topic_goal_pose, data_class=Float64MultiArray, callback=self.callback_goal_pose, queue_size=10)

        # Our objects
        self.path_planner       = path_planner()
        self.trajectory_planner = trajectory_planner_simple()
        self.kinematics         = robot_kinematics()
        self.motion_executor    = MotionExecutor(command_topic)

        # Constants and parameters
        #self.curr_joint_states  = rospy.wait_for_message(topic_joint_states, JointState, rospy.Duration(10)).position
        self.curr_goal_pose     = [1.0,0.0,0.0,0.0,-1.0,-0.0,0.0,0.0,-1.0,0.4,0,0.3]
        self.has_a_plan         = new_plan_flag     # Indicates if a motion execution plan is present

        self.err_tolerance      = err_tol
        self.counter = 0
        self.max_dist_between_stutzpunkten = 0.01


    def callback_joint_states(self, msg_in : JointState):
        ''' Callback function for the topic_joint_states. Stores current angles in object variable'''
        self.current_joint_state = msg_in.position

    def callback_goal_pose(self, msg_in : Float64MultiArray):
        ''' Callback function for the topic_goal_pose. Stores current goal pose in object variable 
            and sets the has_a_plan flag to False
        '''
        self.current_goal_pose  = msg_in.data
        self.has_a_plan         = False
        # TODO Input verification -> msg_in.data should be a pose matrix

    def has_active_goal_pose(self):
        ''' Returns true if a goal pose has been set and is not reached yet.
            Returns false if no goal pose has been set or goal pose has been reached
        '''

        if self.curr_goal_pose == None:
            return False #FIXME change into False!!
        
        curr_pose   = self.kinematics.get_pose_from_angles(self.curr_joint_states.position)
        goal_pose   = self.curr_goal_pose
        error       = np.linalg.norm(curr_pose[9:12] - goal_pose[9:12])

        if error <= self.err_tolerance:
            return False
        else:
            return True
        
    def send_commandlist(self):
        # Create the message and send it to the advertised topic
        path = self.get_list()
        angle = path[self.counter]
        msg = Float64MultiArray()
        msg.data = angle
        self.publisher.publish(msg)
        self.counter += 1
        
        #/TODO Error handling nessesary! If list index higher than last length, resend last entrie (hold state)
        #/TODO exit get_list only once
        #/TODO robot join-limit handler
    
    def plan_target_path(self):
        pass
    
    def send_joint_command(self):
        # send single command
        pass

    def get_position_error(self):
        # calcualte position error. If to high, stop execution and wati for reaching target
        #/TODO compare actual state with target state, if error to high do not send new angles, instead send last target angles
        pass

    def get_flag_list(self):
        # if / else flag, replace existing list
        pass

    def set_flag_newlist(self):
        #set flag if new list avaible
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
    command_topic = rospy.get_param("~command_topic", "/joint_position_example_controller_sim/joint_command")
    new_plan_flag = rospy.get_param("~new_plan_flag", False)
    #rospy.logwarn("Operation Mode " + str(operation_mode))
    motion_manager      = MotionManager(command_topic, topic_joint_states, topic_goal_pose, new_plan_flag)

    # Loop

    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        if motion_manager.has_active_goal_pose():
            if motion_manager.has_a_plan:
                motion_manager.plan_motion()
                rospy.logwarn("Has a planed motion.")
                motion_manager.motion_execution()
                # /TODO Detect divergence from planned joints to actual joints
            else:
                # /TODO add last path to goal
                motion_manager.plan_motion()
                rospy.logwarn("Planed motion.")
                motion_manager.motion_execution()
        else:
            if motion_manager.has_a_plan:
                rospy.logwarn("Has a planed motion.")
                motion_manager.motion_execution()
                # /TODO Detect divergence from planned joints to actual joints
            else:
                motion_manager.plan_motion()
                rospy.logwarn("Planed motion.")
                motion_manager.motion_execution()
        rate.sleep()

if __name__ == '__main__':
    
    main(sys.argv)


