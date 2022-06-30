#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

import sys

    # Pre-run
    #   launch launchfile with:
    #       publishers: joint_states
    #       parameter set up für topic names
    #       init_pose (joint space)
    # 
    # Setup PM
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

class ProcessManager:

    def __init__(self):

        # Flags
        self.s0_reset               = True
        self.s1_cv_ready            = False
        self.s2_target_acquired     = False	
        self.s3_pre_incision_reached= False
        self.s4_reverse_active      = False

        self.user_execution_command = False
        self.goal_pose_js_filename  = None


        # ROS inits
        rospy.init_node('process_manager', anonymous=True)
        rospy.Subscriber('~/target_acquired', String, callback_target_acquired)
        rospy.Subscriber('~/goal_pose_js_file', String, callback_goal_pose_js_file)


    def callback_target_acquired(self):
        '''
        '''
        self.s2_target_acquired = True

    def callback_goal_pose_js_file(self, msg_in : String):
        ''' Callback function for the goal_pose_js topic. Stores current goal pose in object variable 
        '''
        self.s1_cv_ready            = True
        self.goal_pose_js_filename  = msg_in


    def get_user_execution_command(self):
        ''' 
        '''
        self.user_execution_command = rospy.get_param('~user_execution_command')

    def reset_user_execution_command(self):
        '''
        '''
        rospy.set_param('~user_execution_command', False)



    def main_process(self):

        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            # State transitions are triggered by parameters or user inputs

            # State 0: Reset state-----------------------------------------------------------------
            if self.s0_reset:
                self.s4_reverse_active = False


            # State 1: Camera Calibration and Target Acquisition-----------------------------------
            if self.s1_cv_ready and self.user_execution_command:
                self.s0_reset = False

                


                # Do until target acquired
                if self.s2_target_acquired: 
                    self.s1_cv_ready = False
                    self.reset_user_execution_command()
                

            # State 2: Move to pre-incision point--------------------------------------------------
            if self.s2_target_acquired and self.user_execution_command:
                
                # If finished
                if self.s3_pre_incision_reached:
                    self.s2_target_acquired = False
                    self.reset_user_execution_command()


            # State 3: Execute incision------------------------------------------------------------
            if self.s3_pre_incision_reached and self.user_execution_command:

                # If finished
                if self.s4_reverse_active:
                    self.s3_pre_incision_reached = False
                    self.reset_user_execution_command()


            # State 4: Reverse incision------------------------------------------------------------
            if self.s4_reverse_active and self.user_execution_command:
                self.reset_user_execution_command()



            # Call method to check for user input to get execute_next_state
            self.get_user_execution_command()

            rate.sleep()

            # Received new goal pose for CV
            # If old goal pose != new goal pose && !needle_goal_published
            #   then goal_pose_reached = false

            # Received new goal pose for incision
            #   Move to init_pose (for collision avoidance)
            #   Wait for user input (to change needle)
            #   Move to to pre-incision point
            #   Wait
            #   Execute incision



if __name__ == '__main__':

    ProcessManager()
    ProcessManager.main_process()