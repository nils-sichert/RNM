#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from motion_manager import MotionManager

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

        self.act_goal_id            = None
        self.old_goal_id            = None

        # ROS inits
        rospy.init_node('process_manager', anonymous=True)

        # Objects
        self.motion_manager = MotionManager()

        # Load Ros Parameter

        # Ros Publisher
        self.goal_pose_reached_pub = rospy.Publisher("~goal_pose_reached", Bool, queue_size=1)


        # Ros Subscriber
        self.target_acquired_sub = rospy.Subscriber('~/target_acquired', Bool, self.callback_target_acquired)
        self.needle_goal_pose_sub = rospy.Subscriber('~/needle_goal_pose', JointState, self.callback_needle_goal_pose)
        self.goal_pose_sub = rospy.Subscriber('~/goal_pose_cs', JointState, self.callback_goal_pose_js )

    # Callbacks
    def callback_target_acquired(self, msg):
        '''
        Callback function for needle target point: True = needle Target point is published and activ | False = needle Target point is not published or not activ
        '''
        self.s2_target_acquired = msg

    def callback_goal_pose_js(self, msg):
        ''' 
        Callback function for the goal_pose_js topic. Stores current goal pose in object variable 
        '''
        self.s1_cv_ready            = True
        self.goal_pose_js           = msg.position
        self.act_goal_id            = msg.name
        rospy.logwarn("Got Goal_Pose_JS with ID:" + self.act_goal_id)

    def callback_needle_goal_pose(self):
        """
        Callback function for the target pose of the needle containing a location (x,y,z) and a roatation matrice (R); position[0:9] = Rotation & position [9:12] = position
        """
        self.needle_goal_pose = msg.position
    
    # Publish Methods
    def set_goal_pose_reached_pub(self, value):
        """
        Value must be True = position reached | False = position not reached
        """
        msg = value
        self.goal_pose_reached_pub(msg)

    # Getter Methods
    def get_user_execution_command(self):
        ''' 
        '''
        self.user_execution_command = rospy.get_param('~user_execution_command', False)
    
    def get_init_pose(self):
        ''' 
        '''
        self.init_pose = rospy.get_param('~init_pose')

    # Resetter Methods
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
                
                if not self.old_goal_id == self.act_goal_id:
                    self.old_goal_id = self.act_goal_id
                    self.set_goal_pose_reached_pub = False
                    self.motion_manager.go_to_goal_js(self.goal_pose_js) #FIXME Add function
                    self.set_goal_pose_reached_pub = True


                # Do until target acquired
                if self.s2_target_acquired:                 # Is True if VS has published needle goal pose. Will be published True by cv
                    self.motion_manager.move2start(self.get_init_pose)
                                                            # TODO In case of aborting program due to Safety barrier, write all values into file
                    self.s1_cv_ready = False
                    self.reset_user_execution_command()
                    rospy.logwarn("[PM] Has successful completed CV.")
                

            # State 2: Move to pre-incision point--------------------------------------------------
            if self.s2_target_acquired and self.user_execution_command:
                self.s3_pre_incision_reached = self.motion_manager.move_start2preincision(self.needle_goal_pose)  # FIXME Add function, 
                                                                                                # calculate two list with help of needle goal:   
                                                                                                # 1. Start -> Pre-Inj: calculated_trajectory_start2preinc.csv
                                                                                                # 2. Pre-Inj -> Target: calculated_trajectory_preinj2target.csv
                                                                                                # execute motion executor: calculated_trajectory_start2preinc.csv
                                                                                                # return True

                # If finished
                if self.s3_pre_incision_reached:
                    self.s2_target_acquired = False
                    self.reset_user_execution_command()
                    rospy.logwarn("[PM] Has successful completed Movement to Pre-Incision Point.")


            # State 3: Execute incision------------------------------------------------------------
            if self.s3_pre_incision_reached and self.user_execution_command:
                self.s4_reverse_active = self.motion_manager.move_preincision2target()                      # FIXME Add function
                                                                                                # execute motion executor: calculated_trajectory_preinc2target.csv
                                                                                                # return True
                # If finished
                if self.s4_reverse_active:
                    self.s3_pre_incision_reached = False
                    self.reset_user_execution_command()
                    rospy.logwarn("[PM] Has successful completed Incision.")


            # State 4: Reverse incision------------------------------------------------------------
            if self.s4_reverse_active and self.user_execution_command:
                self.motion_manager.move_target2start
                self.reset_user_execution_command()
                rospy.logwarn("[PM] Has successful reversed Robot to inital pose.")


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