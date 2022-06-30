#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int16
from motion_manager import MotionManager
import time

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
        self.s3_at_pre_insertion    = False
        self.s4_reverse_active      = False

        self.user_execution_command = False
        self.goal_pose_js_filename  = None

        self.crr_goal_pose_id       = None
        self.old_goal_pose_id       = None

        # ROS inits
        rospy.init_node('process_manager', anonymous=True)

        # Objects
        self.motion_manager = MotionManager()

        # Load ROS Parameter
        self.MOVEMENT_SPEED = rospy.get_param("~/movement_speed", 0.01)/1000 # speed of robot endeffector in m/s; /1000 because of updaterate of 1000Hz

        # ROS Publisher
        self.pub_goal_pose_reached  = rospy.Publisher("~goal_pose_reached", Int16, queue_size=1)

        # ROS Subscriber
        self.sub_needle_goal_pose   = rospy.Subscriber('~/needle_goal_pose', Float64MultiArray, self.callback_needle_goal_pose)
        self.sub_goal_pose_js       = rospy.Subscriber('~/goal_pose_js', Float64MultiArray, self.callback_goal_pose_js )

        time.sleep(1)

    # Callbacks
    def callback_goal_pose_js(self, msg : Float64MultiArray):
        ''' Callback function for the goal_pose_js topic. Stores current goal pose in object variable 
            and sets the s2_target_acquired flag = True 
        '''
        self.s2_target_acquired     = True
        self.goal_pose_js           = msg.data[0:7]
        self.crr_goal_pose_id       = msg.data[8]
        rospy.logwarn("[PM] Got Goal_Pose_JS with ID:" + self.crr_goal_pose_id)

    def callback_needle_goal_pose(self, msg : JointState):
        ''' Callback function for the target pose of the needle containing a location (x,y,z) and 
            a roatation matrice (R); position[0:9] = Rotation & position [9:12] = position
        '''
        # /TODO: received message will probably be in the same format as in callback_goal_pose_js
        self.needle_goal_pose = msg.position
    
    # Publish Methods
    def pub_goal_pose_reached(self, goal_pose_id : int):
        ''' Publishes the confirmation ID of a reached goal pose
        '''
        msg      = Int16()
        msg.data = goal_pose_id
        self.pub_goal_pose_reached(msg)

    # Getter Methods
    def get_user_execution_command(self):
        ''' /TODO
        '''
        self.user_execution_command = rospy.get_param('~user_execution_command', False)
    
    def get_init_pose(self):
        ''' /TODO
        '''
        self.init_pose = rospy.get_param('~init_pose')

    def is_topic_published(self, topic_name : str):
        ''' Checks the rostopic list for the given topic 
        '''
        topics  = rospy.get_published_topics()
        topics  = [topics[i][0] for i in range(len(topics))]
        return (topic_name in topics)

    # Resetter Methods
    def reset_user_execution_command(self):
        ''' /TODO
        '''
        rospy.set_param('~user_execution_command', False)


    def main_process(self):

        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            # State transitions are triggered by outside flags and user inputs

            # State 0: Reset state-----------------------------------------------------------------
            if self.s0_reset:
                self.s4_reverse_active = False

                # Exit State - when CV published topics 
                if self.is_topic_published('/goal_pose_js'):
                    self.s0_reset    = False
                    self.s1_cv_ready = True
                    self.reset_user_execution_command()
                    rospy.logwarn("[PM] s0 -> s1 Topics from CV detected")
                    rospy.logwarn("[PM] Waiting for user execution command...")


            # State 1: Camera Calibration and Target Acquisition-----------------------------------
            if self.s1_cv_ready and self.user_execution_command:
                
                if not self.old_goal_pose_id == self.crr_goal_pose_id:
                    self.old_goal_pose_id = self.crr_goal_pose_id
                    self.motion_manager.move2goal_js(self.goal_pose_js, self.MOVEMENT_SPEED)    #FIXME Add function
                    self.pub_goal_pose_reached(self.crr_goal_pose_id)


                # Exit State - when target acquired
                if self.s2_target_acquired:                 # Is True if VS has published needle goal pose
                    self.motion_manager.move2goal_js(self.init_pose, self.MOVEMENT_SPEED)
                                                            # TODO In case of aborting program due to Safety barrier, write all values into file
                    self.s1_cv_ready = False
                    self.reset_user_execution_command()
                    rospy.logwarn("[PM] s1 -> s2 CV has successfully acquired target")
                    rospy.logwarn("[PM] Waiting for user execution command...")
                

            # State 2: Move to pre-insertion point--------------------------------------------------
            if self.s2_target_acquired and self.user_execution_command:
                self.s3_at_pre_insertion = self.motion_manager.move_start2preinsertion(self.needle_goal_pose, self.MOVEMENT_SPEED)  # FIXME Add function, 
                                                                                                # calculate two list with help of needle goal:   
                                                                                                # 1. Start -> Pre-Inj: calculated_trajectory_start2preinc.csv
                                                                                                # 2. Pre-Inj -> Target: calculated_trajectory_preinj2target.csv
                                                                                                # execute motion executor: calculated_trajectory_start2preinc.csv
                                                                                                # return True
                
                # Exit State
                if self.s3_at_pre_insertion:
                    self.s2_target_acquired = False
                    self.reset_user_execution_command()
                    rospy.logwarn("[PM] s2 -> s3 Successfully completed movement to pre-insertion point")
                    rospy.logwarn("[PM] Waiting for user execution command...")


            # State 3: Execute insertion------------------------------------------------------------
            if self.s3_at_pre_insertion and self.user_execution_command:
                self.s4_reverse_active = self.motion_manager.move_preinsertion2target()                      # FIXME Add function
                                                                                                # execute motion executor: calculated_trajectory_preinc2target.csv
                                                                                                # return True
                # Exit State
                if self.s4_reverse_active:
                    self.s3_at_pre_insertion = False
                    self.reset_user_execution_command()
                    rospy.logwarn("[PM] s3 -> s4 Successfully completed needle insertion")
                    rospy.logwarn("[PM] Waiting for user execution command...")


            # State 4: Reverse insertion------------------------------------------------------------
            if self.s4_reverse_active and self.user_execution_command:
                self.motion_manager.move_target2start
                self.reset_user_execution_command()
                rospy.logwarn("[PM] s4 -> s0 Successfully reversed robot to inital pose")
                rospy.logwarn("[PM] Waiting for user execution command...")

                # Exit State


            # Looping tasks------------------------------------------------------------------------

            # Call method to check for user input to get execute_next_state
            self.get_user_execution_command()
            rate.sleep()

            # Received new goal pose for CV
            # If old goal pose != new goal pose && !needle_goal_published
            #   then goal_pose_reached = false

            # Received new goal pose for insertion
            #   Move to init_pose (for collision avoidance)
            #   Wait for user input (to change needle)
            #   Move to to pre-insertion point
            #   Wait
            #   Execute insertion



if __name__ == '__main__':

    ProcessManager()
    ProcessManager.main_process()