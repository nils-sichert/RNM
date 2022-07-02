#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int16
from std_msgs.msg import String
from motion_manager import MotionManager
import time
import numpy as np

# TODO State 4 need to be implemented

class ProcessManager:

    def __init__(self):

        # Task commands
        self.taskcmd_camera_calibration = "camera_calibration_start"
        # TODO have more taskcmds for other CV nodes

        # Flags
        self.s0_reset               = True
        self.s1_cv_ready            = False
        self.s2_target_acquired     = False	
        self.s3_at_pre_insertion    = False
        self.s4_reverse_active      = False

        
        self.goal_pose_js_filename  = None

        self.crr_goal_pose_id   = None
        self.old_goal_pose_id   = None

        self.crr_task_command   = None

        # Process flow
        self.all_states = [ 's0_start',                 # Exit when PM is ready -> useful if next states should be skipped
                            's1_wait_for_cv',           # Exit when CV nodes ready -> Ready flags set
                            's2_camera_calibration',    # Exit when CC finished -> camera calibrated, result file on disk
                            's3_handeye_calibration',   # Exit when HC finished -> handeye calibrated, result file on disk
                            's4_model_registration',    # Exit when MR finished -> target acquired, target pose on disk
                            's5_pre_insertion',         # Exit when robot at pre insertion pose
                            's6_insertion',             # Exit when robot at insertion pose
                            's7_post_insertion',        # Exit when robot at post insertion pose (reverse to pre insertion pose)
                            's8_end',                   # Never exit unless user command
                            'sX_wait_for_user'
                            ]

        self.curr_state = 's0_start'            # Current state
        self.user_execution_command = False     # Confirm next state with user_execution_command
        self.user_next_state        = None      # Ovrride next state with user_next_state


        # ROS inits
        rospy.init_node('process_manager', anonymous=True)
        rospy.logwarn('[PM] Init started...')

        # Load ROS Parameter
        self.joint_state_topic  = rospy.get_param('/joint_state_topic')
        self.joint_command_topic= rospy.get_param('/joint_command_topic')
        self.MOVEMENT_SPEED     = rospy.get_param('/movement_speed', 0.1)/1000 # speed of robot endeffector in m/s; /1000 because of updaterate of 1000Hz
        self.INIT_POSE          = np.array([-0.21133107464982753, -0.7980917344176978, 0.5040977626328044, -2.1988260275772613, -0.06275970955855316, 1.4630513990722382, 0.9288285106498062])
        
        # Objects
        self.motion_manager     = MotionManager(self.joint_command_topic, self.joint_state_topic)
       
        # Ros Helper/ Debuging
        self.user_execution_command = rospy.set_param('/user_execution_command', False)

        # ROS Subscriber
        self.sub_needle_goal_pose   = rospy.Subscriber('/needle_goal_pose', Float64MultiArray, self.callback_needle_goal_pose)
        self.sub_goal_pose_js       = rospy.Subscriber('/goal_pose_js', Float64MultiArray, self.callback_goal_pose_js )

        # ROS Publisher (do last to signal node ready)
        self.pub_task_command       = rospy.Publisher('/task_command', String, queue_size=1)      
        self.pub_goal_pose_reached  = rospy.Publisher('/goal_pose_reached', Int16, queue_size=1)

        time.sleep(1)
        rospy.logwarn('[PM] Init finished')

    # Callbacks
    def callback_goal_pose_js(self, msg : Float64MultiArray):
        ''' Callback function for the goal_pose_js topic. Stores current goal pose in object variable 
            and sets the s2_target_acquired flag = True 
        '''
        self.s2_target_acquired     = False
        self.goal_pose_js           = msg.data[0:-1]
        self.crr_goal_pose_id       = int(msg.data[-1])
        rospy.logwarn("[PM] Got Goal_Pose_JS with ID:" + str(self.crr_goal_pose_id)) 

    def callback_needle_goal_pose(self, msg : Float64MultiArray):
        ''' Callback function for the target pose of the needle containing a location (x,y,z) and 
            a roatation matrice (R); position[0:9] = Rotation & position [9:12] = position
        '''
       
        self.s2_target_acquired     = True
        self.needle_goal_pose = msg.data
    
    # Publish Methods
    def publish_goal_pose_reached(self, goal_pose_id : int):
        ''' Publishes the confirmation ID of a reached goal pose
        '''
        msg      = Int16()
        msg.data = goal_pose_id
        self.pub_goal_pose_reached.publish(msg)

    def publish_task_command(self, task_command : str):
        ''' Publishes the current task command
        '''
        self.crr_task_command = task_command
        msg = String()
        msg.data = task_command
        rospy.logwarn(f'[PM] Sending task_command: {task_command}')
        self.pub_task_command.publish(msg)

    # Getter Methods
    def get_user_execution_command(self):
        ''' Read user execution command from rosparam
        '''
        new_state = rospy.get_param('/user_execution_command', False)
        if not self.user_execution_command and new_state:
            rospy.logwarn("[PM] Received user execution command")
        self.user_execution_command = new_state
   
    def get_user_next_state(self):
        ''' Read user next state from rosparam
        '''
        new_user_next_state = rospy.get_param('/user_next_state', 'None')
        if self.user_next_state == 'None' and not new_user_next_state == 'None':
            rospy.logwarn(f"[PM] Received user next state: {new_user_next_state}")
        self.user_next_state = new_user_next_state

    def is_topic_published(self, topic_name : str):
        ''' Checks the rostopic list for the given topic 
        '''
        topics  = rospy.get_published_topics()
        topics  = [topics[i][0] for i in range(len(topics))]
        return (topic_name in topics)

    # Resetter Methods
    def reset_user_execution_command(self):
        ''' Resets the user_execution_command
        '''
        self.user_execution_command = rospy.set_param('/user_execution_command', False)

    def reset_user_next_state(self):
        ''' Resets the user_next_state
        '''
        self.user_next_state = rospy.set_param('/user_next_state', 'None')

    # Process flow control methods
    def is_in_state(self, state_name : str):
        ''' Returns bool if process_manager is in state_name state '''
        return self.curr_state == state_name

    def go_to_next_state(self):
        ''' Reads curr_state and sets next_state according to all_states 
            Then waits for user input to confirm next state or overrule
        '''
        
        # Check if state is in all_states, get state index
        curr_state  = self.curr_state
        assert curr_state in self.all_states, 'curr_state not found in all_states'
        state_idx   = self.all_states.index(curr_state)

        # Set next state. Set to none, if last state has been reached
        if state_idx == len(self.all_states) - 1:
            rospy.logwarn("[PM] Reached end state, no next state specified")
            next_state = None
        else:
            next_state  = self.all_states[state_idx + 1]

        rospy.logwarn(f"[PM] curr_state = {self.curr_state} --> next_state = {next_state}")

        # Wait for user confirmation or user specified state
        rospy.logwarn("[PM] Waiting for user input...")
        while True:
            # Confirm next state selection with user_execution_command 
            if self.user_execution_command:
                self.reset_user_execution_command()
                break

            # Overrule next state selection with user_next_state command
            if not self.user_next_state == 'None':
                next_state = self.user_next_state
                self.reset_user_next_state()
                break

            self.get_user_execution_command()
            self.get_user_next_state()
        
        # Assign curr_state to next_state and continue PM
        self.curr_state = next_state
        rospy.logwarn(f'[PM] Go to state = {self.curr_state}')



    def main_process(self):

        rospy.logwarn("[PM] Started main process")

        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            # State transitions are triggered by outside flags and user inputs

            # S0 Start-----------------------------------------------------------------------------
            # Go directly to next state. This allows to do init process and accept user input at start
            if self.is_in_state('s0_start'):
                pass


            # S0 Wait for CV-----------------------------------------------------------------------
            if self.is_in_state('s0_wait_for_cv'):

                if self.is_topic_published('/goal_pose_js'):
                    self.s0_reset    = False
                    self.s1_cv_ready = True
                    self.reset_user_execution_command()
                    rospy.logwarn("[PM] s0 -> s1 Topics from CV detected")
                    rospy.logwarn("[PM] Waiting for user execution command...")

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
                self.publish_task_command(self.taskcmd_camera_calibration)  # TODO: only publish once
                
                if not self.old_goal_pose_id == self.crr_goal_pose_id:
                    self.old_goal_pose_id = self.crr_goal_pose_id
                    self.motion_manager.move2goal_js(self.goal_pose_js, self.MOVEMENT_SPEED)   
                    self.publish_goal_pose_reached(self.crr_goal_pose_id)                   #FIXME Package int in int16 type


                # Exit State - when target acquired
                if self.s2_target_acquired:                 # Is True if VS has published needle goal pose
                    rospy.logwarn("[PM] Target acquired, moving to inital pose.")
                    self.motion_manager.move2goal_js(self.INIT_POSE, self.MOVEMENT_SPEED)
                    
                    self.s1_cv_ready = False
                    self.reset_user_execution_command()
                    rospy.logwarn("[PM] s1 -> s2 CV has successfully acquired target")
                    rospy.logwarn("[PM] Waiting for user execution command...")
                    # TODO In case of aborting program due to Safety barrier, write all values into file

            # State 2: Move to pre-insertion point--------------------------------------------------
            if self.s2_target_acquired and self.user_execution_command:
                self.s3_at_pre_insertion = self.motion_manager.move_start2preinsertion(self.needle_goal_pose, self.MOVEMENT_SPEED) 
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
                self.s4_reverse_active = self.motion_manager.move_preinsertion2target(self.MOVEMENT_SPEED)                     
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
                self.motion_manager.move_target2init_pose()  #FIXME Add function
                self.reset_user_execution_command()
                rospy.logwarn("[PM] s4 -> s0 Successfully reversed robot to inital pose")
                rospy.logwarn("[PM] Waiting for user execution command...")

                # Exit State


            # Looping tasks------------------------------------------------------------------------

            # Call method to check for user input to get execute_next_state
            self.get_user_execution_command()
            rate.sleep()



if __name__ == '__main__':

    process_manager = ProcessManager()
    process_manager.main_process()