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

        # Flags
        # /TODO delete if finally unnecessary
        #self.s0_reset               = True     
        #self.s1_cv_ready            = False
        #self.s3_at_pre_insertion    = False
        #self.s4_reverse_active      = False

        # /TODO currenty placeholder, replace with approriate flags to signal CV node done status
        self.s2_target_acquired     = False	
        
        self.goal_pose_js_filename  = None

        self.crr_goal_pose_id   = None
        self.old_goal_pose_id   = None

        # Internal process flow control------------------------------------------------------------
        self.all_states = [ 's0_start',                 # Exit when PM is ready -> useful if next states should be skipped
                            's1_wait_for_cv',           # Exit when CV nodes ready -> Ready flags set
                            's2_camera_calibration',    # Exit when CC finished -> camera calibrated, result file on disk
                            's3_handeye_calibration',   # Exit when HC finished -> handeye calibrated, result file on disk
                            's4_model_registration',    # Exit when MR finished -> target acquired, target pose on disk
                            's5_pre_insertion',         # Exit when robot at pre insertion pose
                            's6_insertion',             # Exit when robot at insertion pose
                            's7_post_insertion',        # Exit when robot at post insertion pose (reverse to pre insertion pose)
                            's8_end'                    # Exit immediately -> useful for end tasks
                            ]

        self.curr_state = 's0_start'            # Current state
        self.user_execution_command = False     # Confirm next state with user_execution_command
        self.user_next_state        = None      # Ovrride next state with user_next_state

        self.first_run_in_state     = False     # Set to true when state transition in go_to_next_state
                                                # set to false after first run in state in is_first_run_in_state

        # Extenral process flow control------------------------------------------------------------

        # Last send task command
        self.crr_task_command   = None
        
        # Task commands (constant)
        self.TASKCMD_camera_calibration     = "camera_calibration_start"
        self.TASKCMD_handeye_calibration    = "handeye_calibration_start"
        self.TASKCMD_model_registration     = "model_registration_start"
                                            
        # Task finished messages (constant)
        self.TASKFIN_camera_calibration     = "camera_calibration_finished"   
        self.TASKFIN_handeye_calibration    = "handeye_calibration_finished"                  
        self.TASKFIN_model_registration     = "model_registration_finished"

        # Flags for signalling CV processes
        self.CC_finished    = False
        self.HEC_finished   = False
        self.MR_finished    = False

        # Objects----------------------------------------------------------------------------------
        self.motion_manager     = MotionManager(self.joint_command_topic, self.joint_state_topic)

        # ROS inits--------------------------------------------------------------------------------
        rospy.init_node('process_manager', anonymous=True)
        rospy.logwarn('[PM] Init started...')

        # Load ROS Parameter
        self.joint_state_topic  = rospy.get_param('/joint_state_topic')
        self.joint_command_topic= rospy.get_param('/joint_command_topic')
        self.MOVEMENT_SPEED     = rospy.get_param('/movement_speed', 0.1)/1000 # speed of robot endeffector in m/s; /1000 because of updaterate of 1000Hz
        self.INIT_POSE          = np.array([-0.21133107464982753, -0.7980917344176978, 0.5040977626328044, -2.1988260275772613, -0.06275970955855316, 1.4630513990722382, 0.9288285106498062])
             
        # ROS Helper/ Debuging
        self.user_execution_command = rospy.set_param('/user_execution_command', False)

        # ROS Subscriber
        self.sub_needle_goal_pose   = rospy.Subscriber('/needle_goal_pose', Float64MultiArray, self.callback_needle_goal_pose)
        self.sub_goal_pose_js       = rospy.Subscriber('/goal_pose_js', Float64MultiArray, self.callback_goal_pose_js )
        self.sub_task_finished      = rospy.Subscriber('/task_finished', Float64MultiArray, self.callback_goal_pose_js )

        # ROS Publisher (do this last to signal that PM node is ready)
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
    
    def callback_task_finished(self, msg : String):
        ''' Reads task_finished messeges and sets appropriate flags to True'''
        msg     = msg.data
        rospy.logwarn(f'[PM] Received task_finished message "{msg}"')
        
        if msg == self.TASKFIN_camera_calibration:
            self.CC_finished = True
        if msg == self.TASKFIN_handeye_calibration:
            self.HEC_finished = True
        if msg == self.TASKFIN_model_registration:
            self.MR_finished = True

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
        assert state_name in self.all_states, f'state_name "{state_name}" not found in all_states'
        return self.curr_state == state_name

    def is_first_run_in_state(self):
        ''' Returns if this is the first run in the current state, such that some tasks can
            be activated only once.
        '''
        if self.first_run_in_state:
            self.first_run_in_state = False
            return True
        else:
            return False        

    def go_to_next_state(self):
        ''' Reads curr_state and sets next_state according to all_states 
            Then waits for user input to confirm next state or overrule
        '''
        
        # Check if state is in all_states, get state index
        curr_state  = self.curr_state
        assert curr_state in self.all_states, f'curr_state "{curr_state}" not found in all_states'
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

            # Update user inputs
            self.get_user_execution_command()
            self.get_user_next_state()
        
        # Assign curr_state to next_state and continue PM
        self.curr_state = next_state
        self.do_once_in_state = True
        rospy.logwarn(f'[PM] Go to state = {self.curr_state}')


    def main_process(self):

        rospy.logwarn("[PM] Started main process")

        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            # State transitions are triggered by outside flags and user inputs

            # S0 Start-----------------------------------------------------------------------------
            # Go directly to next state. This allows to do init process and accept user input at start
            if self.is_in_state('s0_start'):
                self.go_to_next_state()

            
            # S1 Wait for CV-----------------------------------------------------------------------
            # Wait until CV nodes are ready (non blocking)
            # /TODO this is currently done by only checking if /goal_pose_js topic is published
            # /TODO this is not sufficient to test if ALL CV nodes are ready
            if self.is_in_state('s1_wait_for_cv'):

                if self.is_topic_published('/goal_pose_js'):
                    rospy.logwarn("[PM] Topics from CV detected")
                    self.go_to_next_state()

            
            # S2 Do camera calibration-------------------------------------------------------------
            # Start camera calibration, go to goal poses, wait for CC results (non blocking)
            if self.is_in_state('s2_camera_calibration'):
                
                if self.is_first_run_in_state():
                    self.publish_task_command(self.TASKCMD_camera_calibration)
                
                if not self.old_goal_pose_id == self.crr_goal_pose_id:
                    self.old_goal_pose_id = self.crr_goal_pose_id
                    self.motion_manager.move2goal_js(self.goal_pose_js, self.MOVEMENT_SPEED)   
                    self.publish_goal_pose_reached(self.crr_goal_pose_id)

                # Exit state - when camera is calibrated
                if self.CC_finished:
                    rospy.logwarn("[PM] Camera calibration finished")
                    #self.motion_manager.move2goal_js(self.INIT_POSE, self.MOVEMENT_SPEED)
                    self.go_to_next_state()


            # S3 Do Hand eye calibration-----------------------------------------------------------
            # Start Hand eye calibration, go to goal poses, wait for HEC results (non blocking)
            if self.is_in_state('s3_handeye_calibration'):
                
                if self.is_first_run_in_state():
                    self.publish_task_command(self.TASKCMD_handeye_calibration)
                
                if not self.old_goal_pose_id == self.crr_goal_pose_id:
                    self.old_goal_pose_id = self.crr_goal_pose_id
                    self.motion_manager.move2goal_js(self.goal_pose_js, self.MOVEMENT_SPEED)   
                    self.publish_goal_pose_reached(self.crr_goal_pose_id)

                # Exit state - when hand eye is calibrated
                if self.HEC_finished:
                    rospy.logwarn("[PM] Hand eye calibration finished")
                    #self.motion_manager.move2goal_js(self.INIT_POSE, self.MOVEMENT_SPEED)
                    self.go_to_next_state()


            # S4 Do Model registrtaion-------------------------------------------------------------
            # Start model registrtation, go to goal poses, wait for MR results / target acquired (non blocking)
            if self.is_in_state('s4_model_registration'):
                
                if self.is_first_run_in_state():
                    self.publish_task_command(self.TASKCMD_model_registration)
                
                if not self.old_goal_pose_id == self.crr_goal_pose_id:
                    self.old_goal_pose_id = self.crr_goal_pose_id
                    self.motion_manager.move2goal_js(self.goal_pose_js, self.MOVEMENT_SPEED)   
                    self.publish_goal_pose_reached(self.crr_goal_pose_id)

                # Exit state - when model registrtation is done and target is acquired
                if self.MR_finished:
                    rospy.logwarn("[PM] Model registration finished")
                    self.go_to_next_state()


            # S5 Move to pre-insertion pose--------------------------------------------------------
            # First move robot to initial pose then move robot to pre-insertion pose (blocking)
            if self.is_in_state('s5_pre_insertion'):
                self.motion_manager.move2goal_js(self.INIT_POSE, self.MOVEMENT_SPEED)
                motion_done = self.motion_manager.move_start2preinsertion(self.needle_goal_pose, self.MOVEMENT_SPEED) 
                
                # Exit state - when robot at pre insertion pose
                if motion_done:
                    rospy.logwarn("[PM] Successfully completed movement to pre-insertion point")
                    self.go_to_next_state()


            # S6 Execute insertion-----------------------------------------------------------------
            # Insert needle to target, exit when at pose (blocking)
            if self.is_in_state('s6_insertion'):
                motion_done = self.motion_manager.move_preinsertion2target(self.MOVEMENT_SPEED)                     

                # Exit state - when robot at insertion pose
                if motion_done:
                    rospy.logwarn("[PM] Needle inserted")
                    self.go_to_next_state()


            # S7 Post insertion--------------------------------------------------------------------
            # Reverse needle insertion, exit when at pose (blocking)
            # /TODO replace motion function with correct "reverse needle" function
            if self.is_in_state('s7_post_insertion'):
                self.motion_manager.move_target2init_pose() # /FIXME Add function
                motion_done = True                          # /FIXME Get result from motion function

                # Exit state - when robot at post insertion pose
                if motion_done:
                    rospy.logwarn("[PM] Successfully reversed robot to inital pose")
                    self.go_to_next_state()


            # S8 End state-------------------------------------------------------------------------
            # Do end tasks, finish state and await new user commands (non blocking)
            if self.is_in_state('s8_end'):
                self.go_to_next_state()


            # Looping tasks------------------------------------------------------------------------
            rate.sleep()



if __name__ == '__main__':

    process_manager = ProcessManager()
    process_manager.main_process()