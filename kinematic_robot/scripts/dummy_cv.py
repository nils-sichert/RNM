#!/usr/bin/env python3
import numpy as np
import time
import os
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int16

class DummyCV:
    def __init__(self, pose_list_dir):

        # ROS inits
        rospy.init_node('dummy_cv', anonymous=True)
        rospy.logwarn('[dCV] Init started...')
        
        # ROS Subscriber
        self.sub_goal_pose_reached  = rospy.Subscriber('/goal_pose_reached', Int16, self.callback_goal_pose_reached)
        self.sub_user_input_dummy   = rospy.Subscriber('/user_input_dummy', String, self.callback_user_input_dummy)

        # ROS Publisher (do last to signal node ready)
        self.pub_goal_pose_js       = rospy.Publisher('/goal_pose_js', Float64MultiArray, queue_size=1)
        self.pub_needle_goal_pose   = rospy.Publisher('/needle_goal_pose', Float64MultiArray, queue_size=1)

        # Flags
        self.at_desired_goal_pose   = False
        self.go_to_next_pose        = False

        # Misc
        self.joint_state_topic      = rospy.get_param('joint_state_topic', "/joint_states")
        self.pose_list_dir          = pose_list_dir
        self.pose_list              = np.load(os.path.join(os.path.dirname(__file__), pose_list_dir))
        self.curr_pose_id           = 0

        time.sleep(1)
        rospy.logwarn('[dCV] Init finished')


    def callback_user_input_dummy(self, msg):
        ''' This function simply simulates a delayed response from the program
        '''
        self.go_to_next_pose = True
        rospy.logwarn("[dCV] User input received")

    def callback_goal_pose_reached(self, msg : Int16):
        ''' Checks the received confirmation ID and sets at_desired_goal_pose flag accordingly
        '''
        received_id = msg.data
        if received_id == self.curr_pose_id:
            rospy.logwarn(f"[dCV] Received correct confirmation ID {received_id}")
            self.at_desired_goal_pose = True
        else:
            rospy.logwarn(f"[dCV] !ERR! Received incorrect confirmation ID {received_id}")

    def publish_desired_goal_pose(self, pose_js : np.array, pose_id : int):
        ''' Publihes current desired goal pose and current pose id using the goal_pose_js_pub.
            Also sets the self.curr_pose_id to the id that is sent in the message.
            Parameters:
                pose_js (List): Current goal pose in joint space
                pose_id (int): Current goal pose ID
        '''
        self.curr_pose_id   = pose_id

        pose_js = np.append(pose_js, pose_id)
        msg         = Float64MultiArray()
        msg.data    = pose_js

        self.pub_goal_pose_js.publish(msg)

        rospy.logwarn(f"[dCV] Send new goal_pose_js with ID {pose_id}")

    def publish_needle_goal_pose(self, pose):

        msg         = Float64MultiArray()
        msg.data    = pose

        self.pub_needle_goal_pose.publish(msg)
        return

    def get_curr_joint_state(self):
        ''' Get actual current joint states
        '''
        act_curr_pose   = rospy.wait_for_message(self.joint_state_topic, JointState, rospy.Duration(10))
        act_curr_pose   = np.array(act_curr_pose.position)
        return act_curr_pose

    def important_stuff(self):
        ''' This function simulates important stuff which takes time
        '''
        delay = 1
        time.sleep(delay)
        return

    def is_topic_published(self, topic_name : str):
        ''' Checks the rostopic list for the given topic 
        '''
        topics  = rospy.get_published_topics()
        topics  = [topics[i][0] for i in range(len(topics))]
        return (topic_name in topics)


    def do_stuff_with_recorded_poses(self):
      
        # Init process
        self.pose_list       = np.load(os.path.join(os.path.dirname(__file__), self.pose_list_dir))
        self.go_to_next_pose = True
      
        # Wait until process_manager is ready
        rospy.logwarn('[dCV] Waiting for PM...')
        while not self.is_topic_published('/goal_pose_reached'): pass
        rospy.logwarn('[dCV] Topics from PM detected')
        time.sleep(2)
        """
        # Loop over all desired waypoints
        for pose_id in range(0):
            pose = self.pose_list[pose_id]
            # Send next desired goal pose
            self.at_desired_goal_pose   = False
            self.publish_desired_goal_pose(pose, pose_id)

            # Wait to reach desired goal pose
            while not self.at_desired_goal_pose:
                if self.at_desired_goal_pose:
                    break
            
            # Do something important
            #act_curr_pose = self.get_curr_joint_state()
            self.important_stuff()
        """
        
        # Do more important stuff
        self.publish_needle_goal_pose(self.calculate_needle_goal_pose())

        # /TODO: Signal finished calibration/registrtation to start 

    def calculate_needle_goal_pose(self):
        #FIXME actual needle goal pose calculation
        needle_goal_pose  = np.array([ 0.70105746, -0.63595987 , 0.32260423 ,-0.71068701 ,-0.66031765,  0.24270266,  0.05867211, -0.39941914, -0.91488903 , 0.29202788,  0.14586335,  0.62162211])
        
        return needle_goal_pose

if __name__ == '__main__':
    
    dummy_cv = DummyCV('Path/collected_joint_list.npy')
    dummy_cv.do_stuff_with_recorded_poses()
    
    rospy.spin()

