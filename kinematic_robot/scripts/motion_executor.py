#!/usr/bin/env python3
import rospy
import os
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import csv
import sys
from robot_kinematics import robot_kinematics
import numpy as np

class MotionExecutor:
    def __init__(self, command_topic):
        
        if "sim" in command_topic:
            msg = rospy.wait_for_message("/joint_states", JointState, rospy.Duration(10))
        else:
            msg = rospy.wait_for_message("/franka_state_controller/joint_states_desired", JointState, rospy.Duration(10))
            # joint_states_desired is correct

        self.pub     = rospy.Publisher(command_topic, Float64MultiArray, queue_size=1)
        self.initial_joints = np.array(msg.position)
        self.publish_list = []
        self.move2start_list = []
        self.movement_speed = 0.02/1000
        self.robot_kinematics = robot_kinematics()

    def run(self):
        self.get_joint_list()
        self.move_to_start()
        self.publish_joint()
        
    def get_joint_list(self):
        with open((os.path.join(os.path.dirname(__file__),"trajectory/calculated_trajectory_simple_1ms.csv"))) as f:
                        reader = csv.reader(f, delimiter=",")
                        for row in reader:
                            self.publish_list.append([float(row[0]),float(row[1]),float(row[2]),float(row[3]),float(row[4]),float(row[5]),float(row[6])])
        rospy.logwarn("Got Joint List")
        return 0

    def move_to_start(self):
        #FIXME mit trajectory_planner kombinieren -> Methode schreiben
        current_pose = self.robot_kinematics.get_pose_from_angles(self.initial_joints)
        start_pose = self.robot_kinematics.get_pose_from_angles(self.publish_list[0])     # /FIXME confirm correction
        start_join = self.publish_list[0]
        dist = np.linalg.norm(start_pose[9:12] -  current_pose[9:12])
        steps = int(dist/self.movement_speed)
        delta_joints_per_step = (self.publish_list[0] - self.initial_joints)/steps
        rate    = rospy.Rate(1000)
        tmp_list = []
        for j in range(steps+1):

            joint = self.initial_joints + j*delta_joints_per_step
            msg = Float64MultiArray()
            msg.data = joint
            self.pub.publish(msg)
            rate.sleep()
        rospy.logwarn("Moved to Start-Point")

    def publish_joint(self):
        # publish new joint every 1ms
        rate    = rospy.Rate(1000)
        for i in range(len(self.publish_list)):
            
            joint = self.publish_list[i]
            msg = Float64MultiArray()
            msg.data = joint
            self.pub.publish(msg)
            rate.sleep()
        
        rospy.logwarn("Published all Joints")
        rospy.logwarn("repeat last join:" + str(joint))
        while not rospy.is_shutdown():
            self.pub.publish(msg)
            rate.sleep()



def main(argv):
    rospy.init_node("variable_publisher")
    command_topic = rospy.get_param("~command_topic", "/joint_position_example_controller_sim/joint_command")
    rospy.logwarn("Using topic: " + str(command_topic))

    MotionExecution = MotionExecutor(command_topic)
    MotionExecution.run()

if __name__ == '__main__':
    main(sys.argv)

