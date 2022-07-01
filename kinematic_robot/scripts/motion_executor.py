#!/usr/bin/env python3
import rospy
import os
from std_msgs.msg import Float64MultiArray
import csv
import sys
import numpy as np

class MotionExecutor:
    def __init__(self, command_topic, robot_kinematics):
        
    
        self.pub     = rospy.Publisher(command_topic, Float64MultiArray, queue_size=1)
        self.robot_kinematics = robot_kinematics

    def run(self, filename, current_pose, MOVEMENT_SPEED):
        self.get_joint_list(filename)
        self.move_to_start(current_pose, MOVEMENT_SPEED)
        self.publish_joint()
        
    def get_joint_list(self, filename):
        self.publish_list = []
        with open((os.path.join(os.path.dirname(__file__),filename))) as f:
                        reader = csv.reader(f, delimiter=",")
                        for row in reader:
                            self.publish_list.append([float(row[0]),float(row[1]),float(row[2]),float(row[3]),float(row[4]),float(row[5]),float(row[6])])
        rospy.logwarn("Got Joint List")
        return 0

    def move_to_start(self, current_pose, MOVEMENT_SPEED):
        """
        Moves robot in desired start position. To get there a linear path between the current joint state and the target joint state 
        will be interpolatet. The movment speed is be set and should not be above 0.02 m/s.
        """
        #FIXME mit trajectory_planner kombinieren -> Methode schreiben
        #FIXME fängt nicht komplett auf Start an (Z-Koordiante??)

        # get current cartesian robot state
        current_pose_cartesian = self.robot_kinematics.get_pose_from_angles(current_pose)

        # get target cartesian robot state
        start_pose = self.robot_kinematics.get_pose_from_angles(self.publish_list[0])     # /FIXME confirm correction and error handling if List is empty
        delta_pose = start_pose - current_pose_cartesian
        err_tol = 1e-2
        if np.abs(delta_pose).max() > err_tol:
            # calculate distance between cartesian coordinates of current and start position
            dist = np.linalg.norm(start_pose[9:12] -  current_pose[9:12])

            # divide distance by the movement speed to calculate number of nessesary interpolations to reach the movement speed during an updaterate of 1000 Hz.
            steps = int(dist/MOVEMENT_SPEED)
            delta_joints_per_step = (self.publish_list[0] - current_pose)/steps
            
            # set Updaterate to 1000 Hz and publish every 1ms a new joint state
            rate    = rospy.Rate(1000)
            for j in range(steps+1):

                joint = current_pose + j*delta_joints_per_step
                msg = Float64MultiArray()
                msg.data = joint
                self.pub.publish(msg)
                rate.sleep()
            rospy.logwarn("Moved to Start-Point")   
        else:
            rospy.logwarn("Be on start position")
        
        return

    def publish_joint(self):
        """
        /TODO write comment
        """
        # publish new joint every 1ms
        rate    = rospy.Rate(1000)

        rospy.logwarn("Start publishing joints.")
        for i in range(len(self.publish_list)):
            
            joint = self.publish_list[i]
            msg = Float64MultiArray()
            msg.data = joint
            self.pub.publish(msg)
            rate.sleep()
        
        rospy.logwarn("Published all Joints.")
        
        
# for testing purpose
def main(argv):
    rospy.init_node("variable_publisher")
    command_topic = rospy.get_param("~command_topic", "/joint_position_example_controller_sim/joint_command")
    rospy.logwarn("Using topic: " + str(command_topic))

    MotionExecution = MotionExecutor(command_topic)
    MotionExecution.run()

if __name__ == '__main__':
    main(sys.argv)

