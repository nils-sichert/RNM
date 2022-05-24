#!/usr/bin/python3
from math import cos

import rospy
import sys
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Float64MultiArray


class CRobotArm:
    def __init__(self, move_distance, command_topic):
        """ Initialize C_Robot_Arm
            :param move_distance:
            :param command_topic:
        """

        # Check if in simulation or in real hardware, wait for first message
        if "sim" in command_topic:
            self.joint_state_topic  = "/joint_states"
        else:
            self.joint_state_topic  = "/franka_state_controller/joint_states_desired"

        msg_in = rospy.wait_for_message(self.joint_state_topic, JointState, rospy.Duration(10))

        # Get parameter for object
        self.initial_position   = msg_in.position
        self.move_distance      = move_distance

        # Set up the publisher
        self.publisher = rospy.Publisher(command_topic, Float64MultiArray, queue_size=1)
        self.counter = 0

    def move_to_angles(self, target_angles):
        """ Publishes joint command messages to reach desired target angles. Respects maximum move distance.
            :param target_angles:
            :return: nothing
        """
        msg_in      = rospy.wait_for_message(self.joint_state_topic, JointState, rospy.Duration(10))
        curr_angles = msg_in.position
        next_angles = np.zeros((7, 1))

        # Iterate over joints
        for i in range(7):

            delta   = target_angles[i] - curr_angles[i]
            if delta > self.move_distance:
                delta = self.move_distance
            elif delta < -self.move_distance:
                delta = -self.move_distance

            next_angles[i] = curr_angles[i] + delta

        # Create the message and send it to the advertised topic
        msg_out         = Float64MultiArray()
        msg_out.data    = next_angles
        self.publisher.publish(msg_out)

    def move_to_angles_smoother(self, target_angles):
        """ Publishes joint command messages to reach desired target angles. Respects maximum move distance.
            All joints stop at the same time. The joint with the longest movement takes the max step length, all other
            joints move at a scaled step length.
            :param target_angles:
            :return: nothing
        """
        msg_in      = rospy.wait_for_message(self.joint_state_topic, JointState, rospy.Duration(10))
        curr_angles = msg_in.position
        next_angles = np.zeros((7, 1))

        max_diff    = max([abs(t - c) for t, c in zip(target_angles, curr_angles)])

        # Iterate over joints
        for i in range(7):

            delta   = target_angles[i] - curr_angles[i]
            if delta > self.move_distance:
                delta = self.move_distance * (abs(delta)/max_diff)
            elif delta < -self.move_distance:
                delta = -self.move_distance * (abs(delta)/max_diff)

            next_angles[i] = curr_angles[i] + delta

        # Create the message and send it to the advertised topic
        msg_out         = Float64MultiArray()
        msg_out.data    = next_angles
        self.publisher.publish(msg_out)

    def wiggle_dat_bot(self):
        goal = np.zeros((7, 1))
        delta_angle = (1. - cos(np.pi / 5.0 * self.counter/1000.)) * self.move_distance * np.pi / 180.
        for i in range(7):
            if i == 4:
                goal[i] = self.initial_position[i] - delta_angle
            else:
                goal[i] = self.initial_position[i] + delta_angle
        self.counter += 1

        # Create the message and send it to the advertised topic
        msg_out         = Float64MultiArray()
        msg_out.data    = goal
        self.publisher.publish(msg_out)


def main(argv):
    # Initialize ROS connection
    rospy.init_node("simple_trajectory_player", argv)

    # Read parameters specified in the .launch file
    # The "~" in the beginning means that it will use parameters from the node
    # e.g. /node_name/parameter_name can then be
    # read by rospy.get_param("parameter_name", default)
    move_dist       = rospy.get_param("~joint_move_dist", 10)
    command_topic   = rospy.get_param("~command_topic", "/default_command_topic")

    rospy.logwarn("Using topic " + str(command_topic) + " and distance " + str(move_dist))
    robot_arm = CRobotArm(move_dist, command_topic)

    target_angles_deg = [60, 45, 30, 40, 50, 60, 70]
    target_angles_rad = [v * np.pi / 180 for v in target_angles_deg]

    # Loop infinitely with a fixed frequency of 1000 hz
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #robot_arm.wiggle_dat_bot()
        robot_arm.move_to_angles_smoother(target_angles_rad)
        rate.sleep()


if __name__ == '__main__':
    main(sys.argv)
