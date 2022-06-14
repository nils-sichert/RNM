#!/usr/bin/python3
from math import cos

import rospy
import sys
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Float64MultiArray


class CRobotArm:
    def __init__(self, move_distance, command_topic, joint_state_topic):
        """ TODO: Docstring """

        # Get parameter for object, wait for joint_state_topic to publish
        self.joint_state_topic  = joint_state_topic
        self.current_angles     = rospy.wait_for_message(self.joint_state_topic, JointState, rospy.Duration(10)).position
        self.move_distance      = move_distance

        # Set up the publishers and subscribers
        self.publisher  = rospy.Publisher(command_topic, Float64MultiArray, queue_size=20000)
        self.subscriber = rospy.Subscriber(name=joint_state_topic, data_class=JointState, callback=self.callback_joint_states, queue_size=10)

    def callback_joint_states(self, msg_in : JointState):
        """ Callback function for the joint_state_topic. Stores current angles in object variable"""
        self.current_angles = msg_in.position

    def move_to_angles(self, target_angles):
        """ Publishes joint command messages to reach desired target angles. Respects maximum move distance.
            :param target_angles:
            :return: nothing
        """
        #msg_in      = rospy.wait_for_message(self.joint_state_topic, JointState, rospy.Duration(10))
        curr_angles = self.current_angles
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

        curr_angles = self.current_angles
        next_angles = np.zeros((7, 1))

        max_diff    = max([abs(t - c) for t, c in zip(target_angles, curr_angles)])

        # Iterate over joints and assign next_angles
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

    def test_moveList(self):
        curr_angles = self.current_angles
        next_angles = np.zeros((1000, 7))

        joint_1     = np.linspace(curr_angles[0], -80/180 * np.pi, 1000)
        joint_2     = np.linspace(curr_angles[1], curr_angles[1], 1000)
        joint_3     = np.linspace(curr_angles[2], curr_angles[2], 1000)
        joint_4     = np.linspace(curr_angles[3], curr_angles[3], 1000)
        joint_5     = np.linspace(curr_angles[4], curr_angles[4], 1000)
        joint_6     = np.linspace(curr_angles[5], curr_angles[5], 1000)
        joint_7     = np.linspace(curr_angles[6], curr_angles[6], 1000)

        move_list   = np.zeros((7 * 1000))
        for i in range(1000):
            move_list[7 * i + 0]    = joint_1[i]
            move_list[7 * i + 1]    = joint_2[i]
            move_list[7 * i + 2]    = joint_3[i]
            move_list[7 * i + 3]    = joint_4[i]
            move_list[7 * i + 4]    = joint_5[i]
            move_list[7 * i + 5]    = joint_6[i]
            move_list[7 * i + 6]    = joint_7[i]


        rospy.logwarn(f"move_list created with {len(move_list)} entries")

        # Create the message and send it to the advertised topic
        msg_out         = Float64MultiArray()
        msg_out.data    = move_list
        self.publisher.publish(msg_out)


def main(argv):
    # Initialize ROS connection
    # TODO: Where does this node name appear? Is the name overridden by the launch file?
    rospy.init_node("simple_trajectory_player", argv)
    # TODO: listener and publisher inint could go here

    # Read parameters specified in the .launch file
    # The "~" in the beginning means that it will use parameters from the node
    # e.g. /node_name/parameter_name can then be
    # read by rospy.get_param("parameter_name", default)
    move_dist       = rospy.get_param("~joint_move_dist", 10)
    command_topic   = rospy.get_param("~command_topic", "/default_command_topic")

    # Check if in simulation or in real hardware, set joint_state_topic accordingly
    if "sim" in command_topic:
        joint_state_topic  = "/joint_states"
    else:
        joint_state_topic  = "/franka_state_controller/joint_states_desired"

    rospy.logwarn("Using topic " + str(command_topic) + " and distance " + str(move_dist))
    robot_arm = CRobotArm(move_dist, command_topic, joint_state_topic)

    target_angles_deg = [-60, 45, -30, 40, 50, 60, 70]
    target_angles_rad = [v * np.pi / 180 for v in target_angles_deg]

    #robot_arm.move_to_angles_smoother(target_angles_rad)

    # Loop infinitely with a fixed frequency of 1000 hz
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        #robot_arm.wiggle_dat_bot()
        robot_arm.move_to_angles_smoother(target_angles_rad)
        rate.sleep()


if __name__ == '__main__':
    main(sys.argv)
