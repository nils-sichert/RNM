#!/usr/bin/python3

import rospy
from sensor_msgs.msg import JointState


def callback(msg : JointState):
    joint_pos   = msg.position
    print(joint_pos)


def listener():
    # Init node
    rospy.init_node('listener', anonymous=True)

    # Get parameters

    # Subscribe to topic
    rospy.Subscriber(name="/joint_states", data_class=JointState, callback=callback, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    listener()
