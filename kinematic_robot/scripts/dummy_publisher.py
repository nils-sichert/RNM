#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import JointState


def main():
    
    # Init
    rospy.init_node("dummy_publisher")

    publish_joint_states = rospy.Publisher(name="/joint_states", data_class=JointState, queue_size=20)

    message = JointState()

    message.name     = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    message.position = [np.deg2rad(10), np.deg2rad(5), np.deg2rad(0), np.deg2rad(10), np.deg2rad(5), np.deg2rad(0),np.deg2rad(10)]
    message.velocity = [0, 0, 0, 0, 0, 0, 0]


    # Loop
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
    
        publish_joint_states.publish(message)

        rate.sleep()
    

if __name__ == '__main__':
    
    main()
