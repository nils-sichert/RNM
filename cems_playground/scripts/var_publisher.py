#!/usr/bin/python3
from math import cos

""" var_publisher.py is used mainly for developing and debugging. Variable messages can be published here.
"""

import rospy
import sys
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Float64MultiArray


def main(argv):
    # Initialize ROS connection
    rospy.init_node("variable_publisher", argv)
    pub     = rospy.Publisher('/target_angles', Float64MultiArray, queue_size=1)

    target_angles_deg = [60, 45, 30, 40, 50, 60, 70]
    target_angles_rad = [v * np.pi / 180 for v in target_angles_deg]

    rate    = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(target_angles_rad)
        rate.sleep()


# Start script
if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass