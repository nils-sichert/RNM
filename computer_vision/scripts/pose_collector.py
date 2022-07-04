#!/usr/bin/env python

import rospy
import cv2 as cv
import sys
from sensor_msgs.msg import JointState, PointCloud2, PointField
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy.linalg import inv
import os
import time
import ctypes
import struct


import os



class pose_collector():


    def __init__(self):

        self.MR_path_dataset = os.path.join(os.path.dirname(__file__), 'CV_model_registration_data/dataset001')
        self.CC_path_dataset = os.path.join(os.path.dirname(__file__), 'calibration_datasets/dataset001')

        # Set this path to self.CC_path_dataset or self_MR_path_dataset
        self.path_dataset = self.CC_path_dataset



        self.node_name = "pose_collector"
        self.joint_topic = "/franka_state_controller/joint_states_desired"
        self.position_reached_topic = "/position_reached"
        self.current_joints = []
        self.collected_joint_list = []
        self.current_rgb_frame = []  # Current RGB Frame From Callback Function
        self.current_ir_frame = []  # Current IR Frame From Callback Function
        self.counter = 0

        rospy.init_node(self.node_name)
        self.joint_state = rospy.Subscriber(self.joint_topic, JointState, self.joint_state_callback)
        self.position_reached = rospy.Subscriber(self.position_reached_topic, Bool, self.position_reached_callback)
        self.bridge = CvBridge()

        # Camera Setup
        self.rgb_frame          = rospy.Subscriber("/rgb/image_raw", Image, self.rgb_image_callback)
        self.ir_frame           = rospy.Subscriber("/ir/image_raw", Image, self.ir_image_callback)
        self.ir_frame           = rospy.Subscriber("/ir/image_raw", PointCloud2, self.pc_callback)
        #/points2

        
    # Store Current RGB Frame In Class Variable
    def rgb_image_callback(self, ros_rgb_img):
        try:
            self.current_rgb_frame = self.bridge.imgmsg_to_cv2(ros_rgb_img, "bgr8")
        except CvBridgeError:
            print ("error bridging ROS Image-Message to OpenCV Image")

    def callback(self, ros_point_cloud : PointCloud2):
        self.pc2 = PointCloud2()
        self.pc2 = ros_point_cloud


    def save_pointcloud2(self):
        
        xyz = np.array([[0,0,0]])
        rgb = np.array([[0,0,0]])
        #self.lock.acquire()
        gen = pc2.read_points(self.pc2, skip_nans=True)
        int_data = list(gen)

        for x in int_data:
            test = x[3] 
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            # prints r,g,b values in the 0-255 range
                        # x,y,z can be retrieved from the x[0],x[1],x[2]
            xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
            rgb = np.append(rgb,[[r,g,b]], axis = 0)

        out_pcd = o3d.geometry.PointCloud()    
        out_pcd.points = o3d.utility.Vector3dVector(xyz)
        out_pcd.colors = o3d.utility.Vector3dVector(rgb)
        o3d.io.write_point_cloud(self.path_dataset"PCD_1.ply",out_pcd)


    # Store Current IR Frame In Class Variable
    def ir_image_callback(self, ros_ir_img):
        try:
            self.current_ir_frame = self.bridge.imgmsg_to_cv2(ros_ir_img, "bgr8")   
        except CvBridgeError:
            print ("error bridging ROS Image-Message to OpenCV Image")

    def joint_state_callback(self, joint_state):
        #print(joint_state.position)
        self.current_joints = joint_state.position
        
    def position_reached_callback(self, data):
        print("Collected Joint")
        print(self.current_joints)
        #Save desired joints to array
        self.collected_joint_list.append(self.current_joints)

        #Save RGB/IR images
        cv.imwrite(self.path_dataset + "/rgb_" + str(self.counter) + ".png", self.current_rgb_frame) 
        cv.imwrite(self.path_dataset + "/ir_" + str(self.counter) + ".png", self.current_ir_frame)
        self.counter += 1 

        #Saves Pointcloud in self.path_dataset
        self.save_pointcloud2()
        rospy.logwarn(f"[Pose Collector] Collected current_joints, saved rgb and ir image and PC2 with ID {self.counter}")

    def save_collected_joint_list(self):
        self.collected_joint_list = np.array(self.collected_joint_list)
        print(os.path.dirname)
        np.save(os.path.join(self.path_dataset, "collected_joint_list.npy"), self.collected_joint_list)
    


def main(args):       
    try:
        Pose_Collector = pose_collector()
        rospy.logwarn("Starting pose collector")
                
        while not rospy.is_shutdown():
            rospy.spin()
        print("After Shutdown")
     
        Pose_Collector.save_collected_joint_list()
        #joint_test = np.load("/home/rnm/catkin_ws/src/panda/panda_vision/scripts/joint_list/camera_calibration"/collected_joint_list.npy")
        #print(joint_test)


    except KeyboardInterrupt:
        print ("Shutting down pose_collector node.")
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)