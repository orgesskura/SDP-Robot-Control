import numpy as np
import cv2
import time
import sys

import roslib
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

from object_detection_utils import *
from camera_feedback_control import *


class Object_Detector:
    
    def __init__(self):
        # initialization
        rospy.init_node('Object_Detector', anonymous=True)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        # image subscribers
        self.water_img_sub = rospy.Subscriber("water_img",Image,self.get_base_image)
        self.front_img_sub = rospy.Subscriber("front_img",Image,self.get_object_image)
        # result publisher
        self.object_detect_pub = rospy.Publisher("objest_exist", Bool, queue_size=10)

    def get_base_image(self,data):
        # Recieve the image
        try:
            self.water_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def get_object_image(self,data):
        # Recieve the image
        try:
            self.front_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def ros_object_detect(show=True):
        frame_base = self.water_img
        frame_object = self.front_img
        # get segmentated image
        seg_img = segment_object(frame_base,frame_object)
        obj_img = get_main_object(seg_img)
        # detect object exist or not
        obj_exist = object_exist(obj_img)
        
        # show the images
        if show:
            showImg = cv2.resize(frame_object,IMG_SIZE)
            base_img = cv2.resize(frame_base,IMG_SIZE)
            # show
            cv2.imshow('window1', showImg)
            cv2.imshow('window2', obj_img)
            cv2.imshow('window3', base_img)
            # cv2.waitKey(1)

        # publish the result
        result = Bool()
        result.data = obj_exist
        self.object_detect_pub.publish(result)

        rate = rospy.Rate(50)
        rate.sleep()

if __name__ == '__main__':
    obj_det = Object_Detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")