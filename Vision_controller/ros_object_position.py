import numpy as np
import cv2
import time
import sys

import roslib
import rospy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from object_detection_utils import *
from camera_feedback_control import *

class Object_Position:
    def __init__(self):
        # initialization
        rospy.init_node('Object_Detector', anonymous=True)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        # image subscribers
        self.water_img_sub = rospy.Subscriber("water_img",Image,self.get_base_image)
        self.front_img_sub = rospy.Subscriber("front_img",Image,self.get_object_image)
        # result publishers
        self.horizontal_gap_pub = rospy.Publisher("horizontal_gap", Float64, queue_size=10)
        self.direction_pub = rospy.Publisher("direction", String, queue_size=10)

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
    
    def ros_object_position(self)
        # camera for getting base image of the water surface without object
        frame_base = self.water_img
        # camera for getting the image of object floating on the water surface
        frame_object = self.front_img

        # get segmentated image
        seg_img = segment_object(frame_base,frame_object)
        obj_img = get_main_object(seg_img)
        # detect object exist or not
        obj_exist = object_exist(obj_img)
        # if object exist
        if obj_exist:
            # get center of object
            center = get_object_center(obj_img)
            # get the gap between center of the detected object and center of the object_camera image
            gap = center[0] - X_CENTER
            # get direction which boat should ahead
            direction = left_forward_right(gap)
        else:
            gap = None
            direction = None

        # ploting the object detection process
        if show and obj_exist:
            showImg = cv2.resize(frame_object,IMG_SIZE)
            showImg[:,X_CENTER-5:X_CENTER+5,0] = np.ones_like(showImg[:,X_CENTER-5:X_CENTER+5,0])
            showImg = cv2.circle(showImg,(center[0],center[1]),radius=4,color=(255,0,0),thickness=-1)
            base_img = cv2.resize(frame_base,IMG_SIZE)
            # show
            cv2.imshow('window1', showImg)
            cv2.imshow('window2', obj_img)
            cv2.imshow('window3', base_img)
            # cv2.waitKey(1)
        elif show:
            showImg = cv2.resize(frame_object,IMG_SIZE)
            showImg[:,X_CENTER-5:X_CENTER+5,0] = np.ones_like(showImg[:,X_CENTER-5:X_CENTER+5,0])
            base_img = cv2.resize(frame_base,IMG_SIZE)
            # show
            cv2.imshow('window1', showImg)
            cv2.imshow('window2', obj_img)
            cv2.imshow('window3', base_img)
            # cv2.waitKey(1)
        else:
            pass

        # publish the results
        result1 = Float64()
        result1.data = gap
        result2 = String()
        result2.data = direction
        horizontal_gap_pub.publish(result1)
        direction_pub.publish(result2)

        rate = rospy.Rate(50)
        rate.sleep()

if __name__ == '__main__':
    obj_pos = Object_Position()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")