#! /usr/bin/python3

import numpy as np
import cv2
import time
import sys
import os

import rospy
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from vision_utils_ver2 import *
from remove_background import getBackground, getHorizon, removeBackground

OBJECT_EXIST_RATE = 0.5
timeout = 10
front_image = None
water_image = None
bridge = CvBridge()
object_center = None


def image_object_detect(img_base,img_obj):
    global object_center
    # get segmentated image
    seg_img = segment_object(img_base,img_obj)
    #cv2.imshow("seg_image", seg_img); cv2.waitKey(1)
    # get image with background
    background = getBackground(img_base,img_obj)
    # get horizontal line (y-coordinate)
    horizon = getHorizon(background)
    # remove anything upper horizontal line
    seg_img2 = removeBackground(seg_img,horizon)
    #cv2.imshow("seg_image2", seg_img2); cv2.waitKey(1)
    # get main object
    img_with_contour, main_obj_contour = get_main_object(seg_img2)
    #cv2.imshow("obj_image", img_with_contour); cv2.waitKey(1)
    nearby_row = 180
    img_with_contour[nearby_row,:] = 255
    cv2.imshow("obj_image", img_with_contour); cv2.waitKey(1)
    # detect object exist or not
    obj_exist, object_size = object_exist(main_obj_contour)
    object_center = None
    if obj_exist:
        object_center = get_object_center(main_obj_contour)
    return (obj_exist, object_center, object_size, horizon)

def update_front_image(img):
    global bridge, front_image
    front_image = bridge.imgmsg_to_cv2(img)
    cv2.imwrite('../../images/front.png', front_image)
    # cv2.imshow("cv2_front", front_image)
    # cv2.waitKey(1)

def update_water_image(img):
    global bridge, water_image
    water_image = bridge.imgmsg_to_cv2(img)
    cv2.imwrite('../../images/water.png', water_image)
    # cv2.imshow("cv2_back", water_image)
    # cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node(name="object_detection_node")
    rate = rospy.Rate(20) # Hz
    front_image_sub = rospy.Subscriber("/front_camera_view", Image, queue_size=1, callback=update_front_image)
    water_image_sub = rospy.Subscriber("/water_camera_view", Image, queue_size=1, callback=update_water_image)
    is_object_detected_pub = rospy.Publisher("/is_object_detected", Bool, queue_size=1)
    object_dist_from_center_pub = rospy.Publisher("/object_dist_from_center", Float64, queue_size=1)
    object_size_pub = rospy.Publisher("/object_size", Float64, queue_size=1)
    object_y_pos_pub = rospy.Publisher("/object_y_pos", Float64, queue_size=1)
    
    while not rospy.is_shutdown():
        if front_image is not None and water_image is not None:
            object_exists, object_center, object_size, horizon = image_object_detect(water_image, front_image)
            if object_exists:
                is_object_detected_pub.publish(True)
                dist = get_Horizontal_gap(object_center)
                object_dist_from_center_pub.publish(dist)
                object_size_pub.publish(object_size)
                object_y_pos_pub.publish(object_center[1]) # y coord
            else:
                #print("No object detected...")
                is_object_detected_pub.publish(False)
        rate.sleep()
