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

from object_detection_utils import *
from camera_feedback_control import *

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
    img_with_contour, main_obj_contour = get_main_object(seg_img)
    # cv2.imshow("obj_image", img_with_contour)
    # cv2.waitKey(1)
    # detect object exist or not
    obj_exist, object_size = object_exist(main_obj_contour)
    object_center = None
    if obj_exist:
        object_center = get_object_center(main_obj_contour)
    return (obj_exist, object_center, object_size)

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

# look at the image for timeout sec, and judge whether there is object or not
def video_object_detect(rate, show=True):
    global water_image, front_image
    # initialization
    obj_exist_rates = []
    start = time.time()

    while True:
        rate.sleep()
        # camera for getting base image of the water surface without object
        frame_base = water_image
        # camera for getting the image of object floating on the water surface
        frame_object = front_image

        if frame_base is None or frame_object is None:
            continue

        # get segmentated image
        seg_img = segment_object(frame_base, frame_object)
        obj_img = get_main_object(seg_img)
        # detect object exist or not
        obj_exist = object_exist(obj_img)
        obj_exist_rates.append(obj_exist)

        # ploting the object detection process
        if show:
            showImg = cv2.resize(frame_object,IMG_SIZE)
            base_img = cv2.resize(frame_base,IMG_SIZE)
            # show
            cv2.imshow('window1', showImg)
            cv2.imshow('window2', obj_img)
            cv2.imshow('window3', base_img)
        else:
            pass

        # quit with ESC
        if cv2.waitKey(30) & 0xff == 27:
            break

        # quit with timeout
        if time.time() - start > timeout:
            break

        # prepare for next frame
        end_flag_base, frame_base = cap_base.read()
        end_flag_object, frame_object = cap_object.read()
    
    # calculate what is the rate of the object detected and decide whether object exist or not
    exists_rate = sum(obj_exist_rates) / len(obj_exist_rates)
    return True if exists_rate > OBJECT_EXIST_RATE else False

if __name__ == '__main__':
    rospy.init_node(name="object_detection_node")
    rate = rospy.Rate(20) # Hz
    front_image_sub = rospy.Subscriber("/front_camera_view", Image, queue_size=1, callback=update_front_image)
    water_image_sub = rospy.Subscriber("/water_camera_view", Image, queue_size=1, callback=update_water_image)
    is_object_detected_pub = rospy.Publisher("/is_object_detected", Bool, queue_size=1)
    object_dist_from_center_pub = rospy.Publisher("/object_dist_from_center", Float64, queue_size=1)
    object_size_pub = rospy.Publisher("/object_size", Float64, queue_size=1)
    
    while not rospy.is_shutdown():
        if front_image is not None and water_image is not None:
            object_exists, object_center, object_size = image_object_detect(water_image, front_image)
            if object_exists:
                is_object_detected_pub.publish(True)
                dist = get_Horizontal_gap(object_center)
                object_dist_from_center_pub.publish(dist)
                object_size_pub.publish(object_size)
            else:
                #print("No object detected...")
                is_object_detected_pub.publish(False)
        rate.sleep()



    # if sys.argv[1]=="camera":
    #     # use camera
    #     cap1 = cv2.VideoCapture(0)
    #     cap2 = cv2.VideoCapture(1)
    # else:
    #     # get video object
    #     cap1 = cv2.VideoCapture(sys.argv[1])
    #     cap2 = cv2.VideoCapture(sys.argv[2])
    # # run object detecter
    # exist = video_object_detect(cap1,cap2)
    # print("Object Exist or not:",exist)
    # # finish
    # cv2.destroyAllWindows()
    # cap1.release()
    # cap2.release()