import numpy as np
import cv2
import time
import sys

from object_detection_utils import *
from camera_feedback_control import *

def image_object_detect(img_base,img_obj):
    # get segmentated image
    seg_img = segment_object(frame_base,frame_object)
    obj_img = get_main_object(seg_img)
    # detect object exist or not
    obj_exist = object_exist(obj_img)
    return obj_exist

# high parameters
OBJECT_EXIST_RATE = 0.5
timeout = 10

# look at the image for timeout sec, and judge whether there is object or not
def video_object_detect(cap_base,cap_object,show=True):
    # initialization
    end_flag_base, end_flag_object = True, True
    obj_exist_rates = []
    start = time.time()

    while end_flag_base and end_flag_object:
        # camera for getting base image of the water surface without object
        end_flag_base, frame_base = cap_base.read()
        # camera for getting the image of object floating on the water surface
        end_flag_object, frame_object = cap_object.read()

        # get segmentated image
        seg_img = segment_object(frame_base,frame_object)
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
    if sys.argv[1]=="camera":
        # use camera
        cap1 = cv2.VideoCapture(0)
        cap2 = cv2.VideoCapture(1)
    else:
        # get video object
        cap1 = cv2.VideoCapture(sys.argv[1])
        cap2 = cv2.VideoCapture(sys.argv[2])
    # run object detecter
    exist = video_object_detect(cap1,cap2)
    print("Object Exist or not:",exist)
    # finish
    cv2.destroyAllWindows()
    cap1.release()
    cap2.release()