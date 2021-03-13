import numpy as np
import cv2
import sys

from object_detection_utils import *
from camera_feedback_control import *

def video_object_detect(cap_base,cap_object,show=True):
    # initialization
    end_flag_base, end_flag_object = True, True

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
        # if object exist
        if obj_exist:
            # get center of object
            center = get_object_center(obj_img)
            # get the gap between center of the detected object and center of the object_camera image
            gap = center[0] - X_CENTER
            # get direction which boat should ahead
            direction = left_forward_right(gap)
            #print("Object exist. Horizontal Gap is ",gap,". Boat should ",direction)
        else:
            center = None
            gap = None
            direction = None
            #print("Object is not detected")
        # return (yield) the values
        yield gap, direction

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
        elif show:
            showImg = cv2.resize(frame_object,IMG_SIZE)
            showImg[:,X_CENTER-5:X_CENTER+5,0] = np.ones_like(showImg[:,X_CENTER-5:X_CENTER+5,0])
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

        # prepare for next frame
        end_flag_base, frame_base = cap_base.read()
        end_flag_object, frame_object = cap_object.read()

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
    for gap,direction in video_object_detect(cap1,cap2):
        print("Object exist. Horizontal Gap is ",gap,". Boat should ",direction)
    # finish
    cv2.destroyAllWindows()
    cap1.release()
    cap2.release()