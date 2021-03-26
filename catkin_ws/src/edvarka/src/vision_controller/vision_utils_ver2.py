import numpy as np
import cv2

# high parameters
IMG_SIZE = (256,256)
X_CENTER = int(256//2)
OBJECT_THRESH = int((256*256) * 0.000005)

################################################################################################
# object detection functions
################################################################################################

# method to sbtract base image from object image
def image_subtraction(img1,img2):
    sub =  cv2.subtract(img1,img2)
    sub -= np.min(sub)
    return sub

# method to segment image using threshold
def binarization(img):
    cv2.imshow("view", img)
    cv2.waitKey(1)
    t = 50
    red = img[:,:,2] < t
    green = img[:,:,1] < t
    blue = img[:,:,0] < t
    mask = red & green & blue
    binary = np.zeros((256, 256))
    binary[~mask] = 255
    return binary

# method to get segmented image from base and object images
def segment_object(base_img,obj_img):
    seg_img = binarization(image_subtraction(obj_img,base_img))
    return seg_img

# filter the segmented image
def get_main_object(seg_img):
    seg_img = seg_img.astype(np.uint8)
    contours = cv2.findContours(seg_img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0]
    if len(contours) == 0:
        return seg_img, None
    max_seg_idx = np.argmax([cv2.contourArea(c) for c in contours])
    max_seg = contours[max_seg_idx]
    seg_img = cv2.drawContours(seg_img, max_seg, -1, (0, 0, 128), -1)
    return seg_img, max_seg

# decide object exist or not from segmented image
def object_exist(main_object):
    if main_object is None:
        return (False, None)
    white_pixels_num = cv2.contourArea(main_object)
    return (True, white_pixels_num) if white_pixels_num > OBJECT_THRESH else (False, None)

################################################################################################
# object position detection functions
################################################################################################

# get the center of the object in segmented image
def get_object_center(img):
    M = cv2.moments(img)
    x = int(M['m10'] / M['m00'])
    y = int(M['m01'] / M['m00'])
    return np.array([x, y])

# get the gap between center of the detected object and center of the camera image
def get_Horizontal_gap(center):
    gap = center[0] - X_CENTER
    return gap


if __name__ == "__main__":
    img1 = cv2.imread("water1.png")
    img2 = cv2.imread("obj_back3.png")
    exist,center,size,h = image_object_detect(img1,img2)
    if exist:
        showImg = cv2.resize(img2,IMG_SIZE)
        showImg = cv2.circle(showImg,(center[0],center[1]),radius=4,color=(255,0,0),thickness=-1)
        showImg[h-1:h+1,:,1] = 255
        cv2.imshow("result",showImg)
        cv2.waitKey(1)