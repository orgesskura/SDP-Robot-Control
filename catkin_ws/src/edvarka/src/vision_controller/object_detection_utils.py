import cv2
import numpy as np

# high parameters
IMG_SIZE = (256,256)
X_CENTER = int(256//2)
OBJECT_THRESH = int((256*256) * 0.05)

################################################################################################
# object segmentation functions
################################################################################################

# method for converting images to uint8
def convertUnit8(img):
  img2 = np.zeros_like(img)
  cv2.normalize(img, img2, 0, 255, cv2.NORM_MINMAX)
  img2 = np.uint8(img2)
  return img2

# method to sbtract base image from object image
def image_subtraction(img1,img2):
    sub =  cv2.subtract(img1,img2)
    sub -= np.min(sub)
    # cv2.imshow("sub", sub)
    # cv2.waitKey(1)
    return sub

# method to segment image using threshold
def binarization(img):
    t = 30
    red = img[:,:,2] < t
    green = img[:,:,1] < t
    blue = img[:,:,0] < t
    mask = red & green & blue
    binary = np.zeros((256, 256))
    binary[~mask] = 255
    return binary

# method to normarize the image (RGB normalizations)
def normalize(img):
    norm_image = cv2.normalize(img, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    return norm_image

# method to get segmented image from base and object images
def segment_object(base_img,obj_img):
    seg_img = binarization(image_subtraction(obj_img,base_img))
    return seg_img

# filter the segmented image
def get_main_object(seg_img):
    imge = cv2.erode(seg_img,np.ones((1,1),np.uint8))
    return imge

# decide object exist or not from segmented image
def object_exist(seg_img):
    pixels = seg_img.reshape(-1)
    print(pixels)
    white_pixels_num = len([p for p in pixels if p == 255])
    return True if white_pixels_num > OBJECT_THRESH else False

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
