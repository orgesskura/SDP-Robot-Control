import cv2
import numpy as np

# high parameters
IMG_SIZE = (256,256)
X_CENTER = int(256//2)
OBJECT_THRESH = int((256*256) * 0.1)

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
    #convert images to uint8 as descibed in the paper
    #img1 = img1[:180][:]
    #img2 = img2[:][:280]
    #print(img1.shape)
    #print(img2.shape)
    img3 = convertUnit8(img1)
    img4 = convertUnit8(img2)
    return  cv2.subtract(img3,img4)

# method to segment image using threshold
def binarization(img):
    #convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Otsu's thresholding after Gaussian filtering
    blur = cv2.GaussianBlur(gray,(5,5),0)
    ret,threshold = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    return threshold

# method to normarize the image (RGB normalizations)
def normalize(img):
    norm_image = cv2.normalize(img, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    return norm_image

# method to get segmented image from base and object images
def segment_object(base_img,obj_img):
    base_img = normalize(cv2.resize(base_img,IMG_SIZE))
    obj_img = normalize(cv2.resize(obj_img,IMG_SIZE))
    seg_img = binarization(image_subtraction(obj_img,base_img))
    return seg_img

# filter the segmented image
def get_main_object(seg_img):
    imge = cv2.erode(seg_img,np.ones((8,8),np.uint8))
    return imge

# decide object exist or not from segmented image
def object_exist(seg_img):
    pixels = seg_img.reshape(-1)
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
