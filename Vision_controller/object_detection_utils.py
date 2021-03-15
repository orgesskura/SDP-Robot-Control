import cv2
import numpy as np
import matplotlib.pyplot as plt

# high parameters
IMG_SIZE = (256,256)
X_CENTER = int(256//2)
OBJECT_THRESH = int((256*256) * 0.001)

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
    img3 = convertUnit8(img1)
    img4 = convertUnit8(img2)
    return  cv2.subtract(img3,img4)

# method to sbtract base image from object image
def image_subtraction2(img1,img2):
    sub =  cv2.subtract(img1,img2)
    sub -= np.min(sub)
    # cv2.imshow("sub", sub)
    # cv2.waitKey(1)
    return sub

# method to segment image using threshold
def binarization(img):
    #convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Otsu's thresholding after Gaussian filtering
    #blur = cv2.GaussianBlur(img,(5,5),0)
    #ret,threshold = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    threshold = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,81,10)
    return threshold

def binarization2(img):
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
    base_img = cv2.resize(base_img,IMG_SIZE)
    obj_img = cv2.resize(obj_img,IMG_SIZE)
    seg_img = binarization2(image_subtraction2(obj_img,base_img))
    return seg_img

# filter the segmented image
def get_main_object(seg_img):
    imge = cv2.erode(seg_img,np.ones((8,8),np.uint8))
    return imge

def get_main_object3(seg_img):
    seg_img = seg_img.astype(np.uint8)
    contours = cv2.findContours(seg_img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0]
    if len(contours) == 0:
        return seg_img, None
    max_seg_idx = np.argmax([cv2.contourArea(c) for c in contours])
    max_seg = contours[max_seg_idx]
    seg_img = cv2.drawContours(seg_img, max_seg, -1, (0, 0, 128), -1)
    return seg_img, max_seg

def get_main_object2(seg_img):
    contours = cv2.findContours(seg_img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0]
    max_seg_idx = np.argmax([cv2.contourArea(c) for c in contours])
    max_seg = contours[max_seg_idx]
    seg_img = cv2.drawContours(seg_img, max_seg, -1, 255, -1)
    return seg_img, max_seg

# cobine segmentation and filter
def main_object_segment(img1,img2):
    seg_img = segment_object(img1,img2)
    imge, max_seg = get_main_object2(seg_img)
    return imge, max_seg

def object_exist2(main_object):
    if main_object is None:
        return False
    white_pixels_num = cv2.contourArea(main_object)
    print(white_pixels_num)
    return True if white_pixels_num > OBJECT_AREA_THRESH else False

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


# for testing purpose
if __name__ == '__main__':
    img1 = normalize(cv2.imread("water.png"))
    img2 = normalize(cv2.imread("front1.png"))
    segimg, contour = main_object_segment(img1,img2)
    segimg = (255-segimg)
    obj_exist = object_exist(segimg)
    if obj_exist:
        center = get_object_center(segimg)
        showImg = cv2.resize(img2,IMG_SIZE)
        showImg[:,X_CENTER-5:X_CENTER+5,0] = np.ones_like(showImg[:,X_CENTER-5:X_CENTER+5,0])
        showImg = cv2.circle(showImg,(center[0],center[1]),radius=4,color=(255,0,0),thickness=-1)
        state = "Object detected in the video"
    else:
        showImg = cv2.resize(img2,IMG_SIZE)
        state = "Object detected in the video"
    # show
    showImg = cv2.putText(showImg, state, (100,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 3, cv2.LINE_AA)
    cv2.imshow('window1', showImg)
    cv2.imshow('window2', segimg)
        