import numpy as np
import cv2

# high parameters
IMG_SIZE = (256,256)
last_horizon = 255

################################################################################################
# background segmentation functions
################################################################################################

# method for converting images to uint8
def convertUnit8(img):
  img2 = np.zeros_like(img)
  cv2.normalize(img, img2, 0, 255, cv2.NORM_MINMAX)
  img2 = np.uint8(img2)
  return img2

def image_subtraction(img1,img2):
    sub =  cv2.subtract(img1,img2)
    sub -= np.min(sub)
    return sub

def getBackground(img1,img2):
    # subtract the images
    img_sub = image_subtraction(img1,img2)
    #convert to grayscale
    gray = cv2.cvtColor(img_sub, cv2.COLOR_BGR2GRAY)
    # normarize the image
    gray_norm = convertUnit8(gray)
    # Otsu's thresholding after Gaussian filtering
    blur = cv2.GaussianBlur(gray_norm,(5,5),0)
    ret,threshold = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    return threshold

def getEucLength(line):
    x1,y1,x2,y2 = line
    v1 = np.array([x1,y1])
    v2 = np.array([x2,y2])
    return np.linalg.norm(v1-v2)

def getYcoordinate(line):
    x1,y1,x2,y2 = line
    return int((y1+y2)/2)

def getHorizon(img):
    img = convertUnit8(img)
    edges = cv2.Canny(img, 150, 300, L2gradient=True)
    cv2.imshow("Edges",edges); cv2.waitKey(1)
    lines = cv2.HoughLinesP(edges,rho=1,theta=np.pi/360,threshold=80,minLineLength=50,maxLineGap=5)
    lines = [l[0] for l in lines]
    #print(lines)
    idx = np.argmax([getEucLength(line) for line in lines])
    max_line = lines[idx]
    horizon = int((max_line[1] + max_line[3])/2)
    return horizon

def getHorizon2(img):
    img = convertUnit8(img)
    edges = cv2.Canny(img, 150, 300, L2gradient=True)
    cv2.imshow("Edges",edges); cv2.waitKey(1)
    #lines = cv2.HoughLinesP(edges,rho=1,theta=np.pi/360,threshold=80,minLineLength=50,maxLineGap=5)
    lines = cv2.HoughLinesP(edges, cv2.HOUGH_PROBABILISTIC, np.pi/180, 30, 50, 5)
    global last_horizon
    if lines is None:
        return last_horizon
    lines = [l[0] for l in lines]
    #print(lines)
    idx = np.argmax([getYcoordinate(line) for line in lines])
    max_line = lines[idx]
    horizon = max(max_line[1], max_line[3])
    last_horizon = horizon
    return horizon

def removeBackground(img,h):
    img_copy = img.copy()
    img_copy[:h+5] = 0
    return img_copy
