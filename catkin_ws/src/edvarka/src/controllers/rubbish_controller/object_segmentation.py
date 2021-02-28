import cv2
import numpy as np
import matplotlib.pyplot as plt

IMG_SIZE = (256,256)

#method for converting images to uint8
def convertUnit8(img):
  img2 = np.zeros_like(img)
  cv2.normalize(img, img2, 0, 255, cv2.NORM_MINMAX)
  img2 = np.uint8(img2)
  return img2

def image_subtraction(img1,img2):
    #convert images to uint8 as descibed in the paper
    #img1 = img1[:180][:]
    #img2 = img2[:][:280]
    print(img1.shape)
    print(img2.shape)
    img3 = convertUnit8(img1)
    img4 = convertUnit8(img2)
    return  cv2.subtract(img3,img4)

def binarization(img):
    #convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Otsu's thresholding after Gaussian filtering
    blur = cv2.GaussianBlur(gray,(5,5),0)
    ret,threshold = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    return threshold

def segment_object(base_img,img):
    base_img = cv2.imread(base_img,1)
    base_img = cv2.resize(base_img,IMG_SIZE)
    obj_img = cv2.imread(img,1)
    obj_img = cv2.resize(obj_img,IMG_SIZE)
    seg_img = binarization(image_subtraction(obj_img,base_img))
    
    fig, axs = plt.subplots(1,3)
    fig.suptitle('BaseImage, ObjectImage, SegmentedImage')
    axs[0].imshow(base_img)
    axs[1].imshow(obj_img)
    axs[2].imshow(seg_img)
    plt.show()

    return seg_img

def object_exist(seg_img):
    return True if 255 in seg_img else False

def get_main_object(seg_img):
    imge = cv2.erode(seg_img,np.ones((8,8),np.uint8))
    
    fig, axs = plt.subplots(1,2)
    fig.suptitle('SegmentedImage, FilteredImages')
    axs[0].imshow(seg_img)
    axs[1].imshow(imge)
    plt.show()

    return imge


if __name__ == '__main__':
    imgb = "base.jpg"
    imgo = "object.jpg"
    imgs = segment_object(imgb,imgo)
    print("Object exist? : ",object_exist(imgs))
    img = get_main_object(imgs)

    imgb = "base.jpg"
    imgo = "base.jpg"
    imgs = segment_object(imgb,imgo)
    print("Object exist? : ",object_exist(imgs))
    img = get_main_object(imgs)
