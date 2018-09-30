# cv2CannyHough.py

import cv2
import numpy as np

def nothing(x):
    pass

cap = cv2.VideoCapture(0)

cv2.namedWindow('image')
cv2.createTrackbar('linearResHoughGrid','image',1,30,nothing)
cv2.createTrackbar('angularResHoughGrid','image',1,30,nothing)
cv2.createTrackbar('lowThreshIntersec','image',1,75,nothing)
cv2.createTrackbar('minLineLength','image',5,200,nothing)
cv2.createTrackbar('maxLineGap','image',1,50,nothing)

kernel_size = 11
low_threshold = 38
high_threshold = 38

#Best settings in Bedroom w 6mm lense:
# best -- for video, stability across frames
# is important for optical flow calculation
# kernel: 13
# low threshold = 38
# high threshold = 38

def frameArray(newFrame):

    pass



while(1):

    ret, frame2 = cap.read()

    # MANIPULATION
    #
    #

    #grayscale
    gray = cv2.cvtColor(frame2, cv2.COLOR_RGB2GRAY) 

    # blur
    blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    # canny thresholds
    edges = cv2.Canny(blur_gray, low_threshold, high_threshold)

    # show result
    cv2.imshow('frame',edges)
    
    
    # look for kill & destroy
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
