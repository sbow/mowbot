# cv2canny.py
# this file includes a "trackbar"; which are sliders
# allowing for the tuning of the canny filter
# Interesingly / not expected: found best values for
# low / high threshold with respect to stability across
# frames inside my ceiling lit office were equal and fairly
# low values... also fairly large value for gausian blur kernal
# This probably is somewhat due to the low-light conditions in
# the office; hence the large kernel / blending operation.


import cv2
import numpy as np


def nothing(x):
    pass

cap = cv2.VideoCapture(0)

cv2.namedWindow('image')
cv2.createTrackbar('halfkernelSize','image',1,25,nothing)
cv2.createTrackbar('lowThresh','image',0,200,nothing)
cv2.createTrackbar('highThresh','image',50,250,nothing)

cv2.setTrackbarPos('halfkernelSize','image',9)
cv2.setTrackbarPos('lowThresh','image',6)
cv2.setTrackbarPos('highThresh','image',18)


kernel_size = 13
low_threshold = 38
high_threshold = 38

#Best settings in Bedroom w 6mm lense:
# best -- for video, stability across frames
# is important for optical flow calculation
# kernel: 13
# low threshold = 38
# high threshold = 38

while(1):

    ret, frame2 = cap.read()

    # MANIPULATION
    #
    #

    # Convert to Grayscale
    gray = cv2.cvtColor(frame2, cv2.COLOR_RGB2GRAY)

    # Get User Input
    kernel_size = cv2.getTrackbarPos('halfkernelSize','image')*2+1
    low_threshold = cv2.getTrackbarPos('lowThresh','image')
    high_threshold = cv2.getTrackbarPos('highThresh','image')

    # blur
    blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    # canny thresholds
    edges = cv2.Canny(blur_gray, low_threshold, high_threshold)

    # masked edges
    mask = np.zeros_like(edges)
    ignore_mask_color = 255

    # define 4 sided polygon to mask
    imshape = frame2.shape
    vertices = np.array([[(0,imshape[0]),(0, 0), (imshape[1], 0),(imshape[1],imshape[0])]], dtype=np.int32)
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    masked_edges = cv2.bitwise_and(edges, mask)

    cv2.imshow('mowbot_mowfeed', edges)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
