# vidCapture.py
# example cv2 - Shaun Bowman
# Dec 2 2017

import cv2
import matplotlib.pyplot as plt

def show_webcam(mirror=False):
    # 1 below is 2nd webcam
    cam = cv2.VideoCapture(0)
    ret_val, img = cam.read()
    #cv2.imshow('my webcam', img)
    #cv2.waitKey(0)
    #cv2.waitKey(2000) # wait ms
    #cv2.destroyAllWindows()
    
    print img.size

    cv2.imshow('my webcam', img)
    cv2.waitKey(0)

#while(True):
for x in range(1):
    show_webcam()

