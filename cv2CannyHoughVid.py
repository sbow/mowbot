# cv2CannyHough.py

import cv2
import numpy as np
cap = cv2.VideoCapture(0)
ret, frame1 = cap.read()
prvs = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
hsv = np.zeros_like(frame1)
hsv[...,1] = 255

while(1):

    ret, frame2 = cap.read()

    # MANIPULATION
    gray = cv2.cvtColor(frame2, cv2.COLOR_RGB2GRAY)
    
    # sobel edge detection
#    frame = gray
#    sobelx64f = cv2.Sobel(frame, cv2.CV_64F, 1, 0, ksize=3)
#    abs_sobel64f = np.absolute(sobelx64f)
#    sobel_8u = np.uint8(abs_sobel64f)
#    modFrame = sobel_8u
#    gray = modFrame
#
    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    # canny thresholds
    low_threshold = 100
    high_threshold = 200
    edges = cv2.Canny(blur_gray, low_threshold, high_threshold)

    # masked edges
    mask = np.zeros_like(edges)
    ignore_mask_color = 255

    # define 4 sided polygon to mask
    imshape = frame2.shape
    vertices = np.array([[(0,imshape[0]),(0, 0), (imshape[1], 0),(imshape[1],imshape[0])]], dtype=np.int32)
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    masked_edges = cv2.bitwise_and(edges, mask)

    # Define the Hough transform parameters
    # Make a blank the same size as our image to draw on
    rho = 3 # distance resolution in pixels of the Hough grid
    theta = 3*np.pi/180 # angular resolution in radians of the Hough grid
    threshold = 15     # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 90 #minimum number of pixels making up a line
    max_line_gap = 4    # maximum gap in pixels between connectable line segments
    line_image = np.copy(frame2)*0 # creating a blank to draw lines on

    # Run Hough on edge detected image
    # Output "lines" is an array containing endpoints of detected line segments
    lines = cv2.HoughLinesP(masked_edges, rho, theta, threshold, np.array([]),min_line_length,max_line_gap)

    # Iterate over the output "lines" and draw lines on a blank image
    if lines is not None:
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),10)
    
                # Create a "color" binary image to combine with line
                # image
                color_edges = np.dstack((edges, edges, edges)) 
                #lines_edges = cv2.addWeighted(color_edges, 0.8,line_image, 1, 0) 
                lines_edges = cv2.addWeighted(frame2, 0.8, line_image, 1, 0) 
    
        # Display resulting frame
        cv2.imshow('frame',lines_edges)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
