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

    # masked edges
    mask = np.zeros_like(edges)
    ignore_mask_color = 255

    # define 4 sided polygon to mask
    imshape = frame2.shape
    vertices = np.array([[(0,imshape[0]),(0, 0), (imshape[1], 0),(imshape[1],imshape[0])]], dtype=np.int32)
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    masked_edges = cv2.bitwise_and(edges, mask)

    # Get User Input
    rho_in = cv2.getTrackbarPos('linearResHoughGrid','image')
    the_in = cv2.getTrackbarPos('angularResHoughGrid','image')
    thres_in = cv2.getTrackbarPos('lowThreshIntersec','image')
    min_line_in = cv2.getTrackbarPos('minLineLength','image')
    max_gap_in = cv2.getTrackbarPos('maxLineGap','image')

    # Define the Hough transform parameters
    # Make a blank the same size as our image to draw on
    rho = rho_in # distance resolution in pixels of the Hough grid
    theta = the_in*np.pi/180 # angular resolution in radians of the Hough grid
    threshold = thres_in    # minimum number of votes (intersections in Hough grid cell)
    min_line_length = min_line_in #minimum number of pixels making up a line
    max_line_gap = max_gap_in    # maximum gap in pixels between connectable line segments
    #rho = 1 # distance resolution in pixels of the Hough grid
    #theta = 1*np.pi/180 # angular resolution in radians of the Hough grid
    #threshold = 35     # minimum number of votes (intersections in Hough grid cell)
    #min_line_length = 40 #minimum number of pixels making up a line
    #max_line_gap = 3    # maximum gap in pixels between connectable line segments
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
                lines_edges = cv2.addWeighted(color_edges, 0.8,line_image, 1, 0) 
                #lines_edges = cv2.addWeighted(frame2, 0.8, line_image, 1, 0) 
    
        # Display resulting frame
        cv2.imshow('frame',lines_edges)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
