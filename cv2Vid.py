import numpy as np
import cv2

cap = cv2.VideoCapture(0)

while(True):
    #Capture, frame by frame
    ret, frame = cap.read()

    # Whatever operations we want to do to frame
    modFrame = frame
    #modFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #modFrame = frame #cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    #modFrame = frame #cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #modFrame[:,:,2] = 0
    
    #ret2, frame2 = cap.read()
    #modFrame = cv2.addWeighted(frame, 0.5, frame2, 0.5, 0)
   
    #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #sobelx64f = cv2.Sobel(frame, cv2.CV_64F, 1, 0, ksize=3)
    #abs_sobel64f = np.absolute(sobelx64f)
    #sobel_8u = np.uint8(abs_sobel64f)
    #modFrame = sobel_8u

    # Display resulting frame
    cv2.imshow('frame',modFrame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When all done, release the camera
cap.release()
cv2.destroyAllWindows()

