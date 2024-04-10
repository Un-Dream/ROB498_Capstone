import cv2 
import numpy as np 

cap = cv2.VideoCapture(url of our video stream or IP of camera or 0) 

while 1: 
    ret,frame =cap.read()  
    into_hsv =cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) 
    L_limit=np.array([98,50,50]) # setting the blue lower limit 
    U_limit=np.array([139,255,255]) # setting the blue upper limit 
    #using blue to match video      
  
    b_mask=cv2.inRange(into_hsv,L_limit,U_limit) 
    blue=cv2.bitwise_and(frame,frame,mask=b_mask) 
    cv2.imshow('Original',frame) 
    cv2.imshow('Blue Detector',blue) # for our verification
  
    if cv2.waitKey(1)==27: 
        break
    # this function will be triggered when the ESC key is pressed and the while loop will terminate
    # made this a while not dependent on flight since it should still be looking when it holds a hover on a location
cap.release() 
  
cv2.destroyAllWindows() 