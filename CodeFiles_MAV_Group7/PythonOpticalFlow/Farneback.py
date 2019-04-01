import cv2
import numpy as np

cap = cv2.VideoCapture("TestVideo/CyberZoo4.mp4")
ret, frame1 = cap.read()
prvs = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
hsv = np.zeros_like(frame1)
hsv[...,1] = 255
#framenr = 0
while(1):
    ret, frame2 = cap.read()
    next = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
    flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
    hsv[...,0] = ang*180/np.pi/2
    hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
    bgr = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)

    #name2 = 'Frames2/frame' + str(framenr) + '.jpg'
    #cv2.imwrite(name2, bgr);
    #framenr+=1

    
    cv2.imshow('frame2',bgr)
    if cv2.waitKey(1) & 0xFF == ord('q') or ret == False:
         break
    prvs = next
cap.release()
cv2.destroyAllWindows()



