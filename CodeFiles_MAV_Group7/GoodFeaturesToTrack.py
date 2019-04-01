import numpy as np
import cv2


cap = cv2.VideoCapture('TestVideo/CyberZoo6.mp4')


while(True):

    # Capture frame-by-frame
    ret, img = cap.read()
    if ret == True:
        # Our operations on the frame come here
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners = cv2.goodFeaturesToTrack(gray, 1000,0.2,10)
        corners = np.int0(corners)
        for i in corners:
            x,y = i.ravel()
            cv2.circle(img,(x,y),3,255,-1)


    
    # Display the resulting frame
    cv2.imshow('frame',img)
    if cv2.waitKey(1) & 0xFF == ord('q') or ret == False:
        break
    
    
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()



