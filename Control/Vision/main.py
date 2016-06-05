import numpy as np
import cv2

img = cv2.imread('test1.jpg',0)

img = cv2.medianBlur(img,5)
cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20, param1=50,param2=28,minRadius=0,maxRadius=0)

circles = np.uint16(np.around(circles))

for i in circles[0]:
    mask = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
    cv2.circle(mask,(i[0],i[1]),i[2], (255,255,255), -1)
    cv2.imshow(str(id(i)), cv2.resize(mask, (854, 480)))
    # cv2.resizeWindow(str(id(i)), 400, 400)
    # draw the outer circle
    cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
    cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

cv2.imshow('detected circles',cimg)
cv2.waitKey(0)
cv2.destroyAllWindows()