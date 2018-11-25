import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

img = cv.imread('valvulas/valvula12.jpg')

img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

#inclinacao da parte movel
hsv = [120, 190] #h and s, v is not useful
thresh_hsv = [20, 30]

min_hsv = np.array([hsv[0] - thresh_hsv[0], hsv[1] - thresh_hsv[1], 0])
max_hsv = np.array([hsv[0] + thresh_hsv[0], hsv[1] + thresh_hsv[1], 255])
mask_hsv = cv.inRange(img_hsv, min_hsv, max_hsv)

result_hsv = cv.bitwise_and(img_hsv, img_hsv, mask = mask_hsv)
result_gray = cv.cvtColor(cv.cvtColor(result_hsv, cv.COLOR_HSV2BGR), cv.COLOR_BGR2GRAY)

_, th = cv.threshold(result_gray,1,255,cv.THRESH_BINARY)

im2, contours, hierarchy = cv.findContours(th, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

contour = contours[0]
for cnt in contours:
    if cv.contourArea(cnt) > cv.contourArea(contour):
        contour = cnt
    
cv.drawContours(th,[contour],0,255,-1)

rect_mob = cv.minAreaRect(contour)
box = cv.boxPoints(rect_mob)
box = np.int0(box)
cv.drawContours(img,[box],0,(0,255,0),2)

#inclinacao do corpo da valvula
blur = cv.GaussianBlur(img_gray,(5,5),0)
ret2, th1 = cv.threshold(blur,0,255,cv.THRESH_OTSU)

th1 = cv.bitwise_not(th1)

th2 = cv.bitwise_and(th1, cv.bitwise_not(th))

kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(11,11))
th2 = cv.morphologyEx(th2,cv.MORPH_OPEN, kernel)

im2, contours, hierarchy = cv.findContours(th2, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

contour = contours[0]
for cnt in contours:
    if cv.contourArea(cnt) > cv.contourArea(contour):
        contour = cnt

#cv.drawContours(img, [contour], -1, (0,255,0), 3)
rect_bod = cv.minAreaRect(contour)
box = cv.boxPoints(rect_bod)
box = np.int0(box)
cv.drawContours(img,[box],0,(0,0,255),2)

if(rect_mob[1][1] > rect_mob[1][0]):
    angle_mob = 90 + rect_mob[2]
else:
    angle_mob = 180 + rect_mob[2]

if(rect_bod[1][1] > rect_bod[1][0]):
    angle_bod = 90 + rect_bod[2]
else:
    angle_bod = 180 + rect_bod[2]

print('Mobile Angle: ', angle_mob)
print('Body Angle: ', angle_bod)


titles = ['Original Image', 'Mobile Part', 'Body', 'Body without mobile']
images = [cv.cvtColor(img, cv.COLOR_BGR2RGB), th, th1, th2]

for i in xrange(4):
    plt.subplot(2,2,i+1),plt.imshow(images[i],'gray')
    plt.title(titles[i])
    plt.xticks([]),plt.yticks([])
plt.show()
