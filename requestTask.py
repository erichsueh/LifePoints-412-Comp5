#!/usr/bin/env python
import rospy
import cv2
import os
import sys

global xPos
global yPos
global haveCursor

def getCursor(event, x, y, flags, params):
    global xPos, yPos
    global haveCursor
    if event == cv2.EVENT_LBUTTONDOWN:
        xPos = x
        yPos = y
        haveCursor = False
        print x
        print y

haveCursor = True
xPos = -1
yPos = -1
cv2.namedWindow("Map")
cv2.setMouseCallback("Map", getCursor)
map = cv2.imread("map.png")
cv2.imshow("Map", map)

while(haveCursor):   
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

if(xPos != -1):

    os.system("rostopic pub -1 /taskRequested std_msgs/String x" + str(xPos) + "y" + str(yPos))
    cv2.destroyWindow("Map")