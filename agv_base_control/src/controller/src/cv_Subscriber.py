#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

br = CvBridge()

def callback(data):
    try:
        cv_image = br.imgmsg_to_cv2(data,"passthrough")
    except CvBridgeError as e:
        print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60:
        cv2.circle(cv_image, (50, 50), 10, 255)
    cv2.imshow("Image Subscriber Window", cv_image)
    cv2.waitKey(3)

def main():
    #ros config
    rospy.init_node("CV_Subscriber", anonymous=True)

    sub = rospy.Subscriber("/vision",Image,callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down!")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
