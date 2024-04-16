#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

msgsTopic = "/vision"

def capture_camera():
    # Checks and deletes the output file
    # You cant have a existing file or it will through an er
    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    
    # ROS config
    # Publish messages with Image type
    pub = rospy.Publisher(msgsTopic,Image, queue_size=10)

    # Set rospy node name
    rospy.init_node('vision_control', anonymous=True)

    # Go through the loop 10 times per second
    rate = rospy.Rate(10)
    
    # Create a Video Capture Object
    cap = cv2.VideoCapture(0)

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    currentFrame = 0
    currentFlag = 0
    # Get current width of frame
    width = cap.get(3)   # float
    # Get current height of frame
    height = cap.get(4) # float
    # Define the codec and create VideoWriter object
    
    if cap.isOpened():
        while not rospy.is_shutdown():
            ret_val, img = cap.read()
            #print("Captured image is: ",img)
            #print("Captured image type is: ", type(img))
            if(ret_val == True):
                # Print bugging information to the terminal
                rospy.loginfo('publishing video frame')
                cv2.imshow('CSI Camera',img)
                print("[INFO] Frame Number:",currentFrame)
                #out.write(img)
                print("Type of image before publishing is: ", type(img))
                # Converts Opencv image to ROS image message
                pub.publish(br.cv2_to_imgmsg(img,"passthrough"))

                if(currentFrame%4==0):
                    currentFlag+=1
                currentFrame+=1
                if cv2.waitKey(1)&0xFF == ord('q'):
                    break
            else:
                break
        rate.sleep()
        cap.release()
        cv2.destroyAllWindows()
    else:
        print('Unable to open camera')


if __name__ == '__main__':
    capture_camera()
