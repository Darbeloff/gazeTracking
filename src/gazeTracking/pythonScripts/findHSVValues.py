#!/usr/bin/python

# 2.12 Lab 4 object detection: a node for de-noising
# Luke Roberto Oct 2017

import rospy
import numpy as np
import cv2  # OpenCV module
from matplotlib import pyplot as plt
import time

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math

rospy.init_node('colorThresh', anonymous=True)

# Bridge to convert ROS Image type to OpenCV Image type
cv_bridge = CvBridge()

def nothing(x):
    pass

def main():
    # create HSV tracker bar
    cv2.namedWindow('HSV_Thresholding')
    cv2.createTrackbar('L - h', 'HSV_Thresholding', 0, 255, nothing)
    cv2.createTrackbar('U - h', 'HSV_Thresholding', 255, 255, nothing)
    cv2.createTrackbar('L - s', 'HSV_Thresholding', 0, 255, nothing)
    cv2.createTrackbar('U - s', 'HSV_Thresholding', 255, 255, nothing)
    cv2.createTrackbar('L - v', 'HSV_Thresholding', 0, 255, nothing)
    cv2.createTrackbar('U - v', 'HSV_Thresholding', 255, 255, nothing)

    rospy.Subscriber('/pupil_capture/image_raw', Image, colorThreshCallback)
    print("Subscribing")
    rospy.spin()


def colorThreshCallback(msg):
    # convert ROS image to opencv format
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # visualize it in a cv window
    cv2.imshow("Original_Image", cv_image)
    cv2.waitKey(3)

    ################ HSV THRESHOLDING ####################
    # conver to HSV
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # get threshold values
    l_h = cv2.getTrackbarPos('L - h', 'HSV_Thresholding')
    u_h = cv2.getTrackbarPos('U - h', 'HSV_Thresholding')
    l_s = cv2.getTrackbarPos('L - s', 'HSV_Thresholding')
    u_s = cv2.getTrackbarPos('U - s', 'HSV_Thresholding')
    l_v = cv2.getTrackbarPos('L - v', 'HSV_Thresholding')
    u_v = cv2.getTrackbarPos('U - v', 'HSV_Thresholding')
    lower_bound_HSV = np.array([l_h, l_s, l_v])
    upper_bound_HSV = np.array([u_h, u_s, u_v])
    # threshold
    mask_HSV = cv2.inRange(hsv_image, lower_bound_HSV, upper_bound_HSV)

    # get display image
    disp_image_HSV = cv2.bitwise_and(cv_image,cv_image, mask= mask_HSV)
    cv2.imshow("HSV_Thresholding", disp_image_HSV)
    cv2.waitKey(3)

if __name__=='__main__':
    main()
