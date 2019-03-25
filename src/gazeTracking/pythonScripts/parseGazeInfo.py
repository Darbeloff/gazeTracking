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

rospy.init_node('parseGaze', anonymous=True)

# Bridge to convert ROS Image type to OpenCV Image type
cv_bridge = CvBridge()

# declare some global variables
global firstTime, oldGrayIm, oldFeaturePoints, lk_params, mask, color
firstTime = True
oldGrayIm = None
oldFeaturePoints = None
lk_params = None
mask = None
color = None

def main():
    # ros subscribers
    rospy.Subscriber('/pupil_capture/image_raw', Image, parseGazeCallback)

    # ros maintenance
    rospy.spin()


def parseGazeCallback(msg):
    # some globals
    global firstTime, oldGrayIm, oldFeaturePoints, lk_params, mask, color

    # convert ROS image to opencv format
    try:
        cvIm = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # setup for feature detection...get first frame
    if firstTime:
        # get first frame and convert to gray
        oldIm = cvIm
        oldGrayIm = cv2.cvtColor(oldIm, cv2.COLOR_BGR2GRAY)

        # get feature points
        oldFeaturePoints = getFeaturesShiTomasi(oldGrayIm)

        # Create a mask image for drawing purposes
        mask = np.zeros_like(oldIm)
        color = np.random.randint(0,255,(100,3))

        # setup lk params
        lk_params = dict( winSize  = (15,15), maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # flip bool
        firstTime = False
    # optical flow
    else:
        # get frame_gray
        currIm = cvIm
        currGrayIm = cv2.cvtColor(currIm, cv2.COLOR_BGR2GRAY)

        # calculate optical flow
        newFeaturePoints, st, err = cv2.calcOpticalFlowPyrLK(oldGrayIm, currGrayIm, oldFeaturePoints, None, **lk_params)

        # Select good points
        goodNew = newFeaturePoints[st==1]
        goodOld = oldFeaturePoints[st==1]

        # draw the tracks
        for i,(new,old) in enumerate(zip(goodNew,goodOld)):
            a,b = new.ravel()
            c,d = old.ravel()
            mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
            currIm = cv2.circle(currIm,(a,b),5,color[i].tolist(),-1)
        dispIm = cv2.add(currIm,mask)
        cv2.imshow('Tracking',dispIm)
        cv2.waitKey(3)

        # Now update the previous frame and previous points
        oldGrayIm = currGrayIm.copy()
        oldFeaturePoints = goodNew.reshape(-1,1,2)

def getFeaturesHSV(im):
    lower_bound_HSV = np.array([30, 0, 0])
    upper_bound_HSV = np.array([55, 255, 255])

    # threshold
    mask_HSV = cv2.inRange(im, lower_bound_HSV, upper_bound_HSV)

    # get display image
    disp_image_HSV = cv2.bitwise_and(im,im, mask= mask_HSV)
    cv2.imshow("HSV_Thresholding", disp_image_HSV)
    cv2.waitKey(3)

def getFeaturesShiTomasi(im):
    # params for ShiTomasi corner detection
    feature_params = dict( maxCorners = 100, qualityLevel = .7, minDistance = 7, blockSize = 7 )

    # detect feature
    return cv2.goodFeaturesToTrack(im, mask = None, **feature_params)


if __name__=='__main__':
    # lets go
    main()
