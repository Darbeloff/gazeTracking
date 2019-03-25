#!/usr/bin/env python
import rospy
import time
import numpy as np
import cv2
from matplotlib import pyplot as plt

# ros
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray

# pupil
from pupil_ros_plugin.msg import gaze_positions, gaze, pupil, pupil_positions


class demoProcessor:
    def __init__(self):
        # subscribers
        #self.worldImSub = rospy.Subscriber("/usb_cam/image_raw", Image, self.worldImCallback) # Cam
        self.worldImSub = rospy.Subscriber("/pupil_capture/image_raw", Image, self.worldImCallback) # Cam
        self.gazePosSub = rospy.Subscriber("/pupil_capture/gaze", gaze_positions, self.gazePosCallback)

        # cv bridge for conversions between ros and cv
        self.cv_bridge = CvBridge()

        # images from/for ros
        self.gotImage = False
        self.cvIm = None # original, unchanged, conversion of rosIm
        self.rosIm = None # ros sensor image type that is receieved from readCamera
        self.labeledIm = None # display image

        # optical flow stuff
        self.firstTime = True
        self.oldGrayIm = None
        self.oldFeaturePoints = None
        self.goodNewFeaturePoints = None
        self.currFeaturePointLocation = None
        self.globalCoordinate = None

        # optical flow display
        self.mask = None
        self.color = None

        # Parameters for lucas kanade optical flow
        self.lk_params = dict( winSize  = (15,15), maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # gaze variables
        self.gotGaze = False
        self.gaze = None
        self.globalDim = None
        self.currGazeLocation = None
        self.globalGazeLocation = []

############################# Subscriber Callback functions ####################
    def worldImCallback(self, data):
        try:
            self.cvIm = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")

            # setup for feature detection...get first frame
            if self.firstTime:
                # get first frame and convert to gray...also get shape
                self.oldIm = self.cvIm
                self.oldGrayIm = cv2.cvtColor(self.oldIm, cv2.COLOR_BGR2GRAY)
                self.globalDim = self.cvIm.shape

                # get feature points
                self.oldFeaturePoints = self.getFeaturesShiTomasi(self.oldGrayIm)

                # get first feature point as "global" coordinate
                self.globalCoordinate = self.oldFeaturePoints[0]
                self.labeledIm = self.cvIm

                # Create a mask image for drawing purposes
                self.mask = np.zeros_like(self.oldIm)
                self.color = np.random.randint(0,255,(100,3))

                # flip bool
                self.firstTime = False
            # optical flow
            else:
                # get frame_gray
                currIm = self.cvIm
                currGrayIm = cv2.cvtColor(currIm, cv2.COLOR_BGR2GRAY)

                # calculate optical flow
                self.newFeaturePoints, st, err = cv2.calcOpticalFlowPyrLK(self.oldGrayIm, currGrayIm, self.oldFeaturePoints, None, **self.lk_params)

                # Select good points
                self.goodNewFeaturePoints = self.newFeaturePoints[st==1]
                goodOldFeaturePoints = self.oldFeaturePoints[st==1]

                # draw the tracks
                for i,(new,old) in enumerate(zip(self.goodNewFeaturePoints,goodOldFeaturePoints)):
                    a,b = new.ravel()
                    c,d = old.ravel()
                    mask = cv2.line(self.mask, (a,b),(c,d), self.color[i].tolist(), 2)
                    currIm = cv2.circle(currIm,(a,b),5,self.color[i].tolist(),-1)
                dispIm = cv2.add(currIm,self.mask)
                cv2.imshow('Tracking',dispIm)
                cv2.waitKey(3)

                # Now update the previous frame and previous points
                self.oldGrayIm = currGrayIm.copy()
                self.oldFeaturePoints = self.goodNewFeaturePoints.reshape(-1,1,2)

                # we can continue now
                self.gotImage = True
        except CvBridgeError as e:
            print('failed')
            print(e)


    def gazePosCallback(self, data):
        try:
            self.gaze = data.gazes
            self.gazePosPixX = self.globalDim[1]*self.gaze[0].norm_pos.x
            self.gazePosPixY = self.globalDim[0]*(1-self.gaze[0].norm_pos.y)

            # we can continue now
            self.gotGaze = True
        except:
            pass

############################# Get features (for LK) functions ##################
    def getFeaturesHSV(im):
        lower_bound_HSV = np.array([30, 0, 0])
        upper_bound_HSV = np.array([55, 255, 255])

        # threshold
        mask_HSV = cv2.inRange(im, lower_bound_HSV, upper_bound_HSV)

        # get display image
        disp_image_HSV = cv2.bitwise_and(im,im, mask= mask_HSV)
        cv2.imshow("HSV_Thresholding", disp_image_HSV)
        cv2.waitKey(3)

    def getFeaturesShiTomasi(self,im):
        # params for ShiTomasi corner detection
        feature_params = dict( maxCorners = 100, qualityLevel = .7, minDistance = 7, blockSize = 7 )

        # detect feature
        return cv2.goodFeaturesToTrack(im, mask = None, **feature_params)

############################# Buffer functions #################################
    def getLatestFeaturePointLocation(self):
        try:
            self.currFeaturePointLocation = self.goodNewFeaturePoints[0]
        except:
            pass

    def getLatestGazeLocation(self):
        try:
            self.currGazeLocation = np.array([self.gazePosPixX, self.gazePosPixY])
        except:
            pass

############################# Gaze global location functions ###################
    def gazeGlobalLocation(self):
        # get latest gaze and feature point location
        self.getLatestFeaturePointLocation()
        self.getLatestGazeLocation()
        print('global:' + str(self.globalCoordinate))
        print('feature:' + str(self.currFeaturePointLocation))
        print('gaze:' + str(self.currGazeLocation))
        # calculate current global location of gaze (relative to the feature points)
        self.globalGazeLocation.append(self.currGazeLocation + (self.globalCoordinate[0] - self.currFeaturePointLocation))


        # draw on image
        x = int(self.globalGazeLocation[-1][0])
        y = int(self.globalGazeLocation[-1][1])
        print('x:' + str(x))
        print('y:' + str(y))

        # draw circle on image
        cv2.circle(self.labeledIm, (x,y), 10, (0,255,0),-1)

        # display
        cv2.imshow('Whole Image',cv2.resize(self.labeledIm,(0,0),fx=1,fy=1))
        cv2.waitKey(3)
