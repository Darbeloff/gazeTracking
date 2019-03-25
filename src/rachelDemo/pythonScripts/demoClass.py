#!/usr/bin/env python
import rospy
import time
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from pupil_ros_plugin.msg import gaze_positions, gaze, pupil, pupil_positions


class demoProcessor:
    def __init__(self):
        # subscribers
        #self.worldImSub = rospy.Subscriber("/usb_cam/image_raw", Image, self.worldImCallback) # Cam
        self.worldImSub = rospy.Subscriber("/pupil_capture/image_raw", Image, self.worldImCallback) # Cam
        self.gazePosSub = rospy.Subscriber("/pupil_capture/gaze", gaze_positions, self.gazePosCallback)

        # cv bridge for conversions between ros and cv
        self.bridge = CvBridge()

        # images from/for ros
        self.cvIm = [] # original, unchanged, conversion of rosIm
        self.rosIm = [] # ros sensor image type that is receieved from readCamera

        # other variables
        self.gaze = []
        self.noImage = True


############################# Subscriber Callback functions ####################
    def worldImCallback(self, data):
        # convert to an opencv image
        try:
            self.cvIm = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # crop image about gaze position
            globalDim = self.cvIm.shape
            gazePosPixX = globalDim[0]*self.gaze[0].norm_pos.x
            gazePosPixY = globalDim[1]*self.gaze[0].norm_pos.y
            gazeCropRegion = 150

            # get x and y coordinates for crop
            gazeCropXBeg = int(gazePosPixX-gazeCropRegion)
            gazeCropXEnd = int(gazePosPixX+gazeCropRegion)
            gazeCropYBeg = int(gazePosPixY-gazeCropRegion)
            gazeCropYEnd = int(gazePosPixY+gazeCropRegion)

            # confirm those coordinates make sense
            if gazeCropXBeg < 0:
            	gazeCropXBeg = 0
                gazeCropXEnd = gazeCropXEnd - gazeCropXBeg
            if gazeCropXEnd > globalDim[0]:
            	gazeCropXEnd = globalDim[0]
                gazeCropXBeg = gazeCropXBeg - (gazeCropXEnd-globalDim[0])
            if gazeCropYBeg < 0:
            	gazeCropYBeg = 0
                gazeCropYEnd = gazeCropYEnd - gazeCropYBeg
            if gazeCropYEnd > globalDim[0]:
            	gazeCropYEnd = globalDim[1]
                gazeCropYBeg = gazeCropYBeg - (gazeCropYEnd-globalDim[1])


            # crop
            gazeCropIm = self.cvIm[gazeCropXBeg:gazeCropXEnd,gazeCropYBeg:gazeCropYEnd]

            # downsample both images
            gazeCropIm = cv2.resize(gazeCropIm,(0,0),fx=0.1,fy=0.1)
            self.cvIm = cv2.resize(self.cvIm,(0,0),fx=0.1,fy=0.1)


            # first with full image
            # convert to gray
            #globalGrayIm = cv2.cvtColor(self.cvIm, cv2.COLOR_BGR2GRAY)
            #cropGrayIm = cv2.cvtColor(gazeCropIm, cv2.COLOR_BGR2GRAY)

            # filter
            #globalFiltIm = cv2.bilateralFilter(globalGrayIm, 5, 50, 50)  # keeps edges a little nicer
            #croplFiltIm = cv2.bilateralFilter(cropGrayIm, 5, 50, 50)  # keeps edges a little nicer

            ############ adaptive thresholding, using bin size dictated by the size of the image
            #globalAdaptThreshBinSize = int(3*globalDim[0]) - 1
            #globalAdaptThreshIm = cv2.adaptiveThreshold(globalFiltIm, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, globalAdaptThreshBinSize, 2)
            #cropAdaptThreshBinSize = int(0.5*gazeCropRegion*2) - 1
            #cropAdaptThreshIm = cv2.adaptiveThreshold(croplFiltIm, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, cropAdaptThreshBinSize, 2)

            ############# rgb thresholding
            #cropHSV = cv2.cvtColor(gazeCropIm,cv2.COLOR_BGR2HSV)
            #globalHSV = cv2.cvtColor(self.cvIm,cv2.COLOR_BGR2HSV)
            #lowerBound = np.array([110,50,50])
            #upperBound = np.array([130,255,255])
            #cropMask = cv2.inRange(cropHSV, lowerBound, upperBound)
            #cropResult = cv2.bitwise_and(gazeCropIm, gazeCropIm, mask= cropMask)
            #globalMask = cv2.inRange(globalHSV, lowerBound, upperBound)
            #globalResult = cv2.bitwise_and(self.cvIm, self.cvIm, mask= globalMask)

            print('got here')

            ########### harris corner detection
            # make gray
            globalGrayIm = cv2.cvtColor(self.cvIm, cv2.COLOR_BGR2GRAY)
            cropGrayIm = cv2.cvtColor(gazeCropIm, cv2.COLOR_BGR2GRAY)

            # cast as float32 and detect
            cropDST = cv2.cornerHarris(np.float32(cropGrayIm),2,3,0.04)
            globalDST = cv2.cornerHarris(np.float32(globalGrayIm),2,3,0.04)

            # dilate
            cropDil = cv2.dilate(cropDST,None)
            globalDil = cv2.dilate(globalDST,None)

            # threshold
            gazeCropIm[cropDil>0.01*cropDil.max()]=[0,0,255]
            self.cvIm[globalDil>0.01*globalDil.max()]=[0,0,255]

            # display
            cv2.imshow('Whole Image',cv2.resize(self.cvIm,(0,0),fx=10,fy=10))
            cv2.waitKey(3)

            cv2.imshow('Cropped Image',cv2.resize(gazeCropIm,(0,0),fx=10,fy=10))
            cv2.waitKey(3)
        except:
            pass


    def gazePosCallback(self, data):
        self.gaze = data.gazes
