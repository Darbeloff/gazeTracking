#!/usr/bin/env python
import sys
import rospy

import parseGazeClass

def main():
    # initialize node
    rospy.init_node('parseGazeMain', anonymous = True)

    # create class
    parseGaze = parseGazeClass.demoProcessor()

    # send send send
    pubRate = 5 # hertz
    rate = rospy.Rate(pubRate) # hertz

    # check everythings working (getting camera feed, lighting looks good) etc.

    while (not rospy.is_shutdown()):
        if parseGaze.gotImage and parseGaze.gotGaze:
            # determine global location of gaze
            parseGaze.gazeGlobalLocation()

        # sleep to maintain loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
