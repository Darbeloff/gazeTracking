#!/usr/bin/env python
import sys
import rospy

import demoClass

def main():
	# initialize node
	rospy.init_node('demoMain', anonymous = True)

	# create class
	demo = demoClass.demoProcessor()

	# send send send
	pubRate = 50 # hertz
	rate = rospy.Rate(pubRate) # hertz

	#while demo.noImage:
		#demo.getLatestImage()

	while (not rospy.is_shutdown()):
		rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
