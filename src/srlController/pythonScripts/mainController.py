# author Jacob Guggenheim
#!/usr/bin/env python
import rospy
import time
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from pupil_ros_plugin.msg import gaze_positions, gaze, pupil, pupil_positions
from apriltags2_ros.msg import AprilTagDetectionArray


# define some global variables
def defineGlobalVars():
	global humArmVel
	humArmVel = 0
	global humGripVel
	humGripVel = 0
	global encVal
	encVal = 0
	global aprilTagPos
	aprilTagPos = []
	global pupilPos
	pupilPos = []



def humCallback(data):
	# max velocity
	maxHumVel = 2 # degrees per second?

	# make deadband because joy is annoying
	joyVel = data.axes[1]
	if abs(data.axes[1]) < .1:
		joyVel = 0

	global humArmVel
	humArmVel = float(maxHumVel*joyVel)

	# now get gripper velocity
	global humGripVel
	if data.buttons[0] == 1:
		humGripVel = -1
	elif data.buttons[1] == 1:
		humGripVel = 1
	else:
		humGripVel = 0


def ardCallback(data):
	global encVal
	encVal = float(data.data[0])

def aprilCallback(data):
	global aprilTagPos

	# for now just check if we got something at all...
	aprilTagPos = data.detections
	print(aprilTagPos)

def pupilCallback(data):
	global pupilPos
	pupilPos = data.norm_pos

def start():
	# initialize node
	rospy.init_node('mainController', anonymous = True)

	# commands publisher
	global commands2ArduinoPub
	commands2ArduinoPub = rospy.Publisher('commands2Arduino', Float32MultiArray, queue_size = 10)

	# subscribe to joy, our human command source
	humSubscriber = rospy.Subscriber("joy", Joy, humCallback)

	# subscribe to arduino limit switch
	ardSubscriber = rospy.Subscriber('ard2Control', Float32MultiArray, ardCallback)

	# subscribe to april tags
	aprilSubscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, aprilCallback)

	# subscribe to pupil labs gaze tracking
	pupilSubscriber = rospy.Subscriber('/pupil_capture/gaze', gaze, pupilCallback)

	# go to send command loop
	sendCommand()

	# rospy.spin()
	rospy.spin()

def sendCommand():
	# send send send
	pubRate = 50 # hertz
	rate = rospy.Rate(pubRate) # hertz

	# parameters
	stallTime = 3 # how long we stop during state 2 and 4
	armCommandState = 1
	commandSignals = [0,0]
	commandSignalsForPub = Float32MultiArray(data=commandSignals)
	autoCommandEffort = 1
	humanCommand = False

	while (not rospy.is_shutdown()):
		commandSignals[0] = humArmVel
		commandSignals[1] = humGripVel

		# publish that signal
		commands2ArduinoPub.publish(commandSignalsForPub)

		# sleep to maintain loop rate
		rate.sleep()


if __name__ == '__main__':
    try:
	defineGlobalVars()
	start()
    except rospy.ROSInterruptException:
        pass
