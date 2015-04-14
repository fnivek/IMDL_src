#!/usr/bin/env python

# This file does forward kinematics for the data from the encoders
import rospy
import numpy
from std_msgs.msg import Float64MultiArray

def vels_cb():
	pass

def main():
	# Ros initilization
	#Init ros
	rospy.init_node('encoder_fwd_kinematics', anonymous=False)

	rospy.Subscriber("motor_vels", Float64MultiArray, vels_cb)

	rospy.spin()

if __name__ == '__main__':
	main()