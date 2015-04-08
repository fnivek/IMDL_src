#!/usr/bin/env python
import rospy
import serial

def main():
	#Init ros
	rospy.init_node('hardware_interface', anonymous=False)

	rospy.spin()

if __name__ == '__main__':
	main()