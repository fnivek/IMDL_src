#!/usr/bin/env python

# Sums all pfields that it is listining to and publishes result at 10 hz
# TODO make it smarter what if something publishes at 1 Hz... the Kinect cough cough

import rospy
from geometry_msgs.msg import Twist
from percept_generators.msg import pfield

motor_pub = rospy.Publisher("motor_cmd", Twist, queue_size = 10)

def newPfield(field):
	msg = Twist()
	msg.linear.x = field.x
	msg.angular.z = field.y
	motor_pub.publish(msg)

def main():
	# Ros initilization
	#Init ros
	rospy.init_node('pfield_sum', anonymous=False)

	rospy.Subscriber("/percepts/sonar_pfield", pfield, newPfield)

	rospy.spin()

if __name__ == '__main__':
	main()