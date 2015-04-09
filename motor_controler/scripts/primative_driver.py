#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

left_duty_pub = rospy.Publisher("left_duty", Float32, queue_size = 10)
right_duty_pub = rospy.Publisher("right_duty", Float32, queue_size = 10)


#Callback for motor command
#		
# Equation:
#	TODO:
#
#
def motorCmdCb(twist):
	# Temporarily treat this like force...
	#	| Fx |   | 1     1 | | Fm1 |
	#	| T  | = | L/2  -L/2| | Fm2 |
	#
	#	Fm1 = F/2 + T / L
	#	Fm2 = F/2 - T /L
	#
	#	L = 6in
	#							Fudge constants
	vel = twist.linear.x		/ 10
	ang_vel = twist.angular.z	/ 10

	# Set just force component
	left_duty = vel / 2 + ang_vel / 0.1524
	right_duty = vel / 2 - ang_vel / 0.1524

	# Scale the duty cycle
	diff = abs(left - right)
	if abs(left - right) > 2:
		rospy.logwarn('Can not produced desired twist')
		if left < right:
			left = -1 
			right = 1
		else:
			left = 1 
			right = -1

	if(left > 1):
		rospy.logwarn('Left duty cycle above max')
		shift = left - 1 
		left = 1 
		right -= shift

	elif left < -1:
		rospy.logwarn('Left duty cycle below min')
		shift = left + 1
		left = -1
		right -= shift

	if(right > 1):
		rospy.logwarn('Right duty cycle above max')
		shift = right - 1 
		right = 1 
		left -= shift

	elif right < -1:
		rospy.logwarn('Right duty cycle below min')
		shift = right + 1
		right = -1
		left -= shift

	left_duty_pub.publish(Float32(left_duty))
	right_duty_pub.publish(Float32(right_duty))


def main():
	# Ros initilization
	#Init ros
	rospy.init_node('primative_driver', anonymous=False)

	rospy.Subscriber("motor_cmd", Twist, motorCmdCb)

	rospy.spin()

if __name__ == '__main__':
	main()