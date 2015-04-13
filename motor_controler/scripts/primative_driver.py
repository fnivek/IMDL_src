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
	diff = abs(left_duty - right_duty)
	if abs(left_duty - right_duty) > 2:
		rospy.logwarn('Can not produced desired twist')
		if left_duty < right_duty:
			left_duty = -1 
			right_duty = 1
		else:
			left_duty = 1 
			right_duty = -1

	if(left_duty > 1):
		rospy.logwarn('Left duty cycle above max')
		shift = left_duty - 1 
		left_duty = 1 
		right_duty -= shift

	elif left_duty < -1:
		rospy.logwarn('Left duty cycle below min')
		shift = left_duty + 1
		left_duty = -1
		right_duty -= shift

	if(right_duty > 1):
		rospy.logwarn('Right duty cycle above max')
		shift = right_duty - 1 
		right_duty = 1 
		left_duty -= shift

	elif right_duty < -1:
		rospy.logwarn('Right duty cycle below min')
		shift = right_duty + 1
		right_duty = -1
		left_duty -= shift

	left_duty_pub.publish(Float32(left_duty))
	right_duty_pub.publish(Float32(right_duty))


def main():
	# Ros initilization
	#Init ros
	rospy.init_node('primative_driver', anonymous=False)

	rospy.Subscriber("motor_cmd", Twist, motorCmdCb)

	rospy.spin()



if __name__ == '__main__':
	try:
		main()
	except e:
		print e
	finally:		
		print 'Warning: motors left in final state before node was shutdown'