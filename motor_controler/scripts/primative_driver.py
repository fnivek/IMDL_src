#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

left_duty_pub = rospy.Publisher("left_duty", Float32, queue_size = 10)
right_duty_pub = rospy.Publisher("right_duty", Float32, queue_size = 10)

def scaleDuty(duty):
	if(duty > 1):
		rospy.logwarn('Duty cycle above max')
		return 1
	elif duty < -1:
		rospy.logwarn('Duty cycle below min')
		return -1
	else:
		return duty

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
	#													L = 6in
	left_duty = scaleDuty(twist.linear.x / 2 + twist.angular.z / 0.1524) 
	right_duty = scaleDuty(twist.linear.x / 2 - twist.angular.z / 0.1524)

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