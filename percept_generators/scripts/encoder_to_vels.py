#!/usr/bin/env python

# This file does forward kinematics for the data from the encoders
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32MultiArray

vels_pub = rospy.Publisher("motor_vels", Float64MultiArray, queue_size = 10)

last_encoder = [0, 0]
last_time = 0
rads_per_tick = 2 * 3.14159265359 / 3730

def encoder_cb(msg):
	global last_encoder
	global last_time
	now = rospy.get_time()
	time_step = now - last_time
	# Not sure what causes this but this prevents it
	if time_step < 0.001:
		return

	last_time = now

	data = msg.data
	diff = [0,0]
	for index in range(2):
		if(last_encoder[index] > 0xF000 and data[index] < 1000):
			diff[index] = (0xFFFF - last_encoder[index]) + data[index]
		elif(last_encoder[index] < 1000 and data[index] > 0xF000):
			diff[index] = -1 * (last_encoder[index] + (0xFFFF - data[index]))
		else:
			diff[index] = last_encoder[index] - data[index]

	# Reset last_encoder
	last_encoder = data

	# Convert encoder diff to angular velocity and publish
	#	diff * (rads/ticks) / time_step
	for index in range(2):
		diff[index] = diff[index] * rads_per_tick / time_step

	out = Float64MultiArray()
	out.data = diff
	vels_pub.publish(out)



def main():
	# Ros initilization
	#Init ros
	rospy.init_node('encoder_to_vels', anonymous=False)

	rospy.Subscriber("motor_position", Int32MultiArray, encoder_cb)

	rospy.spin()

if __name__ == '__main__':
	main()