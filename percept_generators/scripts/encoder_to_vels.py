#!/usr/bin/env python

# This file does forward kinematics for the data from the encoders
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32MultiArray

from collections import deque

vels_pub = rospy.Publisher("motor_vels", Float64MultiArray, queue_size = 10)

last_diff_encoder = [0, 0]
last_abs_encoder = [0, 0]
sizeOfDeques = 3
encoder_diff_values = [deque() for x in range(2)]
first = True

last_time = 0
rads_per_tick = 2 * 3.14159265359 / 3730

def avg(seq):
	return reduce(lambda x, y: x + y, seq) / len(seq)

def encoder_cb(msg):
	global last_diff_encoder
	global last_abs_encoder
	global last_time
	global encoder_diff_values
	global first

	# Timer stuff
	now = rospy.get_time()
	time_step = now - last_time
	# Not sure what causes this but this prevents it
	if time_step < 0.001:
		return
	last_time = now
	
	data = msg.data

	if first is True:
		# Get initial last_abs_encoder
		last_abs_encoder = data
		first = False
	
	# get diff
	diff = [0,0]
	for index in range(2):
		if(last_abs_encoder[index] > 0xF000 and data[index] < 1000):
			diff[index] = (0xFFFF - last_abs_encoder[index]) + data[index]
		elif(last_abs_encoder[index] < 1000 and data[index] > 0xF000):
			diff[index] = -1 * (last_abs_encoder[index] + (0xFFFF - data[index]))
		else:
			diff[index] = last_abs_encoder[index] - data[index]

	#print 'Diff: %f, %f' % (diff[0], diff[1])

	# Reset last_abs_encoder
	last_abs_encoder = data

	# Record Last diff
	index = 0
	if len(encoder_diff_values[0]) >= sizeOfDeques:
		for encoder_tick in diff:
			encoder_diff_values[index].popleft()
			encoder_diff_values[index].append(encoder_tick)
			index += 1
	else:
		for encoder_tick in diff:
			encoder_diff_values[index].append(encoder_tick)
			index += 1

	#print 'Encoder diff values:', encoder_diff_values

	# Get avg
	encoder_avg = map(avg, encoder_diff_values)

	#print 'Encoder avg values:', encoder_avg

	# Convert encoder diff to angular velocity and publish
	#	avg_diff * (rads/ticks) / time_step
	ang_vel = [0, 0]
	for index in range(2):
		ang_vel[index] = encoder_avg[index] * rads_per_tick / time_step

	out = Float64MultiArray()
	out.data = ang_vel
	vels_pub.publish(out)



def main():
	# Ros initilization
	#Init ros
	rospy.init_node('encoder_to_vels', anonymous=False)

	rospy.Subscriber("/hardware_interface/motor_position", Int32MultiArray, encoder_cb)

	rospy.spin()

if __name__ == '__main__':
	main()