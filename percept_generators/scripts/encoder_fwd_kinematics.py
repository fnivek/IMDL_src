#!/usr/bin/env python

# This file does forward kinematics for the data from the encoders
#	ICC - instantaneous center of curvature
#	R - distance of ICC to the center of wheel base
#	w - anular velocity about ICC
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

# Distance between centers of wheels
wheel_base = 0.3048 # 0.3048 m = 12 in
wheel_radius = 0.06

last_time = 0
last_w = 0
last_R = 0

# Use avg of last R and last w with current
# multiply by the time between cb's
def vels_cb(msg):
	global last_time
	global last_R
	global last_w
	# Time step
	now = rospy.get_time()
	time_step = now - last_time
	last_time = now

	Vl = msg.data[0]
	Vr = msg.data[1]

	if(Vl == Vr):
		print 'Forward Motion: %f' % (Vl * time_step * wheel_radius) # w * t * r = arc length
		return
	# Instantanious values
	R = (wheel_base * (Vl + Vr)) / (2 * (Vr - Vl))
	w = (Vr - Vl) / wheel_base

	R_avg = (R + last_R) / 2
	w_avg = (w + last_w) / 2

	last_R = R
	last_w = w

	# Kinematic equations assuming no slip
	print time_step
	dtheta = time_step * w_avg
	dx = R_avg * np.sin(dtheta)
	dy = R_avg * (1 - np.cos(dtheta))

	print (dx, dy, dtheta)


def main():
	# Ros initilization
	#Init ros
	rospy.init_node('encoder_fwd_kinematics', anonymous=False)

	rospy.Subscriber("motor_vels", Float64MultiArray, vels_cb)

	rospy.spin()

if __name__ == '__main__':
	main()