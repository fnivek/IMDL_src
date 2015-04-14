#!/usr/bin/env python

# This file does forward kinematics for the data from the encoders
#	ICC - instantaneous center of curvature
#	R - distance of ICC to the center of wheel base
#	w - anular velocity about ICC
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
import tf

class node:
	def __init__(self):
		self.motor_vel_sub = rospy.Subscriber('motor_vels', Float64MultiArray, self.vels_cb)
		self.listener = tf.TransformListener()
		self.last_time = 0
		self.last_R = 0
		self.last_w = 0
		self.wheel_base = 0.3048 # 0.3048 m = 12 in
		self.wheel_radius = 0.06

	# Use avg of last R and last w with current
	# multiply by the time between cb's
	def vels_cb(self, msg):

		# Time step
		now = rospy.get_time()
		time_step = now - self.last_time
		self.last_time = now

		Vl = msg.data[0]
		Vr = msg.data[1]

		dx = 0
		dy = 0
		dtheta = 0

		if(Vl == Vr):
			dx = Vl * time_step * self.wheel_radius # w * t * r = arc length
		else:
			# Instantanious values
			R = (self.wheel_base * (Vl + Vr)) / (2 * (Vr - Vl))
			w = (Vr - Vl) / self.wheel_base

			R_avg = (R + self.last_R) / 2
			w_avg = (w + self.last_w) / 2

			self.last_R = R
			self.last_w = w

			# Kinematic equations assuming no slip
			dtheta = time_step * w_avg
			dx = R_avg * np.sin(dtheta)
			dy = R_avg * (1 - np.cos(dtheta))

		trans = [0,0,0]	#x y z
		rot = [0,0,0,1] # x y z w

		try:
			(trans, rot) = self.listener.lookupTransform('/world', '/base_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr('TF error: %s' % e)
			# TODO: make sure we don't reset our position on an error

		# Transform cords
		trash, trash, theta = tf.transformations.euler_from_quaternion(rot)
		mag = np.sqrt(dx*dx + dy*dy)
		dx = np.cos(theta) * mag
		dy = np.sin(theta) * mag
		rot = tf.transformations.quaternion_multiply(rot, tf.transformations.quaternion_from_euler(0, 0, dtheta))		
		
		# Set up a tf brodcaster
		br = tf.TransformBroadcaster()
		br.sendTransform((trans[0] + dx, trans[1] + dy, 0),
						 rot,
						 rospy.Time.now(),
						 'base_link',
						 'world')

def main():
	# Ros initilization
	#Init ros
	rospy.init_node('encoder_fwd_kinematics', anonymous=False)

	this_node = node()

	rospy.spin()

if __name__ == '__main__':
	main()